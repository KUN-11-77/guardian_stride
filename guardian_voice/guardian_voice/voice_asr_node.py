#!/usr/bin/env python3
"""麦克风 → arecord 子进程 → VAD → faster-whisper → /voice/text_in

用 arecord 子进程代替 sounddevice/PortAudio(后者在本机不稳定)。
`alsa_device` 参数可指向任何 ALSA 设备(BOYA、RealSense、内置麦均可)。
"""
import queue
import subprocess
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from faster_whisper import WhisperModel
except ImportError:
    WhisperModel = None


class VoiceAsrNode(Node):
    def __init__(self):
        super().__init__('voice_asr_node')

        # ---------- 参数 ----------
        self.declare_parameter('alsa_device', 'plughw:1,0')   # BOYA mini 默认
        self.declare_parameter('native_sample_rate', 48000)     # 设备原生采样率
        self.declare_parameter('sample_rate', 16000)             # ASR 目标采样率
        self.declare_parameter('frame_duration_ms', 30)
        self.declare_parameter('silence_threshold_ms', 700)
        self.declare_parameter('max_utterance_s', 8.0)
        self.declare_parameter('energy_threshold', 0.012)
        self.declare_parameter('asr_model', 'Systran/faster-whisper-tiny')
        self.declare_parameter('asr_language', 'zh')
        self.declare_parameter('asr_device', 'cpu')
        self.declare_parameter('asr_compute_type', 'int8')

        self.alsa_device = self.get_parameter('alsa_device').value
        self.native_sr = int(self.get_parameter('native_sample_rate').value)
        self.sample_rate = int(self.get_parameter('sample_rate').value)
        self.frame_ms = int(self.get_parameter('frame_duration_ms').value)
        self.silence_ms = int(self.get_parameter('silence_threshold_ms').value)
        self.max_utt_s = float(self.get_parameter('max_utterance_s').value)
        self.energy_thr = float(self.get_parameter('energy_threshold').value)
        asr_model_name = self.get_parameter('asr_model').value
        self.asr_lang = self.get_parameter('asr_language').value
        asr_device = self.get_parameter('asr_device').value
        asr_compute = self.get_parameter('asr_compute_type').value

        # ---------- 发布 ----------
        self.text_pub = self.create_publisher(String, '/voice/text_in', 10)

        # ---------- 加载 ASR 模型 ----------
        if WhisperModel is None:
            self.get_logger().error('faster-whisper 未安装，无法启动 ASR')
            raise RuntimeError('faster-whisper missing')
        self.get_logger().info(
            f'加载 ASR 模型 {asr_model_name} ({asr_device}/{asr_compute})...'
        )
        self.asr = WhisperModel(
            asr_model_name, device=asr_device, compute_type=asr_compute
        )
        self.get_logger().info('ASR 模型加载完毕。')

        # ---------- arecord 子进程 ----------
        # BOYA mini 是 48000Hz,S16_LE,单声道
        self._decim = max(1, self.native_sr // self.sample_rate)
        self.frame_samples = self.native_sr * self.frame_ms // 1000
        self.audio_q: queue.Queue[np.ndarray] = queue.Queue()
        self._stopped = False
        self._utt_lock = threading.Lock()

        self.arecord_proc = subprocess.Popen(
            ['arecord', '-D', self.alsa_device,
             '-f', 'S16_LE', '-r', str(self.native_sr),
             '-c', '1', '-t', 'raw', '-q'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=0,
            start_new_session=True,   # 脱离父进程组,父死时不拖累
        )
        threading.Thread(target=self._arecord_reader, daemon=True).start()
        threading.Thread(target=self._vad_loop, daemon=True).start()

        # ---------- VAD 状态 ----------
        self.buffer: list[np.ndarray] = []
        self.silent_frames = 0
        self.silence_frame_thr = max(1, int(self.silence_ms / self.frame_ms))
        self.max_utt_frames = int(self.max_utt_s * 1000 / self.frame_ms)

        self.get_logger().info(
            f'voice_asr_node 启动 device={self.alsa_device} '
            f'native_sr={self.native_sr} → sr={self.sample_rate} '
            f'decim={self._decim} silence_thr={self.energy_thr}'
        )

    def _arecord_reader(self) -> None:
        """从 arecord 子进程的 stdout 持续读 raw PCM,decimate 后入队。"""
        chunk_bytes = self.frame_samples * 2  # S16_LE = 2 bytes/sample
        while not self._stopped:
            try:
                chunk = self.arecord_proc.stdout.read(chunk_bytes)
            except Exception as e:
                self.get_logger().error(f'arecord 读取失败: {e}')
                break
            if not chunk:
                break
            if len(chunk) < chunk_bytes:
                # 末尾不完整帧,补零
                chunk = chunk + b'\x00' * (chunk_bytes - len(chunk))
            data = np.frombuffer(chunk, dtype=np.int16).astype(np.float32) / 32768.0
            if self._decim > 1:
                data = data[::self._decim]
            self.audio_q.put(data)

    def _vad_loop(self) -> None:
        """VAD:累积语音帧,静音超时后送 ASR。"""
        while rclpy.ok() and not self._stopped:
            try:
                frame = self.audio_q.get(timeout=0.5)
            except queue.Empty:
                continue

            rms = float(np.sqrt(np.mean(frame * frame) + 1e-12))
            is_speech = rms > self.energy_thr

            with self._utt_lock:
                if is_speech:
                    self.buffer.append(frame)
                    self.silent_frames = 0
                elif self.buffer:
                    self.buffer.append(frame)  # 静音帧也带上,避免截断尾音
                    self.silent_frames += 1

                triggered = (
                    self.buffer
                    and self.silent_frames >= self.silence_frame_thr
                ) or (len(self.buffer) >= self.max_utt_frames)

                if triggered and self.buffer:
                    audio = np.concatenate(self.buffer)
                    self.buffer = []
                    self.silent_frames = 0
                    threading.Thread(
                        target=self._run_asr, args=(audio,), daemon=True
                    ).start()

    def _run_asr(self, audio: np.ndarray) -> None:
        try:
            segments, _ = self.asr.transcribe(
                audio,
                language=self.asr_lang,
                vad_filter=False,
                condition_on_previous_text=False,
            )
            text = ''.join(seg.text for seg in segments).strip()
        except Exception as e:
            self.get_logger().error(f'ASR 推理失败: {e}')
            return

        if not text:
            return
        self.get_logger().info(f'ASR: {text}')
        self.text_pub.publish(String(data=text))

    def destroy_node(self):
        self._stopped = True
        try:
            if self.arecord_proc and self.arecord_proc.poll() is None:
                self.arecord_proc.terminate()
                self.arecord_proc.wait(timeout=2)
        except Exception as e:
            self.get_logger().warn(f'清理 arecord 子进程失败: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceAsrNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # 已经关过就算了


if __name__ == '__main__':
    main()