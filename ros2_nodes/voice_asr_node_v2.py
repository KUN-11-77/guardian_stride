#!/usr/bin/env python3
"""BOYA 麦克风 → arecord → VAD → faster-whisper → /voice/text_in (v2)

改进:
1. TTS 播放时自动抑制 ASR，防止录入机器自己的回答造成死循环
2. 中文 initial_prompt 提升识别准确率
3. 更激进的静音检测 + 尾音保留
"""
import queue
import subprocess
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from faster_whisper import WhisperModel
except ImportError:
    WhisperModel = None

# 中文 navigation 常用词的 prompt，帮助 whisper 偏向正确词汇
ZH_PROMPT = (
    "前方 后方 左侧 右侧 左边 右边 前边 后边 障碍物 有没有 有没有人 安全 危险"
    "可以走 不能走 往哪走 怎么走 停下 继续 多远 几米 你好 谢谢 在吗"
    "请 告诉我 帮忙 前面 后面 哪里 这边 那边"
)


class VoiceAsrNode(Node):
    def __init__(self):
        super().__init__('voice_asr_node')

        # ---------- 参数 ----------
        self.declare_parameter('alsa_device', 'plughw:1,0')
        self.declare_parameter('native_sample_rate', 48000)
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('frame_duration_ms', 30)
        self.declare_parameter('silence_threshold_ms', 600)       # 缩短静音等
        self.declare_parameter('max_utterance_s', 10.0)           # 最长 10s
        self.declare_parameter('energy_threshold', 0.008)          # 降低阈值
        self.declare_parameter('asr_model', 'Systran/faster-whisper-tiny')
        self.declare_parameter('asr_language', 'zh')
        self.declare_parameter('asr_device', 'cpu')
        self.declare_parameter('asr_compute_type', 'int8')
        self.declare_parameter('tts_mute_duration_s', 5.0)         # TTS 后静音

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
        self.tts_mute_s = float(self.get_parameter('tts_mute_duration_s').value)

        # ---------- TTS 回音抑制 ----------
        self.tts_until = 0.0          # 在此时间戳之前抑制 ASR
        self.tts_lock = threading.Lock()
        self.create_subscription(
            String, '/tts/say', self._on_tts, 10)
        self.get_logger().info('已订阅 /tts/say，TTS 播放时将自动抑制麦克风录入')

        # ---------- 发布 ----------
        self.text_pub = self.create_publisher(String, '/voice/text_in', 10)

        # ---------- 加载 ASR 模型 ----------
        if WhisperModel is None:
            self.get_logger().error('faster-whisper 未安装')
            raise RuntimeError('faster-whisper missing')
        self.get_logger().info(
            f'加载 ASR 模型 {asr_model_name} ({asr_device}/{asr_compute})...'
        )
        self.asr = WhisperModel(
            asr_model_name, device=asr_device, compute_type=asr_compute
        )
        self.get_logger().info('ASR 模型加载完毕。')

        # ---------- arecord 子进程 ----------
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
            start_new_session=True,
        )
        threading.Thread(target=self._arecord_reader, daemon=True).start()
        threading.Thread(target=self._vad_loop, daemon=True).start()

        # ---------- VAD 状态 ----------
        self.buffer: list[np.ndarray] = []
        self.silent_frames = 0
        self.silence_frame_thr = max(1, int(self.silence_ms / self.frame_ms))
        self.max_utt_frames = int(self.max_utt_s * 1000 / self.frame_ms)

        self.get_logger().info(
            f'voice_asr_node v2 启动 device={self.alsa_device} '
            f'native_sr={self.native_sr}→sr={self.sample_rate} '
            f'decim={self._decim} thr={self.energy_thr} tts_mute={self.tts_mute_s}s'
        )

    # ======================== TTS 抑制 ========================
    def _on_tts(self, msg: String) -> None:
        """收到 TTS 消息时，设置抑制窗口。"""
        duration = max(self.tts_mute_s, len(msg.data) * 0.15)  # ~150ms/字
        with self.tts_lock:
            self.tts_until = time.time() + duration
        self.get_logger().debug(
            f'TTS 抑制 {duration:.1f}s (到 {self.tts_until:.0f})')

    def _is_muted(self) -> bool:
        """检查当前是否在 TTS 抑制期内。"""
        with self.tts_lock:
            return time.time() < self.tts_until

    # ======================== 音频采集 ========================
    def _arecord_reader(self) -> None:
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
                chunk = chunk + b'\x00' * (chunk_bytes - len(chunk))
            data = np.frombuffer(chunk, dtype=np.int16).astype(np.float32) / 32768.0
            if self._decim > 1:
                data = data[::self._decim]
            self.audio_q.put(data)

    # ======================== VAD + ASR ========================
    def _vad_loop(self) -> None:
        while rclpy.ok() and not self._stopped:
            try:
                frame = self.audio_q.get(timeout=0.5)
            except queue.Empty:
                continue

            # ** TTS 抑制: 直接丢弃音频帧 **
            if self._is_muted():
                continue

            rms = float(np.sqrt(np.mean(frame * frame) + 1e-12))
            is_speech = rms > self.energy_thr

            with self._utt_lock:
                if is_speech:
                    self.buffer.append(frame)
                    self.silent_frames = 0
                elif self.buffer:
                    self.buffer.append(frame)
                    self.silent_frames += 1

                triggered = (
                    self.buffer
                    and self.silent_frames >= self.silence_frame_thr
                ) or (len(self.buffer) >= self.max_utt_frames)

                if triggered and self.buffer:
                    # 再次检查 TTS 抑制(防止刚结束就触发)
                    if self._is_muted():
                        self.buffer = []
                        self.silent_frames = 0
                        continue
                    audio = np.concatenate(self.buffer)
                    self.buffer = []
                    self.silent_frames = 0
                    threading.Thread(
                        target=self._run_asr, args=(audio,), daemon=True
                    ).start()

    def _run_asr(self, audio: np.ndarray) -> None:
        """运行 ASR 推理，使用 initial_prompt 提升中文识别。"""
        try:
            segments, _ = self.asr.transcribe(
                audio,
                language=self.asr_lang,
                initial_prompt=ZH_PROMPT,
                vad_filter=True,                        # 启用内置 VAD
                vad_parameters=dict(
                    threshold=0.5,
                    min_speech_duration_ms=250,
                    min_silence_duration_ms=400,
                ),
                condition_on_previous_text=False,
                without_timestamps=True,
            )
            text = ''.join(seg.text for seg in segments).strip()
        except Exception as e:
            self.get_logger().error(f'ASR 推理失败: {e}')
            return

        if not text:
            return

        # 过滤掉明显是 TTS 回声的文本 (太长的句子可能是机器说的)
        if len(text) > 80:
            self.get_logger().warn(f'疑似 TTS 回声被过滤: {text[:50]}...')
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
            self.get_logger().warn(f'清理 arecord 失败: {e}')
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
            pass


if __name__ == '__main__':
    main()
