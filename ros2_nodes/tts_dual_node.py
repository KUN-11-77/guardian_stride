#!/usr/bin/env python3
"""/tts/say → edge-tts(在线) / piper(离线兜底) → PulseAudio USB扬声器"""
import asyncio, io, os, subprocess, sys, tempfile, threading, time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# edge-tts (在线)
try:
    import edge_tts
    HAVE_EDGE = True
except ImportError:
    HAVE_EDGE = False

# piper-tts (离线)
try:
    from piper.voice import PiperVoice
    import wave
    PIPER_MODEL = os.path.expanduser("~/piper_models/zh_CN-huayan-medium.onnx")
    PIPER_CONFIG = PIPER_MODEL + ".json"
    HAVE_PIPER = os.path.exists(PIPER_MODEL)
    if HAVE_PIPER:
        _PIPER_VOICE = PiperVoice.load(PIPER_MODEL, config_path=PIPER_CONFIG)
        PIPER_SR = _PIPER_VOICE.config.sample_rate
except ImportError:
    HAVE_PIPER = False
    _PIPER_VOICE = None

# 播放器
try:
    import pygame
    pygame.mixer.init()
    PYGAME_OK = True
except Exception:
    pygame = None
    PYGAME_OK = False


class DualTTSNode(Node):
    def __init__(self):
        super().__init__("tts_node")
        self.declare_parameter("voice", "zh-CN-XiaoxiaoNeural")
        self.declare_parameter("rate", "+0%")
        self.declare_parameter("prefer", "edge")  # edge / piper / auto
        
        self.voice = self.get_parameter("voice").value
        self.prefer = self.get_parameter("prefer").value
        self._lock = asyncio.Lock()
        self._edge_ok = HAVE_EDGE
        
        self.sub = self.create_subscription(String, "/tts/say", self._on_say, 10)
        
        self.get_logger().info(
            f"TTS Dual 启动 | edge_tts={HAVE_EDGE} piper={HAVE_PIPER} "
            f"pygame={PYGAME_OK} prefer={self.prefer}")
    
    def _on_say(self, msg: String):
        text = msg.data.strip()
        if not text: return
        self.get_logger().info(f"TTS: {text}")
        threading.Thread(target=self._run, args=(text,), daemon=True).start()
    
    def _run(self, text: str):
        # 策略: edge 优先, 失败则 piper 兜底
        wav_path = None
        source = "unknown"
        
        # 1. 尝试 edge-tts
        if self._edge_ok and self.prefer != "piper":
            try:
                asyncio.run(self._edge_speak(text))
                source = "edge"
                # edge 自己播放了, 返回
                self.get_logger().info("  edge-tts 成功")
                return
            except Exception as e:
                self.get_logger().warn(f"  edge-tts 失败: {str(e)[:80]}")
                self._edge_ok = False  # 标记不可用, 后续跳过
        
        # 2. fallback: piper 离线
        if HAVE_PIPER:
            try:
                wav_path = self._piper_synthesize(text)
                source = "piper"
            except Exception as e:
                self.get_logger().error(f"  piper 失败: {str(e)[:80]}")
        
        # 3. 播放
        if wav_path and os.path.exists(wav_path):
            self._play_wav(wav_path)
            self.get_logger().info(f"  {source} 播放完成 ({os.path.getsize(wav_path)/1024:.0f}KB)")
            os.unlink(wav_path)
        elif not HAVE_PIPER:
            self.get_logger().error("无可用TTS引擎")
    
    async def _edge_speak(self, text: str):
        """edge-tts 合成 + pygame 播放"""
        async with self._lock:
            communicate = edge_tts.Communicate(text, voice=self.voice)
            buf = io.BytesIO()
            async for chunk in communicate.stream():
                if chunk["type"] == "audio":
                    buf.write(chunk["data"])
            if buf.tell() == 0:
                raise RuntimeError("空音频")
            
            if PYGAME_OK:
                buf.seek(0)
                pygame.mixer.music.load(buf, "mp3")
                pygame.mixer.music.play()
                while pygame.mixer.music.get_busy():
                    await asyncio.sleep(0.05)
            else:
                # fallback: 写临时文件用 aplay
                with tempfile.NamedTemporaryFile(suffix=".mp3", delete=False) as f:
                    f.write(buf.getvalue())
                    tmp = f.name
                subprocess.run(["aplay", tmp], timeout=10)
                os.unlink(tmp)
    
    def _piper_synthesize(self, text: str) -> str:
        """piper TTS → WAV 文件"""
        chunks = []
        for audio in _PIPER_VOICE.synthesize(text):
            chunks.append(audio)
        
        if not chunks:
            raise RuntimeError("piper 无音频输出")
        
        all_audio = np.concatenate([c.audio for c in chunks])
        wav_path = tempfile.mktemp(suffix=".wav")
        
        with wave.open(wav_path, "wb") as wf:
            wf.setframerate(_PIPER_VOICE.config.sample_rate)
            wf.setsampwidth(2)
            wf.setnchannels(1)
            wf.writeframes(all_audio.tobytes())
        
        return wav_path
    
    def _play_wav(self, path: str):
        """播放WAV文件, 优先pygame, 回退aplay"""
        if PYGAME_OK:
            pygame.mixer.music.load(path)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                time.sleep(0.05)
        else:
            subprocess.run(["aplay", path], timeout=10)


def main():
    rclpy.init()
    node = DualTTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
