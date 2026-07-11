#!/usr/bin/env python3
"""/tts/say → edge-tts 合成中文 → 通过默认音频设备（USB 音箱）播放。"""
import asyncio
import io
import tempfile
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import edge_tts
except ImportError:
    edge_tts = None

try:
    import pygame
    pygame.mixer.init()
    _PYGAME_OK = True
except Exception as e:  # noqa: BLE001
    pygame = None
    _PYGAME_OK = False
    print(f'[tts_node] pygame.mixer.init 失败: {e}')


class TtsNode(Node):
    def __init__(self):
        super().__init__('tts_node')

        self.declare_parameter('voice', 'zh-CN-XiaoxiaoNeural')
        self.declare_parameter('rate', '+0%')
        self.declare_parameter('volume', '+0%')
        self.declare_parameter('save_dir', '')  # 为空则不落盘

        self.voice = self.get_parameter('voice').value
        self.rate = self.get_parameter('rate').value
        self.volume = self.get_parameter('volume').value
        self.save_dir = self.get_parameter('save_dir').value

        self.sub = self.create_subscription(String, '/tts/say', self._on_say, 10)
        self._lock = asyncio.Lock()
        self.get_logger().info(
            f'tts_node 启动 voice={self.voice} pygame={_PYGAME_OK} '
            f'edge_tts={edge_tts is not None}'
        )

    def _on_say(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        self.get_logger().info(f'TTS: {text}')
        try:
            asyncio.run(self._speak(text))
        except Exception as e:
            self.get_logger().error(f'TTS 失败: {e}')

    async def _speak(self, text: str) -> None:
        if edge_tts is None:
            self.get_logger().warn('edge-tts 未安装，跳过语音输出')
            return
        async with self._lock:
            communicate = edge_tts.Communicate(
                text,
                voice=self.voice,
                rate=self.rate,
                volume=self.volume,
            )
            buf = io.BytesIO()
            async for chunk in communicate.stream():
                if chunk['type'] == 'audio':
                    buf.write(chunk['data'])
            if buf.tell() == 0:
                self.get_logger().warn('TTS 没有生成音频')
                return

            if self.save_dir:
                import os
                os.makedirs(self.save_dir, exist_ok=True)
                fp = os.path.join(self.save_dir, f'{abs(hash(text)) % 10**8}.mp3')
                with open(fp, 'wb') as f:
                    f.write(buf.getvalue())

            if _PYGAME_OK:
                buf.seek(0)
                pygame.mixer.music.load(buf, 'mp3')
                pygame.mixer.music.play()
                while pygame.mixer.music.get_busy():
                    await asyncio.sleep(0.1)
            else:
                with tempfile.NamedTemporaryFile(
                    suffix='.mp3', delete=False
                ) as f:
                    f.write(buf.getvalue())
                    tmp = f.name
                self.get_logger().info(f'已生成 {tmp}（未播放）')


def main(args=None):
    rclpy.init(args=args)
    node = TtsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
