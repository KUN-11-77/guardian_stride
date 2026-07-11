#!/usr/bin/env python3
"""/voice/text_in → 关键词匹配 → 查障碍距离 → /voice/text_out + /tts/say。"""
import re
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


class VoiceAssistantNode(Node):
    def __init__(self):
        super().__init__('voice_assistant_node')

        self.declare_parameter('obstacle_threshold_m', 1.5)
        self.threshold = float(self.get_parameter('obstacle_threshold_m').value)

        self.sub_text = self.create_subscription(
            String, '/voice/text_in', self._on_text, 10
        )
        self.pub_text = self.create_publisher(String, '/voice/text_out', 10)
        self.pub_say = self.create_publisher(String, '/tts/say', 10)

        # 缓存最新障碍距离
        self.dists = {'front': None, 'left': None, 'right': None}
        self.create_subscription(
            Float32, '/obstacle/front_distance',
            lambda m: self._cache('front', m.data), 10
        )
        self.create_subscription(
            Float32, '/obstacle/left_distance',
            lambda m: self._cache('left', m.data), 10
        )
        self.create_subscription(
            Float32, '/obstacle/right_distance',
            lambda m: self._cache('right', m.data), 10
        )

        self.get_logger().info(f'voice_assistant_node 启动 threshold={self.threshold}m')

    def _cache(self, key: str, val: float) -> None:
        self.dists[key] = val

    def _say(self, text: str) -> None:
        self.get_logger().info(f'回复: {text}')
        self.pub_text.publish(String(data=text))
        self.pub_say.publish(String(data=text))

    def _on_text(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        self.get_logger().info(f'听到: {text}')
        reply = self._match(text)
        if reply:
            self._say(reply)

    def _describe(self, direction: str, distance: Optional[float]) -> str:
        cn = {'front': '前方', 'left': '左侧', 'right': '右侧'}[direction]
        if distance is None:
            return f'{cn}感知还没准备好，请稍等'
        if distance < 0:
            return f'{cn}超出量程，没看到东西'
        if distance < self.threshold:
            return f'警告，{cn} {distance:.1f} 米处有障碍物，请停下'
        return f'{cn}畅通，最近物体在 {distance:.1f} 米'

    def _match(self, text: str) -> Optional[str]:
        # 前方 / 前面 / 前边
        if any(k in text for k in ['前方', '前面', '前边']) and any(
            k in text for k in ['障碍', '东西', '物体', '情况']
        ):
            return self._describe('front', self.dists['front'])

        if any(k in text for k in ['左侧', '左边', '左方']) and any(
            k in text for k in ['障碍', '东西', '物体', '情况']
        ):
            return self._describe('left', self.dists['left'])

        if any(k in text for k in ['右侧', '右边', '右方']) and any(
            k in text for k in ['障碍', '东西', '物体', '情况']
        ):
            return self._describe('right', self.dists['right'])

        if any(k in text for k in ['后方', '后面', '后边']) and any(
            k in text for k in ['障碍', '东西', '物体', '情况']
        ):
            return '我没有后视摄像头，看不到后面'

        # 自我介绍 / 打招呼
        if any(k in text for k in ['你是谁', '介绍一下', '叫什么', '名字']):
            return '我是 guardian 语音助手，可以回答前方和两侧有没有障碍物'

        if any(k in text for k in ['你好', '在吗', '嗨']):
            return '我在，请说'

        if '谢谢' in text or '感谢' in text:
            return '不客气'

        return '没听懂，你可以问：前方有没有障碍物？左侧呢？右侧呢？'


def main(args=None):
    rclpy.init(args=args)
    node = VoiceAssistantNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
