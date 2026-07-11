#!/usr/bin/env python3
"""Intelligent voice assistant for blind navigation using local Ollama LLM.

Subscribes: /voice/text_in (ASR), obstacle distances, traversability
Publishes: /tts/say (TTS), /voice/text_out (text response)

Uses Ollama qwen2.5:0.5b for understanding user intent and generating
helpful, context-aware navigation guidance for visually impaired users.
"""
import json
import re
import subprocess
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String


SYSTEM_PROMPT = """你是一个盲人助行导航系统的语音助手，名字叫"守护者"。
你的用户是盲人或视障人士，正在借助外骨骼助行器行走。

你的职责：
1. 回答用户关于周围环境的问题（前方/左侧/右侧有没有障碍物）
2. 根据传感器数据提供安全导航建议（向哪边走更安全）
3. 理解各种口语化表达（"前边有啥" = 前方障碍物查询）

回答规则：
- 总是用简洁的中文口语回答，不超过2句话（会被TTS朗读）
- 如果前方有障碍物，明确告知距离和方向建议
- 如果传感器数据为None或负数，说明该方向"超出量程，没探测到物体"
- 回答风格：温暖、简洁、直接，像是导盲犬在说话
- 不要说"根据传感器数据"之类的话，直接说结果
- 距离单位用米，精确到小数点后1位

当前传感器数据：
{context}

用户说：{query}
请用中文回答："""


class LLMVoiceAssistant(Node):
    def __init__(self):
        super().__init__('llm_voice_assistant')

        self.declare_parameter('obstacle_threshold_m', 1.5)
        self.declare_parameter('ollama_model', 'qwen2.5:0.5b')
        self.threshold = float(self.get_parameter('obstacle_threshold_m').value)
        self.model = self.get_parameter('ollama_model').value

        # Subscribers
        self.sub_text = self.create_subscription(
            String, '/voice/text_in', self._on_text, 10)
        self.pub_text = self.create_publisher(String, '/voice/text_out', 10)
        self.pub_say = self.create_publisher(String, '/tts/say', 10)

        # Cache sensor data
        self.dists = {'front': None, 'left': None, 'right': None}
        self.nearest_obstacle = None
        self.traversability = None

        self.create_subscription(
            Float32, '/obstacle/front_distance',
            lambda m: self._cache_dist('front', m.data), 10)
        self.create_subscription(
            Float32, '/obstacle/left_distance',
            lambda m: self._cache_dist('left', m.data), 10)
        self.create_subscription(
            Float32, '/obstacle/right_distance',
            lambda m: self._cache_dist('right', m.data), 10)
        self.create_subscription(
            Float32, '/nearest_obstacle_m',
            lambda m: setattr(self, 'nearest_obstacle', m.data), 10)

        self.get_logger().info(
            f'LLM Voice Assistant started — model={self.model} threshold={self.threshold}m')

    def _cache_dist(self, key, val):
        self.dists[key] = val

    def _build_context(self) -> str:
        """Build context string from current sensor readings."""
        parts = []
        for key, label in [('front', '前方'), ('left', '左侧'), ('right', '右侧')]:
            d = self.dists.get(key)
            if d is None:
                parts.append(f"{label}: 传感器未就绪")
            elif d < 0:
                parts.append(f"{label}: 超出量程，安全")
            elif d < self.threshold:
                parts.append(f"{label}: {d:.1f}米有障碍物⚠️危险")
            else:
                parts.append(f"{label}: {d:.1f}米，安全")
        if self.nearest_obstacle is not None:
            parts.append(f"最近障碍物: {self.nearest_obstacle:.1f}米")
        return '\n'.join(parts)

    def _call_ollama(self, prompt: str, timeout: float = 8.0) -> str:
        """Call Ollama API for response."""
        try:
            proc = subprocess.run(
                ['ollama', 'run', self.model, prompt],
                capture_output=True, text=True, timeout=timeout,
                env={**__import__('os').environ, 'OLLAMA_NUM_PARALLEL': '1'}
            )
            if proc.returncode == 0 and proc.stdout.strip():
                return proc.stdout.strip()
            else:
                self.get_logger().warn(f'Ollama failed: {proc.stderr[:200]}')
                return None
        except subprocess.TimeoutExpired:
            self.get_logger().warn('Ollama timeout')
            return None
        except Exception as e:
            self.get_logger().error(f'Ollama error: {e}')
            return None

    def _say(self, text: str) -> None:
        self.get_logger().info(f'回复: {text}')
        self.pub_text.publish(String(data=text))
        self.pub_say.publish(String(data=text))

    def _on_text(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        self.get_logger().info(f'听到: {text}')

        # Quick fallback for very short queries
        if len(text) <= 2 and text in ['嗯', '啊', '哦', '好', '行']:
            self._say('我在听，请说')
            return

        # Build context and query LLM
        context = self._build_context()
        prompt = SYSTEM_PROMPT.format(context=context, query=text)

        # Run LLM in thread to avoid blocking ROS spinner
        threading.Thread(
            target=self._run_llm, args=(prompt, text), daemon=True
        ).start()

    def _run_llm(self, prompt: str, original_text: str) -> None:
        reply = self._call_ollama(prompt)

        if reply:
            # Clean up common LLM artifacts
            reply = reply.strip().strip('"').strip("'")
            # Remove any "回答：" prefix
            reply = re.sub(r'^(回答|回复|答)[:：]\s*', '', reply)
            # Truncate to reasonable length for TTS
            if len(reply) > 150:
                reply = reply[:147] + '...'
            self._say(reply)
        else:
            # Fallback to simple keyword matching
            self._say(self._fallback_match(original_text))

    def _describe(self, direction: str, distance: Optional[float]) -> str:
        cn = {'front': '前方', 'left': '左侧', 'right': '右侧'}[direction]
        if distance is None:
            return f'{cn}感知还没准备好，请稍等'
        if distance < 0:
            return f'{cn}超出量程，没看到东西'
        if distance < self.threshold:
            return f'警告，{cn} {distance:.1f} 米处有障碍物，请停下'
        return f'{cn}畅通，最近物体在 {distance:.1f} 米'

    def _fallback_match(self, text: str) -> str:
        """Simple keyword fallback if LLM is unavailable."""
        if any(k in text for k in ['前方', '前面', '前边']):
            return self._describe('front', self.dists['front'])
        if any(k in text for k in ['左侧', '左边', '左方']):
            return self._describe('left', self.dists['left'])
        if any(k in text for k in ['右侧', '右边', '右方']):
            return self._describe('right', self.dists['right'])
        if any(k in text for k in ['你是谁', '叫什么', '名字']):
            return '我是守护者，你的盲人助行语音助手，可以告诉你周围有没有障碍物'
        if any(k in text for k in ['你好', '在吗', '嗨']):
            return '我在，请问'
        if '谢谢' in text or '感谢' in text:
            return '不客气'
        return '你可以问我：前方有没有障碍物？左边呢？右边呢？'


def main(args=None):
    rclpy.init(args=args)
    node = LLMVoiceAssistant()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
