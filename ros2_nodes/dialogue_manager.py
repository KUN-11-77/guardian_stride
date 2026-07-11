#!/usr/bin/env python3
"""Guardian Dialogue Manager - 智能对话管理器 v1"""
import rclpy, time, threading, re
from enum import Enum
from rclpy.node import Node
from std_msgs.msg import Float32, String

class Mode(Enum):
    SILENT = 0; NAVIGATE = 1; EXPLORE = 2; QUERY = 3

class Risk(Enum):
    SAFE = 0; CAUTION = 1; DANGER = 2

class DialogueManager(Node):
    def __init__(self):
        super().__init__("dialogue_manager")
        self.front = None; self.left = None; self.right = None
        self.prev_risk = Risk.SAFE
        self.mode = Mode.SILENT
        self.user_speaking = False
        self.last_user_speech = 0.0
        self.last_tts = 0.0
        self.announce_history = {}
        self.wake_words = ["小导", "guardian", "守护者"]

        self.create_subscription(Float32, "/obstacle/front_distance", lambda m: setattr(self, "front", m.data), 10)
        self.create_subscription(Float32, "/obstacle/left_distance", lambda m: setattr(self, "left", m.data), 10)
        self.create_subscription(Float32, "/obstacle/right_distance", lambda m: setattr(self, "right", m.data), 10)
        self.create_subscription(String, "/voice/text_in", self._on_user_speech, 10)

        self.pub_tts = self.create_publisher(String, "/tts/say", 10)
        self.pub_motor = self.create_publisher(Float32, "/motor/emergency_stop", 10)
        self.pub_text_out = self.create_publisher(String, "/voice/text_out", 10)

        self.create_timer(0.2, self._loop)
        self.get_logger().info("DialogueManager: 静默优先 事件驱动 用户可打断")
        self._say("Guardian就绪")

    def _on_user_speech(self, msg: String):
        text = msg.data.strip()
        if not text: return
        self.user_speaking = True
        self.last_user_speech = time.time()
        self.get_logger().info(f"USER: {text}")
        self._parse_intent(text)

    def _parse_intent(self, text: str):
        t = text.lower()
        if any(kw in t for kw in ["带我去", "导航到", "去", "怎么走"]):
            self.mode = Mode.NAVIGATE
            goal = re.sub(r".*(?:带我去|导航到|去|怎么走到?)\s*", "", text).strip()
            self._say(f"好的，开始导航到{goal}" if goal else "请问要去哪里")
            return
        if any(kw in t for kw in ["描述", "周围", "附近有什么", "环境"]):
            self.mode = Mode.EXPLORE
            self._describe_environment()
            return
        if any(kw in t for kw in ["安静", "闭嘴", "别说了"]):
            self.mode = Mode.SILENT
            self._say("好的，静默模式")
            return
        prev = self.mode
        self.mode = Mode.QUERY
        self._answer_question(text)
        self.mode = prev

    def _loop(self):
        now = time.time()
        if self.user_speaking and now - self.last_user_speech > 1.5:
            self.user_speaking = False
        if self.user_speaking: return

        risk = self._assess_risk()
        if risk == Risk.DANGER and self.prev_risk != Risk.DANGER:
            self._danger_alert()
        elif risk == Risk.CAUTION and self.prev_risk == Risk.SAFE:
            if self.mode != Mode.SILENT:
                self._caution_alert()
        elif risk == Risk.SAFE and self.prev_risk in (Risk.CAUTION, Risk.DANGER):
            if self.mode != Mode.SILENT:
                self._say("道路恢复畅通")
        self.prev_risk = risk

        if self.mode == Mode.NAVIGATE and now - self.last_tts > 8.0:
            self._nav_guidance()

    def _assess_risk(self):
        f = self.front if (self.front is not None and self.front > 0.1) else 999
        if f < 0.5: return Risk.DANGER
        elif f < 1.2: return Risk.CAUTION
        return Risk.SAFE

    def _danger_alert(self):
        f = self.front; cm = f * 100
        self._say(f"小心！前方{cm:.0f}厘米", priority=True)
        self.pub_motor.publish(Float32(data=-2.0))
        threading.Thread(target=lambda: (time.sleep(3), self.pub_motor.publish(Float32(data=0.0)))).start()

    def _caution_alert(self):
        f = self.front
        l = self.left if (self.left is not None and self.left > 0.1) else 999
        r = self.right if (self.right is not None and self.right > 0.1) else 999
        better = "左" if l > r else "右"
        self._say(f"注意前方{f:.1f}米，建议往{better}")

    def _nav_guidance(self):
        f = self.front if (self.front is not None and self.front > 0.1) else 999
        l = self.left if (self.left is not None and self.left > 0.1) else 999
        r = self.right if (self.right is not None and self.right > 0.1) else 999
        if f > 3.0: self._say("直行")
        elif f < 1.5: self._say("左转" if l > r else "右转")

    def _describe_environment(self):
        f = self.front if (self.front is not None and self.front > 0.1) else 999
        l = self.left if (self.left is not None and self.left > 0.1) else 999
        r = self.right if (self.right is not None and self.right > 0.1) else 999
        parts = []
        parts.append(f"前方{f:.1f}米有障碍物" if f < 3.0 else "前方畅通")
        parts.append(f"左侧{f:.1f}米有物体" if l < 2.0 else "左侧安全")
        parts.append(f"右侧{f:.1f}米有物体" if r < 2.0 else "右侧安全")
        self._say("当前环境：" + "。".join(parts))

    def _answer_question(self, text: str):
        t = text.lower(); f = self.front; l = self.left; r = self.right
        if any(k in t for k in ["前面", "前方", "前边"]):
            self._say(f"前方{f:.1f}米有障碍物" if (f and f < 3.0) else "前方安全"); return
        if any(k in t for k in ["左边", "左侧"]):
            self._say(f"左侧{l:.1f}米有物体" if (l and l < 2.0) else "左侧安全"); return
        if any(k in t for k in ["右边", "右侧"]):
            self._say(f"右侧{r:.1f}米有物体" if (r and r < 2.0) else "右侧安全"); return
        if "你好" in t or "在吗" in t: self._say("我在，请说"); return
        self.pub_text_out.publish(String(data=text))
        self._say("让我想想")

    def _say(self, text: str, priority: bool = False):
        now = time.time(); key = text[:20]
        if key in self.announce_history and now - self.announce_history[key] < 30: return
        self.announce_history[key] = now; self.last_tts = now
        self.get_logger().info(f"TTS [{self.mode.name}]: {text}")
        self.pub_tts.publish(String(data=text))

def main():
    rclpy.init(); rclpy.spin(DialogueManager()); rclpy.shutdown()

if __name__ == "__main__": main()
