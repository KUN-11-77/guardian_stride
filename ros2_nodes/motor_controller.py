#!/usr/bin/env python3
"""
DK-2500 步进电机 CAN 控制器 + 安全状态机
==========================================
CAN总线: PEAK PCAN-USB @ can0, 1Mbps
电机: 2×步进电机 (膝盖两侧), 绳索驱动

安全逻辑:
  - 前方障碍<0.8m  → 双电机收紧 = STOP
  - 左侧障碍<0.5m  → 左收紧右放松 = 转向右
  - 右侧障碍<0.5m  → 右收紧左放松 = 转向左
  - 前方畅通       → 双电机放松 = 行走
  - 绳索短, 快速响应, 小角度步进

CAN协议: DJI C610兼容 (CAN ID 0x200, 4×int16电流值)
可配置为通用步进电机CAN协议
"""
import rclpy, struct, socket, time, threading
from rclpy.node import Node
from std_msgs.msg import Float32, String

# CAN frame format (SocketCAN)
CAN_MTU = 16
CAN_EFF_FLAG = 0x80000000

class CANBus:
    """SocketCAN wrapper for PEAK PCAN-USB."""
    def __init__(self, iface='can0'):
        self.iface = iface
        self.sock = None
        self._lock = threading.Lock()

    def open(self):
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((self.iface,))

    def send(self, can_id, data_bytes):
        """Send CAN frame. data_bytes should be <= 8 bytes."""
        with self._lock:
            if self.sock is None:
                return
            try:
                data = data_bytes.ljust(8, b'\x00')[:8]
                frame = struct.pack('=IB3x8s', can_id & 0x7FF, 8, data)
                self.sock.send(frame)
            except Exception:
                pass

    def close(self):
        if self.sock:
            self.sock.close()
            self.sock = None


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Motor config
        self.declare_parameter('can_id_left', 0x200)
        self.declare_parameter('can_id_right', 0x1FF)
        self.declare_parameter('max_current_ma', 3000)    # 最大电流mA
        self.declare_parameter('hold_current_ma', 500)    # 维持力矩mA
        self.declare_parameter('obstacle_stop_m', 0.8)   # 停车距离
        self.declare_parameter('obstacle_warn_m', 1.5)   # 警告距离
        self.declare_parameter('step_angle_deg', 1.8)    # 步进角

        self.can_left = self.get_parameter('can_id_left').value
        self.can_right = self.get_parameter('can_id_right').value
        self.max_ma = self.get_parameter('max_current_ma').value
        self.hold_ma = self.get_parameter('hold_current_ma').value
        self.stop_m = self.get_parameter('obstacle_stop_m').value
        self.warn_m = self.get_parameter('obstacle_warn_m').value

        # CAN bus
        self.can = CANBus('can0')
        try:
            self.can.open()
            self.get_logger().info('CAN bus opened on can0')
        except Exception as e:
            self.get_logger().error(f'CAN open failed: {e} — using mock mode')

        # Sensor cache
        self.front_dist = 6.0
        self.left_dist = 6.0
        self.right_dist = 6.0
        self.voice_cmd = ''
        self.emergency = False

        # Motor state
        self.left_torque = 0     # -1.0 to 1.0
        self.right_torque = 0

        # Subscriptions
        self.create_subscription(Float32, '/obstacle/front_distance',
                                 lambda m: setattr(self, 'front_dist', m.data), 10)
        self.create_subscription(Float32, '/obstacle/left_distance',
                                 lambda m: setattr(self, 'left_dist', m.data), 10)
        self.create_subscription(Float32, '/obstacle/right_distance',
                                 lambda m: setattr(self, 'right_dist', m.data), 10)
        self.create_subscription(Float32, '/nearest_obstacle_m',
                                 lambda m: setattr(self, 'front_dist',
                                                   min(self.front_dist, m.data) if m.data > 0 else self.front_dist), 10)
        self.create_subscription(String, '/voice/text_out',
                                 lambda m: setattr(self, 'voice_cmd', m.data), 10)

        # 100Hz motor update
        self.create_timer(0.01, self._update)
        # 1Hz heartbeat
        self.create_timer(1.0, self._heartbeat)

        self.get_logger().info(
            f'MotorController ready | CAN: can0@{1000000}bps | '
            f'L:0x{self.can_left:03X} R:0x{self.can_right:03X} | '
            f'Stop<{self.stop_m}m Warn<{self.warn_m}m')

    def _update(self):
        """Safety FSM → motor torque commands."""
        f, l, r = self.front_dist, self.left_dist, self.right_dist

        # Default: walk forward (loose ropes)
        lt, rt = 0.0, 0.0

        # EMERGENCY: obstacle very close ahead
        if f < self.stop_m:
            lt, rt = -1.0, -1.0  # both tighten = STOP
        # Obstacle on left → turn right (tighten left, loosen right)
        elif l < self.stop_m * 0.6:
            lt, rt = -0.8, 0.3
        # Obstacle on right → turn left (tighten right, loosen left)
        elif r < self.stop_m * 0.6:
            lt, rt = 0.3, -0.8
        # Warning zone ahead → slow down
        elif f < self.warn_m:
            lt, rt = -0.3, -0.3
        # All clear → light guidance forward
        else:
            lt, rt = 0.2, 0.2

        # Smooth transition
        alpha = 0.3
        self.left_torque = self.left_torque * (1-alpha) + lt * alpha
        self.right_torque = self.right_torque * (1-alpha) + rt * alpha

        # Send CAN commands
        self._send_motor(self.can_left, self.left_torque)
        self._send_motor(self.can_right, self.right_torque)

    def _send_motor(self, can_id, torque):
        """Send torque command as CAN frame.
        torque: -1.0 (full tighten) to 1.0 (full release)
        """
        # Convert to current in mA (-max to +max)
        current = int(torque * self.max_ma)
        current = max(-self.max_ma, min(self.max_ma, current))

        # Pack as int16 (little-endian)
        data = struct.pack('<h', current) + b'\x00' * 6
        self.can.send(can_id, data)

    def _heartbeat(self):
        """Send keepalive + log status."""
        f = self.front_dist
        status = 'STOP' if f < self.stop_m else 'WARN' if f < self.warn_m else 'GO'
        self.get_logger().info(
            f'[{status}] F:{f:.1f}m L:{self.left_dist:.1f}m R:{self.right_dist:.1f}m | '
            f'Motor L:{self.left_torque:+.2f} R:{self.right_torque:+.2f}',
            throttle_duration_sec=5.0)

    def destroy_node(self):
        # Safe stop: hold both motors
        for _ in range(3):
            self._send_motor(self.can_left, 0.0)
            self._send_motor(self.can_right, 0.0)
            time.sleep(0.01)
        self.can.close()
        super().destroy_node()


def main():
    rclpy.init()
    rclpy.spin(MotorController())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
