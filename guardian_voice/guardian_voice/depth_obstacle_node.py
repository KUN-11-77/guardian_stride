#!/usr/bin/env python3
"""订阅 RealSense 深度图，发布 /obstacle/{front,left,right}_distance（米）。

复用 gs_perception/scripts/odometry.py:182-191 的中心深度取法（16UC1 毫米 → 中位数）。
"""
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32


VALID_MIN_MM = 200     # RealSense D435 最小可靠深度 ≈ 0.3m
VALID_MAX_MM = 5000    # ~5m 之后精度严重下降


def sample_roi(depth: np.ndarray, cx: int, cy: int, half: int) -> float:
    """取 (cx, cy) 周围 (2*half+1)^2 ROI 的有效深度中位数（米）。"""
    h, w = depth.shape[:2]
    x0, x1 = max(0, cx - half), min(w, cx + half + 1)
    y0, y1 = max(0, cy - half), min(h, cy + half + 1)
    roi = depth[y0:y1, x0:x1].astype(np.float32)
    valid = roi[(roi > VALID_MIN_MM) & (roi < VALID_MAX_MM)]
    if valid.size < 5:
        return -1.0
    return float(np.median(valid)) / 1000.0


class DepthObstacleNode(Node):
    def __init__(self):
        super().__init__('depth_obstacle_node')

        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('roi_half', 12)
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('obstacle_threshold_m', 1.5)

        topic = self.get_parameter('depth_topic').value
        self.half = int(self.get_parameter('roi_half').value)
        rate = float(self.get_parameter('publish_rate_hz').value)

        self.bridge = CvBridge()
        self.latest_depth: Optional[np.ndarray] = None
        self.stale_frames = 0  # 长时间无新数据则视为失效

        self.sub = self.create_subscription(Image, topic, self._depth_cb, 10)
        self.front_pub = self.create_publisher(Float32, '/obstacle/front_distance', 10)
        self.left_pub = self.create_publisher(Float32, '/obstacle/left_distance', 10)
        self.right_pub = self.create_publisher(Float32, '/obstacle/right_distance', 10)

        self.timer = self.create_timer(1.0 / rate, self._publish_distances)
        self.get_logger().info(
            f'depth_obstacle_node 订阅 {topic}, roi_half={self.half}, rate={rate}Hz'
        )

    def _depth_cb(self, msg: Image) -> None:
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge 失败: {e}')
            return
        if depth.dtype not in (np.uint16, np.float32):
            return
        self.latest_depth = depth
        self.stale_frames = 0

    def _publish_distances(self) -> None:
        self.stale_frames += 1
        if self.latest_depth is None or self.stale_frames > 30:
            return
        h, w = self.latest_depth.shape[:2]
        front = sample_roi(self.latest_depth, w // 2, h // 2, self.half)
        left = sample_roi(self.latest_depth, w // 4, h // 2, self.half)
        right = sample_roi(self.latest_depth, 3 * w // 4, h // 2, self.half)
        self.front_pub.publish(Float32(data=front))
        self.left_pub.publish(Float32(data=left))
        self.right_pub.publish(Float32(data=right))


def main(args=None):
    rclpy.init(args=args)
    node = DepthObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
