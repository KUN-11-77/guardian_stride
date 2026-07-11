#!/usr/bin/env python3
"""Depth image normalizer: 16-bit mono → 8-bit colorized for web streaming.

The web_video_server can't properly encode 16-bit mono depth as MJPEG.
This node converts depth to a color-mapped 8-bit image for the dashboard.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class DepthVizNode(Node):
    def __init__(self):
        super().__init__('depth_viz_node')

        self.bridge = CvBridge()
        self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw',
            self._on_depth, 10)
        self.pub = self.create_publisher(
            Image, '/depth/colorized', 10)

        self.get_logger().info('DepthVizNode started → /depth/colorized')

    def _on_depth(self, msg: Image):
        try:
            # 16-bit mono depth in mm
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        # Clip to 0-6000mm (6m), normalize to 0-255
        depth_clipped = np.clip(depth.astype(np.float32), 0, 6000)
        depth_norm = (depth_clipped / 6000.0 * 255).astype(np.uint8)

        # Apply TURBO colormap for better visualization
        colorized = cv2.applyColorMap(depth_norm, cv2.COLORMAP_TURBO)

        # Mark invalid (0 = no data) as black
        invalid_mask = (depth < 1) | (depth > 6000)
        colorized[invalid_mask] = [20, 20, 20]

        out_msg = self.bridge.cv2_to_imgmsg(colorized, 'bgr8')
        out_msg.header = msg.header
        self.pub.publish(out_msg)


def main():
    rclpy.init()
    node = DepthVizNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
