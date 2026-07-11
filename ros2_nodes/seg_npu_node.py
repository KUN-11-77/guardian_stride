#!/usr/bin/env python3
"""
ADE20K Semantic Segmentation on DK-2500
========================================
Model:  SegFormer-B0 ADE20K FP32 (150 classes, indoor/outdoor)
Device: CPU 14-thread OpenVINO (ADE20K too large for NPU SRAM)
NPU:    Reserved for ASR voice pipeline (faster-whisper INT8)
Speed:  ~150-200ms/frame OpenVINO CPU
"""
import rclpy, cv2, numpy as np, openvino as ov, time
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

MODEL_PATH = '/home/guardian/Desktop/ros2_ws/src/guardian_stride/gs_perception/models/segformer_b0_ade_fp32.xml'

# ADE20K color map: green=walkable, red=obstacle, yellow=stairs, cyan=door
COLORS = np.full((150, 3), [100, 100, 100], dtype=np.uint8)  # default grey
WALKABLE = {0, 3, 12, 13, 14, 15, 16, 29, 91}
OBSTACLE = {1, 4, 5, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 82, 83, 84, 85, 86, 87, 88, 89}
for c in WALKABLE: COLORS[c] = [80, 210, 60]    # GREEN
for c in OBSTACLE: COLORS[c] = [60, 50, 240]     # RED
for c in [2, 10]: COLORS[c] = [200, 140, 60]     # sky → BLUE-GREY
for c in [80, 81]: COLORS[c] = [0, 230, 250]     # stairs → YELLOW
for c in [88, 89]: COLORS[c] = [220, 200, 40]     # door → CYAN

LABELS = {0:'unlabeled',1:'wall',2:'building',3:'sky',12:'floor',13:'road',
          14:'sidewalk',15:'path',19:'chair',20:'table',30:'person',
          80:'stairs',81:'escalator',88:'door',89:'gate'}

class SegNPUNode(Node):
    def __init__(self):
        super().__init__('seg_npu')
        self.bridge = CvBridge()
        self.rgb = None
        self.count = 0

        self.sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self._cb, 10)
        self.pub = self.create_publisher(Image, '/segmentation/overlay', 10)

        # Load OpenVINO
        self.get_logger().info(f'Loading ADE20K model...')
        core = ov.Core()
        model = core.read_model(MODEL_PATH)
        model.reshape({model.input(0).any_name: [1, 3, 640, 480]})

        # CPU 14-thread inference
        core.set_property('CPU', {
            'INFERENCE_NUM_THREADS': '14',
            'PERFORMANCE_HINT': 'LATENCY',
            'ENABLE_CPU_PINNING': 'YES'
        })
        self.compiled = core.compile_model(model, 'CPU')
        self.infer = self.compiled.create_infer_request()
        self.in_key = model.input(0).any_name
        self.out_key = model.output(0).any_name
        self.log_cfg = {'throttle_duration_sec': 3.0}
        self.get_logger().info('CPU 14-thread ready | NPU reserved for ASR')

        self.create_timer(0.05, self._infer)  # ~20Hz max

    def _cb(self, msg):
        self.rgb = msg

    def _infer(self):
        if self.rgb is None:
            return
        try:
            t0 = time.time()
            cv_img = self.bridge.imgmsg_to_cv2(self.rgb, 'bgr8')
            h, w = cv_img.shape[:2]

            # Preprocess
            img = cv2.resize(cv_img, (640, 480))
            rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
            inp = np.transpose(rgb, (2, 0, 1))[np.newaxis, ...]

            # Inference
            logits = self.infer.infer({self.in_key: inp})[self.out_key]
            seg_small = np.argmax(logits[0], axis=0).astype(np.uint8)
            seg = cv2.resize(seg_small, (w, h), interpolation=cv2.INTER_LINEAR).astype(np.uint8)
            ms = (time.time()-t0)*1000

            # Colorize
            colors = COLORS[seg]  # fast vectorized lookup
            overlay = cv2.addWeighted(cv_img, 0.5, colors, 0.5, 0)

            # Stats
            trav = np.isin(seg, list(WALKABLE)).sum() / seg.size * 100
            counts = np.bincount(seg.flatten(), minlength=150)
            top = int(np.argmax(counts))
            top_name = LABELS.get(top, str(top))
            cv2.putText(overlay, f'ADE20K CPU14 {ms:.0f}ms | Walk:{trav:.0f}% | {top_name}',
                        (8, h-8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200,200,200), 1)

            self.pub.publish(self.bridge.cv2_to_imgmsg(overlay, 'bgr8'))
            self.count += 1
            self.get_logger().info(f'#{self.count} {ms:.0f}ms top={top_name}', **self.log_cfg)

        except Exception as e:
            self.get_logger().error(str(e)[:150])

def main():
    rclpy.init()
    rclpy.spin(SegNPUNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
