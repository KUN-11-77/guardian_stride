#!/usr/bin/env python3
"""NPU 导航节点 v5: MiDaS NPU深度 + RealSense地面平面 → 精确可通行检测"""
import rclpy, cv2, numpy as np, time
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge

FPS = 10.0

class NPUNavNode(Node):
    def __init__(self):
        super().__init__("npu_nav_node")
        self.bridge = CvBridge()
        self.rgb = None
        self.rs_depth = None  # RealSense aligned depth (mm)
        
        import openvino as ov
        core = ov.Core()
        model = core.read_model("/home/guardian/midas_small_ir.xml")
        self.npu = core.compile_model(model, "NPU")
        self.infer = self.npu.create_infer_request()
        self.in_key = model.input(0).any_name
        self.get_logger().info("MiDaS-small NPU ✓")
        
        self.create_subscription(Image, "/camera/camera/color/image_raw", self._cb_rgb, 10)
        self.create_subscription(Image, "/camera/camera/aligned_depth_to_color/image_raw", self._cb_depth, 10)
        self.pub_depth = self.create_publisher(Image, "/npu/depth", 10)
        self.pub_obst = self.create_publisher(Float32, "/npu/nearest_obstacle", 10)
        self.pub_npu_util = self.create_publisher(Float32, "/sys/npu_util", 10)
        
        self.infer_times = []
        self.timer = self.create_timer(1.0 / FPS, self._infer)
        self.count = 0
        self.get_logger().info("NPU导航 v5 10Hz (MiDaS+RS地面平面)")
    
    def _cb_rgb(self, msg): self.rgb = msg
    def _cb_depth(self, msg): self.rs_depth = msg
    
    def _infer(self):
        if self.rgb is None: return
        try:
            t0 = time.time()
            cv_img = self.bridge.imgmsg_to_cv2(self.rgb, "bgr8")
            h, w = cv_img.shape[:2]
            
            # === NPU MiDaS 深度估计 ===
            rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            inp = cv2.resize(rgb, (256, 256)).astype(np.float32) / 255.0
            inp = np.transpose(inp, (2, 0, 1))[np.newaxis, ...]
            
            self.infer.infer({self.in_key: inp})
            disparity = self.infer.get_output_tensor(0).data[0]
            
            d_min, d_max = float(np.min(disparity)), float(np.max(disparity))
            if d_max > d_min:
                midas_depth = 1.0 / (disparity / d_max + 0.01)
                midas_depth = (midas_depth - midas_depth.min()) / (midas_depth.max() - midas_depth.min()) * 7.8 + 0.2
            else:
                midas_depth = np.full((256, 256), 1.0, dtype=np.float32)
            midas_depth = cv2.resize(midas_depth, (w, h))
            
            # === RealSense 地面平面估计 (几何方法，精确) ===
            if self.rs_depth is not None:
                rs = self.bridge.imgmsg_to_cv2(self.rs_depth, desired_encoding="passthrough")
                # 处理16UC1或32FC1
                if rs.dtype == np.uint16:
                    rs_m = rs.astype(np.float32) * 0.001  # mm → m
                else:
                    rs_m = rs.astype(np.float32)
                
                # 地面平面: 取底部中心区域深度中位数
                bh = int(h * 0.75)
                bw_l, bw_r = int(w * 0.25), int(w * 0.75)
                ground_roi = rs_m[bh:, bw_l:bw_r]
                valid_ground = ground_roi[(ground_roi > 0.1) & (ground_roi < 6.0)]
                ground_z = float(np.median(valid_ground)) if len(valid_ground) > 300 else 2.0
                
                # 可通行 = 在ground_z ± 35cm范围内
                rs_traversable = (np.abs(rs_m - ground_z) < 0.35) & (rs_m > 0.1) & (rs_m < 6.0)
                
                # 障碍物 = 显著高于地面 (>20cm) + 在3m内
                rs_obstacle = (rs_m < ground_z - 0.2) & (np.abs(rs_m - ground_z) > 0.3) & (rs_m > 0.1) & (rs_m < 3.0)
                
                # 融合 MiDaS 和 RealSense: 
                # - RealSense 有数据 → 用RS判断 (精确)
                # - RealSense 无数据(0) → 用 MiDaS 梯度 (补充)
                gy, gx = np.gradient(midas_depth)
                midas_grad = np.sqrt(gy**2 + gx**2)
                midas_trav = (midas_grad < 0.5) & (midas_depth < 5.0)
                
                # RS有效pixels用RS判断，RS无效区域用MiDaS补充
                rs_valid = (rs_m > 0.1) & (rs_m < 6.0)
                traversable = np.where(rs_valid, rs_traversable, midas_trav)
                
                # 障碍物: RS优先，MiDaS大梯度补充
                midas_obst = (midas_grad > 0.8) & (midas_depth < 2.0)
                obstacle = np.where(rs_valid, rs_obstacle, midas_obst)
                
                # 最近障碍物距离
                if obstacle.any() and rs_valid.any():
                    obst_depths = rs_m[obstacle & rs_valid]
                    if len(obst_depths) > 10:
                        nearest = float(np.min(obst_depths))
                    else:
                        nearest = -1.0
                else:
                    nearest = -1.0
                
                use_rs = True
            else:
                # 无RS深度，退回到纯MiDaS梯度法
                gy, gx = np.gradient(midas_depth)
                grad_mag = np.sqrt(gy**2 + gx**2)
                traversable = (grad_mag < 0.5) & (midas_depth < 5.0)
                obstacle = (grad_mag > 0.8) & (midas_depth < 2.0)
                nearest = float(np.min(midas_depth[obstacle])) if obstacle.any() else -1.0
                rs_m = midas_depth  # fallback
                use_rs = False
            
            ms = (time.time() - t0) * 1000
            
            # === NPU 利用率 ===
            frame_ms = 1000.0 / FPS
            self.infer_times.append(ms)
            if len(self.infer_times) > 20: self.infer_times.pop(0)
            avg_ms = np.mean(self.infer_times)
            npu_pct = min(100.0, avg_ms / frame_ms * 100.0)
            
            # === 可视化 ===
            display = cv_img.copy().astype(np.float32)
            
            # 绿色 = 可通行
            gmask = np.zeros_like(display)
            gmask[traversable] = [60, 210, 80]
            display = cv2.addWeighted(display, 0.65, gmask, 0.35, 0)
            
            # 红色 = 障碍物
            rmask = np.zeros_like(display)
            rmask[obstacle] = [60, 50, 240]
            display = cv2.addWeighted(display, 1.0, rmask, 0.45, 0)
            
            display = display.astype(np.uint8)
            
            cv2.putText(display, f"NPU {ms:.0f}ms util={npu_pct:.0f}%", (10, 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)
            cv2.putText(display, f"Walk={traversable.mean()*100:.0f}% Near={nearest:.1f}m",
                       (10, h-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60,255,100), 1)
            source = "RS+MiDaS" if use_rs else "MiDaS-only"
            cv2.putText(display, source, (w-120, h-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200,200,200), 1)
            
            self.pub_depth.publish(self.bridge.cv2_to_imgmsg(display, "bgr8"))
            self.pub_obst.publish(Float32(data=nearest))
            self.pub_npu_util.publish(Float32(data=npu_pct))
            
            self.count += 1
            if self.count % 30 == 0:
                self.get_logger().info(
                    f"#{self.count} NPU {ms:.0f}ms util={npu_pct:.0f}% "
                    f"walk={traversable.mean()*100:.0f}% near={nearest:.1f}m "
                    f"ground_z={ground_z:.1f}m" if use_rs else f"#{self.count} MiDaS-only")
        except Exception as e:
            self.get_logger().error(str(e)[:200])

def main():
    rclpy.init()
    rclpy.spin(NPUNavNode())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
