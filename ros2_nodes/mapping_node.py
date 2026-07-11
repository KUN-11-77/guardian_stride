#!/usr/bin/env python3
"""DK-2500 俯视建图 + 伪LiDAR + A*路径规划 + 可通行检测 (v2 - clean)."""
import rclpy, cv2, numpy as np, math, time, heapq, threading
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge

MAP_SZ, MAP_RES = 200, 0.05  # 10m x 10m @ 5cm
MC = MAP_SZ // 2

class MappingNode(Node):
    def __init__(self):
        super().__init__('mapping_node')
        self.bridge = CvBridge()
        self.depth_img = None
        self.rgb_img = None
        self.grid = np.zeros((MAP_SZ, MAP_SZ), dtype=np.int32)  # obstacle hits
        self.free = np.zeros((MAP_SZ, MAP_SZ), dtype=np.float32)
        self.pose = [0.0, 0.0, 0.0]  # x, y, yaw
        self.goal = (MC + 40, MC)    # 2m ahead
        self.path = []
        self.proc = False

        # Camera intrinsics (D435i depth aligned)
        self.fx = 640.0; self.fy = 640.0
        self.cx = 640.0; self.cy = 360.0

        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self._cb_depth, 10)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self._cb_rgb, 10)

        self.pub_map = self.create_publisher(Image, '/topdown/map', 10)
        self.pub_overlay = self.create_publisher(Image, '/segmentation/overlay', 10)
        self.pub_obst = self.create_publisher(Float32, '/nearest_obstacle_m', 10)

        # IMU poll 50Hz
        self.create_timer(0.02, self._imu)
        # Mapping 10Hz
        self.create_timer(0.1, self._update)
        # Viz 5Hz
        self.create_timer(0.2, self._viz)

        self.get_logger().info('MappingNode v2 ready')

    def _cb_depth(self, msg):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    def _cb_rgb(self, msg):
        self.rgb_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _imu(self):
        """Read IMU, integrate odometry."""
        try:
            import os
            gs = 0.001065; as_ = 0.000598  # D435i defaults
            with open('/sys/bus/iio/devices/iio:device0/in_anglvel_z_raw') as f: gz = float(f.read()) * gs
            with open('/sys/bus/iio/devices/iio:device1/in_accel_x_raw') as f: ax = float(f.read()) * as_
            dt = 0.02
            self.pose[2] += gz * dt
            self.pose[0] += ax * math.cos(self.pose[2]) * dt * 0.5
            self.pose[1] += ax * math.sin(self.pose[2]) * dt * 0.5
        except:
            pass

    def _update(self):
        if self.depth_img is None or self.proc:
            return
        self.proc = True
        try:
            d = self.depth_img.astype(np.float64) * 0.001  # mm → m
            h, w = d.shape
            px, py, pyaw = float(self.pose[0]), float(self.pose[1]), float(self.pose[2])
            ca, sa = math.cos(pyaw), math.sin(pyaw)

            # Step through depth image
            for v in range(4, h, 5):
                vi = int(v)
                for u in range(4, w, 5):
                    ui = int(u)
                    z = float(d[vi, ui])
                    if z < 0.15 or z > 6.0:
                        continue
                    x = (float(ui) - self.cx) * z / self.fx
                    y = (float(vi) - self.cy) * z / self.fy
                    # World coordinates
                    wx = px + x * ca - z * sa
                    wy = py + x * sa + z * ca
                    gx = int(round(wx / MAP_RES)) + MC
                    gy = int(round(wy / MAP_RES)) + MC
                    if 0 <= gx < MAP_SZ and 0 <= gy < MAP_SZ:
                        # Obstacle if above ground (>0.3m high) and within 3m
                        if abs(y + 1.0) > 0.3 and z < 3.0:
                            self.grid[gy, gx] = min(255, self.grid[gy, gx] + 1)
                        elif z < 0.5:
                            self.grid[gy, gx] = min(255, self.grid[gy, gx] + 1)
                        else:
                            self.free[gy, gx] = 0.0

            # Plan path
            self.path = self._astar()

        except Exception as e:
            self.get_logger().error(str(e)[:200])
        finally:
            self.proc = False

    def _astar(self):
        """A* on obstacle grid."""
        obs = (self.grid > 2).astype(np.uint8)
        kernel = np.ones((5, 5), np.uint8)
        cost = cv2.dilate(obs, kernel).astype(np.float32) * 100.0
        s, g = (MC, MC), self.goal
        if cost[g[1], g[0]] >= 100:
            return []
        h_, w_ = cost.shape
        vis = np.zeros((h_, w_), bool)
        ds = np.full((h_, w_), np.inf)
        par = {}
        ds[s[1], s[0]] = 0
        q = [(math.hypot(g[0]-s[0], g[1]-s[1]), s)]
        while q:
            _, (cx, cy) = heapq.heappop(q)
            if (cx, cy) == g:
                path = []
                while (cx, cy) != s:
                    path.append((cx, cy))
                    cx, cy = par[(cx, cy)]
                path.reverse()
                return path
            if vis[cy, cx]:
                continue
            vis[cy, cx] = True
            for dx, dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)]:
                nx, ny = cx+dx, cy+dy
                if 0 <= nx < w_ and 0 <= ny < h_ and not vis[ny, nx] and cost[ny, nx] < 100:
                    nd = ds[cy, cx] + math.hypot(dx, dy)
                    if nd < ds[ny, nx]:
                        ds[ny, nx] = nd
                        par[(nx, ny)] = (cx, cy)
                        heapq.heappush(q, (nd + math.hypot(g[0]-nx, g[1]-ny), (nx, ny)))
        return []

    def _viz(self):
        if self.rgb_img is None:
            return
        try:
            # === TOP-DOWN MAP ===
            tviz = np.zeros((MAP_SZ, MAP_SZ, 3), np.uint8)
            tviz[:, :] = [15, 15, 25]
            tviz[self.grid <= 2] = [35, 35, 50]
            tviz[self.grid > 2] = [60, 40, 200]
            # Path
            for i, (pxx, pyy) in enumerate(self.path):
                if i % 2 == 0:
                    cv2.circle(tviz, (pxx, pyy), 2, (0, 255, 100), -1)
            # Robot
            cv2.circle(tviz, (MC, MC), 5, (0, 255, 200), -1)
            # Goal
            cv2.circle(tviz, self.goal, 5, (0, 200, 255), -1)
            # Direction
            pyaw = self.pose[2]
            cv2.line(tviz, (MC, MC), (MC + int(20*math.cos(pyaw)), MC + int(20*math.sin(pyaw))), (255, 255, 0), 2)

            tviz_big = cv2.resize(tviz, (400, 400), interpolation=cv2.INTER_NEAREST)
            cv2.putText(tviz_big, f'10x10m 5cm/cell', (5, 390), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150,150,150), 1)
            cv2.putText(tviz_big, f'Obstacles: {int(np.sum(self.grid > 2))}', (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200,100,100), 1)

            self.pub_map.publish(self.bridge.cv2_to_imgmsg(tviz_big, 'bgr8'))

            # === TRAVERSABILITY OVERLAY ===
            h, w = self.rgb_img.shape[:2]
            depth_m = cv2.resize(self.depth_img.astype(np.float32), (w, h), interpolation=cv2.INTER_NEAREST) * 0.001

            # Ground plane from bottom-center region
            bot = depth_m[int(h*0.75):, int(w*0.2):int(w*0.8)]
            bv = bot[(bot > 0.1) & (bot < 6.0)]
            gz = float(np.median(bv)) if len(bv) > 300 else 2.0

            # Traversable: within 35cm of ground plane
            trav = np.abs(depth_m - gz) < 0.35
            # Obstacle: significantly above ground or very close
            obst = (depth_m < gz - 0.2) & (np.abs(depth_m - gz) > 0.3)
            obst |= (depth_m < 0.5)
            # Near threat: <1m
            near = obst & (depth_m < 1.0) & (depth_m > 0.1)

            overlay = self.rgb_img.copy()
            gmask = np.zeros_like(overlay); gmask[trav] = [80, 210, 60]
            overlay = cv2.addWeighted(overlay, 0.6, gmask, 0.4, 0)
            rmask = np.zeros_like(overlay); rmask[obst] = [60, 50, 240]
            overlay = cv2.addWeighted(overlay, 1.0, rmask, 0.35, 0)
            ymask = np.zeros_like(overlay); ymask[near] = [0, 230, 250]
            overlay = cv2.addWeighted(overlay, 1.0, ymask, 0.5, 0)

            cv2.putText(overlay, 'GREEN=Walkable RED=Obstacle YELLOW=<1m', (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1)
            cv2.putText(overlay, f'Ground: {gz:.1f}m  Walkable: {trav.sum()/trav.size*100:.0f}%', (10, h-10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (60,210,80), 2)
            self.pub_overlay.publish(self.bridge.cv2_to_imgmsg(overlay, 'bgr8'))

            # Nearest obstacle
            if obst.any():
                od = depth_m[obst]; od = od[od > 0.1]
                self.pub_obst.publish(Float32(data=float(np.min(od)) if len(od) > 0 else -1.0))

        except Exception as e:
            self.get_logger().error(str(e)[:200])

def main():
    rclpy.init()
    rclpy.spin(MappingNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
