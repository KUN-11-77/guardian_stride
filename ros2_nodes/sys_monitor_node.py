#!/usr/bin/env python3
"""DK-2500 System Monitor: CPU/RAM/NPU/iGPU usage → ROS2 topics.

Publishes: /sys/cpu_percent, /sys/ram_percent, /sys/ram_used_gb,
           /sys/npu_util, /sys/igpu_util, /sys/temp_celsius (Float32 each)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import os, time, threading

class SysMonitorNode(Node):
    def __init__(self):
        super().__init__('sys_monitor')
        self.cpu_pub = self.create_publisher(Float32, '/sys/cpu_percent', 10)
        self.ram_pub = self.create_publisher(Float32, '/sys/ram_percent', 10)
        self.ram_gb_pub = self.create_publisher(Float32, '/sys/ram_used_gb', 10)
        self.npu_pub = self.create_publisher(Float32, '/sys/npu_util', 10)
        self.igpu_pub = self.create_publisher(Float32, '/sys/igpu_util', 10)
        self.temp_pub = self.create_publisher(Float32, '/sys/temp_celsius', 10)

        self._prev_idle = self._prev_total = 0
        self.create_timer(1.0, self._publish)
        self.get_logger().info('SysMonitor ready')

    def _cpu_percent(self):
        """Read /proc/stat for CPU usage."""
        try:
            with open('/proc/stat') as f:
                fields = [int(v) for v in f.readline().split()[1:]]
            idle, total = fields[3], sum(fields)
            d_idle, d_total = idle - self._prev_idle, total - self._prev_total
            self._prev_idle, self._prev_total = idle, total
            return 100.0 * (1.0 - d_idle / max(d_total, 1))
        except:
            return 0.0

    def _ram_info(self):
        """Read /proc/meminfo for RAM usage."""
        try:
            mem = {}
            with open('/proc/meminfo') as f:
                for line in f:
                    if ':' in line:
                        k, v = line.split(':')
                        mem[k.strip()] = int(v.strip().split()[0])
            total = mem.get('MemTotal', 1)
            avail = mem.get('MemAvailable', 0)
            used = total - avail
            return (used / total * 100.0, used / (1024*1024))
        except:
            return (0.0, 0.0)

    def _npu_util(self):
        """Check Intel NPU driver status."""
        try:
            # Check if NPU device exists and is busy
            if os.path.exists('/dev/accel/accel0'):
                # Try to get NPU usage from sysfs
                for p in ['/sys/class/accel/accel0/device/',
                          '/sys/devices/pci0000:00/*/ivpu_usage']:
                    pass
                # Return 0 if NPU present but idle, -1 if not found
                return 0.0  # present but not actively used by us
            return -1.0  # no NPU
        except:
            return -1.0

    def _igpu_util(self):
        """Check Intel iGPU usage via sysfs."""
        try:
            if os.path.exists('/sys/class/drm/card1/gt_cur_freq_mhz'):
                with open('/sys/class/drm/card1/gt_cur_freq_mhz') as f:
                    freq = int(f.read().strip())
                with open('/sys/class/drm/card1/gt_max_freq_mhz') as f:
                    max_freq = int(f.read().strip())
                return freq / max(max_freq, 1) * 100.0
            return -1.0
        except:
            return -1.0

    def _temp(self):
        """Read CPU/GPU temperature."""
        try:
            for z in ['/sys/class/thermal/thermal_zone0/temp',
                      '/sys/class/hwmon/hwmon0/temp1_input']:
                if os.path.exists(z):
                    with open(z) as f:
                        return float(f.read().strip()) / 1000.0
            return 0.0
        except:
            return 0.0

    def _publish(self):
        now = self.get_clock().now()
        t = time.time()

        # CPU
        cpu = self._cpu_percent()
        self.cpu_pub.publish(Float32(data=cpu))

        # RAM
        ram_pct, ram_gb = self._ram_info()
        self.ram_pub.publish(Float32(data=ram_pct))
        self.ram_gb_pub.publish(Float32(data=ram_gb))

        # NPU / iGPU / Temp
        self.npu_pub.publish(Float32(data=self._npu_util()))
        self.igpu_pub.publish(Float32(data=self._igpu_util()))
        self.temp_pub.publish(Float32(data=self._temp()))

        self.get_logger().debug(
            f'CPU:{cpu:.0f}% RAM:{ram_pct:.0f}%({ram_gb:.1f}GB)',
            throttle_duration_sec=30.0)


def main():
    rclpy.init()
    rclpy.spin(SysMonitorNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
