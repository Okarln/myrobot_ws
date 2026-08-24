#!/usr/bin/python3
# 注:必须用绝对路径。容器内 /usr/bin/env python3 会解析到宿主的 anaconda
# (PATH 靠前),其 Python 版本与 rclpy 的 C 扩展不兼容,import 即崩。
"""cmd_vel 安全护栏:根据激光距离放行/减速/刹停线速度。

链路位置:teleop / 脚本 --/cmd_vel--> [本节点] --/cmd_vel_safe--> diff_drive_controller
(控制器订阅哪个话题由 URDF 里 gazebo 插件的 remapping 决定)

行为:
  - 只看"行进方向"的扇形区域:前进看车头 ±60°,后退看车尾 ±60°
  - 障碍距离 d > slow_dist(0.8m):指令原样放行
  - stop_dist(0.25m)< d < slow_dist:线速度按距离线性压低,越近越慢
  - d < stop_dist:线速度清零刹停(角速度不受限,原地转还能脱困)
  - 后退时若车尾扇区干净,倒车不受影响

注意:激光雷达平面高度必须低于矮墙顶端,否则看不见墙(见 URDF 里
laser_joint 的注释)。
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class CmdVelGuard(Node):
    def __init__(self):
        super().__init__('cmd_vel_guard')
        self.declare_parameter('stop_dist', 0.30)       # 雷达到障碍 < 此值刹停(车前缘0.2,留10cm)
        self.declare_parameter('slow_dist', 0.60)       # 低于此距离开始线性减速
        self.declare_parameter('self_mask', 0.30)        # 0.4新底盘轮子回波~0.2-0.28m
        self.declare_parameter('sector_half_deg', 30.0)  # 行进方向扇形半角;±60°会把窄通道侧墙当前方障碍
        self.front = math.inf
        self.rear = math.inf
        self.create_subscription(LaserScan, '/scan', self.on_scan,
                                  qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)
        self.get_logger().info('cmd_vel 护栏启动: /cmd_vel -> /cmd_vel_safe')

    def on_scan(self, m):
        half = math.radians(self.get_parameter('sector_half_deg').value)
        mask = self.get_parameter('self_mask').value
        f, r = math.inf, math.inf
        for i, d in enumerate(m.ranges):
            if m.range_min < d < m.range_max and d > mask:   # 滤掉 inf/nan/盲区/自遮挡
                a = m.angle_min + i * m.angle_increment
                if abs(a) <= half:                       # 车头扇区
                    f = min(f, d)
                elif abs(abs(a) - math.pi) <= half:      # 车尾扇区
                    r = min(r, d)
        self.front, self.rear = f, r

    def on_cmd(self, m):
        stop = self.get_parameter('stop_dist').value
        slow = self.get_parameter('slow_dist').value
        out = Twist()
        out.angular = m.angular                          # 转向永远放行
        v = m.linear.x
        d = self.front if v > 0 else (self.rear if v < 0 else math.inf)
        if v != 0.0 and d < slow:
            scale = 0.0 if d <= stop else (d - stop) / (slow - stop)
            out.linear.x = v * scale
            if scale == 0.0:
                self.get_logger().warn(
                    f'行进方向 {d:.2f}m 内有障碍,线速度已刹停',
                    throttle_duration_sec=2.0)
        else:
            out.linear.x = v
        self.pub.publish(out)


def main():
    rclpy.init()
    node = CmdVelGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
