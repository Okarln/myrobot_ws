#!/usr/bin/python3
# 注:必须绝对路径,env python3 会解析到宿主 anaconda,rclpy 导入即崩(同 cmd_vel_guard)
"""扫描自遮挡过滤:/scan -> /scan_filtered

雷达居中装在底盘下方,四个轮子的回波固定出现在 ~0.57-0.66m。
护栏和 slam_toolbox 用各自的 min 距离参数滤掉了它们,但 Nav2 的
代价地图和 AMCL 直接订阅原始 /scan,会把"自己的轮子"当障碍,
机器人随身带着一圈紫色致命区,规划器永远出不了门。

本节点把 mask 距离以内的回波置为 +inf,输出干净的 /scan_filtered,
供 nav2_params.yaml 里的代价地图与 AMCL 使用。
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')
        # 0.4x0.4 新底盘:轮子回波 ~0.2-0.28m;0.30 足够遮住自遮挡,
        # 再大窄通道两侧的墙会被当成"自己"滤掉,局部代价地图看不见墙
        self.declare_parameter('self_mask', 0.30)
        self.mask = self.get_parameter('self_mask').value
        self.pub = self.create_publisher(LaserScan, '/scan_filtered', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan,
                                  qos_profile_sensor_data)
        self.get_logger().info('扫描自遮挡过滤启动: /scan -> /scan_filtered')

    def on_scan(self, m):
        out = LaserScan()
        out.header = m.header
        out.angle_min = m.angle_min
        out.angle_max = m.angle_max
        out.angle_increment = m.angle_increment
        out.time_increment = m.time_increment
        out.scan_time = m.scan_time
        out.range_min = m.range_min
        out.range_max = m.range_max
        out.ranges = [
            float('inf') if d < self.mask else d
            for d in m.ranges
        ]
        out.intensities = m.intensities
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ScanFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
