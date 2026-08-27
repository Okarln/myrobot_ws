#!/usr/bin/env python3
"""纯里程计定位的 map→odom 调整器:RViz "2D Pose Estimate" 点击重定位。

替代 nav_odom_test.launch.py 原来的静态 map→odom TF:
  - 启动时按 start_x/start_y/start_yaw 钉扎(与原静态 TF 行为一致);
  - RViz 工具栏 "2D Pose Estimate"(发布 /initialpose)点击车的真实位姿后,
    结合当前 EKF 的 odom→base_link 反解新的 map→odom 并持续发布,
    车在地图上立即"跳"到点击位置,后续里程计照常累积。

原理(odom→base_link 始终由 EKF 独占,本节点只管 map→odom):
    T_map_odom = T_map_base(点击) · T_odom_base(当前)^-1
    θ_mo = θ_click - θ_odom
    t_mo = (x_click, y_click) - R(θ_mo)·(x_odom, y_odom)

注意与 AMCL 的区别:这是"一次性搬移",不吸收后续漂移;漂移靠
直线/旋转标定(见 docs/wheel_calibration.md)和 IMU/雷达闭环压制。
"""
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MapTfAdjust(Node):

    def __init__(self):
        super().__init__('map_tf_adjust')
        self.declare_parameter('start_x', 11.0)
        self.declare_parameter('start_y', 11.0)
        self.declare_parameter('start_yaw', 1.57)
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('rate', 10.0)

        p = self.get_parameters(['start_x', 'start_y', 'start_yaw',
                                 'odom_topic', 'rate'])
        # map→odom 偏移 (x, y, yaw),点击 /initialpose 时更新
        self.offset = (p[0].value, p[1].value, p[2].value)

        self.odom_pose = None                     # 最近一帧 odom→base_link
        self.create_subscription(Odometry, p[3].value, self.on_odom, 20)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose',
                                 self.on_initialpose, 10)

        self.tf_pub = TransformBroadcaster(self)
        self.create_timer(1.0 / p[4].value, self.publish_tf)

        self.get_logger().info(
            f'map→odom 初始钉扎 ({self.offset[0]:.2f}, {self.offset[1]:.2f}, '
            f'{math.degrees(self.offset[2]):.1f}°);RViz 用 2D Pose Estimate 点击重定位')

    # ── 当前 odom→base_link(EKF) ─────────────────────────────
    def on_odom(self, msg: Odometry):
        pos = msg.pose.pose.position
        self.odom_pose = (pos.x, pos.y,
                          yaw_from_quat(msg.pose.pose.orientation))

    # ── RViz 2D Pose Estimate 点击 ───────────────────────────
    def on_initialpose(self, msg: PoseWithCovarianceStamped):
        frame = msg.header.frame_id or 'map'
        if frame != 'map':
            self.get_logger().warn(f'忽略 initialpose:frame_id={frame} ≠ map')
            return
        if self.odom_pose is None:
            self.get_logger().warn('还没收到里程计,忽略本次点击')
            return
        pos = msg.pose.pose.position
        x_c, y_c = pos.x, pos.y
        th_c = yaw_from_quat(msg.pose.pose.orientation)
        x_o, y_o, th_o = self.odom_pose

        th_mo = th_c - th_o
        c, s = math.cos(th_mo), math.sin(th_mo)
        x_mo = x_c - (c * x_o - s * y_o)
        y_mo = y_c - (s * x_o + c * y_o)

        old = self.offset
        self.offset = (x_mo, y_mo, th_mo)
        self.get_logger().info(
            f'重定位:点击 ({x_c:.2f}, {y_c:.2f}, {math.degrees(th_c):.1f}°) → '
            f'map→odom ({old[0]:.2f},{old[1]:.2f},'
            f'{math.degrees(old[2]):.1f}°) ⇒ ({x_mo:.2f},{y_mo:.2f},'
            f'{math.degrees(th_mo):.1f}°)')
        self.publish_tf()

    def publish_tf(self):
        x, y, yaw = self.offset
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf_pub.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = MapTfAdjust()
    import signal
    signal.signal(signal.SIGTERM, lambda *a: (_ for _ in ()).throw(KeyboardInterrupt))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
