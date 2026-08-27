#!/usr/bin/env python3
"""手推里程计精度测量:电机零电流卸力,手推车,纯编码器测距,RViz 实时轨迹。

链路(全程无 PID、无 cmd_vel、无 EKF):
    wheel_bridge /wheel_state (100Hz 编码器 rpm,固件顺序 [LF,RL,RR,FR])
        └─► push_odom 正解算 → 中值积分位姿(与 chassis_driver 同法)
              ├─► /push_odom     (Odometry, pose+twist)
              ├─► /push_path     (Path, RViz 轨迹)
              ├─► /joint_states  (RViz 轮子动画)
              └─► TF odom→base_link
    启动即向 /wheel_current_cmd 发零电流(0x01 直控帧)卸力电机;
    wheel_bridge 的 10Hz 保活会持续重发,之后手推,编码器照常上报。

用法:
  1. ros2 launch myrobot_real push_test.launch.py wheel_port:=/dev/wheel_stm32
     (勿与 robot.launch.py 同跑:wheel_kinematics 的保活零速会抱死轮子,
       EKF 也会争抢 odom→base_link TF)
  2. 手推车走已知距离(如沿 5m 卷尺直线),或推一圈回到起点
  3. RViz 看 /push_path 绿色轨迹;Ctrl+C 打印精度报告并写 CSV

精度判读:
  - 直线:报告"路程" ÷ 卷尺实距 = 里程计比例
        → 新 wheel_radius = 旧 wheel_radius × 实际距离 / 里程路程
  - 闭环:推回起点后"闭环误差"即纯漂移(横移打滑+积分误差)
  - 横向推:麦轮滚轮被动滚,编码器几乎无读数 → 横移测不到属正常现象
"""
import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster

from myrobot_real.wheel_kinematics import WHEEL_MIRRORED, WHEEL_NAMES


class PushOdom(Node):

    def __init__(self):
        super().__init__('push_odom')

        # —— 几何参数(默认与 config/wheel_kinematics.yaml 一致,可 -p 覆盖)——
        self.declare_parameter('wheel_radius', 0.0763)    # 轮半径 [m]
        self.declare_parameter('half_diagonal', 0.625)    # lx+ly [m]
        self.declare_parameter('gear_ratio', 19.2)        # M3508 减速比
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)        # 本节点是唯一 TF 源
        self.declare_parameter('path_period', 0.2)        # 轨迹采样周期 [s]
        self.declare_parameter('path_max', 10000)         # 轨迹点上限
        self.declare_parameter('free_motors', True)       # 启动即零电流卸力
        self.declare_parameter('free_repub_s', 3.0)       # 零电流重发周期 [s]
        self.declare_parameter('out', '')                 # CSV 路径,空则自动

        p = self.get_parameters(['wheel_radius', 'half_diagonal', 'gear_ratio',
                                 'odom_frame', 'base_frame', 'publish_tf',
                                 'path_period', 'path_max', 'free_motors',
                                 'free_repub_s', 'out'])
        self.r = p[0].value
        self.l = p[1].value
        self.rad_s_per_rpm = 2.0 * math.pi / 60.0 / p[2].value
        self.odom_frame = p[3].value
        self.base_frame = p[4].value
        self.publish_tf = p[5].value
        self.path_period = p[6].value
        self.path_max = p[7].value
        free = p[8].value
        self.out = p[10].value

        self.pub_odom = self.create_publisher(Odometry, 'push_odom', 50)
        self.pub_path = self.create_publisher(Path, 'push_path', 10)
        self.pub_js = self.create_publisher(JointState, 'joint_states', 50)
        self.pub_current = self.create_publisher(Float32MultiArray, 'wheel_current_cmd', 10)
        self.tf_pub = TransformBroadcaster(self) if self.publish_tf else None

        self.create_subscription(Float32MultiArray, 'wheel_state', self.on_wheel_state, 50)

        # —— 积分状态(中值积分,与 chassis_driver_node.cpp 同法)——
        self.x = self.y = self.yaw = 0.0
        self.last_t = None
        self.last_path_t = 0.0
        self.t0 = None
        self.path_len = 0.0        # 累计路程 [m]
        self.yaw_total = 0.0       # 累计转角(绝对值) [rad]
        self.samples = 0
        self.v_max = 0.0
        self.joint_pos = [0.0] * 4
        self.path = Path()
        self.path.header.frame_id = self.odom_frame
        self.rows = []

        if free:
            self.send_free_current()
            # bridge 掉线重启后保活会清空,周期重发兜底
            self.create_timer(p[9].value, self.send_free_current)
        self.get_logger().info(
            f'手推里程计就绪: r={self.r} lx+ly={self.l} '
            f'零电流卸力={"开" if free else "关"};推送后 Ctrl+C 出报告')

    # ── 零电流卸力:0x01 直控帧,固件输出力矩为 0,轮子自由转动 ──
    def send_free_current(self):
        m = Float32MultiArray()
        m.data = [0.0] * 4
        self.pub_current.publish(m)

    # ── 正解:4 电机 rpm → 车体速度(与 wheel_kinematics 一致)──
    def forward(self, fw_rpm):
        w_fl = fw_rpm[0] * self.rad_s_per_rpm
        w_rl = fw_rpm[1] * self.rad_s_per_rpm
        w_rr = -fw_rpm[2] * self.rad_s_per_rpm   # 还原右侧镜像
        w_fr = -fw_rpm[3] * self.rad_s_per_rpm
        r, l = self.r, self.l
        vx = r * (w_fl + w_fr + w_rl + w_rr) / 4.0
        vy = r * (-w_fl + w_fr + w_rl - w_rr) / 4.0
        wz = (-w_fl + w_fr - w_rl + w_rr) * r / (4.0 * l)
        return vx, vy, wz

    # ── 反馈链:100Hz wheel_state → 积分 + 发布 ────────────────
    def on_wheel_state(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            return
        rpm = msg.data[:4]
        now = self.get_clock().now()
        now_msg = now.to_msg()
        t = now.nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = t

        vx, vy, wz = self.forward(rpm)
        self.v_max = max(self.v_max, math.hypot(vx, vy))

        if self.last_t is not None:
            dt = min(max(t - self.last_t, 1e-4), 0.1)   # 首帧/断流防跳变
            yaw_mid = self.yaw + wz * dt * 0.5          # 中值积分
            self.x += (vx * math.cos(yaw_mid) - vy * math.sin(yaw_mid)) * dt
            self.y += (vx * math.sin(yaw_mid) + vy * math.cos(yaw_mid)) * dt
            self.yaw += wz * dt
            self.path_len += math.hypot(vx, vy) * dt
            self.yaw_total += abs(wz) * dt
            for i in range(4):
                self.joint_pos[i] += (rpm[i] * self.rad_s_per_rpm *
                                      (-1.0 if WHEEL_MIRRORED[i] else 1.0)) * dt
        self.last_t = t
        self.samples += 1
        self.rows.append((t - self.t0, self.x, self.y, self.yaw,
                          vx, vy, wz, *rpm))

        # /push_odom:位姿+速度
        odom = Odometry()
        odom.header.stamp = now_msg
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw * 0.5)
        odom.pose.pose.orientation.w = math.cos(self.yaw * 0.5)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.pub_odom.publish(odom)

        # TF odom→base_link
        if self.tf_pub is not None:
            tr = TransformStamped()
            tr.header.stamp = now_msg
            tr.header.frame_id = self.odom_frame
            tr.child_frame_id = self.base_frame
            tr.transform.translation.x = self.x
            tr.transform.translation.y = self.y
            tr.transform.rotation.z = math.sin(self.yaw * 0.5)
            tr.transform.rotation.w = math.cos(self.yaw * 0.5)
            self.tf_pub.sendTransform(tr)

        # /joint_states:RViz 轮子转动(逐轮回波可见,可查单侧编码器故障)
        js = JointState()
        js.header.stamp = now_msg
        js.name = list(WHEEL_NAMES)
        js.velocity = [rpm[i] * self.rad_s_per_rpm *
                       (-1.0 if WHEEL_MIRRORED[i] else 1.0) for i in range(4)]
        js.position = list(self.joint_pos)
        self.pub_js.publish(js)

        # /push_path:轨迹采样
        if t - self.last_path_t >= self.path_period:
            self.last_path_t = t
            pose = PoseStamped()
            pose.header.stamp = now_msg
            pose.header.frame_id = self.odom_frame
            pose.pose.position.x = self.x
            pose.pose.position.y = self.y
            pose.pose.orientation.z = math.sin(self.yaw * 0.5)
            pose.pose.orientation.w = math.cos(self.yaw * 0.5)
            self.path.poses.append(pose)
            if len(self.path.poses) > self.path_max:
                self.path.poses.pop(0)
            self.path.header.stamp = now_msg
            self.pub_path.publish(self.path)

    # ── 退出报告 + CSV ────────────────────────────────────────
    def report(self):
        if self.samples == 0:
            self.get_logger().warn('未收到任何 wheel_state,无报告(bridge 起来了吗?)')
            return
        dur = (self.last_t - self.t0) if (self.last_t and self.t0) else 0.0
        net = math.hypot(self.x, self.y)
        lines = [
            '═══════ 手推里程计精度报告 ═══════',
            f'时长 {dur:.1f}s  样本 {self.samples}  峰值速度 {self.v_max:.2f} m/s',
            f'路程(轮式累计) {self.path_len:.3f} m   累计转角 {math.degrees(self.yaw_total):.1f}°',
            f'终点 x={self.x:.3f} y={self.y:.3f} yaw={math.degrees(self.yaw):.1f}°   净位移 {net:.3f} m',
        ]
        if net < 0.5 and self.path_len > 1.0:
            lines.append(f'闭环误差 {net:.3f} m(已推回起点,此即纯漂移)')
        lines.append('直线标定: 新 wheel_radius = 0.0763 × 卷尺实距 / 路程 '
                     f'(={0.0763:.4f} × 实距 / {self.path_len:.3f})')
        if not self.out:
            d = os.path.expanduser('~/wheel_logs')
            os.makedirs(d, exist_ok=True)
            self.out = os.path.join(d, time.strftime('push_%Y%m%d_%H%M%S.csv'))
        with open(self.out, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y', 'yaw', 'vx', 'vy', 'wz',
                        'rpm_fl', 'rpm_rl', 'rpm_rr', 'rpm_fr'])
            w.writerows(self.rows)
        lines.append(f'CSV → {self.out}')
        self.get_logger().info('\n'.join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = PushOdom()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():   # SIGINT 时 rclpy 已关 context,防双重 shutdown
            rclpy.shutdown()


if __name__ == '__main__':
    main()
