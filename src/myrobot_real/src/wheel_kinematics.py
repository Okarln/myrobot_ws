#!/usr/bin/env python3
"""麦轮运动学节点:cmd_vel ⇄ 四轮电机转速,替代实车上位机 PID。

控制链(下发):
    /cmd_vel (m/s, rad/s) ──逆解──► /wheel_speed_cmd (4×电机转子 rpm, 固件顺序)
                                  └─► wheel_bridge 串口 0x10 帧(板载速度环 PID 闭环)

反馈链(上报):
    wheel_bridge /wheel_state (100Hz 4×rpm 反馈) ──正解──► /odom (nav_msgs/Odometry)
                                                      └──► /joint_states (RViz 轮转)

固件轮序(=C620 电调 ID):[0]左前 0x201 [1]左后 0x202 [2]右后 0x203 [3]右前 0x204
右侧电机镜像安装:电机 rpm 与物理轮速符号相反(与 myrobot_controller/mecanum.hpp 一致)。

安全:
  - cmd_vel 超时(默认 0.5s)自动下发零速(固件另有 500ms 掉线保护双保险);
  - 10Hz 保活重发最后目标(wheel_bridge 也会重发,双保险);
  - /chassis_estop 服务(std_srvs/Trigger)→ wheel_estop 急停帧。
"""

import math
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import Trigger

# 固件顺序到物理轮的映射:false=左侧(电机=物理), true=右侧(镜像取反)
WHEEL_NAMES = ["left_front_wheel_joint", "left_back_wheel_joint",
               "right_back_wheel_joint", "right_front_wheel_joint"]
WHEEL_MIRRORED = [False, False, True, True]


class WheelKinematics(Node):

    def __init__(self):
        super().__init__('wheel_kinematics')

        # —— 几何参数(实测:轮径 76mm,前后轮距 47.5cm,左右 77.5cm)——
        self.declare_parameter('wheel_radius', 0.0763)   # 轮半径 [m]
        self.declare_parameter('half_diagonal', 0.625)   # lx+ly [m]
        self.declare_parameter('gear_ratio', 19.2)       # M3508 减速比
        self.declare_parameter('max_wheel_rpm', 9000.0)  # 固件硬限幅
        self.declare_parameter('cmd_timeout', 0.5)       # cmd_vel 超时 [s]
        self.declare_parameter('keepalive_hz', 10.0)     # 指令保活频率
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        # 里程计协方差(打滑时轮式量可信度下降,可调大)
        self.declare_parameter('vx_cov', 0.03)
        self.declare_parameter('vy_cov', 0.05)
        self.declare_parameter('wz_cov', 0.03)

        p = self.get_parameters(['wheel_radius', 'half_diagonal', 'gear_ratio',
                                 'max_wheel_rpm', 'cmd_timeout', 'keepalive_hz',
                                 'odom_frame', 'base_frame'])
        self.r = p[0].value
        self.l = p[1].value
        self.gear = p[2].value
        self.max_rpm = p[3].value
        self.cmd_timeout = p[4].value
        self.keepalive_hz = p[5].value
        self.odom_frame = p[6].value
        self.base_frame = p[7].value
        self.rpm_per_rad_s = 60.0 / (2.0 * math.pi) * self.gear
        self.rad_s_per_rpm = 1.0 / self.rpm_per_rad_s

        self.cmd_lock = threading.Lock()
        self.target = [0.0, 0.0, 0.0]        # 最近的 vx vy wz
        self.last_cmd_time = self.get_clock().now()

        self.pub_cmd = self.create_publisher(Float32MultiArray, 'wheel_speed_cmd', 10)
        self.pub_odom = self.create_publisher(Odometry, 'odom', 50)
        self.pub_js = self.create_publisher(JointState, 'joint_states', 50)

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(Float32MultiArray, 'wheel_state', self.on_wheel_state, 50)

        self.create_service(Trigger, 'chassis_estop', self.on_estop)
        self.create_timer(1.0 / self.keepalive_hz, self.tick_keepalive)

        self.get_logger().info(
            f'麦轮运动学就绪: r={self.r} lx+ly={self.l} gear={self.gear}')

    # ── 逆解:机器人速度 → 4 轮电机 rpm(固件顺序)──────────────
    def inverse(self, vx, vy, wz):
        r, l = self.r, self.l
        w_fl = (vx - vy - l * wz) / r   # 物理轮速 rad/s
        w_fr = (vx + vy + l * wz) / r
        w_rl = (vx + vy - l * wz) / r
        w_rr = (vx - vy + l * wz) / r
        rpm = [w_fl * self.rpm_per_rad_s,   # fw[0] 左前
               w_rl * self.rpm_per_rad_s,   # fw[1] 左后
               -w_rr * self.rpm_per_rad_s,  # fw[2] 右后(镜像)
               -w_fr * self.rpm_per_rad_s]  # fw[3] 右前(镜像)
        return [max(-self.max_rpm, min(self.max_rpm, v)) for v in rpm]

    # ── 正解:4 轮电机 rpm → 机器人速度 ────────────────────────
    def forward(self, fw_rpm):
        w_fl = fw_rpm[0] * self.rad_s_per_rpm
        w_rl = fw_rpm[1] * self.rad_s_per_rpm
        w_rr = -fw_rpm[2] * self.rad_s_per_rpm   # 还原镜像
        w_fr = -fw_rpm[3] * self.rad_s_per_rpm
        r, l = self.r, self.l
        vx = r * (w_fl + w_fr + w_rl + w_rr) / 4.0
        vy = r * (-w_fl + w_fr + w_rl - w_rr) / 4.0
        wz = (-w_fl + w_fr - w_rl + w_rr) * r / (4.0 * l)
        return vx, vy, wz

    # ── 控制链 ─────────────────────────────────────────────────
    def on_cmd_vel(self, msg: Twist):
        with self.cmd_lock:
            self.target = [msg.linear.x, msg.linear.y, msg.angular.z]
            self.last_cmd_time = self.get_clock().now()
        self.send_wheels(self.inverse(*self.target))

    def send_wheels(self, rpm):
        m = Float32MultiArray()
        m.data = [float(v) for v in rpm]
        self.pub_cmd.publish(m)

    def tick_keepalive(self):
        with self.cmd_lock:
            stale = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9 > self.cmd_timeout
            target = list(self.target)
        if stale and any(abs(v) > 1e-6 for v in target):
            self.get_logger().warn('cmd_vel 超时,下发零速', throttle_duration_sec=2.0)
            with self.cmd_lock:
                self.target = [0.0, 0.0, 0.0]
                target = [0.0, 0.0, 0.0]
        self.send_wheels(self.inverse(*target))

    def on_estop(self, _req, resp):
        with self.cmd_lock:
            self.target = [0.0, 0.0, 0.0]
            self.last_cmd_time = self.get_clock().now()
        self.send_wheels([0.0] * 4)
        resp.success = True
        resp.message = '已下发零速(急停帧由 wheel_bridge 处理 /wheel_estop)'
        return resp

    # ── 反馈链:100Hz wheel_state → /odom + /joint_states ─────
    def on_wheel_state(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            return
        rpm = msg.data[:4]
        vx, vy, wz = self.forward(rpm)

        now = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        cv = self.get_parameters(['vx_cov', 'vy_cov', 'wz_cov'])
        odom.twist.covariance[0] = cv[0].value       # vx
        odom.twist.covariance[7] = cv[1].value       # vy
        odom.twist.covariance[35] = cv[2].value      # vyaw
        # 位姿由 EKF 统一积分(odom0 只供速度量),这里留 0
        self.pub_odom.publish(odom)

        js = JointState()
        js.header.stamp = now
        js.name = list(WHEEL_NAMES)
        js.velocity = [rpm[i] * self.rad_s_per_rpm * (-1.0 if WHEEL_MIRRORED[i] else 1.0)
                       for i in range(4)]
        self.pub_js.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = WheelKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_wheels([0.0] * 4)   # 退出零速
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
