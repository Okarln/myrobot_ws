#!/usr/bin/env python3
"""麦轮底盘里程计标定工具:轮半径 / 轮距对角 / 横移打滑率。

原理(wheel_kinematics.py 正解):
    vx = r·(w_fl+w_fr+w_rl+w_rr)/4          轮半径 r 缩放所有平移
    vy = r·(-w_fl+w_fr+w_rl-w_rr)/4         (同 r)
    wz = (-w_fl+w_fr-w_rl+w_rr)·r/(4l)      轮距对角 l 只缩放旋转

标定顺序必须 直线 → 旋转(r 出现在 wz 里,先修 r 再修 l,迭代收敛)。

流程(每步之间停车等回车,全程 20Hz 指令流,异常/Ctrl-C 自动零速+急停):
  1) 直线:以 vx 推车,里程计走满目标距离后停,量实际距离 → r_new = r·(实际/里程)
  2) 旋转:以 wz 原地转 N 圈,量实际角度 → l_new = l·(里程角度/实际角度)
  3) 横移:以 vy 平移,量实际距离 → 打滑率诊断(无参数可修,报告 actual/odom)

用法:
  终端1 只跑底盘栈(勿与 Nav2 同跑,避免 velocity_smoother 抢 /cmd_vel):
    ros2 launch myrobot_real robot.launch.py wheel_port:=/dev/ttyUSB0 \\
        use_imu:=false use_lidar:=false
  终端2:
    ros2 run myrobot_real wheel_calibration --test all
  走护栏通道(nav_real 雷达链)时:
    ros2 run myrobot_real wheel_calibration --test all --cmd-topic /cmd_vel_safe

安全:
  - 速度/时长/距离全部限幅;里程计 2s 无进展(撞墙/卡住)自动停车;
  - 退出与异常先发零速再调 chassis_estop;Ctrl-C 额外发 /wheel_estop 固件刹停。
"""
import argparse
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from std_msgs.msg import Empty
from std_srvs.srv import Trigger


def yaw_from_quat(q):
    """四元数 → 偏航角 [rad]。"""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(a):
    """归一化到 (-pi, pi]。"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


class WheelCalibrator(Node):

    def __init__(self, args):
        super().__init__('wheel_calibration')
        self.args = args

        self.cmd_pub = self.create_publisher(Twist, args.cmd_topic, 10)
        self.estop_pub = self.create_publisher(Empty, '/wheel_estop', 10)

        self.odom = None                     # 最近一帧 Odometry
        self.odom_event = threading.Event()
        self.create_subscription(Odometry, args.odom_topic,
                                 self.on_odom, 20)

        # 测试过程累计量(在 on_odom 回调里更新)
        self._start_xy = None
        self._path_len = 0.0
        self._last_xy = None
        self._yaw_turn = 0.0                 # 累计 |Δyaw|,支持多圈
        self._last_yaw = None
        self._progress_ts = time.monotonic()

        self.get_logger().info(
            f'标定就绪: 指令话题 {args.cmd_topic}, 里程计 {args.odom_topic}')

    # ── 里程计回调 ────────────────────────────────────────────
    def on_odom(self, msg: Odometry):
        self.odom = msg
        self.odom_event.set()
        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        if self._start_xy is not None:
            if self._last_xy is not None:
                self._path_len += math.hypot(p.x - self._last_xy[0],
                                              p.y - self._last_xy[1])
            if self._last_yaw is not None:
                self._yaw_turn += abs(wrap_angle(yaw - self._last_yaw))
        self._last_xy = (p.x, p.y)
        self._last_yaw = yaw
        if self._start_xy is not None:
            # 有进展就刷新看门狗时间戳
            if self._path_len > 1e-4 or self._yaw_turn > 1e-4:
                self._progress_ts = time.monotonic()

    def wait_odom(self, timeout=5.0):
        self.odom_event.clear()
        if not self.odom_event.wait(timeout):
            raise RuntimeError(f'{self.args.odom_topic} {timeout}s 无数据,'
                               '检查 EKF/底盘栈是否在运行')
        return self.odom

    def reset_counters(self):
        self.wait_odom()
        p = self.odom.pose.pose.position
        self._start_xy = (p.x, p.y)
        self._path_len = 0.0
        self._yaw_turn = 0.0
        self._progress_ts = time.monotonic()

    def displacement(self):
        p = self.odom.pose.pose.position
        return math.hypot(p.x - self._start_xy[0], p.y - self._start_xy[1])

    # ── 安全 ─────────────────────────────────────────────────
    def send_zero(self):
        for _ in range(5):
            self.cmd_pub.publish(Twist())
            time.sleep(0.05)

    def hard_stop(self):
        """异常退出:零速 + 固件急停(退 LOCAL 模式刹停)。"""
        self.send_zero()
        self.estop_pub.publish(Empty())
        self.get_logger().warn('已发送急停帧 /wheel_estop')

    def gentle_stop(self):
        """正常收尾:零速 + chassis_estop(仅清目标,不变模式)。"""
        self.send_zero()
        cli = self.create_client(Trigger, '/chassis_estop')
        if cli.wait_for_service(timeout_sec=1.0):
            cli.call_async(Trigger.Request())
            self.get_logger().info('已调用 /chassis_estop 清零目标')

    # ── 读取 wheel_kinematics 当前参数 ───────────────────────
    def read_chassis_params(self):
        cli = self.create_client(GetParameters,
                                 '/wheel_kinematics/get_parameters')
        if not cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('wheel_kinematics 不在线,使用命令行默认值')
            return (self.args.wheel_radius, self.args.half_diagonal)
        req = GetParameters.Request()
        req.names = ['wheel_radius', 'half_diagonal']
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        vals = [p.value.double_value for p in fut.result().values]
        return vals[0], vals[1]

    # ── 单次运动测试 ─────────────────────────────────────────
    def drive(self, *, vx=0.0, vy=0.0, wz=0.0,
              stop_path=None, stop_yaw=None, label=''):
        """以给定速度发布 20Hz 指令,直到里程计达到目标或超时。

        stop_path/stop_yaw:里程计侧的目标累计量(米 / 弧度),先到者停车。
        """
        timeout = self.args.timeout
        self.reset_counters()
        rate = 1.0 / self.args.rate
        t0 = time.monotonic()
        try:
            while rclpy.ok():
                if stop_path is not None and self._path_len >= stop_path:
                    break
                if stop_yaw is not None and self._yaw_turn >= stop_yaw:
                    break
                if time.monotonic() - self._progress_ts > 2.0:
                    self.get_logger().error('里程计 2s 无进展(卡住/未上电?),停车')
                    break
                if time.monotonic() - t0 > timeout:
                    self.get_logger().warn(f'达到安全时长 {timeout}s,提前停车')
                    break
                msg = Twist()
                msg.linear.x, msg.linear.y = vx, vy
                msg.angular.z = wz
                self.cmd_pub.publish(msg)
                time.sleep(rate)
        finally:
            self.send_zero()
            time.sleep(0.5)                  # 等完全停稳再测量
        return self._path_len, self._yaw_turn, self.displacement()

    # ── 三个测试 ─────────────────────────────────────────────
    def test_straight(self, radius):
        print(f'\n═══ 直线测试(标定轮半径 r,当前 {radius:.4f} m)═══')
        print(f'请在车前进方向清出 ≥ {self.args.straight_dist + 0.5:.1f} m 空间,'
              '地面贴尺子/做好标记。')
        input('准备就绪后回车开始(随时 Ctrl-C 急停)...')
        path, _, disp = self.drive(vx=self.args.speed,
                                   stop_path=self.args.straight_dist)
        print(f'里程计路径长 {path:.3f} m,起终点直线位移 {disp:.3f} m'
              f'(两者差大 = 打滑或走弯)。')
        print('请测量车实际前进距离(尺子量两个标记点):')
        actual = float(input('实际距离 [m] = '))
        k = actual / path if path > 1e-6 else float('nan')
        r_new = radius * k
        print(f'比例 实际/里程 = {k:.4f}\n'
              f'→ wheel_radius: {radius:.4f} → {r_new:.4f} m')
        return r_new, k

    def test_rotation(self, diagonal):
        print(f'\n═══ 旋转测试(标定轮距对角 lx+ly,当前 {diagonal:.4f} m)═══')
        print('先确认轮半径已标定并生效(重启 launch),否则误差会混入。')
        print(f'车将原地旋转 {self.args.turns} 圈;在地面沿车身方向画基准线,'
              '或用手机指南针记起始航向。')
        input('准备就绪后回车开始...')
        target = 2.0 * math.pi * self.args.turns
        path, yaw_turn, _ = self.drive(wz=self.args.rot_speed, stop_yaw=target)
        print(f'里程计累计转角 {math.degrees(yaw_turn):.1f}° '
              f'({yaw_turn / (2 * math.pi):.3f} 圈)。')
        print('请测量实际转角(输入度数,如 718 或 1440):')
        actual = float(input('实际角度 [°] = '))
        odom_deg = math.degrees(yaw_turn)
        k = odom_deg / actual if actual else float('nan')
        l_new = diagonal * k
        print(f'比例 里程/实际 = {k:.4f}\n'
              f'→ half_diagonal: {diagonal:.4f} → {l_new:.4f} m')
        return l_new, k

    def test_lateral(self):
        print('\n═══ 横移测试(诊断打滑率,无参数可修)═══')
        print(f'麦轮横移必然打滑,此项只用于评估 vy 可信度。'
              f'车将向左侧横移约 {self.args.lat_dist:.2f} m。')
        input('准备就绪后回车开始...')
        path, _, disp = self.drive(vy=self.args.speed,
                                   stop_path=self.args.lat_dist)
        print(f'里程计路径长 {path:.3f} m,直线位移 {disp:.3f} m。')
        actual = float(input('实际横移距离 [m] = '))
        k = actual / path if path > 1e-6 else float('nan')
        slip = (1.0 - k) * 100.0
        print(f'实际/里程 = {k:.4f} → 打滑率约 {slip:.1f}%'
              f'({"正常范围" if 0 <= slip < 15 else "偏大,检查辊子/地面"})')


def main(args=None):
    ap = argparse.ArgumentParser(description='麦轮里程计标定(直线→旋转→横移)')
    ap.add_argument('--test', default='all',
                    choices=['all', 'straight', 'rotation', 'lateral'])
    ap.add_argument('--cmd-topic', default='/cmd_vel',
                    help='速度指令话题(nav_real 护栏链用 /cmd_vel_safe)')
    ap.add_argument('--odom-topic', default='/odometry/filtered')
    ap.add_argument('--speed', type=float, default=0.15, help='测试线速度 [m/s]')
    ap.add_argument('--rot-speed', type=float, default=0.5,
                    help='测试角速度 [rad/s]')
    ap.add_argument('--straight-dist', type=float, default=2.0,
                    help='直线测试目标距离 [m]')
    ap.add_argument('--lat-dist', type=float, default=1.0,
                    help='横移测试目标距离 [m]')
    ap.add_argument('--turns', type=float, default=2.0, help='旋转测试圈数')
    ap.add_argument('--rate', type=float, default=20.0, help='指令频率 [Hz]')
    ap.add_argument('--timeout', type=float, default=60.0,
                    help='单次测试安全时长 [s]')
    ap.add_argument('--wheel-radius', type=float, default=0.0763,
                    help='wheel_kinematics 不在线时的兜底半径 [m]')
    ap.add_argument('--half-diagonal', type=float, default=0.20,
                    help='wheel_kinematics 不在线时的兜底轮距对角 [m]')
    args = ap.parse_args(args)

    rclpy.init()
    node = WheelCalibrator(args)
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    import signal
    signal.signal(signal.SIGTERM, lambda *a: (_ for _ in ()).throw(KeyboardInterrupt))

    radius, diagonal = node.read_chassis_params()
    print(f'\n当前 wheel_kinematics 参数: wheel_radius={radius:.4f} m, '
          f'half_diagonal={diagonal:.4f} m')

    results = {}
    try:
        if args.test in ('all', 'straight'):
            results['radius'] = node.test_straight(radius)
        if args.test in ('all', 'rotation'):
            results['diagonal'] = node.test_rotation(diagonal)
        if args.test in ('all', 'lateral'):
            node.test_lateral()
        node.gentle_stop()
    except KeyboardInterrupt:
        node.hard_stop()
        print('\n已急停。')
    finally:
        print('\n──────── 标定结果汇总 ────────')
        if 'radius' in results:
            k = results['radius'][1]
            print(f'直线: 实际/里程 = {k:.4f}'
                  f'{"  ✔ 达标(<1%)" if abs(k - 1) < 0.01 else "  ✘ 建议更新参数后重测"}')
            print(f'  wheel_radius = {results["radius"][0]:.4f} m')
        if 'diagonal' in results:
            k = results['diagonal'][1]
            print(f'旋转: 里程/实际 = {k:.4f}'
                  f'{"  ✔ 达标(<1%)" if abs(k - 1) < 0.01 else "  ✘ 建议更新参数后重测"}')
            print(f'  half_diagonal = {results["diagonal"][0]:.4f} m')
        if not results:
            print('(未完成任何测试)')
        print('应用方式: 编辑 src/myrobot_real/config/wheel_kinematics.yaml,'
              '然后重启 launch 生效。')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
