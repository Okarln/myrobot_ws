#!/usr/bin/env python3
"""vel_track_test —— cmd_vel 跟踪验证工具(链B 专用)。

发送规定的速度/加速度剖面到 /cmd_vel,同步记录四路信息,结束时输出指标:

  剖面:
    step  阶跃(0→V→0)      验"速度":上升时间/过冲/稳态误差/刹车
    ramp  斜坡(A m/s²→Vr→0)  验"加速度":实际加速度 vs 指令加速度
    all   两个依次跑

  记录(50 Hz,写 CSV):
    /cmd_vel      发出的指令(要求什么)
    /odom         轮反馈正解算的车体速度(轮子说跑了多快)
    /model/myrobot/odometry  gz 真值(车实际跑了多快,打滑裁判)
    /chassis_debug  [目标rpm×4, 反馈rpm×4, 电流×4](轮域证据)

用法(容器内,先起链B):
  ros2 launch myrobot_gazebo gz_chassis_test.launch.py headless:=true rviz:=false
  ros2 run myrobot_controller vel_track_test.py --profile all
  # 参数:--speed 0.5(阶跃目标) --accel 0.4 --ramp-speed 0.3(斜坡目标,对齐 nav2 假设)
"""
import argparse
import csv
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def smooth(xs, win=5):
    out = []
    half = win // 2
    for i in range(len(xs)):
        lo, hi = max(0, i - half), min(len(xs), i + half + 1)
        out.append(sum(xs[lo:hi]) / (hi - lo))
    return out


def slope(xs, ys):
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    num = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    den = sum((x - mx) ** 2 for x in xs)
    return num / den if den > 1e-9 else 0.0


class TrackTest(Node):

    def __init__(self, args):
        super().__init__('vel_track_test')
        # 默认跟随仿真时钟;实车上用 --ros-args -p use_sim_time:=false
        self.declare_parameter('use_sim_time', True)

        self.args = args
        self.started = False
        self.t0 = None
        self.seg_idx = 0
        self.seg_t0 = 0.0
        self.last_print = 0.0
        self.warned_debug = False
        self.records = []

        # ---- 最新观测(互斥访问可忽略:单线程执行器)----
        self.odom = None       # (vx, vy, wz, x, y, yaw)
        self.gt = None         # (vx, vy, wz) 真值;None=未见
        self.debug = None      # 12 floats [tgt×4, fb×4, cur×4]

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 20)
        self.create_subscription(Odometry, '/model/myrobot/odometry',
                                 self._on_gt, 20)
        self.create_subscription(Float32MultiArray, 'chassis_debug',
                                 self._on_debug, 20)
        self.timer = self.create_timer(1.0 / args.hz, self._tick)

        self.segments = self._build_segments(args)

    # ---------- 观测回调 ----------
    def _on_odom(self, msg):
        p, q = msg.pose.pose, msg.pose.pose.orientation
        self.odom = (msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                     msg.twist.twist.angular.z, p.position.x, p.position.y,
                     yaw_of(q))

    def _on_gt(self, msg):
        self.gt = (msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                   msg.twist.twist.angular.z)

    def _on_debug(self, msg):
        if len(msg.data) == 12:
            self.debug = list(msg.data)

    # ---------- 剖面 ----------
    @staticmethod
    def _build_segments(a):
        segs = []
        if a.profile in ('step', 'all'):
            v = a.speed
            segs += [('idle', 1.5, lambda t: 0.0),
                     ('step_hold', a.hold, lambda t, v=v: v),
                     ('step_zero', 5.0, lambda t: 0.0)]
        if a.profile in ('ramp', 'all'):
            acc, vr = a.accel, a.ramp_speed
            up = max(vr / acc, 0.5)
            segs += [('ramp_idle', 1.5, lambda t: 0.0),
                     ('ramp_up', up, lambda t, acc=acc, vr=vr: min(acc * t, vr)),
                     ('ramp_hold', 2.0, lambda t, vr=vr: vr),
                     ('ramp_down', up, lambda t, acc=acc, vr=vr: max(vr - acc * t, 0.0)),
                     ('ramp_end', 1.5, lambda t: 0.0)]
        return segs

    # ---------- 主循环 ----------
    def _tick(self):
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9

        if not self.started:
            if self.odom is None:
                if t - self.last_print > 1.0:
                    self.last_print = t
                    self.get_logger().info('等待 /odom(链B 未就绪?)...')
                return
            self.started = True
            self.t0 = t
            self.seg_t0 = t
            self.get_logger().info(
                f'开始 profile={self.args.profile} '
                f'speed={self.args.speed} accel={self.args.accel} '
                f'真值{"可用" if self.gt is not None else "不可用(未桥接?)"}')

        # 提示链A 跑错了:它没有 chassis_debug
        if self.debug is None and not self.warned_debug and t - self.t0 > 2.0:
            self.warned_debug = True
            self.get_logger().warn(
                '未收到 /chassis_debug —— 链A(gz_sim.launch.py)没有该话题,'
                '请改用 gz_chassis_test.launch.py')

        name, dur, fn = self.segments[self.seg_idx]
        if t - self.seg_t0 >= dur:
            self.seg_idx += 1
            self.seg_t0 = t
            if self.seg_idx >= len(self.segments):
                return self._finish()
            name, dur, fn = self.segments[self.seg_idx]

        cmd = fn(t - self.seg_t0)
        msg = Twist()
        msg.linear.x = cmd
        self.pub.publish(msg)

        o = self.odom or (0.0,) * 6
        g = self.gt or (None, None, None)
        d = self.debug or [None] * 12
        self.records.append({
            't': t - self.t0, 'seg': name, 'cmd_vx': cmd,
            'odom_vx': o[0], 'odom_vy': o[1], 'odom_wz': o[2],
            'odom_x': o[3], 'odom_y': o[4], 'odom_yaw': o[5],
            'gt_vx': g[0],
            'rpm_tgt': sum(d[0:4]) / 4.0 if d[0] is not None else None,
            'rpm_fb': sum(d[4:8]) / 4.0 if d[4] is not None else None,
            'cur': sum(d[8:12]) / 4.0 if d[8] is not None else None,
        })

        if t - self.last_print > 0.5:
            self.last_print = t
            r = self.records[-1]
            err = (r['odom_vx'] - cmd) / cmd * 100 if abs(cmd) > 1e-6 else 0.0
            fb = f"{r['rpm_fb']:.0f}" if r['rpm_fb'] is not None else 'n/a'
            self.get_logger().info(
                f"[{name:9s}] t={r['t']:5.2f} cmd={cmd:5.2f} "
                f"odom={r['odom_vx']:5.2f} ({err:+5.1f}%) fb={fb}rpm")

    # ---------- 指标与收尾 ----------
    def _finish(self):
        self.timer.cancel()
        recs = self.records
        lines = ['', '=' * 62, '  跟踪验证结果', '=' * 62]

        def window(seg):
            return [r for r in recs if r['seg'] == seg]

        # ---- 阶跃:速度跟踪 ----
        hold = window('step_hold')
        if hold:
            v = self.args.speed
            t_start = hold[0]['t']
            xs = [r['t'] for r in recs if r['t'] >= t_start]
            vs = smooth([r['odom_vx'] for r in recs if r['t'] >= t_start])
            i10 = next((i for i, y in enumerate(vs) if y >= 0.1 * v), None)
            i90 = next((i for i, y in enumerate(vs) if y >= 0.9 * v), None)
            if i10 is not None and i90 is not None and i90 > i10:
                lines.append(f"上升时间 10%→90%:  {xs[i90] - xs[i10]:.2f} s")
            peak = max(vs)
            ov = (peak - v) / v * 100
            lines.append(f"峰值速度/过冲:      {peak:.3f} m/s (过冲 {ov:+.1f}%)")
            tail = [r for r in hold if r['t'] > hold[-1]['t'] - 1.0]
            if tail:
                e = sum(abs(r['odom_vx'] - v) for r in tail) / len(tail)
                lines.append(f"稳态误差(末1s):     {e:.3f} m/s ({e / v * 100:.1f}%)")
            if tail and tail[-1]['rpm_tgt'] is not None:
                tgt = sum(r['rpm_tgt'] for r in tail) / len(tail)
                fb = sum(r['rpm_fb'] for r in tail) / len(tail)
                lines.append(f"轮域跟踪(末1s):     目标 {tgt:.0f} rpm / "
                             f"反馈 {fb:.0f} rpm ({fb / tgt * 100 if tgt else 0:.1f}%)")
            # 刹车 + 直线度
            zero = window('step_zero')
            if zero:
                z0 = zero[0]
                stop_i = next((i for i, r in enumerate(zero)
                               if r['odom_vx'] <= 0.02), None)
                if stop_i is not None:
                    st = zero[stop_i]
                    dtb = st['t'] - z0['t']
                    dist = math.hypot(st['odom_x'] - z0['odom_x'],
                                      st['odom_y'] - z0['odom_y'])
                    lines.append(f"刹车时间/距离:      {dtb:.2f} s / {dist:.3f} m")
                else:
                    lines.append('刹车时间/距离:      >记录窗口(未停住!)')
                holdend = hold[-1]
                lat = max(abs(r['odom_y'] - hold[0]['odom_y']) for r in hold)
                lines.append(f"直线度(保持段):      横向偏移 {lat:.3f} m, "
                             f"yaw 漂移 {math.degrees(holdend['odom_yaw'] - hold[0]['odom_yaw']):+.1f}°")

        # ---- 斜坡:加速度跟踪 ----
        up = window('ramp_up')
        if up:
            body = [r for r in up if r['t'] - up[0]['t'] > 0.1]  # 去掉起步死区
            if len(body) > 5:
                a_real = slope([r['t'] for r in body], [r['odom_vx'] for r in body])
                lag = sum(r['cmd_vx'] - r['odom_vx'] for r in body) / len(body)
                vs = [r['odom_vx'] for r in up]
                ts = [r['t'] for r in up]
                dmax = max(abs(vs[i + 1] - vs[i]) / max(ts[i + 1] - ts[i], 1e-6)
                           for i in range(len(vs) - 1))
                lines.append('')
                lines.append(f"指令加速度:          {self.args.accel:.2f} m/s²")
                lines.append(f"实际加速度(拟合):   {a_real:.2f} m/s² "
                             f"({a_real / self.args.accel * 100:.0f}%)")
                lines.append(f"最大瞬时加速度:      {dmax:.2f} m/s²")
                lines.append(f"跟踪滞后(均):       {lag:.3f} m/s")

        # ---- 打滑裁判:odom vs 真值 ----
        both = [r for r in recs if r['gt_vx'] is not None]
        if both:
            slip = max(abs(r['odom_vx'] - r['gt_vx']) for r in both)
            lines.append('')
            lines.append(f"真值对比样本:        {len(both)} 条")
            lines.append(f"max|odom-真值|:      {slip:.3f} m/s "
                         f"({'有明显打滑!' if slip > 0.05 else '无明显打滑'})")
        else:
            lines.append('')
            lines.append('真值对比:            不可用(检查 odometry 插件+桥)')

        # ---- CSV ----
        path = self.args.out or f"/tmp/vel_track_{self.args.profile}_{time.strftime('%H%M%S')}.csv"
        cols = ['t', 'seg', 'cmd_vx', 'odom_vx', 'odom_vy', 'odom_wz',
                'odom_x', 'odom_y', 'odom_yaw', 'gt_vx', 'rpm_tgt', 'rpm_fb', 'cur']
        with open(path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=cols)
            w.writeheader()
            w.writerows(recs)
        lines.append('')
        lines.append(f"CSV: {path}  ({len(recs)} 行)")

        # ---- 可选 PNG ----
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(figsize=(9, 5))
            ax.plot([r['t'] for r in recs], [r['cmd_vx'] for r in recs],
                    'k--', lw=1.5, label='cmd_vel (指令)')
            ax.plot([r['t'] for r in recs], [r['odom_vx'] for r in recs],
                    'b-', lw=1.2, label='odom (轮反馈)')
            if both:
                ax.plot([r['t'] for r in recs], [r['gt_vx'] for r in recs],
                        'g-', lw=1.0, alpha=0.7, label='ground truth (真值)')
            ax.set_xlabel('t [s]')
            ax.set_ylabel('vx [m/s]')
            ax.legend()
            ax.grid(True, alpha=0.3)
            png = path.rsplit('.', 1)[0] + '.png'
            fig.savefig(png, dpi=120, bbox_inches='tight')
            lines.append(f"曲线: {png}")
        except ImportError:
            pass

        print('\n'.join(lines))
        rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--profile', choices=['step', 'ramp', 'all'], default='all')
    ap.add_argument('--speed', type=float, default=0.5, help='阶跃目标速度 [m/s]')
    ap.add_argument('--accel', type=float, default=0.4, help='斜坡加速度 [m/s²]')
    ap.add_argument('--ramp-speed', type=float, default=0.3,
                    help='斜坡段目标速度 [m/s](默认对齐 nav2 假设)')
    ap.add_argument('--hold', type=float, default=4.0, help='阶跃保持时长 [s]')
    ap.add_argument('--hz', type=float, default=50.0, help='指令发布频率')
    ap.add_argument('--out', default='', help='CSV 输出路径')
    args = ap.parse_args()

    rclpy.init()
    node = TrackTest(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.timer.cancel()
        print(f'\n中断,已采 {len(node.records)} 条(不写报告)')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
