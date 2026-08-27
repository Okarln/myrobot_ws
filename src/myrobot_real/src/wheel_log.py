"""轮速跟踪记录:预期转速 vs 实际转速 → CSV + 退出统计。

数据源(单位一致,均为电机转子 rpm,固件顺序 左前/左后/右后/右前):
  /wheel_speed_cmd  Float32MultiArray  4 目标 rpm(wheel_kinematics 逆解 /cmd_vel)
  /wheel_state      Float32MultiArray  100Hz [4 rpm, 4 电流](wheel_bridge 串口反馈)

记录方式:每收到一条 /wheel_state 采样一行,目标值取最近一条 /wheel_speed_cmd
(零阶保持);sample_hz 控制落盘频率(默认 50Hz,0=全采)。

用法:
  ros2 run myrobot_real wheel_log                        # 默认 ~/wheel_logs/下建 CSV
  ros2 run myrobot_real wheel_log --ros-args -p out:=/tmp/t.csv
  ros2 run myrobot_real wheel_log --ros-args -p sample_hz:=10.0

Ctrl+C 结束时打印各轮误差统计(平均/RMS/最大),快速判断哪个轮子跟踪差。

事后画图(容器外也可):
  python3 -c "
  import pandas as pd, matplotlib.pyplot as plt
  df = pd.read_csv('xxx.csv')
  fig, axes = plt.subplots(4, 1, sharex=True, figsize=(10, 8))
  for ax, w in zip(axes, ['fl', 'rl', 'rr', 'fr']):
      ax.plot(df.t, df['cmd_' + w], label='cmd'); ax.plot(df.t, df['fb_' + w], label='fb')
      ax.set_ylabel(w + ' rpm'); ax.legend()
  plt.xlabel('t [s]'); plt.show()"
"""
import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

WHEELS = ['fl', 'rl', 'rr', 'fr']  # 固件顺序:左前/左后/右后/右前


class WheelLog(Node):

    def __init__(self):
        super().__init__('wheel_log')
        self.declare_parameter('out', '')        # 空 = ~/wheel_logs/wheel_时间戳.csv
        self.declare_parameter('sample_hz', 50.0)

        out = self.get_parameter('out').get_parameter_value().string_value
        if not out:
            os.makedirs(os.path.expanduser('~/wheel_logs'), exist_ok=True)
            out = os.path.expanduser(
                '~/wheel_logs/wheel_%s.csv' % time.strftime('%Y%m%d_%H%M%S'))
        self.get_logger().info('记录到 %s' % out)

        self.f = open(out, 'w', newline='')
        self.writer = csv.writer(self.f)
        self.writer.writerow(
            ['t'] + ['cmd_' + w for w in WHEELS] + ['fb_' + w for w in WHEELS]
            + ['err_' + w for w in WHEELS])  # err = fb - cmd

        self.min_dt = 0.0
        hz = self.get_parameter('sample_hz').get_parameter_value().double_value
        if hz > 0:
            self.min_dt = 1.0 / hz

        self.cmd = [float('nan')] * 4   # 最近目标
        self.cmd_t = None               # 目标消息时间(判断目标是否断流)
        self.last_row_t = 0.0
        self.t0 = None
        self.errors = [[] for _ in range(4)]  # 只统计目标有效(非 NaN)期间的误差

        self.create_subscription(Float32MultiArray, 'wheel_speed_cmd',
                                  self.on_cmd, 20)
        self.create_subscription(Float32MultiArray, 'wheel_state',
                                  self.on_state, 50)

    # ── 目标转速(rpm)──────────────────────────────────────────
    def on_cmd(self, msg):
        if len(msg.data) >= 4:
            self.cmd = list(msg.data[:4])
            if self.cmd_t is None:
                self.cmd_t = time.monotonic()

    # ── 实际转速(前 4 位 rpm,后 4 位电流)─────────────────────
    def on_state(self, msg):
        if len(msg.data) < 4:
            return
        now = time.monotonic()
        if self.t0 is None:
            self.t0 = now
        if now - self.last_row_t < self.min_dt:
            return
        self.last_row_t = now

        fb = list(msg.data[:4])
        err = [fb[i] - self.cmd[i] for i in range(4)]
        for i in range(4):
            if not math.isnan(err[i]):
                self.errors[i].append(err[i])
        self.writer.writerow(
            ['%.3f' % (now - self.t0)]
            + ['%.1f' % v for v in self.cmd]
            + ['%.1f' % v for v in fb]
            + ['%.1f' % v for v in err])
        self.f.flush()

    # ── 退出统计────────────────────────────────────────────────
    def report(self):
        self.f.close()
        n = len(self.errors[0])
        if n == 0:
            self.get_logger().warning('没有同时采到目标+实际,无统计(检查两个话题)')
            return
        self.get_logger().info('===== 轮速跟踪统计(%d 个采样)=====' % n)
        for i, w in enumerate(WHEELS):
            e = self.errors[i]
            mean = sum(e) / n
            rms = math.sqrt(sum(v * v for v in e) / n)
            self.get_logger().info(
                '%s: 平均误差 %7.1f  RMS %7.1f  最大 %7.1f rpm'
                % (w, mean, rms, max(abs(v) for v in e)))


def main(args=None):
    rclpy.init(args=args)
    node = WheelLog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
