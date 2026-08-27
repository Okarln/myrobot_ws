#!/usr/bin/env python3
"""STM32 轮速固件模拟器(host_link 协议设备侧),无实车时联调整条链路。

行为镜像 ~/Downloads/CAN 固件:
  收 0x10 四轮目标转速(板载速度环按一阶模型模拟电机跟踪, tau≈0.15s)
  收 0x13 急停 → 目标清零、模式回 LOCAL
  收 0x0F PING → 回 PONG(回显主机 ms + 设备 ms)
  发 0x02 轮状态 100Hz(u16 tick + 4×i16 rpm + 4×i16 电流)
  发 0x03 扩展状态 10Hz(角度/温度/模式/故障)
  发 0x1F 链路状态 1Hz
  500ms 无指令且处于上位机模式 → 触发掉线保护(bit0),目标清零闭环刹停

用法:
  ros2 run myrobot_real mock_stm32          # 创建 /tmp/mock_stm32_tty(伪终端)
  # 另一终端让 wheel_bridge 使用该口:
  ros2 launch myrobot_real robot.launch.py wheel_port:=/tmp/mock_stm32_tty
"""

import math
import os
import pty
import struct
import threading
import time

SOF, TEOF, MAX_PAYLOAD = 0xAA, 0x55, 32
T_CMD_CURRENT, T_FB_STATE, T_FB_EXT = 0x01, 0x02, 0x03
T_PING, T_CMD_SPEED, T_CMD_ESTOP, T_FB_LINK = 0x0F, 0x10, 0x13, 0x1F

TAU = 0.15          # 电机一阶时间常数 [s]
WATCHDOG_MS = 500   # 固件掉线保护阈值


def build(ftype, seq, payload=b''):
    body = bytes([len(payload) & 0xFF, ftype & 0xFF, seq & 0xFF]) + payload
    return bytes([SOF]) + body + bytes([sum(body) & 0xFF, TEOF])


class MockSTM32:

    def __init__(self, master_fd):
        self.master = master_fd
        self.tx_seq = 0
        self.target = [0.0] * 4          # 目标 rpm
        self.rpm = [0.0] * 4             # 实际 rpm(一阶模型)
        self.current = [0.0] * 4         # 电流(简单比例模拟)
        self.mode = 0                    # 0 本地 1 上位机转速 2 上位机电流
        self.last_cmd_ms = None          # 最近指令时刻
        self.watchdog_fired = False
        self.rx_ok = 0
        self.t0 = time.monotonic()

    # ── 串口接收(镜像固件状态机)─────────────────────────────
    def rx_loop(self):
        buf = bytearray()
        last_ts = time.monotonic()
        while True:
            try:
                data = os.read(self.master, 64)
            except OSError:
                break
            now = time.monotonic()
            if buf and now - last_ts > 0.005:
                buf.clear()
            last_ts = now
            for b in data:
                if not buf:
                    if b == SOF:
                        buf.append(b)
                    continue
                buf.append(b)
                if len(buf) == 2 and buf[1] > MAX_PAYLOAD:
                    buf.clear()
                    continue
                if len(buf) >= buf[1] + 6:
                    ln = buf[1]
                    if buf[4 + ln] == (sum(buf[1:4 + ln]) & 0xFF) and buf[5 + ln] == TEOF:
                        self.handle(buf[2], bytes(buf[4:4 + ln]))
                    buf.clear()

    def handle(self, ftype, payload):
        self.rx_ok += 1
        if ftype == T_CMD_SPEED and len(payload) == 8:
            self.target = [float(v) for v in struct.unpack('>4h', payload)]
            self.mode = 1
            self.last_cmd_ms = self.dev_ms()
            self.watchdog_fired = False
        elif ftype == T_CMD_ESTOP:
            self.target = [0.0] * 4
            self.mode = 0
            self.last_cmd_ms = self.dev_ms()
        elif ftype == T_PING and len(payload) == 4:
            self._send(T_PING, payload + struct.pack('>I', self.dev_ms()))

    def _send(self, ftype, payload=b''):
        self.tx_seq = (self.tx_seq + 1) & 0xFF
        os.write(self.master, build(ftype, self.tx_seq, payload))

    def dev_ms(self):
        return int((time.monotonic() - self.t0) * 1000) & 0xFFFFFFFF

    # ── 定时上报(优先级 PONG > 0x02 > 0x03 > 0x1F)──────────
    def run(self):
        period_100 = 0.01
        next_100 = next_10 = next_1 = time.monotonic()
        alpha_full = math.exp(-period_100 / TAU)
        while True:
            now = time.monotonic()
            # 掉线保护
            if (self.mode == 1 and self.last_cmd_ms is not None
                    and self.dev_ms() - self.last_cmd_ms > WATCHDOG_MS):
                if not self.watchdog_fired:
                    self.watchdog_fired = True
                    self.target = [0.0] * 4
                # 保持 LOCAL 闭环刹停

            # 电机一阶跟踪 + 电流估计
            for i in range(4):
                self.rpm[i] = self.target[i] + (self.rpm[i] - self.target[i]) * alpha_full
                self.current[i] = (self.target[i] - self.rpm[i]) * 8.0  # 调试用近似

            if now >= next_100:
                next_100 += period_100
                pay = struct.pack('>H', self.dev_ms() & 0xFFFF)
                pay += struct.pack('>4h', *[int(round(v)) for v in self.rpm])
                pay += struct.pack('>4h', *[int(round(v)) for v in self.current])
                self._send(T_FB_STATE, pay)
            if now >= next_10:
                next_10 += 0.1
                flags = 0x01 if self.watchdog_fired else 0x00
                pay = struct.pack('>4H', 0, 0, 0, 0)
                pay += bytes([30, 30, 30, 30, self.mode, flags])
                self._send(T_FB_EXT, pay)
            if now >= next_1:
                next_1 += 1.0
                self._send(T_FB_LINK, struct.pack('>IIHHB', self.dev_ms(), self.rx_ok, 0, 0, self.mode))
            time.sleep(0.002)


def main():
    master, slave = pty.openpty()
    link = '/tmp/mock_stm32_tty'
    try:
        if os.path.islink(link) or os.path.exists(link):
            os.remove(link)
        os.symlink(os.ttyname(slave), link)
    except OSError as e:
        print(f'警告: 无法创建 {link}: {e}(直接使用下面的从设备路径)')
        link = os.ttyname(slave)
    print(f'mock_stm32 就绪,串口路径: {link}')
    print(f'启动桥接: ros2 run myrobot_real wheel_bridge --ros-args -p port:={link}')
    dev = MockSTM32(master)
    threading.Thread(target=dev.rx_loop, daemon=True).start()
    try:
        dev.run()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
