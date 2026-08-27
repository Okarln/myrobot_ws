#!/usr/bin/env python3
"""
ros2_wheel_bridge.py — STM32 轮速链路 ⇄ ROS 2 桥接节点(协议见 WHEEL_PROTOCOL.md)

串口(USART10, 115200 8N1)桥接为以下话题:

  发布  /wheel_state        Float32MultiArray 100Hz [4 rpm, 4 电流]
        /wheel_ext          Float32MultiArray 10Hz  [4 机械角, 4 温度, 模式, 故障标志]
        /link_status        Float32MultiArray 1Hz   [运行ms, 收帧, 错帧, 丢帧, 模式]
  订阅  /wheel_speed_cmd    Float32MultiArray  4 目标转速 rpm(板上速度环闭环)
        /wheel_current_cmd  Float32MultiArray  4 电流指令(直控, 慎用)
        /wheel_estop        Empty              急停

用法:
  python3 ros2_wheel_bridge.py --port /dev/ttyUSB0 [--baud 115200]

依赖: rclpy(ROS 2 环境), pyserial
"""

import argparse
import struct
import threading
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty, Float32MultiArray

try:
    import serial
except ImportError:
    raise SystemExit("缺少 pyserial: pip install pyserial")

# ---------- 协议常量 ----------
SOF = 0xAA
EOF = 0x55
MAX_PAYLOAD = 32

T_CMD_CURRENT = 0x01   # 下行: 4×i16 电流
T_FB_STATE    = 0x02   # 上行: u16 tick + 4×i16 rpm + 4×i16 电流
T_FB_EXT      = 0x03   # 上行: 4×u16 角度 + 4×u8 温度 + 模式 + 标志
T_PING        = 0x0F   # 下行 PING / 上行 PONG
T_CMD_SPEED   = 0x10   # 下行: 4×i16 目标转速
T_CMD_ESTOP   = 0x13   # 下行: 急停
T_FB_LINK     = 0x1F   # 上行: 链路状态

MODE_NAMES = {0: "LOCAL", 1: "HOST_SPEED", 2: "HOST_CURRENT"}


def build_frame(ftype: int, seq: int, payload: bytes = b"") -> bytes:
    """AA | LEN | TYPE | SEQ | PAYLOAD | CKSUM | 55"""
    body = bytes([len(payload) & 0xFF, ftype & 0xFF, seq & 0xFF]) + payload
    return bytes([SOF]) + body + bytes([sum(body) & 0xFF, EOF])


class FrameParser:
    """镜像固件接收状态机:重同步 + 累加和校验"""

    def __init__(self):
        self.buf = bytearray()
        self.on_frame = None          # callback(type, seq, payload)
        self.last_byte_ts = 0.0

    def feed(self, data: bytes):
        for b in data:
            now = time.monotonic()
            if self.buf and now - self.last_byte_ts > 0.005:
                self.buf.clear()
            self.last_byte_ts = now

            if not self.buf:
                if b == SOF:
                    self.buf.append(b)
                continue

            self.buf.append(b)

            if len(self.buf) == 2 and self.buf[1] > MAX_PAYLOAD:
                self._resync()
                continue

            if len(self.buf) >= self.buf[1] + 6:
                ln = self.buf[1]
                if self.buf[4 + ln] == (sum(self.buf[1:4 + ln]) & 0xFF) \
                        and self.buf[5 + ln] == EOF:
                    if self.on_frame:
                        self.on_frame(self.buf[2], self.buf[3], bytes(self.buf[4:4 + ln]))
                # 好帧坏帧都整帧消费,坏帧直接丢弃
                self.buf.clear()

    def _resync(self):
        i = 1
        while i < len(self.buf) and self.buf[i] != SOF:
            i += 1
        if i < len(self.buf):
            del self.buf[:i]
        else:
            self.buf.clear()


class WheelBridge(Node):

    def __init__(self, port: str, baud: int):
        super().__init__("wheel_bridge")
        self.declare_parameter("frame_id", "base_link")

        self.ser = serial.Serial(port, baud, timeout=0.01)
        self.tx_seq = 0
        self.ping_times = {}                       # client_ms -> 发送时刻
        self.lock = threading.Lock()               # 串口写互斥(急停随时可写)

        # 上行统计
        self.rx_cnt = {"state": 0, "ext": 0, "pong": 0, "link": 0}
        self.rx_err = 0
        self.rx_lost = 0
        self._last_seq = None
        self.last_rtt_ms = float("nan")
        self.last_state = [0.0] * 8
        self.dev_mode = 0

        # 保活指令: 最后一条有效的速度/电流指令, 10Hz 重发
        self.keepalive = None                      # (type, payload)

        self.pub_state = self.create_publisher(Float32MultiArray, "wheel_state", 10)
        self.pub_ext = self.create_publisher(Float32MultiArray, "wheel_ext", 10)
        self.pub_link = self.create_publisher(Float32MultiArray, "link_status", 10)

        self.create_subscription(Float32MultiArray, "wheel_speed_cmd", self.on_speed_cmd, 10)
        self.create_subscription(Float32MultiArray, "wheel_current_cmd", self.on_current_cmd, 10)
        self.create_subscription(Empty, "wheel_estop", self.on_estop, 10)

        self.parser = FrameParser()
        self.parser.on_frame = self.on_frame
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        self.create_timer(0.1, self.tick_keepalive)   # 10Hz 指令保活(≥2Hz 即可)
        self.create_timer(1.0, self.tick_stats)       # 1Hz PING + 状态日志

        self.get_logger().info(f"已打开 {port} @ {baud}, 话题已就绪")

    # ---------- 下行 ----------

    def _send(self, ftype: int, payload: bytes = b""):
        self.tx_seq = (self.tx_seq + 1) & 0xFF
        with self.lock:
            try:
                self.ser.write(build_frame(ftype, self.tx_seq, payload))
            except serial.SerialException as e:
                self.get_logger().error(f"串口发送失败: {e}")

    @staticmethod
    def _pack4(values, limit):
        vals = [max(-limit, min(limit, float(v))) for v in values[:4]]
        while len(vals) < 4:
            vals.append(0.0)
        return struct.pack(">4h", *[int(round(v)) for v in vals])

    def on_speed_cmd(self, msg: Float32MultiArray):
        if len(msg.data) != 4:
            self.get_logger().warn(f"wheel_speed_cmd 需要 4 个数, 忽略(收到 {len(msg.data)})")
            return
        payload = self._pack4(msg.data, 9000)
        self.keepalive = (T_CMD_SPEED, payload)
        self._send(T_CMD_SPEED, payload)

    def on_current_cmd(self, msg: Float32MultiArray):
        if len(msg.data) != 4:
            self.get_logger().warn(f"wheel_current_cmd 需要 4 个数, 忽略(收到 {len(msg.data)})")
            return
        payload = self._pack4(msg.data, 10000)
        self.keepalive = (T_CMD_CURRENT, payload)
        self._send(T_CMD_CURRENT, payload)

    def on_estop(self, _msg: Empty):
        self.keepalive = None
        self._send(T_CMD_ESTOP)
        self.get_logger().warn("已发送急停帧")

    def tick_keepalive(self):
        """10Hz 重发最后一条指令: 固件 500ms 掉线保护要求 ≥2Hz"""
        if self.keepalive is not None:
            self._send(self.keepalive[0], self.keepalive[1])

    def tick_stats(self):
        ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
        self.ping_times[ms] = time.monotonic()
        self._send(T_PING, struct.pack(">I", ms))
        rpm = " ".join(f"{v:6.0f}" for v in self.last_state[:4])
        self.get_logger().info(
            f"rpm[{rpm}] mode={MODE_NAMES.get(self.dev_mode, self.dev_mode)} "
            f"state={self.rx_cnt['state'] * 0.1:5.1f}Hz "
            f"lost={self.rx_lost} err={self.rx_err} rtt={self.last_rtt_ms:.1f}ms")

    # ---------- 上行 ----------

    def rx_loop(self):
        while rclpy.ok():
            try:
                data = self.ser.read(64)
            except serial.SerialException as e:
                self.get_logger().error(f"串口读取失败: {e}")
                break
            if data:
                self.parser.feed(data)

    def on_frame(self, ftype: int, seq: int, payload: bytes):
        # SEQ 丢帧统计(上行方向)
        if self._last_seq is not None:
            self.rx_lost += (seq - self._last_seq - 1) & 0xFF
        self._last_seq = seq

        if ftype == T_FB_STATE and len(payload) == 18:
            vals = struct.unpack(">4h4h", payload[2:18])
            self.last_state = [float(v) for v in vals]
            self.rx_cnt["state"] += 1
            msg = Float32MultiArray()
            msg.data = self.last_state
            self.pub_state.publish(msg)

        elif ftype == T_FB_EXT and len(payload) == 14:
            vals = struct.unpack(">4H4BBB", payload)
            angles, temps = vals[0:4], vals[4:8]
            mode, flags = vals[8], vals[9]
            self.rx_cnt["ext"] += 1
            self.dev_mode = mode
            if flags & 0x01:
                self.get_logger().warn("固件报告: 指令掉线保护已触发(bit0)", throttle_duration_sec=2.0)
            if flags & 0x02:
                self.get_logger().warn("固件报告: 电机 CAN 反馈超时(bit1)", throttle_duration_sec=2.0)
            msg = Float32MultiArray()
            msg.data = [float(a) for a in angles] + [float(t) for t in temps] + [float(mode), float(flags)]
            self.pub_ext.publish(msg)

        elif ftype == T_PING and len(payload) == 8:
            client_ms, _dev_ms = struct.unpack(">II", payload)
            sent = self.ping_times.pop(client_ms, None)
            if sent is not None:
                self.last_rtt_ms = (time.monotonic() - sent) * 1000.0
            self.rx_cnt["pong"] += 1

        elif ftype == T_FB_LINK and len(payload) == 13:
            up_ms, rx_ok, rx_err, rx_lost, mode = struct.unpack(">IIHHB", payload)
            self.rx_cnt["link"] += 1
            msg = Float32MultiArray()
            msg.data = [float(up_ms), float(rx_ok), float(rx_err), float(rx_lost), float(mode)]
            self.pub_link.publish(msg)

    # ---------- 退出安全 ----------

    def shutdown(self):
        try:
            zero = struct.pack(">4h", 0, 0, 0, 0)
            with self.lock:
                self.ser.write(build_frame(T_CMD_SPEED, (self.tx_seq + 1) & 0xFF, zero))
                self.ser.write(build_frame(T_CMD_ESTOP, (self.tx_seq + 2) & 0xFF))
                self.ser.flush()
            self.ser.close()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser(description="STM32 轮速链路 ⇄ ROS 2 桥接")
    ap.add_argument("--port", required=True, help="串口设备, 如 /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--ros-args", action="store_true", help=argparse.SUPPRESS)  # 占位
    args, _unknown = ap.parse_known_args()

    rclpy.init()
    node = WheelBridge(args.port, args.baud)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
