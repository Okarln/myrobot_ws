#pragma once

#include <array>
#include <cstdint>
#include <functional>

namespace chassis {

// 与 STM32 固件 host_link.c 逐字节对应的串口协议。
//
// 帧格式(12 字节,大端):
//   PC   → STM32 电流指令: 0xAA | 0x01 | 4×int16 电流 | cksum | 0x55
//   STM32→ PC    转速反馈: 0xAA | 0x02 | 4×int16 转速 | cksum | 0x55
//   cksum = bytes[1..9] 累加和 & 0xFF
//   电机顺序: [0]左前(0x201) [1]左后(0x202) [2]右后(0x203) [3]右前(0x204)
constexpr uint8_t FRAME_HEADER = 0xAA;
constexpr uint8_t FRAME_TAIL = 0x55;
constexpr size_t FRAME_SIZE = 12;
constexpr uint8_t CMD_CURRENT = 0x01;
constexpr uint8_t FB_SPEED = 0x02;

inline uint8_t frame_checksum(const uint8_t * frame)
{
  uint16_t sum = 0;
  for (size_t i = 1; i <= 9; ++i) {
    sum += frame[i];
  }
  return static_cast<uint8_t>(sum & 0xFF);
}

inline void encode_current_frame(const int16_t current[4], uint8_t * out)
{
  out[0] = FRAME_HEADER;
  out[1] = CMD_CURRENT;
  for (size_t i = 0; i < 4; ++i) {
    const uint16_t v = static_cast<uint16_t>(current[i]);
    out[2 + i * 2] = static_cast<uint8_t>(v >> 8);
    out[3 + i * 2] = static_cast<uint8_t>(v & 0xFF);
  }
  out[10] = frame_checksum(out);
  out[11] = FRAME_TAIL;
}

// 流式解析器:逐字节喂入,凑满一帧且校验通过时回调(与固件状态机一致)。
class FeedbackParser {
public:
  // 回调参数:4 轮电机转速(rpm)
  std::function<void(const int16_t[4])> on_feedback;

  void feed(const uint8_t byte)
  {
    if (state_ == 0) {
      if (byte == FRAME_HEADER) {
        buf_[0] = byte;
        index_ = 1;
        state_ = 1;
      }
      return;
    }

    buf_[index_++] = byte;
    if (index_ < FRAME_SIZE) {
      return;
    }

    // 凑满 12 字节,复位状态机再校验(失败则丢弃,等下一个 0xAA 重新同步)
    state_ = 0;
    index_ = 0;

    if (buf_[1] != FB_SPEED || buf_[10] != frame_checksum(buf_) || buf_[11] != FRAME_TAIL) {
      return;
    }

    int16_t rpm[4];
    for (size_t i = 0; i < 4; ++i) {
      rpm[i] = static_cast<int16_t>((buf_[2 + i * 2] << 8) | buf_[3 + i * 2]);
    }
    if (on_feedback) {
      on_feedback(rpm);
    }
  }

  void reset()
  {
    state_ = 0;
    index_ = 0;
  }

private:
  uint8_t buf_[FRAME_SIZE] = {0};
  uint8_t index_ = 0;
  uint8_t state_ = 0;
};

}  // namespace chassis
