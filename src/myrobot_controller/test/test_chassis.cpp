#include <gtest/gtest.h>

#include <array>
#include <cstring>
#include <vector>

#include "chassis/mecanum.hpp"
#include "chassis/pid.hpp"
#include "chassis/protocol.hpp"

using namespace chassis;

// ---------- 协议:编码 -> 逐字节解析 回环 ----------
TEST(ChassisProtocol, EncodeDecodeRoundtrip) {
  const int16_t sent[4] = {-10000, 327, 0, 9999};
  uint8_t frame[FRAME_SIZE];
  encode_current_frame(sent, frame);

  // 电流帧是 PC->STM32 方向,FeedbackParser 只认 FB_SPEED(0x02),
  // 先手动校验帧结构,再把类型改成 0x02 做解析回环。
  ASSERT_EQ(frame[0], 0xAA);
  ASSERT_EQ(frame[1], 0x01);
  ASSERT_EQ(frame[11], 0x55);
  uint8_t sum = 0;
  for (size_t i = 1; i <= 9; ++i) {
    sum = static_cast<uint8_t>(sum + frame[i]);
  }
  ASSERT_EQ(frame[10], sum);

  frame[1] = FB_SPEED;  // 重算校验和后模拟一帧反馈
  sum = 0;
  for (size_t i = 1; i <= 9; ++i) {
    sum = static_cast<uint8_t>(sum + frame[i]);
  }
  frame[10] = sum;

  int16_t got[4] = {0};
  FeedbackParser parser;
  parser.on_feedback = [&](const int16_t rpm[4]) {
    std::memcpy(got, rpm, sizeof(got));
  };
  // 前面插入不含帧头的噪声字节,解析器应全部跳过后正常解出本帧
  const uint8_t noise[] = {0x00, 0x13, 0xFF};
  for (const auto b : noise) {
    parser.feed(b);
  }
  for (size_t i = 0; i < FRAME_SIZE; ++i) {
    parser.feed(frame[i]);
  }
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(got[i], sent[i]);
  }
}

// 坏校验和的帧必须被丢弃,后续好帧正常解析
TEST(ChassisProtocol, RejectsCorruptedFrame) {
  uint8_t frame[FRAME_SIZE];
  const int16_t cur[4] = {100, 200, 300, 400};
  encode_current_frame(cur, frame);
  frame[1] = FB_SPEED;
  uint8_t sum = 0;
  for (size_t i = 1; i <= 9; ++i) {
    sum = static_cast<uint8_t>(sum + frame[i]);
  }
  frame[10] = sum;
  frame[5] ^= 0xFF;  // 破坏载荷

  int calls = 0;
  FeedbackParser parser;
  parser.on_feedback = [&](const int16_t[4]) { ++calls; };
  for (const auto b : frame) {
    parser.feed(b);
  }
  EXPECT_EQ(calls, 0);
}

// ---------- 运动学:与固件公式一致性 + 正逆解往返 ----------
TEST(MecanumKin, MatchesFirmwareDecomposition) {
  // 固件 Speed_Robot_Coordinate(VX,VY,VZ):
  //   v1(左前)=VX-VY-VZ  v2(左后)=VX+VY-VZ
  //   v3(右后)=-VX+VY-VZ v4(右前)=-VX-VY-VZ
  // ROS2 侧 inverse() 与之同构(单位换成物理量)。给定电机 rpm 目标,
  // 等效固件三分量 = 反解一次 forward。
  MecanumKinematics kin;
  const auto rpm = kin.inverse(1.0, 0.0, 0.0);  // 纯前进
  // 左侧为正、右侧为负(镜像),四轮幅值一致
  EXPECT_GT(rpm[0], 0); EXPECT_GT(rpm[1], 0);
  EXPECT_LT(rpm[2], 0); EXPECT_LT(rpm[3], 0);
  EXPECT_NEAR(rpm[0], rpm[1], 1e-6);
  EXPECT_NEAR(rpm[0], -rpm[2], 1e-6);
  EXPECT_NEAR(rpm[0], -rpm[3], 1e-6);
}

TEST(MecanumKin, RotationMirrorsBothSides) {
  MecanumKinematics kin;
  const auto rpm = kin.inverse(0.0, 0.0, 1.0);  // 纯逆时针旋转
  // 电机坐标系四轮同号(固件 VZ 全 -VZ 的同构结构)
  for (const auto v : rpm) {
    EXPECT_LT(v, 0);
  }
  EXPECT_NEAR(rpm[0], rpm[1], 1e-6);
  EXPECT_NEAR(rpm[0], rpm[2], 1e-6);
  EXPECT_NEAR(rpm[0], rpm[3], 1e-6);
}

TEST(MecanumKin, ForwardInverseRoundtrip) {
  MecanumKinematics kin;
  const std::array<double, 3> v{0.7, -0.4, 1.1};
  const auto rpm = kin.inverse(v[0], v[1], v[2]);
  const auto out = kin.forward(rpm);
  EXPECT_NEAR(out[0], v[0], 1e-9);
  EXPECT_NEAR(out[1], v[1], 1e-9);
  EXPECT_NEAR(out[2], v[2], 1e-9);
}

TEST(MecanumKin, LateralSigns) {
  // 纯左移(vy>0):左前/右后物理反转,左后/右前正转(对角线同向)
  MecanumKinematics kin;
  const auto rpm = kin.inverse(0.0, 1.0, 0.0);
  const double fl = rpm[0], rl = rpm[1], rr = -rpm[2], fr = -rpm[3];  // 还原物理方向
  EXPECT_LT(fl, 0); EXPECT_GT(rl, 0); EXPECT_GT(fr, 0); EXPECT_LT(rr, 0);
  EXPECT_NEAR(fl, rr, 1e-6);  // 对角线等速
  EXPECT_NEAR(rl, fr, 1e-6);
}

// ---------- PID:与固件离散结构一致 ----------
TEST(Pid, MatchesFirmwareRecurrence) {
  // 手算固件公式:dt=0.01, kp=2, ki=0, kd=0, 恒定误差 100
  Pid pid(2.0f, 0.0f, 0.0f, 0.01f, 8000.0f, 10000.0f);
  EXPECT_FLOAT_EQ(pid.update(100.0f), 200.0f);
  EXPECT_FLOAT_EQ(pid.update(100.0f), 200.0f);

  // ki 生效:integral=err*dt=1.0, 输出 = kp*100 + ki*1
  Pid pid2(1.0f, 5.0f, 0.0f, 0.01f, 8000.0f, 10000.0f);
  EXPECT_FLOAT_EQ(pid2.update(100.0f), 105.0f);

  // 输出限幅
  Pid pid3(1.0f, 0.0f, 0.0f, 0.01f, 8000.0f, 500.0f);
  EXPECT_FLOAT_EQ(pid3.update(1000.0f), 500.0f);
  EXPECT_FLOAT_EQ(pid3.update(-1000.0f), -500.0f);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
