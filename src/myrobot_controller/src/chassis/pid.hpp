#pragma once

#include <algorithm>

namespace chassis {

// 从 STM32 固件 pid.c 移植,离散结构完全一致:
//   integral += error * dt (带积分限幅)
//   output = kp*e + ki*integral + kd*(e - prev_e)/dt (带输出限幅)
// 因此固件上整定好的参数(kp=1.95, ki=0.5)可直接沿用,dt 换成本节点实际控制周期。
class Pid {
public:
  Pid() = default;
  Pid(float kp, float ki, float kd, float dt, float integral_limit, float output_limit)
  : kp_(kp), ki_(ki), kd_(kd), dt_(dt),
    integral_limit_(integral_limit), output_limit_(output_limit) {}

  void set_params(float kp, float ki, float kd, float dt,
                  float integral_limit, float output_limit)
  {
    kp_ = kp; ki_ = ki; kd_ = kd; dt_ = dt;
    integral_limit_ = integral_limit; output_limit_ = output_limit;
    reset();
  }

  float update(float error)
  {
    return update(error, dt_);
  }

  // dt 由外部传入(仿真时间/实际步长可能偏离标称周期)
  float update(float error, float dt)
  {
    integral_ += error * dt;
    integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);

    const float derivative = (error - prev_error_) / std::max(dt, 1e-6f);
    prev_error_ = error;

    float out = kp_ * error + ki_ * integral_ + kd_ * derivative;
    out = std::clamp(out, -output_limit_, output_limit_);
    return out;
  }

  void reset()
  {
    integral_ = 0.0f;
    prev_error_ = 0.0f;
  }

private:
  float kp_ = 0.0f, ki_ = 0.0f, kd_ = 0.0f, dt_ = 0.01f;
  float integral_limit_ = 8000.0f, output_limit_ = 10000.0f;
  float integral_ = 0.0f, prev_error_ = 0.0f;
};

}  // namespace chassis
