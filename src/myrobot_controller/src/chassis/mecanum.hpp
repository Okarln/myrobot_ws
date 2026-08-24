#pragma once

#include <array>
#include <cmath>

namespace chassis {

// 四麦轮(X 布局)运动学,从固件 Speed_Robot_Coordinate.c 移植并升级为标准单位。
//
// 坐标系:x 前、y 左、z 上(右手系),wz 逆时针为正 —— 与 ROS2 REP-103 一致。
// 轮子编号(物理):
//   FL=左前  FR=右前  RL=左后  RR=右后
//
// 与固件电调顺序(0x201~0x204)的映射 —— 右侧电机镜像安装,符号取反:
//   fw[0]=左前(FL)   fw[1]=左后(RL)   fw[2]=右后(-RR)   fw[3]=右前(-FR)
// 即"发送/收到的电机 rpm"是电机坐标系;右侧轮子的物理转速 = 反馈值取负。
struct MecanumKinematics {
  double wheel_radius = 0.0763;   // 轮半径 [m]
  double half_diagonal = 0.20;    // (轮距+轴距)/2, 即 lx+ly [m]
  double gear_ratio = 19.2;       // M3508 减速比

  // 逆解算:机器人速度 -> 4 轮电机转子目标转速(rpm,电机坐标系,固件顺序)
  // 标准麦轮公式(物理轮速)后乘减速比,右侧按镜像映射取负。
  std::array<double, 4> inverse(double vx, double vy, double wz) const
  {
    const double r = wheel_radius;
    const double l = half_diagonal;
    const double rpm_per_rad_s = 60.0 / (2.0 * M_PI) * gear_ratio;

    const double w_fl = (vx - vy - l * wz) / r;
    const double w_fr = (vx + vy + l * wz) / r;
    const double w_rl = (vx + vy - l * wz) / r;
    const double w_rr = (vx - vy + l * wz) / r;

    return {
      w_fl * rpm_per_rad_s,   // fw[0] 左前
      w_rl * rpm_per_rad_s,   // fw[1] 左后
      -w_rr * rpm_per_rad_s,  // fw[2] 右后(镜像)
      -w_fr * rpm_per_rad_s   // fw[3] 右前(镜像)
    };
  }

  // 正解算:4 轮电机转速(rpm,电机坐标系,固件顺序)-> 机器人速度
  std::array<double, 3> forward(const std::array<double, 4> & fw_rpm) const
  {
    // 先换回物理轮速(rad/s),右侧符号还原
    const double rad_s_per_rpm = 2.0 * M_PI / 60.0 / gear_ratio;
    const double w_fl = fw_rpm[0] * rad_s_per_rpm;
    const double w_rl = fw_rpm[1] * rad_s_per_rpm;
    const double w_rr = -fw_rpm[2] * rad_s_per_rpm;
    const double w_fr = -fw_rpm[3] * rad_s_per_rpm;

    const double r = wheel_radius;
    const double l = half_diagonal;
    return {
      r * (w_fl + w_fr + w_rl + w_rr) / 4.0,
      r * (-w_fl + w_fr + w_rl - w_rr) / 4.0,
      (-w_fl + w_fr - w_rl + w_rr) * r / (4.0 * l)
    };
  }

  // 单轮电机转速(rpm,电机坐标系)-> 轮轴角速度(rad/s,物理方向)
  double wheel_velocity(const double fw_rpm, const bool mirrored) const
  {
    const double w = fw_rpm * 2.0 * M_PI / 60.0 / gear_ratio;
    return mirrored ? -w : w;
  }
};

}  // namespace chassis
