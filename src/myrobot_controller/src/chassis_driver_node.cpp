// 底盘驱动节点:麦轮逆解算 + 4 轮速度环 PID。
// 控制逻辑从 STM32 固件(Speed_Robot_Coordinate.c / pid.c)迁移至此,
// 固件侧只执行"电流转发到 CAN + 转速上报"。
//
// 三种运行模式(mode 参数):
//   serial    实车:串口发电流帧/收转速帧(对应固件 host_link.c 协议)
//   gazebo    仿真闭环:订阅 /joint_states 取轮速反馈,
//             PID 电流经电流->力矩换算后发 /wheel_effort_controller/commands,
//             由 gz ros2_control 的 effort 控制器驱动仿真机器人
//   simulated 无硬件自测:节点内一阶模型模拟电机响应
//
// 数据流: /cmd_vel(m/s, rad/s) -> 麦轮逆解算(电机 rpm) -> 每轮 PID
//         -> [串口电流帧 | effort 指令] ;反馈 -> /odom, /chassis_debug

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "chassis/mecanum.hpp"
#include "chassis/pid.hpp"
#include "chassis/protocol.hpp"
#include "chassis/serial_port.hpp"

namespace chassis {

using namespace std::chrono_literals;

class ChassisDriver : public rclcpp::Node {
public:
  ChassisDriver()
  : Node("chassis_driver")
  {
    // ---- 参数 ----
    mode_ = declare_parameter<std::string>("mode", "serial");
    // 兼容旧参数:simulated:=true 等价 mode:=simulated
    if (declare_parameter<bool>("simulated", false)) {
      mode_ = "simulated";
    }
    port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    control_rate_ = declare_parameter<double>("control_rate", 100.0);
    cmd_timeout_ = declare_parameter<double>("cmd_vel_timeout", 0.5);
    fb_timeout_ = declare_parameter<double>("feedback_timeout", 0.5);

    kin_.wheel_radius = declare_parameter<double>("wheel_radius", 0.0763);
    kin_.half_diagonal = declare_parameter<double>("half_diagonal", 0.20);
    kin_.gear_ratio = declare_parameter<double>("gear_ratio", 19.2);
    // gazebo 模式:PID 电流(count)换算成轮轴力矩(N·m)
    // M3508+C620: 16384 count ≈ 4.9 N·m(输出轴) → 0.0003 N·m/count
    torque_per_current_ = declare_parameter<double>("torque_per_current", 0.0003);

    max_vx_ = declare_parameter<double>("max_vx", 2.0);
    max_vy_ = declare_parameter<double>("max_vy", 2.0);
    max_wz_ = declare_parameter<double>("max_wz", 4.0);

    const float kp = declare_parameter<double>("pid.kp", 1.95);
    const float ki = declare_parameter<double>("pid.ki", 0.50);
    const float kd = declare_parameter<double>("pid.kd", 0.0);
    const float ilim = declare_parameter<double>("pid.integral_limit", 8000.0);
    const float olim = declare_parameter<double>("pid.output_limit", 10000.0);

    // joint_names 顺序约定:[左前, 左后, 右前, 右后]
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names",
      {"left_front_wheel_joint", "left_back_wheel_joint",
        "right_front_wheel_joint", "right_back_wheel_joint"});
    base_frame_ = declare_parameter<std::string>("base_frame_id", "base_link");
    odom_frame_ = declare_parameter<std::string>("odom_frame_id", "odom");
    publish_tf_ = declare_parameter<bool>("publish_tf", false);  // 与定位包共存时保持 false
    // 纯轮式里程计轨迹累积(/wheel_odom_path),RViz 可直接显示
    publish_path_ = declare_parameter<bool>("publish_path", true);
    path_period_ = declare_parameter<double>("path_sample_period", 0.2);

    for (auto & p : pid_) {
      p.set_params(kp, ki, kd, static_cast<float>(1.0 / control_rate_), ilim, olim);
    }

    // ---- 话题 ----
    sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(10),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        cmd_ = *msg;
        cmd_stamp_ = now();
      });

    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("odom", 50);
    pub_path_ = create_publisher<nav_msgs::msg::Path>("wheel_odom_path", 10);
    pub_debug_ = create_publisher<std_msgs::msg::Float32MultiArray>("chassis_debug", 10);
    if (mode_ != "gazebo") {
      // gazebo 模式下 /joint_states 由 joint_state_broadcaster 发布,
      // 我们只订阅,不再重复发布(否则反馈自环)。
      pub_joint_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    }

    if (mode_ == "serial") {
      if (serial_.open(port_)) {
        RCLCPP_INFO(get_logger(), "串口已打开: %s @115200", port_.c_str());
      } else {
        RCLCPP_WARN(get_logger(),
          "串口 %s 打开失败(设备未插入?),节点继续运行并周期重试", port_.c_str());
      }
      reader_ = std::thread([this] { reader_loop(); });
    } else if (mode_ == "gazebo") {
      sub_joint_ = create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::QoS(50),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) { on_joint_states(msg); });
      pub_effort_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "wheel_effort_controller/commands", 10);
    }

    timer_ = create_wall_timer(
      std::chrono::microseconds(static_cast<int64_t>(1e6 / control_rate_)),
      [this] { control_step(); });

    RCLCPP_INFO(get_logger(),
      "chassis_driver 启动(mode=%s): r=%.3fm L=%.3f gear=%.1f, PID kp=%.2f ki=%.2f",
      mode_.c_str(), kin_.wheel_radius, kin_.half_diagonal, kin_.gear_ratio, kp, ki);
  }

  ~ChassisDriver() override
  {
    running_ = false;
    if (reader_.joinable()) {
      reader_.join();
    }
    if (mode_ == "serial" && serial_.is_open()) {
      int16_t zero[4] = {0, 0, 0, 0};
      uint8_t frame[FRAME_SIZE];
      encode_current_frame(zero, frame);
      serial_.write(frame, FRAME_SIZE);
    }
  }

private:
  // ============ gazebo 模式:轮速反馈(rad/s) -> 电机域 rpm(固件顺序) ============
  void on_joint_states(const sensor_msgs::msg::JointState::ConstSharedPtr & msg)
  {
    // joint_names_ 顺序 [LF, LB, RF, RB];按名字找下标
    double v[4] = {0, 0, 0, 0};
    int found = 0;
    for (size_t n = 0; n < msg->name.size(); ++n) {
      for (size_t j = 0; j < 4; ++j) {
        if (msg->name[n] == joint_names_[j]) {
          v[j] = msg->velocity.size() > n ? msg->velocity[n] : 0.0;
          ++found;
        }
      }
    }
    if (found < 4) {
      return;  // 非四轮完整消息,丢弃
    }

    const double rpm_per_rad_s = 60.0 / (2.0 * M_PI) * kin_.gear_ratio;
    std::lock_guard<std::mutex> lk(fb_mutex_);
    // 物理轮速 -> 电机域(右侧镜像取负),固件顺序 [LF, RL, RR, FR]
    fb_rpm_[0] = v[0] * rpm_per_rad_s;          // 左前
    fb_rpm_[1] = v[1] * rpm_per_rad_s;          // 左后
    fb_rpm_[2] = -v[3] * rpm_per_rad_s;         // 右后(镜像)
    fb_rpm_[3] = -v[2] * rpm_per_rad_s;         // 右前(镜像)
    fb_stamp_ = now();
  }

  void reader_loop()
  {
    std::vector<uint8_t> buf(256);
    parser_.on_feedback = [this](const int16_t rpm[4]) {
      std::lock_guard<std::mutex> lk(fb_mutex_);
      for (int i = 0; i < 4; ++i) {
        fb_rpm_[i] = rpm[i];
      }
      fb_stamp_ = now();
    };

    while (running_ && rclcpp::ok()) {
      const ssize_t n = serial_.read(buf.data(), buf.size());
      if (n > 0) {
        for (ssize_t i = 0; i < n; ++i) {
          parser_.feed(buf[static_cast<size_t>(i)]);
        }
      } else if (n < 0) {
        std::this_thread::sleep_for(200ms);
      }
    }
  }

  void control_step()
  {
    const rclcpp::Time t = now();
    // 实际控制步长(仿真时间可能非实时),夹在合理范围
    float dt = 0.01f;
    if (last_step_.nanoseconds() > 0) {
      dt = static_cast<float>((t - last_step_).seconds());
      dt = std::clamp(dt, static_cast<float>(0.5 / control_rate_),
        static_cast<float>(2.0 / control_rate_));
    }
    last_step_ = t;

    // 周期性重连(仅串口模式且未打开时)
    if (mode_ == "serial" && !serial_.is_open() && (t - last_reconnect_).seconds() > 2.0) {
      last_reconnect_ = t;
      if (serial_.open(port_)) {
        RCLCPP_INFO(get_logger(), "串口重连成功: %s", port_.c_str());
      }
    }

    // ---- 1. 取速度指令,超时归零 ----
    double vx = 0.0, vy = 0.0, wz = 0.0;
    {
      std::lock_guard<std::mutex> lk(cmd_mutex_);
      if ((t - cmd_stamp_).seconds() < cmd_timeout_) {
        vx = std::clamp(cmd_.linear.x, -max_vx_, max_vx_);
        vy = std::clamp(cmd_.linear.y, -max_vy_, max_vy_);
        wz = std::clamp(cmd_.angular.z, -max_wz_, max_wz_);
      } else {
        for (auto & p : pid_) {
          p.reset();  // 停车时清积分,防下次起步积分踢
        }
      }
    }

    // ---- 2. 麦轮逆解算:电机坐标系目标 rpm(固件顺序)----
    const auto target_rpm = kin_.inverse(vx, vy, wz);

    // ---- 3. 取反馈 ----
    std::array<double, 4> fb{};
    bool fb_fresh;
    if (mode_ == "simulated") {
      constexpr double tau = 0.05;  // 电机响应时间常数
      for (size_t i = 0; i < 4; ++i) {
        const double alpha = dt / (tau + dt);
        sim_rpm_[i] += (target_rpm[i] - sim_rpm_[i]) * alpha;
        fb[i] = sim_rpm_[i];
      }
      fb_fresh = true;
    } else {
      std::lock_guard<std::mutex> lk(fb_mutex_);
      fb_fresh = (t - fb_stamp_).seconds() < fb_timeout_;
      fb = fb_fresh ? fb_rpm_ : std::array<double, 4>{};  // 反馈丢失按 0 处理
    }

    // ---- 4. 每轮速度环 PID -> 电流 ----
    int16_t current[4];
    for (size_t i = 0; i < 4; ++i) {
      const float err = static_cast<float>(target_rpm[i]) -
        static_cast<float>(fb_fresh ? fb[i] : 0.0);
      current[i] = static_cast<int16_t>(pid_[i].update(err, dt));
    }

    // ---- 5. 输出 ----
    if (mode_ == "serial") {
      if (serial_.is_open()) {
        uint8_t frame[FRAME_SIZE];
        encode_current_frame(current, frame);
        if (!serial_.write(frame, FRAME_SIZE) && !write_fail_logged_) {
          write_fail_logged_ = true;
          RCLCPP_ERROR(get_logger(), "串口写入失败,将重连");
          serial_.close();
        }
      }
    } else if (mode_ == "gazebo") {
      // 电机域电流 -> 物理轮轴力矩(右侧镜像还原),关节顺序 [LF, LB, RF, RB]
      std_msgs::msg::Float64MultiArray out;
      out.data = {
        current[0] * torque_per_current_,
        current[1] * torque_per_current_,
        -current[3] * torque_per_current_,   // 右前
        -current[2] * torque_per_current_    // 右后
      };
      pub_effort_->publish(out);
    }

    // ---- 6. 里程计(用反馈算,不用目标值)----
    if (fb_fresh) {
      const auto body = kin_.forward(fb);
      const double yaw_mid = yaw_ + body[2] * dt * 0.5;
      x_ += (body[0] * std::cos(yaw_mid) - body[1] * std::sin(yaw_mid)) * dt;
      y_ += (body[0] * std::sin(yaw_mid) + body[1] * std::cos(yaw_mid)) * dt;
      yaw_ += body[2] * dt;
      publish_odom(t, body);

      // ---- 6.5 纯轮式里程计轨迹累积 ----
      if (publish_path_ && (t - last_path_).seconds() >= path_period_) {
        last_path_ = t;
        geometry_msgs::msg::PoseStamped p;
        p.header.stamp = t;
        p.header.frame_id = odom_frame_;
        p.pose.position.x = x_;
        p.pose.position.y = y_;
        const double hp = yaw_ * 0.5;
        p.pose.orientation.z = std::sin(hp);
        p.pose.orientation.w = std::cos(hp);
        path_.poses.push_back(p);
        if (path_.poses.size() > 5000) {
          path_.poses.erase(path_.poses.begin());  // 上限防消息无限膨胀
        }
        path_.header.stamp = t;
        path_.header.frame_id = odom_frame_;
        pub_path_->publish(path_);
      }

      if (mode_ != "gazebo") {
        std::array<double, 4> wheel_vel;  // 物理轮速 rad/s,顺序 [LF, LB, RF, RB]
        wheel_vel[0] = kin_.wheel_velocity(fb[0], false);
        wheel_vel[1] = kin_.wheel_velocity(fb[1], false);
        wheel_vel[2] = kin_.wheel_velocity(fb[3], true);
        wheel_vel[3] = kin_.wheel_velocity(fb[2], true);
        publish_joint_state(t, wheel_vel);
      }
    }

    // ---- 7. 调试话题:[目标rpm×4, 反馈rpm×4, 电流×4] ----
    if (pub_debug_->get_subscription_count() > 0) {
      std_msgs::msg::Float32MultiArray msg;
      msg.data.reserve(12);
      for (size_t i = 0; i < 4; ++i) {
        msg.data.push_back(static_cast<float>(target_rpm[i]));
      }
      for (size_t i = 0; i < 4; ++i) {
        msg.data.push_back(fb_fresh ? static_cast<float>(fb[i]) : 0.0f);
      }
      for (size_t i = 0; i < 4; ++i) {
        msg.data.push_back(static_cast<float>(current[i]));
      }
      pub_debug_->publish(msg);
    }
  }

  void publish_joint_state(const rclcpp::Time & t, const std::array<double, 4> & wheel_vel)
  {
    sensor_msgs::msg::JointState js;
    js.header.stamp = t;
    js.name = joint_names_;
    js.velocity.resize(4);
    js.position.resize(4);
    for (size_t i = 0; i < 4; ++i) {
      js.velocity[i] = wheel_vel[i];
      joint_pos_[i] += wheel_vel[i] / control_rate_;
      js.position[i] = joint_pos_[i];
    }
    pub_joint_->publish(js);
  }

  void publish_odom(const rclcpp::Time & t, const std::array<double, 3> & body)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = t;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    const double half = yaw_ * 0.5;
    odom.pose.pose.orientation.z = std::sin(half);
    odom.pose.pose.orientation.w = std::cos(half);
    odom.twist.twist.linear.x = body[0];
    odom.twist.twist.linear.y = body[1];
    odom.twist.twist.angular.z = body[2];
    pub_odom_->publish(odom);

    if (publish_tf_ && tf_pub_ == nullptr) {
      tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", 10);
    }
    if (publish_tf_) {
      tf2_msgs::msg::TFMessage tf;
      geometry_msgs::msg::TransformStamped tr;
      tr.header.stamp = t;
      tr.header.frame_id = odom_frame_;
      tr.child_frame_id = base_frame_;
      tr.transform.translation.x = x_;
      tr.transform.translation.y = y_;
      tr.transform.rotation = odom.pose.pose.orientation;
      tf.transforms.push_back(tr);
      tf_pub_->publish(tf);
    }
  }

  // ---- 成员 ----
  std::string mode_ = "serial";
  std::string port_;
  double control_rate_ = 100.0, cmd_timeout_ = 0.5, fb_timeout_ = 0.5;
  double max_vx_ = 2.0, max_vy_ = 2.0, max_wz_ = 4.0;
  double torque_per_current_ = 0.0003;
  MecanumKinematics kin_;
  std::array<Pid, 4> pid_;
  std::vector<std::string> joint_names_;
  std::string base_frame_, odom_frame_;
  bool publish_tf_ = false;
  bool publish_path_ = true;
  double path_period_ = 0.2;
  rclcpp::Time last_path_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Path path_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_debug_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_effort_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex cmd_mutex_;
  geometry_msgs::msg::Twist cmd_;
  rclcpp::Time cmd_stamp_{0, 0, RCL_ROS_TIME};

  std::mutex fb_mutex_;
  std::array<double, 4> fb_rpm_{};
  rclcpp::Time fb_stamp_{0, 0, RCL_ROS_TIME};
  std::array<double, 4> sim_rpm_{};

  std::array<double, 4> joint_pos_{};
  double x_ = 0.0, y_ = 0.0, yaw_ = 0.0;
  rclcpp::Time last_step_{0, 0, RCL_ROS_TIME};

  SerialPort serial_;
  FeedbackParser parser_;
  std::thread reader_;
  std::atomic<bool> running_{true};
  rclcpp::Time last_reconnect_{0, 0, RCL_ROS_TIME};
  bool write_fail_logged_ = false;
};

}  // namespace chassis

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<chassis::ChassisDriver>());
  rclcpp::shutdown();
  return 0;
}
