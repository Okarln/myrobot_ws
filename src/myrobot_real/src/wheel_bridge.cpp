#include "rclcpp/rclcpp.hpp"
#include <string>
#include <termios.h>

class WheelBridge: public rclcpp::Node{
    public:
    WheelBridge(): rclcpp::Node("Wheel_Bridge"){
        const std::string port = rclcpp::Node::declare_parameter("port","/dev/ttyUSB0");
        const auto baud = rclcpp::Node::declare_parameter<int>("baud", get_baud(115200)); 
        auto fd_ = open_serial(port,baud);

        auto pub_state_ = create_publisher<std_msgs::msg::Float32MultiArray>("wheel_state", 10); 
        auto pub_ext_ = create_publisher<>("wheel_ext",10);
        auto pub_link_ = create_publisher<>("link_status",10);

        parser_.on_frame_ = [this](auto type, auto seq, auto * p, size_t n) {on_frame(type, seq, p, n);};

        sub_speed_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "wheel_speed_cmd", 10,
            [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr m) { on_speed_cmd(m); });
        sub_current_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "wheel_current_cmd", 10,
            [this](std_msgs::msg::Float32MultiArray::ConstSharedPtr m) { on_current_cmd(m); });
        sub_estop_ = create_subscription<std_msgs::msg::Empty>(
            "wheel_estop", 10,
            [this](std_msgs::msg::Empty::ConstSharedPtr m) { on_estop(m); });
        
        keepalive_timer_ = create_wall_timer(100ms, [this]{ tick_keepalive(); });
        stats_timer_    = create_wall_timer(1s,    [this]{ tick_stats(); });
        
        rx_thread_ = std::thread([this]{ rx_loop(); });
        RCLCPP_INFO(get_logger(), "已打开 %s @ %d", port.c_str(), baud);
    
    }
}



int open_serial(const std::string & port,const int baud)
{
    // ── 第一步:打开设备文件 ──────────────────────────────
    int fd = open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0)
        throw std::runtime_error("open " + port + " 失败: " + std::strerror(errno));

    // ── 第二步:读出当前配置,在其上修改 ──────────────────
    termios tio{};
    if (tcgetattr(fd, &tio) != 0) {
        close(fd);      // 失败路径手动关 fd(构造函数抛异常不会走析构)
        throw std::runtime_error("tcgetattr 失败");
    }

    // ── 第三步:配成 raw 模式(关键)────────────────────
    cfmakeraw(&tio);

    // ── 第四步:波特率 + 8N1 + 流控 ──────────────────────
    cfsetispeed(&tio, baud);          // 输入波特率
    cfsetospeed(&tio, baud);          // 输出波特率
    tio.c_cflag |=  CS8 | CLOCAL | CREAD;
    tio.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);

    // ── 第五步:读超时语义 ───────────────────────────────
    tio.c_cc[VMIN]  = 0;                 // read 不要求最少读到几个字节
    tio.c_cc[VTIME] = 1;                 // 最多等 1×0.1s 就返回

    // ── 第六步:生效 + 清空旧缓冲 ────────────────────────
    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        close(fd);
        throw std::runtime_error("tcsetattr 失败");
    }
    tcflush(fd, TCIOFLUSH);              // 丢弃打开前积压的旧数据
    return fd;
}



speed_t get_baud(int baud)
{
    switch (baud) {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    case 460800:
        return B460800;
    case 500000:
        return B500000;
    case 576000:
        return B576000;
    case 921600:
        return B921600;
    case 1000000:
        return B1000000;
    case 1152000:
        return B1152000;
    case 1500000:
        return B1500000;
    case 2000000:
        return B2000000;
    case 2500000:
        return B2500000;
    case 3000000:
        return B3000000;
    case 3500000:
        return B3500000;
    case 4000000:
        return B4000000;
    default: 
        return -1;
    }
}