#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

namespace wheel {
    constexpr uint8_t SOF         = 0xAA;   // 帧头
    constexpr uint8_t FRAME_EOF   = 0x55;   // 帧尾(不能叫 EOF,那是 <cstdio> 的宏)
    constexpr uint8_t MAX_PAYLOAD = 32;     // 载荷长度上限

    class FrameParser{
        public:
            using FrameCallback = std::function<void(uint8_t type, uint8_t seq,const uint8_t * payload, size_t len)>; 
            FrameCallback on_frame_;

            void feed(const uint8_t * data,size_t n){
                for (size_t i = 0;i < n; ++i)
                    feed_one(data[i]);
            }


        private:
            std::vector<uint8_t> buf_;                     // bytearray → vector<uint8_t>
            std::chrono::steady_clock::time_point last_byte_ts_;

            void feed_one(uint8_t b){
                const auto now = std::chrono::steady_clock::now();
                if (!buf_.empty() &&
                    now - last_byte_ts_ > std::chrono::milliseconds(5)) {
                        buf_.clear();
                    }
                last_byte_ts_ = now;
                if (buf_.empty()) {
                    if (b == SOF) buf_.push_back(b);
                    return;
                }

                buf_.push_back(b);

                if (buf_.size() == 2 && buf_[1] > MAX_PAYLOAD){
                    resync();
                    return;
                }

                if (buf_.size() >= buf_[1] + 6){
                    const size_t ln = buf_[1];
                    const size_t sum = checksum(1 , 4 + ln);
                    if (buf_[ln + 4] == sum && buf_[5 + ln] == FRAME_EOF){
                        if (on_frame_)
                            on_frame_(buf_[2],buf_[3],buf_.data() + 4,ln);
                    }
                    buf_.clear();
                }
            }


            void resync(){
                size_t i = 1;
                while (i < buf_.size() && buf_[i] != SOF){
                    ++i;
                }
                buf_.erase(buf_.begin(),buf_.begin() + i);
            }

            uint8_t checksum(size_t from ,size_t to){
                uint8_t sum = 0;
                for (size_t i = from;i < to; ++i){
                    sum = static_cast<uint8_t>(sum + buf_[i]);
                }
                return sum;
            }

        };


    }

class WheelBridge : public rclcpp::Node {
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_state_, pub_ext_, pub_link_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_speed_, sub_current_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_estop_;
    rclcpp::TimerBase::SharedPtr keepalive_timer_, stats_timer_;

    int fd_{-1};
    std::thread rx_thread_;
    std::mutex tx_lock_;
    wheel::FrameParser parser_;

    uint8_t tx_seq_{0};
    std::mutex keepalive_lock_;
    std::optional<stdLLpair<uint8_t,std::vector<uint8_t>>> keepalive_;
    std::unordered_map<uint32_t,std::chrono::steady_clock::time_point> ping_times_;
    std::mutex ping_lock_;
    std::atomic<uint32_t> cnt_state_{0}, cnt_ext_{0}, cnt_pong_{0}, cnt_link_{0};
    std::atomic<uint32_t> rx_err_{0}, rx_lost_{0};
    std::array<float,8> last_state_{};
    std::atomic<int> dev_mode_{0};
    double last_rtt_ms_{std::numeric_limits<double>::quiet_NaN()};
};