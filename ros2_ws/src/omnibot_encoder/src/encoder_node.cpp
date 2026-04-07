#include "omnibot_encoder/encoder_node.hpp"

#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <cstring>

// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"

// #include <lgpio.h>
// #include <atomic>
// #include <cstdint>
// #include <cmath>

// using namespace std::chrono_literals;

std::atomic<bool> running{true};

void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\nCtrl+C pressed. Stopping..." << std::endl;
        running = false;
    }
}

RotaryEncoder::RotaryEncoder(int handle, int pin_a, int pin_b, int cpr)
    : handle_(handle), pin_a_(pin_a), pin_b_(pin_b), cpr_(cpr)
{
    lgGpioClaimAlert(handle_, 0, LG_BOTH_EDGES, pin_a_, -1);
    lgGpioClaimAlert(handle_, 0, LG_BOTH_EDGES, pin_b_, -1);

    int a = lgGpioRead(handle_, pin_a_);
    int b = lgGpioRead(handle_, pin_b_);
    prev_ab_ = (a << 1) | b;

    lgGpioSetAlertsFunc(handle_, pin_a_, alert_cb, this);
    lgGpioSetAlertsFunc(handle_, pin_b_, alert_cb, this);
}

RotaryEncoder::~RotaryEncoder() {
    lgGpioSetAlertsFunc(handle_, pin_a_, nullptr, nullptr);
    lgGpioSetAlertsFunc(handle_, pin_b_, nullptr, nullptr);
    lgGpioFree(handle_, pin_a_);
    lgGpioFree(handle_, pin_b_);
}

float RotaryEncoder::angle_deg() const {
    return static_cast<float>(count()) * 360.0f / static_cast<float>(cpr_);
}

int64_t RotaryEncoder::count() const {
    return count_.load(std::memory_order_relaxed);
}

void RotaryEncoder::reset() {
    count_.store(0, std::memory_order_relaxed);
}

void RotaryEncoder::alert_cb(int num_alerts, lgGpioAlert_p alerts, void *userdata) {
    auto* self = static_cast<RotaryEncoder*>(userdata);
    if (!self) {
        return;
    }

    for (int i = 0; i < num_alerts; ++i) {
        int gpio  = alerts[i].report.gpio;
        int level = alerts[i].report.level;
        if (gpio == self->pin_a_ || gpio == self->pin_b_) {
            self->on_edge(gpio, level);
        }
    }
}

void RotaryEncoder::on_edge(int gpio, int level) {
    int a = (gpio == pin_a_) ? level : lgGpioRead(handle_, pin_a_);
    int b = (gpio == pin_b_) ? level : lgGpioRead(handle_, pin_b_);
    int curr_ab = (a << 1) | b;

    int8_t step = QEM[(prev_ab_ << 2) | curr_ab];
    count_.fetch_add(step, std::memory_order_relaxed);
    prev_ab_ = curr_ab;
}

// class RotaryEncoder {
// public:
//     RotaryEncoder(int handle, int pin_a, int pin_b, int cpr)
//     : handle_(handle), pin_a_(pin_a), pin_b_(pin_b), cpr_(cpr), count_(0), last_a_(0), last_b_(0)
//     {
//         lgGpioClaimAlert(handle_, 0, LG_BOTH_EDGES, pin_a_, -1);
//         lgGpioClaimAlert(handle_, 0, LG_BOTH_EDGES, pin_b_, -1);

//         int a = lgGpioRead(handle_, pin_a_);
//         int b = lgGpioRead(handle_, pin_b_);
//         prev_ab_ = (a << 1) | b;

//         lgGpioSetAlertsFunc(handle_, pin_a_, alert_cb, this);
//         lgGpioSetAlertsFunc(handle_, pin_b_, alert_cb, this);
//     }

//     ~RotaryEncoder() {
//         lgGpioSetAlertsFunc(handle_, pin_a_, nullptr, nullptr);
//         lgGpioSetAlertsFunc(handle_, pin_b_, nullptr, nullptr);
//         lgGpioFree(handle_, pin_a_);
//         lgGpioFree(handle_, pin_b_);
//     }

//     float angle_deg() const
//     {
//         return static_cast<float>(count()) * 360.0f / static_cast<float>(cpr_);
//     }

//     int64_t count() const
//     {
//         return count_.load(std::memory_order_relaxed);
//     }

//     void reset()
//     {
//         count_.store(0, std::memory_order_relaxed);
//     }

// private:
//     void alert_cb(int num_alerts, lgGpioAlert_p alerts, void *userdata) {
//         auto* self = static_cast<RotaryEncoder*>(userdata);
//         if (!self) {
//             return;
//         }

//         for (int i = 0; i < num_alerts; ++i) {
//             int gpio  = alerts[i].report.gpio;
//             int level = alerts[i].report.level;
//             if (gpio == self->pin_a_ || gpio == self->pin_b_) {
//                 self->on_edge(gpio, level);
//             }
//         }
//     }

//     void on_edge(int gpio, int level) {
//         int a = (gpio == pin_a_) ? level : lgGpioRead(handle_, pin_a_);
//         int b = (gpio == pin_b_) ? level : lgGpioRead(handle_, pin_b_);
//         int curr_ab = (a << 1) | b;

//         int8_t step = QEM[(prev_ab_ << 2) | curr_ab];
//         count_.fetch_add(step, std::memory_order_relaxed);
//         prev_ab_ = curr_ab;
//     }

//     int handle_;
//     int pin_a_;
//     int pin_b_;
//     int cpr_;

//     std::atomic<int64_t> count_{0};
//     int prev_ab_{0};
// };

// // class EncoderNode : public rclcpp::Node {
// // public:
// //     EncoderNode()
// //     : Node("encoder") {
// //         handle_ = lgGpiochipOpen(4)


// //         encoders_.push_back(std::make_unique<EncoderReader>(e1a, e1b));
// //         encoders_.push_back(std::make_unique<EncoderReader>(e2a, e2b));
// //         encoders_.push_back(std::make_unique<EncoderReader>(e3a, e3b));

// //         publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/encoder_ticks", 10);

// //         timer_ = this->create_wall_timer(std::chrono::milliseconds(20),
// //                                          std::bind(&EncoderNode::timerCallback, this));

// //         RCLCPP_INFO(this->get_logger(), "Encoder Node запущена. Публикуем в /encoder_ticks");
// //     }

// //     ~EncoderNode() {
// //         lgGpiochipClose(4);
// //     }

// // private:
// //     void timerCallback() {
// //         auto msg = std_msgs::msg::Int32MultiArray();
// //         msg.data.resize(3);
// //         for (size_t i = 0; i < encoders_.size(); ++i) {
// //             msg.data[i] = encoders_[i]->read();
// //         }
// //         publisher_->publish(msg);
// //     }

// //     std::vector<std::unique_ptr<EncoderReader>> encoders_;
// //     rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
// //     rclcpp::TimerBase::SharedPtr timer_;
// // };

// // int main(int argc, char** argv) {
// //     rclcpp::init(argc, argv);
// //     auto node = std::make_shared<EncoderNode>();
// //     rclcpp::spin(node);
// //     rclcpp::shutdown();
// //     return 0;
// // }

int main() {
    std::signal(SIGINT, signal_handler);

    int handle = lgGpiochipOpen(4);

    const int PIN_A = 10;
    const int PIN_B = 9;
    const int CPR   = 390;

    try {
        RotaryEncoder encoder(handle, PIN_A, PIN_B, CPR);
        std::cout << "=== Rotary Encoder Test (real hardware) ===" << std::endl;
        std::cout << "GPIO_A=" << PIN_A << " GPIO_B=" << PIN_B << " CPR=" << CPR << std::endl;
        std::cout << "Rotate the encoder. Press Ctrl+C to stop.\n" << std::endl;

        while (running) {
            int64_t cnt = encoder.count();
            float angle = encoder.angle_deg();
            std::cout << "\rCount: " << cnt << "   Angle: " << angle << " deg   " << std::flush;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        std::cout << std::endl << "Test finished." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        lgGpiochipClose(handle);
        return 1;
    }

    lgGpiochipClose(handle);
}