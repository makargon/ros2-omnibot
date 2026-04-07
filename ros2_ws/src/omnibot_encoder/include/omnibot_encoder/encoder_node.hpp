#ifndef OMNIBOT_ENCODER__ENCODER_NODE_HPP_
#define OMNIBOT_ENCODER__ENCODER_NODE_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <vector>
#include <csignal>
#include <cstdint>

#include <lgpio.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

// Quadrature decode lookup table (4x).
// Index = (prev_ab << 2) | curr_ab  where ab = (A<<1)|B
// +1 = forward, -1 = reverse, 0 = no change or error
static const int8_t QEM[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0,
};

class RotaryEncoder {
public:
    RotaryEncoder(int handle, int pin_a, int pin_b, int cpr);
    ~RotaryEncoder();

    float angle_deg() const;
    int64_t count() const;
    void reset();

private:
    static void alert_cb(int num_alerts, lgGpioAlert_p alerts, void *userdata);
    void on_edge(int gpio, int level);

    int handle_;
    int pin_a_;
    int pin_b_;
    int cpr_;

    std::atomic<int64_t> count_{0};
    int prev_ab_{0};
};


class EncoderNode : public rclcpp::Node {
public:
    EncoderNode();
    ~EncoderNode();

private:
    void timer_callback();
    void init_encoders();

    int gpio_handle_;
    
    std::unique_ptr<RotaryEncoder> encoder1_;
    std::unique_ptr<RotaryEncoder> encoder2_;
    std::unique_ptr<RotaryEncoder> encoder3_;
    
    struct EncoderPins {
        int pin_a;
        int pin_b;
        int cpr;
    };
    
    std::vector<EncoderPins> encoder_configs_;
    
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// extern std::atomic<bool> running;
// void signal_handler(int signal);

#endif