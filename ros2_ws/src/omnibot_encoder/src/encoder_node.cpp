#include "omnibot_encoder/encoder_node.hpp"

#include <chrono>
#include <iostream>
#include <memory>
#include <vector>

using namespace std::chrono_literals;

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
    if (!self) return;

    for (int i = 0; i < num_alerts; ++i) {
        int gpio  = alerts[i].report.gpio;
        int level = alerts[i].report.level;
        if (gpio == self->pin_a_ || gpio == self->pin_b_)
            self->on_edge(gpio, level);
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

EncoderNode::EncoderNode()
    : Node("encoder_node")
{
    gpio_handle_ = lgGpiochipOpen(4);
    if (gpio_handle_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open GPIO chip 4");
        throw std::runtime_error("Cannot open GPIO chip");
    }

    this->declare_parameter("encoder1.pin_a", 10);
    this->declare_parameter("encoder1.pin_b", 9);
    this->declare_parameter("encoder1.cpr", 390);

    this->declare_parameter("encoder2.pin_a", 13);
    this->declare_parameter("encoder2.pin_b", 19);
    this->declare_parameter("encoder2.cpr", 390);

    this->declare_parameter("encoder3.pin_a", 20);
    this->declare_parameter("encoder3.pin_b", 21);
    this->declare_parameter("encoder3.cpr", 390);

    int e1a = this->get_parameter("encoder1.pin_a").as_int();
    int e1b = this->get_parameter("encoder1.pin_b").as_int();
    int e1cpr = this->get_parameter("encoder1.cpr").as_int();

    int e2a = this->get_parameter("encoder2.pin_a").as_int();
    int e2b = this->get_parameter("encoder2.pin_b").as_int();
    int e2cpr = this->get_parameter("encoder2.cpr").as_int();

    int e3a = this->get_parameter("encoder3.pin_a").as_int();
    int e3b = this->get_parameter("encoder3.pin_b").as_int();
    int e3cpr = this->get_parameter("encoder3.cpr").as_int();

    encoders_.push_back(std::make_unique<RotaryEncoder>(gpio_handle_, e1a, e1b, e1cpr));
    encoders_.push_back(std::make_unique<RotaryEncoder>(gpio_handle_, e2a, e2b, e2cpr));
    encoders_.push_back(std::make_unique<RotaryEncoder>(gpio_handle_, e3a, e3b, e3cpr));

    publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/encoder_ticks", 10);
    timer_ = this->create_wall_timer(20ms, std::bind(&EncoderNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Encoder node started. Publishing to /encoder_ticks");
}

EncoderNode::~EncoderNode() {
    if (gpio_handle_ >= 0) {
        lgGpiochipClose(gpio_handle_);
    }
}

void EncoderNode::timer_callback() {
    auto msg = std_msgs::msg::Int32MultiArray();
    msg.data.resize(encoders_.size());
    for (size_t i = 0; i < encoders_.size(); ++i) {
        msg.data[i] = static_cast<int32_t>(encoders_[i]->count());
    }
    publisher_->publish(msg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EncoderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// TEST


// int main() {
//     std::signal(SIGINT, signal_handler);

//     int handle = lgGpiochipOpen(4);

//     const int PIN_A = 10;
//     const int PIN_B = 9;
//     const int CPR   = 390;

//     try {
//         RotaryEncoder encoder(handle, PIN_A, PIN_B, CPR);
//         std::cout << "=== Rotary Encoder Test (real hardware) ===" << std::endl;
//         std::cout << "GPIO_A=" << PIN_A << " GPIO_B=" << PIN_B << " CPR=" << CPR << std::endl;
//         std::cout << "Rotate the encoder. Press Ctrl+C to stop.\n" << std::endl;

//         while (running) {
//             int64_t cnt = encoder.count();
//             float angle = encoder.angle_deg();
//             std::cout << "\rCount: " << cnt << "   Angle: " << angle << " deg   " << std::flush;
//             std::this_thread::sleep_for(std::chrono::milliseconds(50));
//         }
//         std::cout << std::endl << "Test finished." << std::endl;
//     } catch (const std::exception& e) {
//         std::cerr << "Error: " << e.what() << std::endl;
//         lgGpiochipClose(handle);
//         return 1;
//     }

//     lgGpiochipClose(handle);
// }