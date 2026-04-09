#ifndef PID_WHEEL_NODE_HPP_
#define PID_WHEEL_NODE_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "omnibot_pid_wheel/pid_controller.hpp"

class PIDWheelNode : public rclcpp::Node {
public:
    PIDWheelNode();
    ~PIDWheelNode();

private:
    static constexpr size_t NUM_WHEELS = 3;

    std::vector<PIDController> pid_controllers_;

    std::vector<float> setpoint_;
    std::vector<float> measurement_;

    bool setpoint_received_;
    bool measurement_received_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr speed_sub_;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    void cmd_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void speed_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void timer_callback();
};

#endif