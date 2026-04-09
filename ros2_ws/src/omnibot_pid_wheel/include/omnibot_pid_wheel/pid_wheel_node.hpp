#ifndef PID_WHEEL_NODE_HPP_
#define PID_WHEEL_NODE_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"   // добавлено для энкодеров

#include "omnibot_pid_wheel/pid_controller.hpp"

class PIDWheelNode : public rclcpp::Node {
public:
    PIDWheelNode();
    ~PIDWheelNode();

private:
    static constexpr size_t NUM_WHEELS = 3;

    // PID-контроллеры для каждого колеса
    std::vector<PIDController> pid_controllers_;

    // Целевые и измеренные скорости (м/с)
    std::vector<float> setpoint_;
    std::vector<float> measurement_;

    // Флаги получения данных
    bool setpoint_received_;
    bool measurement_received_;

    double wheel_radius_;
    double ticks_per_rev_;

    std::vector<int32_t> last_ticks_;
    rclcpp::Time last_time_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void cmd_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void encoder_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    void timer_callback();
};

#endif