#ifndef OMNI_WHEEL_HARDWARE__OMNI_WHEEL_HARDWARE_HPP_
#define OMNI_WHEEL_HARDWARE__OMNI_WHEEL_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "realtime_tools/realtime_buffer.h"

namespace omni_wheel_hardware
{

class OmniWheelHardware : public hardware_interface::SystemInterface
{
public:
  OmniWheelHardware();
  virtual ~OmniWheelHardware();

  // ROS2 Control mandatory methods
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ROS node handle for subscribing/publishing
  rclcpp::Node::SharedPtr node_;

  // Subscriber for encoder ticks
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_sub_;
  void encoder_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

  // Publisher for motor speeds
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_speed_pub_;

  // Parameters from URDF
  size_t num_wheels_;
  double ticks_per_rev_;        // encoder ticks per revolution
  double wheel_radius_;         // in meters
  double max_speed_rad_s_;      // maximum speed in rad/s corresponding to normalized 1.0

  // Command and state storage (thread-safe for realtime)
  std::vector<double> cmd_velocities_;      // rad/s
  std::vector<double> state_velocities_;    // rad/s

  // Buffer for latest encoder ticks
  realtime_tools::RealtimeBuffer<std::vector<int>> latest_encoder_ticks_;
  std::vector<int> encoder_ticks_buffer_;

  // Previous ticks for velocity calculation
  std::vector<int> prev_ticks_;
  rclcpp::Time last_encoder_time_;
};

}  // namespace omni_wheel_hardware

#endif  // OMNI_WHEEL_HARDWARE__OMNI_WHEEL_HARDWARE_HPP_