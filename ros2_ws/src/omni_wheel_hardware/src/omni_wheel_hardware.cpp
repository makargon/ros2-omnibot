#include "omni_wheel_hardware/omni_wheel_hardware.hpp"

#include <cmath>
#include <limits>

namespace omni_wheel_hardware
{

OmniWheelHardware::OmniWheelHardware()
: num_wheels_(0),
  ticks_per_rev_(0.0),
  wheel_radius_(0.0),
  max_speed_rad_s_(0.0),
  last_encoder_time_(0, 0, RCL_ROS_TIME)
{
}

OmniWheelHardware::~OmniWheelHardware()
{
}

hardware_interface::CallbackReturn OmniWheelHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Parse parameters from URDF <ros2_control> section
  num_wheels_ = info_.joints.size();
  if (num_wheels_ == 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("OmniWheelHardware"), "No joints defined!");
    return CallbackReturn::ERROR;
  }

  auto get_param = [&](const std::string & name, double & value) -> bool
  {
    auto it = info_.hardware_parameters.find(name);
    if (it == info_.hardware_parameters.end())
    {
      RCLCPP_ERROR(rclcpp::get_logger("OmniWheelHardware"), "Missing parameter: %s", name.c_str());
      return false;
    }
    value = std::stod(it->second);
    return true;
  };

  if (!get_param("ticks_per_rev", ticks_per_rev_)) return CallbackReturn::ERROR;
  if (!get_param("wheel_radius", wheel_radius_)) return CallbackReturn::ERROR;
  if (!get_param("max_speed_rad_s", max_speed_rad_s_)) return CallbackReturn::ERROR;

  // Allocate storage
  cmd_velocities_.resize(num_wheels_, 0.0);
  state_velocities_.resize(num_wheels_, 0.0);
  prev_ticks_.resize(num_wheels_, 0);
  encoder_ticks_buffer_.resize(num_wheels_, 0);
  latest_encoder_ticks_.writeFromNonRT(encoder_ticks_buffer_);

  RCLCPP_INFO(rclcpp::get_logger("OmniWheelHardware"), "Initialized with %zu wheels", num_wheels_);
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OmniWheelHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  node_ = std::make_shared<rclcpp::Node>("omni_wheel_hardware_node");

  // Publisher for motor speeds
  motor_speed_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
    "/motor_speeds", rclcpp::SystemDefaultsQoS());

  // Subscriber for encoder ticks
  encoder_sub_ = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
    "/encoder_ticks", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
      encoder_callback(msg);
    });

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OmniWheelHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset state
  for (size_t i = 0; i < num_wheels_; ++i)
  {
    cmd_velocities_[i] = 0.0;
    state_velocities_[i] = 0.0;
    prev_ticks_[i] = 0;
  }
  last_encoder_time_ = node_->now();
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OmniWheelHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop all motors
  std_msgs::msg::Float32MultiArray stop_msg;
  stop_msg.data.assign(num_wheels_, 0.0f);
  motor_speed_pub_->publish(stop_msg);
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> OmniWheelHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < num_wheels_; ++i)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &state_velocities_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> OmniWheelHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < num_wheels_; ++i)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &cmd_velocities_[i]);
  }
  return command_interfaces;
}

hardware_interface::return_type OmniWheelHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Get latest encoder ticks (non-blocking, realtime safe)
  std::vector<int> ticks;
  if (!latest_encoder_ticks_.readFromRT(ticks))
  {
    // No new data yet
    return hardware_interface::return_type::OK;
  }

  double dt = (time - last_encoder_time_).seconds();
  if (dt < 1e-6)
  {
    return hardware_interface::return_type::OK;
  }

  // Convert ticks to angular velocity (rad/s)
  double rad_per_tick = 2.0 * M_PI / ticks_per_rev_;
  for (size_t i = 0; i < num_wheels_; ++i)
  {
    double delta_rad = (ticks[i] - prev_ticks_[i]) * rad_per_tick;
    state_velocities_[i] = delta_rad / dt;
    prev_ticks_[i] = ticks[i];
  }
  last_encoder_time_ = time;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OmniWheelHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Convert commanded velocities (rad/s) to normalized [-1, 1] values
  std_msgs::msg::Float32MultiArray msg;
  msg.data.resize(num_wheels_);
  for (size_t i = 0; i < num_wheels_; ++i)
  {
    double norm = cmd_velocities_[i] / max_speed_rad_s_;
    msg.data[i] = static_cast<float>(std::clamp(norm, -1.0, 1.0));
  }
  motor_speed_pub_->publish(msg);
  return hardware_interface::return_type::OK;
}

void OmniWheelHardware::encoder_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  if (msg->data.size() != num_wheels_)
  {
    RCLCPP_WARN(node_->get_logger(), "Encoder ticks size mismatch: expected %zu, got %zu",
                num_wheels_, msg->data.size());
    return;
  }
  std::vector<int> ticks(msg->data.begin(), msg->data.end());
  latest_encoder_ticks_.writeFromNonRT(ticks);
}

}  // namespace omni_wheel_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(omni_wheel_hardware::OmniWheelHardware, hardware_interface::SystemInterface)