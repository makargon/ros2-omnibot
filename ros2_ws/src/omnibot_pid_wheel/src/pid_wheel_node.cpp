#include "omnibot_pid_wheel/pid_wheel_node.hpp"
#include "rclcpp/rclcpp.hpp"

PIDWheelNode::PIDWheelNode() : Node("pid_wheel_node"),
    setpoint_(NUM_WHEELS, 0.0f),
    measurement_(NUM_WHEELS, 0.0f),
    setpoint_received_(false),
    measurement_received_(false)
{
    pid_controllers_.resize(NUM_WHEELS);
    for (auto &pid : pid_controllers_) {
        PIDController_Init(&pid);
        pid.Kp = 1.0f;
        pid.Ki = 0.1f;
        pid.Kd = 0.01f;
        pid.tau = 0.05f;
        pid.limMin = -100.0f;
        pid.limMax = 100.0f;
        pid.limMinInt = -50.0f;
        pid.limMaxInt = 50.0f;
        pid.T = 0.05f;
    }

    cmd_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/cmd_wheel", 10,
        std::bind(&PIDWheelNode::cmd_callback, this, std::placeholders::_1));

    speed_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/wheel_speeds", 10,
        std::bind(&PIDWheelNode::speed_callback, this, std::placeholders::_1));

    motor_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/motor_speeds", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&PIDWheelNode::timer_callback, this));
}

PIDWheelNode::~PIDWheelNode() {}

void PIDWheelNode::cmd_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() >= NUM_WHEELS) {
        std::copy_n(msg->data.begin(), NUM_WHEELS, setpoint_.begin());
        setpoint_received_ = true;
    } else {
        RCLCPP_WARN(
            this->get_logger(),
            "cmd_wheel message has insufficient data (%zu < %zu)",
            msg->data.size(), NUM_WHEELS);
    }
}

void PIDWheelNode::speed_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() >= NUM_WHEELS) {
        std::copy_n(msg->data.begin(), NUM_WHEELS, measurement_.begin());
        measurement_received_ = true;
    } else {
        RCLCPP_WARN(
            this->get_logger(),
            "wheel_speed message has insufficient data (%zu < %zu)",
            msg->data.size(), NUM_WHEELS);
    }
}

void PIDWheelNode::timer_callback()
{
    std_msgs::msg::Float32MultiArray motor_msg;
    motor_msg.data.resize(NUM_WHEELS);

    for (size_t i = 0; i < NUM_WHEELS; ++i) {
        float output = PIDController_Update(
            &pid_controllers_[i],
            setpoint_[i],
            measurement_[i]);
        motor_msg.data[i] = output;
    }

    motor_pub_->publish(motor_msg);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PIDWheelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}