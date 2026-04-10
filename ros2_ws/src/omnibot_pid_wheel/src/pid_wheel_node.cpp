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
    
    this->declare_parameter("wheel_radius", 0.05);
    this->declare_parameter("ticks_per_rev", 390.0);
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    ticks_per_rev_ = this->get_parameter("ticks_per_rev").as_double();

    last_ticks_.resize(NUM_WHEELS, 0);
    last_time_ = this->now();

    cmd_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/cmd_wheel", 10,
        std::bind(&PIDWheelNode::cmd_callback, this, std::placeholders::_1));

    encoder_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/encoder_ticks", 10,
        std::bind(&PIDWheelNode::encoder_callback, this, std::placeholders::_1));

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

void PIDWheelNode::timer_callback()
{
    if (!measurement_received_) {
        static int warn_count = 0;
        if (warn_count++ % 100 == 0)
            RCLCPP_WARN(this->get_logger(), "Waiting for encoder data...");
        return;
    }
    
    if (std::all_of(measurement_, measurement_ + NUM_WHEELS, [](double v) { return v == 0.0; })) {
        return;
    }

    static rclcpp::Time last_call = this->now();
    auto now = this->now();
    double dt = (now - last_call).seconds();
    last_call = now;
    if (dt < 0.001) dt = 0.01;
    if (dt > 0.1) dt = 0.01;

    std_msgs::msg::Float32MultiArray motor_msg;
    motor_msg.data.resize(NUM_WHEELS);

    for (size_t i = 0; i < NUM_WHEELS; ++i) {
        pid_controllers_[i].T = dt;
        motor_msg.data[i] = setpoint_[i] == 0 ? 0 : PIDController_Update(
            &pid_controllers_[i],
            setpoint_[i],
            measurement_[i]);
    }

    motor_pub_->publish(motor_msg);
}

void PIDWheelNode::encoder_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
    if (msg->data.size() < NUM_WHEELS) {
        RCLCPP_WARN(this->get_logger(), "Encoder message has insufficient data");
        return;
    }

    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_time_).seconds();

    if (dt < 0.0001) {
        last_time_ = current_time;
        return;
    }
    if (dt > 0.1) {
        RCLCPP_WARN(this->get_logger(), "Large dt: %.3f s, resetting", dt);
        last_time_ = current_time;
        for (size_t i = 0; i < NUM_WHEELS; ++i) {
            last_ticks_[i] = msg->data[i];
        }
        return;
    }

    std::vector<int32_t> delta_ticks(NUM_WHEELS);
    for (size_t i = 0; i < NUM_WHEELS; ++i) {
        delta_ticks[i] = msg->data[i] - last_ticks_[i];
        last_ticks_[i] = msg->data[i];
    }

    for (size_t i = 0; i < NUM_WHEELS; ++i) {
        double revs = static_cast<double>(delta_ticks[i]) / ticks_per_rev_;
        double distance = revs * 2.0 * M_PI * wheel_radius_;
        double speed = distance / dt;
        measurement_[i] = static_cast<float>(speed);
    }

    last_time_ = current_time;
    measurement_received_ = true;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PIDWheelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}