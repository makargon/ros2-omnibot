#include "omnibot_encoder/odometry_node.hpp"

#include <cmath>

OdometryNode::OdometryNode()
    : Node("odometry_node"),
    first_measurement_(true)
{
    this->declare_parameter<double>("robot_radius", 0.2);
    this->declare_parameter<double>("wheel_radius", 0.05);
    this->declare_parameter<double>("wheel_offset", 0.0);
    this->declare_parameter<int>("wheel_count", 3);
    
    params_.wheel_radius = this->get_parameter("wheel_radius").as_double();
    params_.robot_radius = this->get_parameter("robot_radius").as_double();
    params_.wheel_offset = this->get_parameter("wheel_offset").as_double();
    params_.wheel_count  = this->get_parameter("wheel_count").as_int();
    params_.ticks_per_rev = 390;

    odometry_.setParams(robot_radius_, wheel_radius_, wheel_offset_, wheel_count_);
    odometry_.setOdometry(0.0, 0.0, 0.0);

    last_ticks_.resize(params_.wheel_count, 0);
    wheel_angles_rad_.resize(params_.wheel_count, 0.0);

    encoder_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/encoder_ticks", 10,
        std::bind(&OdometryNode::encoder_callback, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    odom_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&OdometryNode::publish_odom_and_tf, this));

    RCLCPP_INFO(
        this->get_logger(),
        "Odometry node started");
    RCLCPP_INFO(
        this->get_logger(),
        "Params: wheel_radius=%.3f, robot_radius=%.3f, ticks_per_rev=%.0f",
        params_.wheel_radius, params_.robot_radius, params_.ticks_per_rev);
}

void OdometryNode::encoder_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
    rclcpp::Time current_time = this->now();

    if (first_measurement_) {
        last_time_ = current_time;
        for (size_t i = 0; i < msg->data.size(); ++i) {
            last_ticks_[i] = msg->data[i];
            wheel_angles_rad_[i] = 0.0;
        }
        odometry_.updateFromPos(wheel_angles_rad_, current_time);
        first_measurement_ = false;
        return;
    }

    double dt = (current_time - last_time_).seconds();
    if (dt < 0.0001) {
        last_time_ = current_time;
        return;
    }
    if (dt > 0.1) {
        RCLCPP_WARN(this->get_logger(), "Large dt detected: %.3f s, resetting", dt);
        last_time_ = current_time;
        for (size_t i = 0; i < 3 && i < msg->data.size(); ++i) {
            last_ticks_[i] = msg->data[i];
        }
        return;
    }

    for (size_t i = 0; i < msg->data.size(); ++i) {
        int32_t delta_ticks = msg->data[i] - last_ticks_[i];
        last_ticks_[i] = msg->data[i];
        
        double delta_angle_rad = (static_cast<double>(delta_ticks) / params_.ticks_per_rev) * 2.0 * M_PI;
        wheel_angles_rad_[i] += delta_angle_rad;
    }

    if (!odometry_.updateFromPos(wheel_angles_rad_, current_time)) {
        RCLCPP_WARN(this->get_logger(), "Odometry update failed");
        return;
    }

    last_time_ = current_time;

    // if (first_measurement_) {
    //     last_time_ = current_time;
    //     odometry_.updateFromPos(wheels_rad, current_time);
    //     first_measurement_ = false;
    //     return;
    // }

    // if (!odometry_.updateFromPos(wheels_rad, current_time)) {
    //     RCLCPP_WARN(this->get_logger(), "Odometry update failed");
    //     return;
    // }

    // publish_odom_and_tf();
    // last_time_ = current_time;
}


void OdometryNode::publish_odom_and_tf()
{
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_footprint";

    odom_msg.pose.pose.position.x = odometry_.getX();
    odom_msg.pose.pose.position.y = odometry_.getY();
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, odometry_.getHeading());
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.pose.covariance.fill(0.0);
    odom_msg.pose.covariance[0] = 0.01;
    odom_msg.pose.covariance[7] = 0.01;
    odom_msg.pose.covariance[35] = 0.01;

    odom_msg.twist.twist.linear.x = odometry_.getLinearXVel();
    odom_msg.twist.twist.linear.y = odometry_.getLinearYVel();
    odom_msg.twist.twist.angular.z = odometry_.getAngularVel();

    odom_msg.twist.covariance.fill(0.0);
    odom_msg.twist.covariance[0] = 0.01;
    odom_msg.twist.covariance[7] = 0.01;
    odom_msg.twist.covariance[35] = 0.01;

    odom_pub_->publish(odom_msg);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->now();
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_footprint";

    tf_msg.transform.translation.x = odometry_.getX();
    tf_msg.transform.translation.y = odometry_.getY();
    tf_msg.transform.translation.z = 0.0;

    tf_msg.transform.rotation = odom_msg.pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf_msg);
}



int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}