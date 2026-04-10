#include "omnibot_encoder/odometry_node.hpp"
#include <cmath>
#include <numeric>
#include <algorithm>

OdometryNode::OdometryNode()
    : Node("odometry_node"),
      x_(0.0), y_(0.0), theta_(0.0),
      vx_(0.0), vy_(0.0), vtheta_(0.0),
      wheel_velocities_(3, 0.0)
{
    // this->declare_parameter("wheel_radius", 0.05);
    // this->declare_parameter("robot_radius", 0.15);
    // this->declare_parameter("ticks_per_rev", 390.0);
    
    params_.wheel_radius = 0.05; //this->get_parameter("wheel_radius").as_double();
    params_.robot_radius = 0.15; //this->get_parameter("robot_radius").as_double();
    params_.ticks_per_rev = 390; //this->get_parameter("ticks_per_rev").as_double();

    encoder_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/encoder_ticks", 10,
        std::bind(&OdometryNode::encoder_callback, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    
    // возможно не нужно
    odom_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&OdometryNode::publish_odom, this));
    
    last_time_ = this->now();
    last_ticks_.resize(3, 0);

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

    std::vector<int32_t> delta_ticks(3);
    for (size_t i = 0; i < 3; ++i) {
        delta_ticks[i] = msg->data[i] - last_ticks_[i];
        last_ticks_[i] = msg->data[i];
    }
    
    for (size_t i = 0; i < 3; ++i) {
        double revs = static_cast<double>(delta_ticks[i]) / params_.ticks_per_rev;
        double wheel_distance = revs * 2.0 * M_PI * params_.wheel_radius;
        wheel_velocities_[i] = wheel_distance / dt;
    }

<<<<<<< HEAD
    vx_ = (2.0 * wheel_velocities_[0] - wheel_velocities_[1] - wheel_velocities_[2]) / 3.0;
    vy_ = (wheel_velocities_[1] - wheel_velocities_[2]) / std::sqrt(3.0);
    vtheta_ = (wheel_velocities_[0] + wheel_velocities_[1] + wheel_velocities_[2]) / (3.0 * params_.robot_radius);
=======
  std::vector<int32_t> delta_ticks(3);
  for (size_t i = 0; i < 3; ++i) {
    delta_ticks[i] = msg->data[i] - last_ticks_[i];
    last_ticks_[i] = msg->data[i];
  }
  
  std::vector<double> wheels_vel(3);
  for (size_t i = 0; i < 3; ++i) {
    double revs = static_cast<double>(delta_ticks[i]) / ticks_per_rev_; // пропорция x/max_x
    // wheels_vel[i] = revs * 2.0 * M_PI * r / dt; // rad/s ---
    wheels_vel[i] = revs * 2.0 * M_PI / dt; // rad/s ---
  }
>>>>>>> bb97fa4 (node kinema test)

    vx_filter_.push_back(vx_);
    vy_filter_.push_back(vy_);
    vtheta_filter_.push_back(vtheta_);
    
    while (vx_filter_.size() > FILTER_WINDOW) vx_filter_.pop_front();
    while (vy_filter_.size() > FILTER_WINDOW) vy_filter_.pop_front();
    while (vtheta_filter_.size() > FILTER_WINDOW) vtheta_filter_.pop_front();
    
    double vx_filt = filter_average(vx_filter_);
    double vy_filt = filter_average(vy_filter_);
    double vtheta_filt = filter_average(vtheta_filter_);

    if (std::abs(vtheta_filt) > 1e-6) {
        double delta_theta = vtheta_filt * dt;
        double R_theta = vtheta_filt;
        double V_robot = std::hypot(vx_filt, vy_filt);
        double angle_robot = std::atan2(vy_filt, vx_filt);
        
        double radius = V_robot / std::abs(R_theta);
        double delta_x = radius * (std::sin(theta_ + delta_theta) - std::sin(theta_));
        double delta_y = -radius * (std::cos(theta_ + delta_theta) - std::cos(theta_));
        
        x_ += delta_x;
        y_ += delta_y;
        theta_ += delta_theta;
    } else {
        x_ += (vx_filt * std::cos(theta_) - vy_filt * std::sin(theta_)) * dt;
        y_ += (vx_filt * std::sin(theta_) + vy_filt * std::cos(theta_)) * dt;
    }
    
    while (theta_ > M_PI) theta_ -= 2.0 * M_PI;
    while (theta_ < -M_PI) theta_ += 2.0 * M_PI;
    
    last_time_ = current_time;

    publish_odom();
}

double OdometryNode::filter_average(const std::deque<double>& window)
{
    if (window.empty()) return 0.0;
    double sum = std::accumulate(window.begin(), window.end(), 0.0);
    return sum / window.size();
}

void OdometryNode::publish_odom()
{
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_footprint";
    
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    
    odom_msg.pose.covariance = {
        0.01, 0, 0, 0, 0, 0,
        0, 0.01, 0, 0, 0, 0,
        0, 0, 0.01, 0, 0, 0,
        0, 0, 0, 0.01, 0, 0,
        0, 0, 0, 0, 0.01, 0,
        0, 0, 0, 0, 0, 0.01
    };
    
    odom_msg.twist.twist.linear.x = vx_;
    odom_msg.twist.twist.linear.y = vy_;
    odom_msg.twist.twist.angular.z = vtheta_;
    
    odom_msg.twist.covariance = {
        0.01, 0, 0, 0, 0, 0,
        0, 0.01, 0, 0, 0, 0,
        0, 0, 0.01, 0, 0, 0,
        0, 0, 0, 0.01, 0, 0,
        0, 0, 0, 0, 0.01, 0,
        0, 0, 0, 0, 0, 0.01
    };
    
    odom_pub_->publish(odom_msg);
    
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = odom_msg.header.stamp;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_footprint";
    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
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