#include "omnibot_encoder/odometry_node.hpp"
#include <cmath>
#include <numeric>
#include <algorithm>

OdometryNode::OdometryNode()
    : Node("odometry_node"),
    timestamp_(this->now()),
    x_(0.0),
    y_(0.0),
    heading_(0.0),
    linear_x_vel_(0.0),
    linear_y_vel_(0.0),
    angular_vel_(0.0),
    robot_radius_(0.15),
    wheel_radius_(0.05),
    ticks_per_rev_(390),
    wheels_old_pos_(0.0)
{
  encoder_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
    "/encoder_ticks", 10,
    std::bind(&OdometryNode::encoder_callback, this, std::placeholders::_1));

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  last_ticks_.resize(3, 0);
  
  odom_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&OdometryNode::publish_odom, this));

  RCLCPP_INFO(
      this->get_logger(),
      "Odometry node started");
  RCLCPP_INFO(
      this->get_logger(),
      "Params: wheel_radius=%.3f, robot_radius=%.3f, ticks_per_rev=%.0f",
      wheel_radius_, robot_radius_, ticks_per_rev_);
}

void OdometryNode::encoder_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  rclcpp::Time current_time = this->now();
  double dt = (current_time - timestamp_).seconds();
  // if (dt < 0.0001) {
  //   timestamp_ = current_time;
  //   return;
  // }
//   if (dt > 0.1) {
//     RCLCPP_WARN(this->get_logger(), "Large dt detected: %.3f s, resetting", dt);
//     timestamp_ = current_time;
//     for (size_t i = 0; i < 3 && i < msg->data.size(); ++i) {
//         last_ticks_[i] = msg->data[i];
//     }
//     return;
//   }

//     std::vector<int32_t> delta_ticks(3);
//     for (size_t i = 0; i < 3; ++i) {
//         delta_ticks[i] = msg->data[i] - last_ticks_[i];
//         last_ticks_[i] = msg->data[i];
//     }
    
//     for (size_t i = 0; i < 3; ++i) {
//         double revs = static_cast<double>(delta_ticks[i]) / params_.ticks_per_rev;
//         double wheel_distance = revs * 2.0 * M_PI * params_.wheel_radius;
//         wheel_velocities_[i] = wheel_distance / dt;
//     }

//     vx_ = (2.0 * wheel_velocities_[0] - wheel_velocities_[1] - wheel_velocities_[2]) / 3.0;
//     vy_ = (wheel_velocities_[1] - wheel_velocities_[2]) / std::sqrt(3.0);
//     vtheta_ = (wheel_velocities_[0] + wheel_velocities_[1] + wheel_velocities_[2]) / (3.0 * params_.robot_radius);

//     vx_filter_.push_back(vx_);
//     vy_filter_.push_back(vy_);
//     vtheta_filter_.push_back(vtheta_);
    
//     while (vx_filter_.size() > FILTER_WINDOW) vx_filter_.pop_front();
//     while (vy_filter_.size() > FILTER_WINDOW) vy_filter_.pop_front();
//     while (vtheta_filter_.size() > FILTER_WINDOW) vtheta_filter_.pop_front();
    
//     double vx_filt = filter_average(vx_filter_);
//     double vy_filt = filter_average(vy_filter_);
//     double vtheta_filt = filter_average(vtheta_filter_);

//     if (std::abs(vtheta_filt) > 1e-6) {
//         double delta_theta = vtheta_filt * dt;
//         double R_theta = vtheta_filt;
//         double V_robot = std::hypot(vx_filt, vy_filt);
//         double angle_robot = std::atan2(vy_filt, vx_filt);
        
//         double radius = V_robot / std::abs(R_theta);
//         double delta_x = radius * (std::sin(theta_ + delta_theta) - std::sin(theta_));
//         double delta_y = -radius * (std::cos(theta_ + delta_theta) - std::cos(theta_));
        
//         x_ += delta_x;
//         y_ += delta_y;
//         theta_ += delta_theta;
//     } else {
//         x_ += (vx_filt * std::cos(theta_) - vy_filt * std::sin(theta_)) * dt;
//         y_ += (vx_filt * std::sin(theta_) + vy_filt * std::cos(theta_)) * dt;
//     }
    
//     while (theta_ > M_PI) theta_ -= 2.0 * M_PI;
//     while (theta_ < -M_PI) theta_ += 2.0 * M_PI;
    
//     last_time_ = current_time;

    publish_odom();
}

// double OdometryNode::filter_average(const std::deque<double>& window)
// {
//     if (window.empty()) return 0.0;
//     double sum = std::accumulate(window.begin(), window.end(), 0.0);
//     return sum / window.size();
// }

void OdometryNode::publish_odom()
{
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_footprint";
    
    // Заполняем позицию
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;
    
    // Заполняем ориентацию (из угла heading_)
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, heading_);
    odom_msg.pose.pose.orientation = tf2::toMsg(q);
    
    // Заполняем ковариации (для простоты - единичные, можно потом настроить)
    odom_msg.pose.covariance = {0.01, 0, 0, 0, 0, 0,
                                 0, 0.01, 0, 0, 0, 0,
                                 0, 0, 0.01, 0, 0, 0,
                                 0, 0, 0, 0.01, 0, 0,
                                 0, 0, 0, 0, 0.01, 0,
                                 0, 0, 0, 0, 0, 0.01};
    
    // Заполняем скорости
    odom_msg.twist.twist.linear.x = linear_x_vel_;
    odom_msg.twist.twist.linear.y = linear_y_vel_;
    odom_msg.twist.twist.angular.z = angular_vel_;
    
    // Ковариации скоростей
    odom_msg.twist.covariance = {0.01, 0, 0, 0, 0, 0,
                                  0, 0.01, 0, 0, 0, 0,
                                  0, 0, 0.01, 0, 0, 0,
                                  0, 0, 0, 0.01, 0, 0,
                                  0, 0, 0, 0, 0.01, 0,
                                  0, 0, 0, 0, 0, 0.01};
    
    odom_pub_->publish(odom_msg);
    
    // Публикуем трансформацию tf
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = odom_msg.header;
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