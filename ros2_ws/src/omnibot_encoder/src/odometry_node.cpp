#include "omnibot_encoder/odometry_node.hpp"
#include <cmath>
#include <numeric>
#include <algorithm>

OdometryNode::OdometryNode()
    : Node("odometry_node"),
    timestamp_(this->now()),
    last_time_ (timestamp_),
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
void OdometryNode::encoder_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (msg->data.size() < 3) return;

    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_time_).seconds();
    if (dt < 0.0001) {
        last_time_ = current_time;
        return;
    }
    if (dt > 0.1) {
        RCLCPP_WARN(this->get_logger(), "Large dt: %.3f, resetting", dt);
        last_time_ = current_time;
        for (size_t i = 0; i < 3; ++i) last_ticks_[i] = msg->data[i];
        return;
    }

    std::vector<int32_t> delta_ticks(3);
    for (size_t i = 0; i < 3; ++i) {
        delta_ticks[i] = msg->data[i] - last_ticks_[i];
        last_ticks_[i] = msg->data[i];
    }

    std::vector<double> w(3);
    for (size_t i = 0; i < 3; ++i) {
        double revs = static_cast<double>(delta_ticks[i]) / ticks_per_rev_;
        w[i] = revs * 2.0 * M_PI / dt;
    }

    double vx = (wheel_radius_ / 1.732) * (w[2] - w[1]);
    double vy = (wheel_radius_ / 3.0) * (2.0 * w[0] - w[1] - w[2]);
    double vtheta = -(wheel_radius_ / (3.0 * robot_radius_)) * (w[0] + w[1] + w[2]);

    // Фильтрация
    vx_filter_.push_back(vx);
    vy_filter_.push_back(vy);
    vtheta_filter_.push_back(vtheta);
    while (vx_filter_.size() > FILTER_WINDOW) vx_filter_.pop_front();
    while (vy_filter_.size() > FILTER_WINDOW) vy_filter_.pop_front();
    while (vtheta_filter_.size() > FILTER_WINDOW) vtheta_filter_.pop_front();

    double vx_filt = filter_average(vx_filter_);
    double vy_filt = filter_average(vy_filter_);
    double vtheta_filt = filter_average(vtheta_filter_);

    // Сохраняем в члены класса для публикации
    linear_x_vel_ = vx_filt;
    linear_y_vel_ = vy_filt;
    angular_vel_ = vtheta_filt;

    // Интеграция
    if (std::abs(vtheta_filt) > 1e-6) {
        double delta_theta = vtheta_filt * dt;
        double radius = std::hypot(vx_filt, vy_filt) / std::abs(vtheta_filt);
        double heading_old = heading_;
        heading_ += delta_theta;
        double delta_x = radius * (std::sin(heading_) - std::sin(heading_old));
        double delta_y = -radius * (std::cos(heading_) - std::cos(heading_old));
        x_ += delta_x;
        y_ += delta_y;
    } else {
        x_ += (vx_filt * std::cos(heading_) - vy_filt * std::sin(heading_)) * dt;
        y_ += (vx_filt * std::sin(heading_) + vy_filt * std::cos(heading_)) * dt;
        heading_ += vtheta_filt * dt;
    }

    // Нормализация угла
    while (heading_ > M_PI) heading_ -= 2.0 * M_PI;
    while (heading_ < -M_PI) heading_ += 2.0 * M_PI;

    last_time_ = current_time;
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