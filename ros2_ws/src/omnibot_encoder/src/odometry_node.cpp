#include "omnibot_encoder/odometry_node.hpp"
#include <cmath>
#include <numeric>
#include <algorithm>

OdometryNode::OdometryNode()
    : Node("odometry_node"),
    timestamp_(0),
    x_(0.0),
    y_(0.0),
    heading_(0.0),
    linear_x_vel_(0.0),
    linear_y_vel_(0.0),
    angular_vel_(0.0),
    robot_radius_(0.0),
    wheel_radius_(0.0),
    wheels_old_pos_(0.0)
{
    // this->declare_parameter("wheel_radius", 0.05);
    // this->declare_parameter("robot_radius", 0.15);
    // this->declare_parameter("ticks_per_rev", 390.0);
    
    wheel_radius_ = 0.05; //this->get_parameter("wheel_radius").as_double();
    robot_radius_ = 0.15; //this->get_parameter("robot_radius").as_double();
    ticks_per_rev_ = 390; //this->get_parameter("ticks_per_rev").as_double();

    encoder_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/encoder_ticks", 10,
        std::bind(&OdometryNode::encoder_callback, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    
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
    if (dt < 0.0001) {
        timestamp_ = current_time;
        return;
    }
    if (dt > 0.1) {
        RCLCPP_WARN(this->get_logger(), "Large dt detected: %.3f s, resetting", dt);
        timestamp_ = current_time;
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
    
    std::vector<double> wheels_vel(3);
    for (size_t i = 0; i < 3; ++i) {
        double revs = static_cast<double>(delta_ticks[i]) / ticks_per_rev_;
        wheels_vel[i] = revs * 2.0 * M_PI / dt; // rad/s
    }

    // мб убрать
    // if (timestamp_.seconds() == 0.0 && timestamp_.nanoseconds() == 0) {
    //     timestamp_ = current_time;
    //     last_time_ = current_time;
    //     return;
    // }

    updateFromVel(wheels_vel, current_time);
}

bool OdometryNode::updateFromPos(const std::vector<double> & wheels_pos, const rclcpp::Time & time)
{
  const double dt = time.seconds() - timestamp_.seconds();
  // We cannot estimate angular velocity with very small time intervals
  if (std::fabs(dt) < 1e-6)
  {
    return false;
  }

  // Estimate angular velocity of wheels using old and current position [rads/s]:
  std::vector<double> wheels_vel(wheels_pos.size());
  for (size_t i = 0; i < static_cast<size_t>(wheels_pos.size()); ++i)
  {
    wheels_vel[i] = (wheels_pos[i] - wheels_old_pos_[i]) / dt;
    wheels_old_pos_[i] = wheels_pos[i];
  }

  if (updateFromVel(wheels_vel, time))
  {
    return true;
  }
  return false;
}

bool OdometryNode::updateFromVel(const std::vector<double> & wheels_vel, const rclcpp::Time & time)
{
  const double dt = time.seconds() - timestamp_.seconds();

  // Compute linear and angular velocities of the robot:
  const Eigen::Vector3d robot_velocity = compute_robot_velocity(wheels_vel);

  // Integrate odometry:
  integrate(robot_velocity(0) * dt, robot_velocity(1) * dt, robot_velocity(2) * dt);

  timestamp_ = time;

  linear_x_vel_ = robot_velocity(0);
  linear_y_vel_ = robot_velocity(1);
  angular_vel_ = robot_velocity(2);

  return true;
}

Eigen::Vector3d OdometryNode::compute_robot_velocity(const std::vector<double> & wheels_vel) const
{
  Eigen::MatrixXd A(wheels_vel.size(), 3);   // Transformation Matrix
  Eigen::VectorXd omega(wheels_vel.size());  // Wheel angular velocities vector

  const double angle_bw_wheels = (2 * M_PI) / static_cast<double>(wheels_vel.size());

  for (size_t i = 0; i < wheels_vel.size(); ++i)
  {
    // Define the transformation matrix
    const double theta = (angle_bw_wheels * static_cast<double>(i));
    A(static_cast<int>(i), 0) = std::sin(theta);
    A(static_cast<int>(i), 1) = -std::cos(theta);
    A(static_cast<int>(i), 2) = -robot_radius_;

    // Define the wheel angular velocities vector
    omega(static_cast<int>(i)) = wheels_vel[i];
  }

  // Compute the robot velocities using SVD decomposition
  const Eigen::Vector3d V =
    A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(omega * wheel_radius_);

  return V;
}

void OdometryNode::integrate(const double & dx, const double & dy, const double & dheading)
{
  if (std::fabs(dheading) < 1e-6)
  {
    // For very small dheading, approximate to linear motion
    x_ = x_ + ((dx * std::cos(heading_)) - (dy * std::sin(heading_)));
    y_ = y_ + ((dx * std::sin(heading_)) + (dy * std::cos(heading_)));
    heading_ = heading_ + dheading;
  }
  else
  {
    const double heading_old = heading_;
    heading_ = heading_ + dheading;
    x_ = x_ + ((dx / dheading) * (std::sin(heading_) - std::sin(heading_old))) +
         ((dy / dheading) * (std::cos(heading_) - std::cos(heading_old)));
    y_ = y_ - (dx / dheading) * (std::cos(heading_) - std::cos(heading_old)) +
         (dy / dheading) * (std::sin(heading_) - std::sin(heading_old));
  }
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

void OdometryNode::resetOdometry()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
}
void OdometryNode::setOdometry(const double & x, const double & y, const double & heading)
{
  x_ = x;
  y_ = y;
  heading_ = heading;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}