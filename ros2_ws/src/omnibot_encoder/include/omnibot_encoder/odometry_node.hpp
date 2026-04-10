#ifndef OMNIBOT_ENCODER_ODOMETRY_NODE_HPP
#define OMNIBOT_ENCODER_ODOMETRY_NODE_HPP

#include <Eigen/Dense>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <deque>
#include <memory>

class OdometryNode : public rclcpp::Node
{
public:
    OdometryNode();

    bool updateFromVel(const std::vector<double> & wheels_vel, const rclcpp::Time & time);
    void setOdometry(const double & x, const double & y, const double & heading);
    void resetOdometry();

    double getX() const { return x_; }
    double getY() const { return y_; }
    double getHeading() const { return heading_; }
    double getLinearXVel() const { return linear_x_vel_; }
    double getLinearYVel() const { return linear_y_vel_; }
    double getAngularVel() const { return angular_vel_; }
    

private:
    void encoder_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    Eigen::Vector3d compute_robot_velocity(const std::vector<double> & wheels_vel) const;
    void integrate(const double & dx, const double & dy, const double & dheading);
    double filter_average(const std::deque<double>& window);
    void publish_odom();
    
    // Current timestamp:
    rclcpp::Time timestamp_;

    // Current pose:
    double x_;        // [m]
    double y_;        // [m]
    double heading_;  // [rads]
    
    // Current velocity:
    double linear_x_vel_;  // [m/s]
    double linear_y_vel_;  // [m/s]
    double angular_vel_;   // [rads/s]

    // Robot kinematic parameters:
    double robot_radius_;  // [m]
    double wheel_radius_;  // [m]
        
    double vx_;
    double vy_;
    double vtheta_;


    // Previous wheel positions/states [rads]:
    std::vector<double> wheels_old_pos_;

    double ticks_per_rev_;
    std::vector<int32_t> last_ticks_;
    
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr odom_timer_;
};

#endif