#ifndef OMNIBOT_ENCODER_ODOMETRY_NODE_HPP
#define OMNIBOT_ENCODER_ODOMETRY_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/msg/transform_stamped.h"
#include "omnibot_encoder/odometry.hpp"

#include <deque>
#include <memory>
#include <vector>

class OdometryNode : public rclcpp::Node
{
public:
    OdometryNode();

private:
    void encoder_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    void publish_odom_and_tf();

    struct RobotParams {
        double wheel_radius;
        double robot_radius;
        double wheel_offset;
        double wheel_count;
        double ticks_per_rev;
    } params_;

    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    Odometry odometry_;
    
    rclcpp::Time last_time_;
    bool first_measurement_;

    std::vector<int32_t> last_ticks_;
    std::vector<double> wheel_angles_rad_;

    rclcpp::TimerBase::SharedPtr odom_timer_;
};

#endif