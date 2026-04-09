#ifndef OMNIBOT_ENCODER_ODOMETRY_NODE_HPP
#define OMNIBOT_ENCODER_ODOMETRY_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <deque>
#include <memory>
#include <vector>

class OdometryNode : public rclcpp::Node
{
public:
    OdometryNode();

private:
    void encoder_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    void compute_odometry();
    void publish_odom();
    void wheel_velocities_from_ticks(const std::vector<int32_t>& current_ticks);
    
    struct RobotParams {
        double wheel_radius;    
        double robot_radius;
        double ticks_per_rev;
    } params_;

    std::vector<int32_t> last_ticks_;
    rclcpp::Time last_time_;
    std::vector<double> wheel_velocities_;

    double x_, y_, theta_;
    double vx_, vy_, vtheta_;

    std::deque<double> vx_filter_, vy_filter_, vtheta_filter_;
    static constexpr size_t FILTER_WINDOW = 5;
    
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr odom_timer_;
    
    double filter_average(const std::deque<double>& window);
};

#endif