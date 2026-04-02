#include <rclcpp/rclcpp.hpp>

class KinematicNode : public rclcpp::Node {
public:
    KinematicNode() : Node("kinematic") {
        this->declare_paremeter("", 0);
        
    }

}