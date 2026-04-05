#include <rclcpp/rclcpp.hpp>

class KinematicNode : public rclcpp::Node {
public:
    KinematicNode() : Node("kinematic") {
        this->declare_paremeter("", 0);
        
    }

}

/* Так, по сути
Нода энкодера выдает только кол-во оборотов раз в n секунд

Скорость обороты/секунды
v = (m * 360/390) / n, где
m - кол-во сигналов на энкодере
n - сколько секунд прошло с прошлого обновления

Так мы знаем скороть трех колес





*/