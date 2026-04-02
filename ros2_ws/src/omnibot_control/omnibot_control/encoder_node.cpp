#include <pigpio.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <chrono>
// #include <memory>
#include <vector>

class EncoderReader {
public:
    EncoderReader(int pin_a, int pin_b)
        : pin_a_(pin_a), pin_b_(pin_b), counter_(0), last_a_(0), last_b_(0) {
        gpioSetMode(pin_a_, PI_INPUT);
        gpioSetMode(pin_b_, PI_INPUT);
        gpioSetPullUpDown(pin_a_, PI_PUD_UP);
        gpioSetPullUpDown(pin_b_, PI_PUD_UP);

        last_a_ = gpioRead(pin_a_);
        last_b_ = gpioRead(pin_b_);

        callback_a_ = gpioSetAlertFuncEx(pin_a_, &EncoderReader::_pulseCallback, this);
        callback_b_ = gpioSetAlertFuncEx(pin_b_, &EncoderReader::_pulseCallback, this);
    }

    ~EncoderReader() {
        gpioSetAlertFuncEx(pin_a_, nullptr, nullptr);
        gpioSetAlertFuncEx(pin_b_, nullptr, nullptr);
    }

    long read() const {
        return counter_;
    }

    void reset() {
        counter_ = 0;
    }

private:
    static void _pulseCallback(int gpio, int level, uint32_t tick, void* userdata) {
        EncoderReader* self = static_cast<EncoderReader*>(userdata);
        self->_onPulse(gpio, level);
    }

    void _onPulse(int gpio, int level) {
        if (gpio == pin_a_)
            last_a_ = level;
        else if (gpio == pin_b_)
            last_b_ = level;

        if (gpio == pin_a_ && level == 1) {
            if (last_b_ == 1)
                counter_++;
            else
                counter_--;
        } else if (gpio == pin_b_ && level == 1) {
            if (last_a_ == 1)
                counter_--;
            else
                counter_++;
        }
    }

    int pin_a_, pin_b_;
    volatile long counter_;
    volatile int last_a_, last_b_;
    int callback_a_, callback_b_;
};

class EncoderNode : public rclcpp::Node {
public:
    EncoderNode() : Node("encoder") {
        this->declare_parameter("encoder1.pin_a", 2);
        this->declare_parameter("encoder1.pin_b", 3);
        this->declare_parameter("encoder2.pin_a", 4);
        this->declare_parameter("encoder2.pin_b", 5);
        this->declare_parameter("encoder3.pin_a", 6);
        this->declare_parameter("encoder3.pin_b", 7);

        int e1a = this->get_parameter("encoder1.pin_a").as_int();
        int e1b = this->get_parameter("encoder1.pin_b").as_int();
        int e2a = this->get_parameter("encoder2.pin_a").as_int();
        int e2b = this->get_parameter("encoder2.pin_b").as_int();
        int e3a = this->get_parameter("encoder3.pin_a").as_int();
        int e3b = this->get_parameter("encoder3.pin_b").as_int();

        if (gpioInitialise() < 0) {
            RCLCPP_ERROR(this->get_logger(), "Ошибка инициализации pigpio! Запустите 'sudo pigpiod'");
            rclcpp::shutdown();
        }

        encoders_.push_back(std::make_unique<EncoderReader>(e1a, e1b));
        encoders_.push_back(std::make_unique<EncoderReader>(e2a, e2b));
        encoders_.push_back(std::make_unique<EncoderReader>(e3a, e3b));

        publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("/encoder_ticks", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(20),
                                         std::bind(&EncoderNode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Encoder Node запущена. Публикуем в /encoder_ticks");
    }

    ~EncoderNode() {
        gpioTerminate();
    }

private:
    void timerCallback() {
        auto msg = std_msgs::msg::Int32MultiArray();
        msg.data.resize(3);
        for (size_t i = 0; i < encoders_.size(); ++i) {
            msg.data[i] = encoders_[i]->read();
        }
        publisher_->publish(msg);
    }

    std::vector<std::unique_ptr<EncoderReader>> encoders_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EncoderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}