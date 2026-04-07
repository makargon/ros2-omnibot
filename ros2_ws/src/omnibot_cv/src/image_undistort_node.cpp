#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>

// Node:
// 1) получает raw-изображение и CameraInfo,
// 2) строит K/D,
// 3) публикует undistorted image.
class ImageUndistortNode : public rclcpp::Node {
public:
  ImageUndistortNode() : Node("image_undistort_node") {
    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/image_raw");
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
    output_image_topic_ = declare_parameter<std::string>("output_image_topic", "/camera/image_undistorted");

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ImageUndistortNode::onImage, this, std::placeholders::_1));

    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, rclcpp::QoS(10),
        std::bind(&ImageUndistortNode::onInfo, this, std::placeholders::_1));

    image_pub_ = create_publisher<sensor_msgs::msg::Image>(output_image_topic_, rclcpp::SensorDataQoS());
  }

private:
  // Сохраняем матрицу камеры и дисторсию из CameraInfo.
  void onInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (msg->k.size() != 9) {
      return;
    }

    const bool has_valid_k = std::any_of(msg->k.begin(), msg->k.end(), [](double v) { return v != 0.0; });
    if (!has_valid_k) {
      return;
    }

    camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double *>(msg->k.data())).clone();
    if (msg->d.empty()) {
      distortion_ = cv::Mat::zeros(1, 5, CV_64F);
    } else {
      distortion_ = cv::Mat(msg->d).clone();
    }
    has_calibration_ = true;
  }

  // Применяем cv::undistort и публикуем кадр в выходной топик.
  void onImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_WARN(get_logger(), "cv_bridge error: %s", e.what());
      return;
    }

    if (!has_calibration_) {
      // Если calibration пока не пришла, не «глушим» pipeline: публикуем исходный кадр.
      image_pub_->publish(*msg);
      return;
    }

    cv::Mat undistorted;
    cv::undistort(cv_ptr->image, undistorted, camera_matrix_, distortion_);

    auto out = cv_bridge::CvImage(msg->header, msg->encoding, undistorted).toImageMsg();
    image_pub_->publish(*out);
  }

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string output_image_topic_;

  bool has_calibration_{false};
  cv::Mat camera_matrix_;
  cv::Mat distortion_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};

// Точка входа ROS 2.
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageUndistortNode>());
  rclcpp::shutdown();
  return 0;
}
