#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <unordered_map>
#include <vector>

// Поза маркера в системе координат поля.
struct FieldMarker {
  double x{};
  double y{};
  double yaw{};
};

// Node:
// 1) детектирует ArUco в кадре,
// 2) оценивает позы маркеров относительно камеры,
// 3) восстанавливает позу робота в системе координат поля.
class ArucoPoseNode : public rclcpp::Node {
public:
  ArucoPoseNode() : Node("aruco_pose_node") {
    image_topic_ = declare_parameter<std::string>("image_topic", "/station/camera/image_undistorted");
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/station/camera/camera_info");
    field_frame_ = declare_parameter<std::string>("field_frame", "aruco_field");
    marker_length_ = declare_parameter<double>("marker_length_m", 0.06);
    robot_marker_id_ = declare_parameter<int>("robot_marker_id", 0);
    world_pose_smoothing_alpha_ = declare_parameter<double>("world_pose_smoothing_alpha", 0.2);
    world_pose_hold_frames_ = declare_parameter<int>("world_pose_hold_frames", 12);
    world_pose_min_markers_ = declare_parameter<int>("world_pose_min_markers", 2);
    world_pose_max_reproj_error_px_ = declare_parameter<double>("world_pose_max_reproj_error_px", 8.0);
    debug_image_topic_ = declare_parameter<std::string>("debug_image_topic", "/aruco/debug_image");
    detections_text_topic_ = declare_parameter<std::string>("detections_text_topic", "/aruco/detections_text");

    const auto dictionary_name = declare_parameter<std::string>("aruco_dictionary", "DICT_4X4_50");
    dictionary_ = cv::aruco::getPredefinedDictionary(dictionaryFromName(dictionary_name));

    const auto ids = declare_parameter<std::vector<int64_t>>("marker_ids", {0});
    const auto xs = declare_parameter<std::vector<double>>("marker_x_m", {0.0});
    const auto ys = declare_parameter<std::vector<double>>("marker_y_m", {0.0});
    const auto yaws = declare_parameter<std::vector<double>>("marker_yaw_rad", {0.0});
    loadFieldMap(ids, xs, ys, yaws);

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ArucoPoseNode::onImage, this, std::placeholders::_1));

    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, rclcpp::QoS(10),
        std::bind(&ArucoPoseNode::onInfo, this, std::placeholders::_1));

    markers_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/aruco/marker_poses", 10);
    robot_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/aruco/robot_pose", 10);
    debug_image_pub_ = create_publisher<sensor_msgs::msg::Image>(debug_image_topic_, rclcpp::SensorDataQoS());
    detections_text_pub_ = create_publisher<std_msgs::msg::String>(detections_text_topic_, 10);
  }

private:
  // Кэш параметров камеры из CameraInfo (K + D).
  void onInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (msg->k.size() != 9) {
      return;
    }

    const bool has_valid_k = std::any_of(msg->k.begin(), msg->k.end(), [](double v) { return v != 0.0; });
    if (has_valid_k) {
      camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double *>(msg->k.data())).clone();
    } else if (msg->width > 0 && msg->height > 0) {
      const double fx = 0.9 * static_cast<double>(msg->width);
      const double fy = 0.9 * static_cast<double>(msg->height);
      const double cx = 0.5 * static_cast<double>(msg->width);
      const double cy = 0.5 * static_cast<double>(msg->height);
      camera_matrix_ = (cv::Mat_<double>(3, 3) << fx, 0.0, cx,
                                               0.0, fy, cy,
                                               0.0, 0.0, 1.0);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "CameraInfo has zero K; using approximate intrinsics from image size.");
    } else {
      return;
    }

    if (msg->d.empty()) {
      distortion_ = cv::Mat::zeros(1, 5, CV_64F);
    } else {
      distortion_ = cv::Mat(msg->d).clone();
    }
    has_camera_info_ = true;
  }

  // Алгоритм (устойчивый world-origin):
  // 1) Детектируем все ArUco в кадре.
  // 2) Для известных (есть в marker_ids/x/y/yaw) собираем 2D-3D соответствия по 4 углам каждого маркера.
  // 3) Если известных маркеров >= world_pose_min_markers:
  //    3.1) solvePnPRansac => грубая camera<-field,
  //    3.2) solvePnP ITERATIVE (c useExtrinsicGuess=true) => уточнение,
  //    3.3) считаем reprojection RMSE и принимаем только если ошибка ниже порога.
  // 4) Принятую позу сглаживаем по времени (translation lerp + quaternion slerp).
  // 5) При кратковременной потере маркеров удерживаем последнюю валидную позу N кадров.
  // 6) Рисуем WORLD(0,0) и оси только из этой глобальной позы (никакой привязки к одиночному маркеру).
  void onImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception &) {
      return;
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(cv_ptr->image, dictionary_, corners, ids);

    if (ids.empty()) {
      publishDebugImage(msg->header, cv_ptr->image);
      publishDetectionsText("no_markers");
      return;
    }

    cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);

    if (!has_camera_info_) {
      std::ostringstream ss;
      ss << "ids=";
      for (size_t i = 0; i < ids.size(); ++i) {
        ss << ids[i] << (i + 1 < ids.size() ? "," : "");
      }
      ss << "; pose=unavailable(camera_info_missing)";
      publishDetectionsText(ss.str());
      publishDebugImage(msg->header, cv_ptr->image);
      return;
    }

    std::vector<cv::Vec3d> rvecs;
    std::vector<cv::Vec3d> tvecs;
    cv::aruco::estimatePoseSingleMarkers(corners, marker_length_, camera_matrix_, distortion_, rvecs, tvecs);

    std::vector<cv::Point3f> field_object_points;
    std::vector<cv::Point2f> field_image_points;

    std::ostringstream ss;
    ss << "count=" << ids.size() << "; ";

    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.stamp = now();
    pose_array.header.frame_id = msg->header.frame_id;

    cv::Matx44d tf_field_camera;
    bool has_tf_field_camera = false;

    for (size_t i = 0; i < ids.size(); ++i) {
      pose_array.poses.push_back(toPose(rvecs[i], tvecs[i]));
      cv::drawFrameAxes(cv_ptr->image, camera_matrix_, distortion_, rvecs[i], tvecs[i], marker_length_ * 0.5F);

      const auto center = 0.25F * (corners[i][0] + corners[i][1] + corners[i][2] + corners[i][3]);
      const auto text_pos = cv::Point(static_cast<int>(center.x) - 20, static_cast<int>(center.y) - 26);
      cv::putText(cv_ptr->image,
                  "ID " + std::to_string(ids[i]),
                  text_pos,
                  cv::FONT_HERSHEY_SIMPLEX,
                  0.95,
                  cv::Scalar(0, 255, 255),
                  2,
                  cv::LINE_AA);

      ss << "id=" << ids[i]
         << "(x=" << tvecs[i][0]
         << ",y=" << tvecs[i][1]
         << ",z=" << tvecs[i][2] << ")";
      if (i + 1 < ids.size()) {
        ss << "; ";
      }

      auto it = field_markers_.find(ids[i]);
      if (it != field_markers_.end()) {
        const auto obj = markerCornersInField(it->second, marker_length_);
        for (size_t k = 0; k < 4; ++k) {
          field_object_points.push_back(obj[k]);
          field_image_points.push_back(corners[i][k]);
        }
      }
    }

    markers_pub_->publish(pose_array);

    bool has_robot_pose = false;
    const int known_markers_seen = static_cast<int>(field_object_points.size() / 4);
    bool world_pose_updated = false;
    double reproj_rmse = -1.0;

    if (known_markers_seen >= world_pose_min_markers_) {
      cv::Vec3d rvec_camera_field;
      cv::Vec3d tvec_camera_field;
      std::vector<int> inliers;

      bool pnp_ok = cv::solvePnPRansac(
          field_object_points,
          field_image_points,
          camera_matrix_,
          distortion_,
          rvec_camera_field,
          tvec_camera_field,
          false,
          150,
          4.0f,
          0.995,
          inliers,
          cv::SOLVEPNP_ITERATIVE);

      if (pnp_ok) {
        // Уточняем решением LM вокруг RANSAC-оценки.
        pnp_ok = cv::solvePnP(
            field_object_points,
            field_image_points,
            camera_matrix_,
            distortion_,
            rvec_camera_field,
            tvec_camera_field,
            true,
            cv::SOLVEPNP_ITERATIVE);
      }

      if (pnp_ok) {
        reproj_rmse = reprojectionRmse(
            field_object_points,
            field_image_points,
            camera_matrix_,
            distortion_,
            rvec_camera_field,
            tvec_camera_field);

        if (reproj_rmse >= 0.0 && reproj_rmse <= world_pose_max_reproj_error_px_) {
          const auto tf_camera_field = toTransform(rvec_camera_field, tvec_camera_field);
          const auto candidate_field_camera = tf_camera_field.inv();

          if (!has_world_pose_) {
            tf_field_camera_filtered_ = candidate_field_camera;
            has_world_pose_ = true;
          } else {
            tf_field_camera_filtered_ = blendTransform(
                tf_field_camera_filtered_,
                candidate_field_camera,
                std::clamp(world_pose_smoothing_alpha_, 0.0, 1.0));
          }
          lost_world_pose_frames_ = 0;
          world_pose_updated = true;
        }
      }
    }

    if (has_world_pose_) {
      if (!world_pose_updated) {
        lost_world_pose_frames_++;
      }
      if (lost_world_pose_frames_ <= world_pose_hold_frames_) {
        tf_field_camera = tf_field_camera_filtered_;
        has_tf_field_camera = true;
      }
    }

    if (has_tf_field_camera) {
      const auto tf_camera_field = tf_field_camera.inv();

      cv::Vec3d origin_rvec;
      cv::Vec3d origin_tvec;
      if (transformToRvecTvec(tf_camera_field, origin_rvec, origin_tvec)) {
        cv::drawFrameAxes(cv_ptr->image, camera_matrix_, distortion_, origin_rvec, origin_tvec, marker_length_ * 1.2F);

        std::vector<cv::Point3f> origin_3d{{0.0F, 0.0F, 0.0F}};
        std::vector<cv::Point2f> origin_2d;
        cv::projectPoints(origin_3d, origin_rvec, origin_tvec, camera_matrix_, distortion_, origin_2d);
        if (!origin_2d.empty()) {
          const auto p = cv::Point(static_cast<int>(origin_2d[0].x), static_cast<int>(origin_2d[0].y));
          cv::circle(cv_ptr->image, p, 7, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
          cv::putText(cv_ptr->image,
                      "WORLD (0,0)",
                      cv::Point(p.x + 10, p.y - 12),
                      cv::FONT_HERSHEY_SIMPLEX,
                      0.95,
                      cv::Scalar(0, 0, 255),
                      2,
                      cv::LINE_AA);
        }
      }

      for (size_t i = 0; i < ids.size(); ++i) {
        if (ids[i] != robot_marker_id_) {
          continue;
        }
        const auto tf_camera_robot = toTransform(rvecs[i], tvecs[i]);
        const auto tf_field_robot = tf_field_camera * tf_camera_robot;

        geometry_msgs::msg::PoseStamped robot_pose;
        robot_pose.header.stamp = now();
        robot_pose.header.frame_id = field_frame_;
        robot_pose.pose = toPose(tf_field_robot);
        robot_pub_->publish(robot_pose);
        has_robot_pose = true;
        break;
      }
    }

    if (!has_tf_field_camera) {
      if (field_object_points.empty()) {
        ss << "; world_origin=unavailable(no_known_marker_seen)";
      } else {
        ss << "; world_origin=unavailable(pnp_failed_or_timeout)";
      }
    } else {
      if (!world_pose_updated) {
        ss << "; world_origin=held";
      } else {
        ss << "; world_origin=visible";
      }
      ss << "; world_origin_mode=multi_marker";
    }

    if (reproj_rmse >= 0.0) {
      ss << "; pnp_rmse_px=" << reproj_rmse;
    }
    if (!has_robot_pose) {
      ss << "; robot_pose=unavailable(robot_marker_not_seen)";
    }

    publishDetectionsText(ss.str());
    publishDebugImage(msg->header, cv_ptr->image);
  }

  void publishDebugImage(const std_msgs::msg::Header &header, const cv::Mat &image) {
    auto out = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image).toImageMsg();
    debug_image_pub_->publish(*out);
  }

  void publishDetectionsText(const std::string &text) {
    std_msgs::msg::String msg;
    msg.data = text;
    detections_text_pub_->publish(msg);
  }

  // Загружает карту поля из параметров marker_ids/x/y/yaw.
  void loadFieldMap(const std::vector<int64_t> &ids,
                    const std::vector<double> &xs,
                    const std::vector<double> &ys,
                    const std::vector<double> &yaws) {
    const size_t n = std::min({ids.size(), xs.size(), ys.size(), yaws.size()});
    for (size_t i = 0; i < n; ++i) {
      field_markers_[static_cast<int>(ids[i])] = FieldMarker{xs[i], ys[i], yaws[i]};
    }
  }

  static geometry_msgs::msg::Pose toPose(const cv::Vec3d &rvec, const cv::Vec3d &tvec) {
    return toPose(toTransform(rvec, tvec));
  }

  static geometry_msgs::msg::Pose toPose(const cv::Matx44d &t) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = t(0, 3);
    pose.position.y = t(1, 3);
    pose.position.z = t(2, 3);

    tf2::Matrix3x3 basis(
        t(0, 0), t(0, 1), t(0, 2),
        t(1, 0), t(1, 1), t(1, 2),
        t(2, 0), t(2, 1), t(2, 2));
    tf2::Quaternion q;
    basis.getRotation(q);

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
  }

  // Rodrigues + translation -> 4x4 transform (camera <- marker).
  static cv::Matx44d toTransform(const cv::Vec3d &rvec, const cv::Vec3d &tvec) {
    cv::Mat rot;
    cv::Rodrigues(rvec, rot);

    cv::Matx44d t = cv::Matx44d::eye();
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        t(r, c) = rot.at<double>(r, c);
      }
      t(r, 3) = tvec[r];
    }
    return t;
  }

  static std::vector<cv::Point3f> markerCornersInField(const FieldMarker &m, double marker_length) {
    const double h = 0.5 * marker_length;
    const double c = std::cos(m.yaw);
    const double s = std::sin(m.yaw);

    const std::vector<cv::Point2d> local{
        {-h, +h},
        {+h, +h},
        {+h, -h},
        {-h, -h},
    };

    std::vector<cv::Point3f> out;
    out.reserve(4);
    for (const auto &p : local) {
      const double x = m.x + c * p.x - s * p.y;
      const double y = m.y + s * p.x + c * p.y;
      out.emplace_back(static_cast<float>(x), static_cast<float>(y), 0.0F);
    }
    return out;
  }

  static double reprojectionRmse(const std::vector<cv::Point3f> &object_points,
                                 const std::vector<cv::Point2f> &image_points,
                                 const cv::Mat &camera_matrix,
                                 const cv::Mat &distortion,
                                 const cv::Vec3d &rvec,
                                 const cv::Vec3d &tvec) {
    if (object_points.empty() || object_points.size() != image_points.size()) {
      return -1.0;
    }

    std::vector<cv::Point2f> projected;
    cv::projectPoints(object_points, rvec, tvec, camera_matrix, distortion, projected);
    if (projected.size() != image_points.size()) {
      return -1.0;
    }

    double sse = 0.0;
    for (size_t i = 0; i < projected.size(); ++i) {
      const double dx = static_cast<double>(projected[i].x - image_points[i].x);
      const double dy = static_cast<double>(projected[i].y - image_points[i].y);
      sse += dx * dx + dy * dy;
    }
    return std::sqrt(sse / static_cast<double>(projected.size()));
  }

  static bool transformToRvecTvec(const cv::Matx44d &t, cv::Vec3d &rvec, cv::Vec3d &tvec) {
    cv::Mat rot(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        rot.at<double>(r, c) = t(r, c);
      }
    }
    cv::Rodrigues(rot, rvec);
    tvec = cv::Vec3d(t(0, 3), t(1, 3), t(2, 3));
    return true;
  }

  static tf2::Quaternion rotationToQuat(const cv::Matx44d &t) {
    tf2::Matrix3x3 basis(
        t(0, 0), t(0, 1), t(0, 2),
        t(1, 0), t(1, 1), t(1, 2),
        t(2, 0), t(2, 1), t(2, 2));
    tf2::Quaternion q;
    basis.getRotation(q);
    q.normalize();
    return q;
  }

  static cv::Matx33d quatToMat(const tf2::Quaternion &q_in) {
    tf2::Quaternion q = q_in;
    q.normalize();
    tf2::Matrix3x3 m(q);
    return cv::Matx33d(
        m[0][0], m[0][1], m[0][2],
        m[1][0], m[1][1], m[1][2],
        m[2][0], m[2][1], m[2][2]);
  }

  static cv::Matx44d blendTransform(const cv::Matx44d &prev, const cv::Matx44d &next, double alpha) {
    const cv::Vec3d t_prev(prev(0, 3), prev(1, 3), prev(2, 3));
    const cv::Vec3d t_next(next(0, 3), next(1, 3), next(2, 3));
    const cv::Vec3d t = (1.0 - alpha) * t_prev + alpha * t_next;

    auto q_prev = rotationToQuat(prev);
    auto q_next = rotationToQuat(next);
    if (q_prev.dot(q_next) < 0.0) {
      q_next = tf2::Quaternion(-q_next.x(), -q_next.y(), -q_next.z(), -q_next.w());
    }
    const auto q = q_prev.slerp(q_next, alpha);
    const auto r = quatToMat(q);

    cv::Matx44d out = cv::Matx44d::eye();
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        out(i, j) = r(i, j);
      }
    }
    out(0, 3) = t[0];
    out(1, 3) = t[1];
    out(2, 3) = t[2];
    return out;
  }

  // Перевод строкового имени словаря в enum OpenCV.
  static cv::aruco::PREDEFINED_DICTIONARY_NAME dictionaryFromName(const std::string &name) {
    static const std::unordered_map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> map{
        {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
        {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
        {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
        {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
        {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
        {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
        {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
        {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
        {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL},
    };
    const auto it = map.find(name);
    return it == map.end() ? cv::aruco::DICT_4X4_50 : it->second;
  }

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string field_frame_;
  std::string debug_image_topic_;
  std::string detections_text_topic_;
  double marker_length_{};
  int robot_marker_id_{};
  double world_pose_smoothing_alpha_{};
  int world_pose_hold_frames_{};
  int world_pose_min_markers_{};
  double world_pose_max_reproj_error_px_{};

  bool has_camera_info_{false};
  bool has_world_pose_{false};
  int lost_world_pose_frames_{0};
  cv::Mat camera_matrix_;
  cv::Mat distortion_;
  cv::Matx44d tf_field_camera_filtered_{cv::Matx44d::eye()};
  cv::Ptr<cv::aruco::Dictionary> dictionary_;

  std::unordered_map<int, FieldMarker> field_markers_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detections_text_pub_;
};

// Точка входа ROS 2.
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoPoseNode>());
  rclcpp::shutdown();
  return 0;
}
