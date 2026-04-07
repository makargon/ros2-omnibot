#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "omnibot_cv/aruco_geometry.hpp"
#include "omnibot_cv/world_pose_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <unordered_map>
#include <vector>

class ArucoPoseNode : public rclcpp::Node {
public:
  ArucoPoseNode() : Node("aruco_pose_node") {
    // Алгоритм простыми шагами:
    // 1) Детектируем ArUco и считаем локальные pose каждого маркера.
    // 2) Для известных маркеров собираем 3D-углы в поле + 2D-углы в кадре.
    // 3) Отдаём соответствия в WorldPoseTracker (RANSAC+refine+RMSE+сглаживание+hold).
    // 4) Если глобальная поза есть, рисуем WORLD(0,0) и публикуем позу робота в field frame.

    image_topic_ = declare_parameter<std::string>("image_topic", "/station/camera/image_undistorted");
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/station/camera/camera_info");
    field_frame_ = declare_parameter<std::string>("field_frame", "aruco_field");
    marker_length_ = declare_parameter<double>("marker_length_m", 0.06);
    robot_marker_id_ = declare_parameter<int>("robot_marker_id", 0);

    debug_image_topic_ = declare_parameter<std::string>("debug_image_topic", "/aruco/debug_image");
    detections_text_topic_ = declare_parameter<std::string>("detections_text_topic", "/aruco/detections_text");
    detected_markers_topic_ = declare_parameter<std::string>("detected_markers_topic", "/aruco/detected_markers_field");

    omnibot_cv::WorldPoseTracker::Params params;
    params.smoothing_alpha = declare_parameter<double>("world_pose_smoothing_alpha", 0.2);
    params.hold_frames = declare_parameter<int>("world_pose_hold_frames", 12);
    params.min_markers = declare_parameter<int>("world_pose_min_markers", 2);
    params.max_reproj_error_px = declare_parameter<double>("world_pose_max_reproj_error_px", 8.0);
    tracker_.setParams(params);

    const auto dict_name = declare_parameter<std::string>("aruco_dictionary", "DICT_4X4_50");
    dictionary_ = cv::aruco::getPredefinedDictionary(omnibot_cv::dictionaryFromName(dict_name));

    const auto ids = declare_parameter<std::vector<int64_t>>("marker_ids", {0});
    const auto xs = declare_parameter<std::vector<double>>("marker_x_m", {0.0});
    const auto ys = declare_parameter<std::vector<double>>("marker_y_m", {0.0});
    const auto yaws = declare_parameter<std::vector<double>>("marker_yaw_rad", {0.0});
    loadFieldMap(ids, xs, ys, yaws);

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_, rclcpp::SensorDataQoS(), std::bind(&ArucoPoseNode::onImage, this, std::placeholders::_1));
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, rclcpp::QoS(10), std::bind(&ArucoPoseNode::onInfo, this, std::placeholders::_1));

    markers_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/aruco/marker_poses", 10);
    robot_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/aruco/robot_pose", 10);
    detected_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(detected_markers_topic_, 10);
    debug_image_pub_ = create_publisher<sensor_msgs::msg::Image>(debug_image_topic_, rclcpp::SensorDataQoS());
    detections_text_pub_ = create_publisher<std_msgs::msg::String>(detections_text_topic_, 10);
  }

private:
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
    } else {
      return;
    }

    distortion_ = msg->d.empty() ? cv::Mat::zeros(1, 5, CV_64F) : cv::Mat(msg->d).clone();
    has_camera_info_ = true;
  }

  void onImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception &) {
      return;
    }

    tracker_.beginFrame();

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(cv_ptr->image, dictionary_, corners, ids);

    if (ids.empty()) {
      tracker_.markMissed();
      publishDetectionsText("no_markers");
      publishDebugImage(msg->header, cv_ptr->image);
      return;
    }

    cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);

    if (!has_camera_info_) {
      tracker_.markMissed();
      publishDetectionsText("markers_detected; world_origin=unavailable(camera_info_missing)");
      publishDebugImage(msg->header, cv_ptr->image);
      return;
    }

    std::vector<cv::Vec3d> rvecs;
    std::vector<cv::Vec3d> tvecs;
    cv::aruco::estimatePoseSingleMarkers(corners, marker_length_, camera_matrix_, distortion_, rvecs, tvecs);

    geometry_msgs::msg::PoseArray marker_poses;
    marker_poses.header.stamp = now();
    marker_poses.header.frame_id = msg->header.frame_id;

    std::vector<omnibot_cv::WorldPoseTracker::PoseCandidate> candidates;
    candidates.reserve(ids.size());
    const auto local_marker_corners = markerCornersLocal(marker_length_);

    for (size_t i = 0; i < ids.size(); ++i) {
      marker_poses.poses.push_back(omnibot_cv::toPose(rvecs[i], tvecs[i]));
      cv::drawFrameAxes(cv_ptr->image, camera_matrix_, distortion_, rvecs[i], tvecs[i], static_cast<float>(marker_length_ * 0.5));

      const auto c = 0.25F * (corners[i][0] + corners[i][1] + corners[i][2] + corners[i][3]);
      cv::putText(cv_ptr->image,
                  "ID " + std::to_string(ids[i]),
                  cv::Point(static_cast<int>(c.x) - 24, static_cast<int>(c.y) - 28),
                  cv::FONT_HERSHEY_SIMPLEX,
                  1.0,
                  cv::Scalar(0, 255, 255),
                  2,
                  cv::LINE_AA);

      const auto it = field_markers_.find(ids[i]);
      if (it != field_markers_.end()) {
        omnibot_cv::WorldPoseTracker::PoseCandidate candidate;
        const auto tf_field_marker = fieldMarkerTransform(it->second);
        const auto tf_camera_marker = omnibot_cv::toTransform(rvecs[i], tvecs[i]);
        candidate.field_to_camera = tf_field_marker * tf_camera_marker.inv();
        candidate.reprojection_rmse_px = omnibot_cv::reprojectionRmse(
            local_marker_corners,
            corners[i],
            camera_matrix_,
            distortion_,
            rvecs[i],
            tvecs[i]);
        if (candidate.reprojection_rmse_px >= 0.0) {
          candidate.weight = 1.0 / (1.0 + candidate.reprojection_rmse_px);
        }
        candidates.push_back(candidate);
      }
    }

    markers_pub_->publish(marker_poses);

    double rmse = -1.0;
    const bool updated = tracker_.updateWithCandidates(candidates, &rmse);
    if (!updated) {
      tracker_.markMissed();
    }

    cv::Matx44d tf_field_camera;
    const bool has_world = tracker_.getPose(tf_field_camera);

    bool has_robot_pose = false;
    size_t field_points_count = 0;
    if (has_world) {
      visualization_msgs::msg::MarkerArray detected_markers;

      for (size_t i = 0; i < ids.size(); ++i) {
        const auto tf_camera_marker = omnibot_cv::toTransform(rvecs[i], tvecs[i]);
        const auto tf_field_marker = tf_field_camera * tf_camera_marker;

        visualization_msgs::msg::Marker point_marker;
        point_marker.header.stamp = msg->header.stamp;
        point_marker.header.frame_id = field_frame_;
        point_marker.ns = "detected_markers_field_points";
        point_marker.id = ids[i];
        point_marker.type = visualization_msgs::msg::Marker::SPHERE;
        point_marker.action = visualization_msgs::msg::Marker::ADD;
        point_marker.pose = omnibot_cv::toPose(tf_field_marker);
        point_marker.scale.x = marker_length_ * 0.30;
        point_marker.scale.y = marker_length_ * 0.30;
        point_marker.scale.z = marker_length_ * 0.30;
        point_marker.color.r = 0.1F;
        point_marker.color.g = 1.0F;
        point_marker.color.b = 0.2F;
        point_marker.color.a = 1.0F;
        point_marker.lifetime.sec = 0;
        point_marker.lifetime.nanosec = 300000000;
        detected_markers.markers.push_back(point_marker);

        visualization_msgs::msg::Marker text_marker;
        text_marker.header.stamp = msg->header.stamp;
        text_marker.header.frame_id = field_frame_;
        text_marker.ns = "detected_markers_field_ids";
        text_marker.id = 100000 + ids[i];
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.pose = point_marker.pose;
        text_marker.pose.position.z += marker_length_ * 0.35;
        text_marker.scale.z = marker_length_ * 0.45;
        text_marker.color.r = 1.0F;
        text_marker.color.g = 1.0F;
        text_marker.color.b = 1.0F;
        text_marker.color.a = 1.0F;
        text_marker.text = std::to_string(ids[i]);
        text_marker.lifetime.sec = 0;
        text_marker.lifetime.nanosec = 300000000;
        detected_markers.markers.push_back(text_marker);
      }
      field_points_count = ids.size();
      detected_markers_pub_->publish(detected_markers);

      const auto tf_camera_field = tf_field_camera.inv();
      cv::Vec3d world_rvec;
      cv::Vec3d world_tvec;
      if (omnibot_cv::transformToRvecTvec(tf_camera_field, world_rvec, world_tvec)) {
        cv::drawFrameAxes(cv_ptr->image, camera_matrix_, distortion_, world_rvec, world_tvec, static_cast<float>(marker_length_ * 1.2));

        std::vector<cv::Point3f> origin_3d{{0.0F, 0.0F, 0.0F}};
        std::vector<cv::Point2f> origin_2d;
        cv::projectPoints(origin_3d, world_rvec, world_tvec, camera_matrix_, distortion_, origin_2d);
        if (!origin_2d.empty()) {
          const cv::Point p(static_cast<int>(origin_2d[0].x), static_cast<int>(origin_2d[0].y));
          cv::circle(cv_ptr->image, p, 7, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
          cv::putText(cv_ptr->image,
                      "WORLD (0,0)",
                      cv::Point(p.x + 10, p.y - 12),
                      cv::FONT_HERSHEY_SIMPLEX,
                      1.0,
                      cv::Scalar(0, 0, 255),
                      2,
                      cv::LINE_AA);
        }
      }

      for (size_t i = 0; i < ids.size(); ++i) {
        if (ids[i] != robot_marker_id_) {
          continue;
        }
        const auto tf_camera_robot = omnibot_cv::toTransform(rvecs[i], tvecs[i]);
        const auto tf_field_robot = tf_field_camera * tf_camera_robot;
        geometry_msgs::msg::PoseStamped robot_pose;
        robot_pose.header.stamp = now();
        robot_pose.header.frame_id = field_frame_;
        robot_pose.pose = omnibot_cv::toPose(tf_field_robot);
        robot_pub_->publish(robot_pose);
        has_robot_pose = true;
        break;
      }
    }

    std::ostringstream text;
    text << "count=" << ids.size();
    text << "; known_markers_seen=" << candidates.size();
    if (has_world) {
      text << (tracker_.isHeld() ? "; world_origin=held" : "; world_origin=visible");
    } else {
      text << "; world_origin=unavailable";
    }
    if (rmse >= 0.0) {
      text << "; pnp_rmse_px=" << rmse;
    }
    text << "; field_points_published=" << field_points_count;
    if (!has_robot_pose) {
      text << "; robot_pose=unavailable(robot_marker_not_seen)";
    }

    publishDetectionsText(text.str());
    publishDebugImage(msg->header, cv_ptr->image);
  }

  static cv::Matx44d fieldMarkerTransform(const omnibot_cv::FieldMarker &m) {
    const double c = std::cos(m.yaw);
    const double s = std::sin(m.yaw);

    cv::Matx44d t = cv::Matx44d::eye();
    t(0, 0) = c;
    t(0, 1) = -s;
    t(1, 0) = s;
    t(1, 1) = c;
    t(0, 3) = m.x;
    t(1, 3) = m.y;
    return t;
  }

  static std::vector<cv::Point3f> markerCornersLocal(double marker_length) {
    const double h = 0.5 * marker_length;
    return {
        cv::Point3f{-static_cast<float>(h), static_cast<float>(h), 0.0F},
        cv::Point3f{static_cast<float>(h), static_cast<float>(h), 0.0F},
        cv::Point3f{static_cast<float>(h), -static_cast<float>(h), 0.0F},
        cv::Point3f{-static_cast<float>(h), -static_cast<float>(h), 0.0F},
    };
  }

  void loadFieldMap(const std::vector<int64_t> &ids,
                    const std::vector<double> &xs,
                    const std::vector<double> &ys,
                    const std::vector<double> &yaws) {
    const size_t n = std::min({ids.size(), xs.size(), ys.size(), yaws.size()});
    for (size_t i = 0; i < n; ++i) {
      field_markers_[static_cast<int>(ids[i])] = omnibot_cv::FieldMarker{xs[i], ys[i], yaws[i]};
    }
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

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string field_frame_;
  std::string debug_image_topic_;
  std::string detections_text_topic_;
  std::string detected_markers_topic_;

  double marker_length_{};
  int robot_marker_id_{};

  bool has_camera_info_{false};
  cv::Mat camera_matrix_;
  cv::Mat distortion_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  std::unordered_map<int, omnibot_cv::FieldMarker> field_markers_;
  omnibot_cv::WorldPoseTracker tracker_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr detected_markers_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detections_text_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoPoseNode>());
  rclcpp::shutdown();
  return 0;
}
