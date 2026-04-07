#include "omnibot_cv/aruco_geometry.hpp"

#include <opencv2/calib3d.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>

namespace omnibot_cv {

cv::aruco::PREDEFINED_DICTIONARY_NAME dictionaryFromName(const std::string &name) {
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

std::vector<cv::Point3f> markerCornersInField(const FieldMarker &m, double marker_length) {
  const double h = 0.5 * marker_length;
  const double c = std::cos(m.yaw);
  const double s = std::sin(m.yaw);

  const std::vector<cv::Point2d> local{{-h, +h}, {+h, +h}, {+h, -h}, {-h, -h}};

  std::vector<cv::Point3f> out;
  out.reserve(4);
  for (const auto &p : local) {
    const double x = m.x + c * p.x - s * p.y;
    const double y = m.y + s * p.x + c * p.y;
    out.emplace_back(static_cast<float>(x), static_cast<float>(y), 0.0F);
  }
  return out;
}

cv::Matx44d toTransform(const cv::Vec3d &rvec, const cv::Vec3d &tvec) {
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

bool transformToRvecTvec(const cv::Matx44d &t, cv::Vec3d &rvec, cv::Vec3d &tvec) {
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

geometry_msgs::msg::Pose toPose(const cv::Matx44d &t) {
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

geometry_msgs::msg::Pose toPose(const cv::Vec3d &rvec, const cv::Vec3d &tvec) {
  return toPose(toTransform(rvec, tvec));
}

double reprojectionRmse(const std::vector<cv::Point3f> &object_points,
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

cv::Matx44d blendTransform(const cv::Matx44d &prev, const cv::Matx44d &next, double alpha) {
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

}  // namespace omnibot_cv
