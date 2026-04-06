#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace omnibot_cv {

struct FieldMarker {
  double x{};
  double y{};
  double yaw{};
};

cv::aruco::PREDEFINED_DICTIONARY_NAME dictionaryFromName(const std::string &name);

std::vector<cv::Point3f> markerCornersInField(const FieldMarker &m, double marker_length);

cv::Matx44d toTransform(const cv::Vec3d &rvec, const cv::Vec3d &tvec);

bool transformToRvecTvec(const cv::Matx44d &t, cv::Vec3d &rvec, cv::Vec3d &tvec);

geometry_msgs::msg::Pose toPose(const cv::Matx44d &t);

geometry_msgs::msg::Pose toPose(const cv::Vec3d &rvec, const cv::Vec3d &tvec);

double reprojectionRmse(const std::vector<cv::Point3f> &object_points,
                        const std::vector<cv::Point2f> &image_points,
                        const cv::Mat &camera_matrix,
                        const cv::Mat &distortion,
                        const cv::Vec3d &rvec,
                        const cv::Vec3d &tvec);

cv::Matx44d blendTransform(const cv::Matx44d &prev, const cv::Matx44d &next, double alpha);

}  // namespace omnibot_cv
