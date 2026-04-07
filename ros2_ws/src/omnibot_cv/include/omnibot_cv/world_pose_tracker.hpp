#pragma once

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <vector>

namespace omnibot_cv {

class WorldPoseTracker {
public:
  struct PoseCandidate {
    cv::Matx44d field_to_camera{cv::Matx44d::eye()};
    double reprojection_rmse_px{-1.0};
    double weight{1.0};
  };

  struct Params {
    double smoothing_alpha{0.2};
    int hold_frames{12};
    int min_markers{2};
    double max_reproj_error_px{8.0};
  };

  WorldPoseTracker();

  explicit WorldPoseTracker(const Params &params);

  void setParams(const Params &params);

  void beginFrame();

  bool updateWithCandidates(const std::vector<PoseCandidate> &candidates, double *out_rmse_px = nullptr);

  bool updateWithPnp(const std::vector<cv::Point3f> &object_points,
                     const std::vector<cv::Point2f> &image_points,
                     const cv::Mat &camera_matrix,
                     const cv::Mat &distortion,
                     double *out_rmse_px = nullptr);

  void markMissed();

  bool getPose(cv::Matx44d &field_to_camera) const;

  bool wasUpdatedThisFrame() const;

  bool isHeld() const;

private:
  Params params_;
  bool has_pose_{false};
  bool updated_this_frame_{false};
  int missed_frames_{0};
  cv::Matx44d field_to_camera_{cv::Matx44d::eye()};
};

}  // namespace omnibot_cv
