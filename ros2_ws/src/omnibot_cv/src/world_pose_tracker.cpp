#include "omnibot_cv/world_pose_tracker.hpp"

#include "omnibot_cv/aruco_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace {

cv::Vec3d translationFromTransform(const cv::Matx44d &t) {
  return cv::Vec3d(t(0, 3), t(1, 3), t(2, 3));
}

tf2::Quaternion rotationToQuat(const cv::Matx44d &t) {
  tf2::Matrix3x3 basis(
      t(0, 0), t(0, 1), t(0, 2),
      t(1, 0), t(1, 1), t(1, 2),
      t(2, 0), t(2, 1), t(2, 2));
  tf2::Quaternion q;
  basis.getRotation(q);
  q.normalize();
  return q;
}

cv::Matx33d quatToMat(const tf2::Quaternion &q_in) {
  tf2::Quaternion q = q_in;
  q.normalize();
  tf2::Matrix3x3 m(q);
  return cv::Matx33d(
      m[0][0], m[0][1], m[0][2],
      m[1][0], m[1][1], m[1][2],
      m[2][0], m[2][1], m[2][2]);
}

cv::Matx44d makeTransform(const cv::Matx33d &rotation, const cv::Vec3d &translation) {
  cv::Matx44d out = cv::Matx44d::eye();
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      out(r, c) = rotation(r, c);
    }
    out(r, 3) = translation[r];
  }
  return out;
}

double poseDistance(const cv::Matx44d &a, const cv::Matx44d &b, double translation_scale_m, double rotation_scale_rad) {
  const auto ta = translationFromTransform(a);
  const auto tb = translationFromTransform(b);
  const double translation_error = cv::norm(ta - tb);

  auto qa = rotationToQuat(a);
  auto qb = rotationToQuat(b);
  const double dot = std::clamp(std::abs(qa.dot(qb)), 0.0, 1.0);
  const double rotation_error = 2.0 * std::acos(dot);

  const double t_scale = std::max(translation_scale_m, 1e-6);
  const double r_scale = std::max(rotation_scale_rad, 1e-6);
  return translation_error / t_scale + rotation_error / r_scale;
}

double rotationError(const cv::Matx44d &a, const cv::Matx44d &b) {
  auto qa = rotationToQuat(a);
  auto qb = rotationToQuat(b);
  const double dot = std::clamp(std::abs(qa.dot(qb)), 0.0, 1.0);
  return 2.0 * std::acos(dot);
}

}  // namespace

namespace omnibot_cv {

WorldPoseTracker::WorldPoseTracker() = default;

WorldPoseTracker::WorldPoseTracker(const Params &params) : params_(params) {}

void WorldPoseTracker::setParams(const Params &params) {
  params_ = params;
}

void WorldPoseTracker::beginFrame() {
  updated_this_frame_ = false;
}

bool WorldPoseTracker::updateWithCandidates(const std::vector<PoseCandidate> &candidates, double *out_rmse_px) {
  if (out_rmse_px) {
    *out_rmse_px = -1.0;
  }

  std::vector<PoseCandidate> valid;
  valid.reserve(candidates.size());
  for (const auto &candidate : candidates) {
    if (!std::isfinite(candidate.field_to_camera(0, 3)) ||
        !std::isfinite(candidate.field_to_camera(1, 3)) ||
        !std::isfinite(candidate.field_to_camera(2, 3))) {
      continue;
    }
    if (candidate.reprojection_rmse_px >= 0.0 && candidate.reprojection_rmse_px > params_.max_reproj_error_px) {
      continue;
    }
    valid.push_back(candidate);
  }

  if (valid.empty()) {
    return false;
  }

  constexpr double kTranslationScaleM = 0.35;
  constexpr double kRotationScaleRad = 0.65;
  const double translation_gate_m = std::max(0.50, 1.5 * kTranslationScaleM);
  const double rotation_gate_rad = std::max(0.80, 1.5 * kRotationScaleRad);

  size_t seed_index = 0;
  if (has_pose_) {
    double best_score = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < valid.size(); ++i) {
      const double score = poseDistance(valid[i].field_to_camera, field_to_camera_, kTranslationScaleM, kRotationScaleRad);
      if (score < best_score) {
        best_score = score;
        seed_index = i;
      }
    }
  } else if (valid.size() > 1) {
    double best_score = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < valid.size(); ++i) {
      double score = 0.0;
      for (size_t j = 0; j < valid.size(); ++j) {
        if (i == j) {
          continue;
        }
        score += poseDistance(valid[i].field_to_camera, valid[j].field_to_camera, kTranslationScaleM, kRotationScaleRad);
      }
      if (score < best_score) {
        best_score = score;
        seed_index = i;
      }
    }
  }

  std::vector<size_t> inliers;
  inliers.reserve(valid.size());
  const auto &seed = valid[seed_index].field_to_camera;
  for (size_t i = 0; i < valid.size(); ++i) {
    const double translation_error = cv::norm(translationFromTransform(valid[i].field_to_camera) - translationFromTransform(seed));
    const double rotation_error_rad = rotationError(valid[i].field_to_camera, seed);
    if (translation_error <= translation_gate_m && rotation_error_rad <= rotation_gate_rad) {
      inliers.push_back(i);
    }
  }

  if (inliers.empty()) {
    inliers.push_back(seed_index);
  }

  auto fused_translation = cv::Vec3d(0.0, 0.0, 0.0);
  double total_weight = 0.0;
  tf2::Quaternion reference_quat = rotationToQuat(valid[inliers.front()].field_to_camera);
  cv::Vec4d fused_quat_sum(0.0, 0.0, 0.0, 0.0);
  double fused_rmse_px = 0.0;
  bool has_rmse = false;

  for (size_t idx : inliers) {
    const auto &candidate = valid[idx];
    double weight = candidate.weight;
    if (!std::isfinite(weight) || weight <= 0.0) {
      weight = 1.0;
    }
    if (candidate.reprojection_rmse_px >= 0.0) {
      weight /= (1.0 + candidate.reprojection_rmse_px * candidate.reprojection_rmse_px);
      fused_rmse_px += weight * candidate.reprojection_rmse_px;
      has_rmse = true;
    }

    const auto translation = translationFromTransform(candidate.field_to_camera);
    fused_translation += weight * translation;
    total_weight += weight;

    auto q = rotationToQuat(candidate.field_to_camera);
    if (reference_quat.dot(q) < 0.0) {
      q = tf2::Quaternion(-q.x(), -q.y(), -q.z(), -q.w());
    }
    fused_quat_sum += cv::Vec4d(q.x() * weight, q.y() * weight, q.z() * weight, q.w() * weight);
  }

  if (total_weight <= 0.0) {
    return false;
  }

  fused_translation /= total_weight;
  const cv::Vec4d fused_quat_vec = fused_quat_sum / total_weight;
  tf2::Quaternion fused_quat(fused_quat_vec[0], fused_quat_vec[1], fused_quat_vec[2], fused_quat_vec[3]);
  if (fused_quat.length2() <= 0.0) {
    return false;
  }
  fused_quat.normalize();

  const cv::Matx44d fused = makeTransform(quatToMat(fused_quat), fused_translation);

  if (!has_pose_) {
    field_to_camera_ = fused;
    has_pose_ = true;
  } else {
    field_to_camera_ = blendTransform(
        field_to_camera_,
        fused,
        std::clamp(params_.smoothing_alpha, 0.0, 1.0));
  }

  if (out_rmse_px && has_rmse) {
    *out_rmse_px = fused_rmse_px / total_weight;
  }

  missed_frames_ = 0;
  updated_this_frame_ = true;
  return true;
}

bool WorldPoseTracker::updateWithPnp(const std::vector<cv::Point3f> &object_points,
                                     const std::vector<cv::Point2f> &image_points,
                                     const cv::Mat &camera_matrix,
                                     const cv::Mat &distortion,
                                     double *out_rmse_px) {
  if (out_rmse_px) {
    *out_rmse_px = -1.0;
  }

  if (object_points.size() != image_points.size() || object_points.size() < 4) {
    return false;
  }

  cv::Vec3d rvec_camera_field;
  cv::Vec3d tvec_camera_field;
  std::vector<int> inliers;

  bool ok = cv::solvePnPRansac(
      object_points,
      image_points,
      camera_matrix,
      distortion,
      rvec_camera_field,
      tvec_camera_field,
      false,
      150,
      4.0f,
      0.995,
      inliers,
      cv::SOLVEPNP_ITERATIVE);

  if (ok) {
    ok = cv::solvePnP(
        object_points,
        image_points,
        camera_matrix,
        distortion,
        rvec_camera_field,
        tvec_camera_field,
        true,
        cv::SOLVEPNP_ITERATIVE);
  }

  if (!ok) {
    return false;
  }

  const double rmse = reprojectionRmse(
      object_points,
      image_points,
      camera_matrix,
      distortion,
      rvec_camera_field,
      tvec_camera_field);

  if (rmse < 0.0 || rmse > params_.max_reproj_error_px) {
    return false;
  }

  const auto tf_camera_field = toTransform(rvec_camera_field, tvec_camera_field);
  PoseCandidate candidate;
  candidate.field_to_camera = tf_camera_field.inv();
  candidate.reprojection_rmse_px = rmse;
  candidate.weight = 1.0;

  const bool updated = updateWithCandidates({candidate}, out_rmse_px);
  if (!updated && out_rmse_px) {
    *out_rmse_px = rmse;
  }
  return updated;
}

void WorldPoseTracker::markMissed() {
  if (has_pose_ && !updated_this_frame_) {
    ++missed_frames_;
  }
}

bool WorldPoseTracker::getPose(cv::Matx44d &field_to_camera) const {
  if (!has_pose_ || missed_frames_ > params_.hold_frames) {
    return false;
  }
  field_to_camera = field_to_camera_;
  return true;
}

bool WorldPoseTracker::wasUpdatedThisFrame() const {
  return updated_this_frame_;
}

bool WorldPoseTracker::isHeld() const {
  return has_pose_ && !updated_this_frame_ && missed_frames_ <= params_.hold_frames;
}

}  // namespace omnibot_cv
