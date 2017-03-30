#pragma once

#include <Eigen/Dense>
#include <vector>

namespace view_setting {

// Each view will have a render view_setting associated with it.
struct ViewSetting {
  ViewSetting() {}
  ViewSetting(int screen_width, int screen_height,
              const Eigen::Vector3d& eye_position,
              const Eigen::Vector3d& up_vector, bool is_orthographic,
              double near_value, double far_value, double angle,
              const Eigen::Vector3d& center)
      : width(screen_width),
        height(screen_height),
        eye(eye_position),
        up(up_vector),
        orthographic(is_orthographic),
        near(near_value),
        far(far_value),
        view_angle(angle),
        camera_center(center) {}
  int width;
  int height;
  Eigen::Vector3d eye;
  Eigen::Vector3d up;
  bool orthographic;
  double near;
  double far;
  double view_angle;
  Eigen::Vector3d camera_center;
};

// Render view_settings are grouped into batches.
struct ViewSettings {
  std::vector<ViewSetting> view_setting_list;
  int which;
};

// Sampling type for render views.
enum RenderSampleType {
  kIcosahedronSample = 0,
  kCylinderSample,
  kUniformRandomSample,
  kGatheredSample,
  kNumRenderSampleTypes
};

}  // namespace view_setting
