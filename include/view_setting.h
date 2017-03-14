#pragma once

#include <Eigen/Dense>
#include <vector>

namespace view_setting {

// Each view will have a render view_setting associated with it.
struct ViewSetting {
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
  kIcosahedronSample=0,
  kCylinderSample,
  kUniformRandomSample,
  kGatheredSample,
  kNumRenderSampleTypes
};

}  // namespace view_setting
