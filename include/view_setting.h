#pragma once

#include "geometry.h"

#include <vector>

#include <Eigen/Dense>

namespace view_setting {

// Each view will have a render view_setting associated with it.
struct ViewSetting {
  ViewSetting() {}
  ViewSetting(const ViewSetting &v)
      : height(v.height),
        width(v.width),
        eye(v.eye),
        up(v.up),
        orthographic(v.orthographic),
        near(v.near),
        far(v.far),
        view_angle(v.view_angle),
        camera_center(v.camera_center) {}
  ViewSetting(int screen_width, int screen_height,
              const Eigen::Vector3d &eye_position,
              const Eigen::Vector3d &up_vector, bool is_orthographic,
              double near_value, double far_value, double angle,
              const Eigen::Vector3d &center)
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
  ViewSettings() : view_setting_list(), which(0), is_sampled(false) {}
  std::vector<ViewSetting> view_setting_list;
  int which;
  bool is_sampled;
};

// Sampling type for render views.
enum RenderSampleType {
  kIcosahedronSample = 0,
  kCylinderSample,
  kUniformRandomSample,
  kGatheredSample,
  kNumRenderSampleTypes
};

// Generate uniform randle samples.
void GenerateUniformRandomSamples(int num_samples,
                                  std::vector<Eigen::Vector3d> *samples);

// Generate cylindrical samples in the XZ plane.
void GenerateCylindricalSamples(int num_samples,
                                std::vector<Eigen::Vector3d> *samples);

// Generate samples using the vertices of an icosahedron.
void GenerateIcosahedronSamples(std::vector<Eigen::Vector3d> *samples);

// Generate a batch of render view_settings.
void GenerateViewSettings(const geometry::Mesh *mesh,
                          RenderSampleType sample_type, int num_samples,
                          int width, int height, ViewSettings *view_settings);
}  // namespace view_setting
