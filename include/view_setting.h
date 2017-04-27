#pragma once

#include "geometry.h"

#include <vector>

#include <Eigen/Dense>

namespace view_setting {

// Each view will have a render view_setting associated with it.
struct ViewSetting {
  ViewSetting() {}
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

// Generate uniform randle samples.
void GenerateUniformRandomSamples(int num_samples,
                                  std::vector<Eigen::Vector3d> *samples);

// Generate cylindrical samples in the XZ plane.
void GenerateCylindricalSamples(int num_samples,
                                std::vector<Eigen::Vector3d> *samples);

// Generate samples using the vertices of an icosahedron.
void GenerateIcosahedronSamples(std::vector<Eigen::Vector3d> *samples);

// Generate a batch of render view_settings.
// Using the input mesh, there will be n view_settings generated.
void GenerateViewSettings(const geometry::Mesh *mesh,
                          RenderSampleType sample_type, int num_samples,
                          ViewSettings *view_settings);
}  // namespace view_setting
