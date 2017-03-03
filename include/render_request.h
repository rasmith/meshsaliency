#pragma once

#include <Eigen/Dense>
#include <vector>

namespace render_request {

// Each view will have a render request associated with it.
struct RenderRequest {
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

// Render requests are grouped into batches.
struct RenderRequests {
  std::vector<RenderRequest> request_list;
  int which;
};

// Sampling type for render views.
enum RenderSampleType {
  kUniformRandomSample,
  kCylinderSample,
  kIcosahedronSample,
  kGatheredSample,
  kNumRenderSampleTypes
};

} // namespace render_request
