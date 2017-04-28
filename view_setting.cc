#include "view_setting.h"

#include "common.h"
#include "geometry.h"

#include <cmath>
#include <vector>

#include <Eigen/Dense>

namespace view_setting {

// Generate uniform randle samples.
void GenerateUniformRandomSamples(int num_samples,
                                  std::vector<Eigen::Vector3d> *samples) {
  double u = 0.0, theta = 0.0;
  srand(0);
  samples->clear();
  for (int i = 0; i < num_samples; ++i) {
    // Get a random number [-1, 1].
    u = -1.0 + (2.0 * rand()) / RAND_MAX;
    // Get a random number [0, 2 * PI].
    theta = 2.0 * (2.0 * rand()) / RAND_MAX * kPi;
    // Generate the eye location.
    double v = sqrt(1 - u * u);
    samples->push_back(Eigen::Vector3d(v * cos(theta), v * sin(theta), u));
  }
}

// Generate cylindrical samples in the XZ plane.
void GenerateCylindricalSamples(int num_samples,
                                std::vector<Eigen::Vector3d> *samples) {
  double delta = 2.0 * kPi / num_samples, theta = 0.0;
  samples->clear();
  for (int i = 0; i < num_samples; ++i) {
    samples->push_back(Eigen::Vector3d(cos(theta), 0.0, sin(theta)));
    theta += delta;
  }
}

// Generate samples using the vertices of an icosahedron.
void GenerateIcosahedronSamples(std::vector<Eigen::Vector3d> *samples) {
  static const Eigen::Vector3d kIcosahedronVertices[12] = {
      {-1.0, kGoldenRatio, 0.0},  {1.0, kGoldenRatio, 0.0},
      {-1.0, -kGoldenRatio, 0.0}, {1.0, -kGoldenRatio, 0.0},
      {0.0, -1.0, kGoldenRatio},  {0.0, 1.0, kGoldenRatio},
      {0.0, -1.0, -kGoldenRatio}, {0.0, 1.0, -kGoldenRatio},
      {kGoldenRatio, 0.0, -1.0},  {kGoldenRatio, 0.0, 1.0},
      {-kGoldenRatio, 0.0, -1.0}, {-kGoldenRatio, 0.0, 1.0},
  };
  samples->clear();
  for (int i = 0; i < 12; ++i) samples->push_back(kIcosahedronVertices[i]);
}

// Generate a batch of render view_settings.
// Using the input mesh, there will be n view_settings generated.
void GenerateViewSettings(const geometry::Mesh *mesh,
                          RenderSampleType sample_type, int num_samples,
                          int width, int height, ViewSettings *view_settings) {
  // The radius to use.
  double radius = 2;

  // Generate samples.
  std::vector<Eigen::Vector3d> samples;
  switch (sample_type) {
    case kIcosahedronSample:
      GenerateIcosahedronSamples(&samples);
      break;
    case kCylinderSample:
      GenerateCylindricalSamples(num_samples, &samples);
      break;
    case kUniformRandomSample:
    default:
      GenerateUniformRandomSamples(num_samples, &samples);
      break;
  }

  // Generate render views based on the choice of sampling.
  ViewSetting view_setting;
  view_settings->view_setting_list.clear();

  for (int i = 0; i < samples.size(); ++i) {
    view_setting = ViewSetting(width, height, radius * samples[i],
                               Eigen::Vector3d(0.0, 1.0, 0.0), false, 0.0001,
                               100, 45.0, Eigen::Vector3d(0.0, 0.0, 0.0));
    view_settings->view_setting_list.push_back(view_setting);
  }
  view_settings->which = 0;
}

}  // namespace view_setting
