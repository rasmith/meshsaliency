#include "common.h"
#include "geometry.h"
#include "logger.h"
#include "view_setting.h"

#include <functional>
#include <string>
#include <vector>

#define IGL_VIEWER_VIEWER_QUIET

#include <igl/look_at.h>
#include <igl/png/writePNG.h>
#include <igl/qslim.h>
#include <igl/readOFF.h>
#include <igl/read_triangle_mesh.h>
#include <igl/viewer/Viewer.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/point_types.h>

using geometry::BoundingBox;
using geometry::Mesh;
using std::placeholders::_1;
using view_setting::RenderSampleType;
using view_setting::ViewSetting;
using view_setting::ViewSettings;

int window_width = 256;
int window_height = 256;

std::string input_directory;

typedef pcl::PointXYZ PclPoint;
typedef pcl::filters::GaussianKernel<PclPoint, PclPoint> PclGaussianKernel;
typedef pcl::search::KdTree<PclPoint> PclKdtree;
typedef pcl::PointCloud<PclPoint> PclPointCloud;

Eigen::Vector3d PclPointToEigen(const PclPoint &point) {
  return Eigen::Vector3d(point.x, point.y, point.z);
}

void ComputeMeshGaussian(const Mesh &mesh, const double *scales, int num_scales,
                         Mesh **output) {
  // Compute Gaussian filter.  Use PCL for this, since libigl sucks.
  PclPointCloud::Ptr input_cloud(new PclPointCloud());
  PclPointCloud::Ptr output_cloud(new PclPointCloud());
  // Search tree for this.  Really don't have to code.
  PclKdtree::Ptr tree(new PclKdtree());
  // Setup the kernel.
  PclGaussianKernel::Ptr kernel(new PclGaussianKernel());
  // Setup the convolution.
  pcl::filters::Convolution3D<PclPoint, PclPoint, PclGaussianKernel>
      convolution;
  // OK set the input cloud and search surface.
  kernel->setInputCloud(input_cloud);
  convolution.setKernel(*kernel);
  convolution.setSearchMethod(tree);
  Mesh *current_mesh;
  for (int i = 0; i < num_scales; ++i) {
    current_mesh = output[i];
    convolution.setRadiusSearch(scales[i]);
    convolution.convolve(*output_cloud);
    for (int j = 0; j < current_mesh->vertices.rows(); ++j)
      current_mesh->vertices.row(j) = PclPointToEigen(output_cloud->points[i]);
  }
}

void ComputeModifiedMeshGaussian(const Mesh &mesh, const double *scales,
                                 int num_scales, Mesh **output) {
  // Compute Gaussian filter.  Use PCL for this, since libigl sucks.
  PclPointCloud::Ptr input_cloud(new PclPointCloud());
  PclPointCloud::Ptr output_cloud(new PclPointCloud());
  // Search tree for this.  Really don't have to code.
  PclKdtree::Ptr tree(new PclKdtree());
  // TODO: Compute the gaussian of each vertex based on its 1-ring neighbors.
}

// Outputs:
//   U  #U by dim list of output vertex posistions (can be same ref as V)
//   G  #G by 3 list of output face indices into U (can be same ref as G)
//   J  #G list of indices into F of birth face
//   I  #U list of indices into V of birth vertices
void ComputeSaliency(const Mesh &mesh, double downsample_factor, double *scales,
                     int num_scales) {
  Eigen::MatrixXd output_vertices;
  Eigen::MatrixXi output_faces;
  Eigen::VectorXi birth_faces;
  Eigen::VectorXi birth_vertices;
  igl::qslim(mesh.vertices, mesh.faces,
             ceil(downsample_factor * mesh.faces.rows()), output_vertices,
             output_faces, birth_faces, birth_vertices);
  std::vector<Mesh *> gaussian_smoothed_meshes(num_scales);
  std::vector<Mesh *> modified_gaussian_smoothed_meshes(num_scales);
  for (int i = 0; i < num_scales; ++i) {
    gaussian_smoothed_meshes[i] = new Mesh;
    modified_gaussian_smoothed_meshes[i] = new Mesh;
  }
  ComputeMeshGaussian(mesh, scales, num_scales, &gaussian_smoothed_meshes[0]);
  ComputeModifiedMeshGaussian(mesh, scales, num_scales,
                              &modified_gaussian_smoothed_meshes[0]);
}

// This is the Viewer initialization callback.
bool ViewerInit(igl::viewer::Viewer &viewer, const ViewSetting *view_setting) {
  window_width = view_setting->width;
  window_height = view_setting->height;
  // Set the window size and viewport before drawing begins.
  glfwSetWindowSize(viewer.window, window_width, window_height);
  glViewport(0, 0, window_width, window_height);
  return false;
}

// This is a pre render callback for the Viewr class.
bool ViewerPreDraw(igl::viewer::Viewer &viewer, const Mesh *mesh,
                   const ViewSetting *view_setting) {
  viewer.core.camera_eye = view_setting->eye.cast<float>();
  viewer.core.camera_up = view_setting->up.cast<float>();
  viewer.core.orthographic = view_setting->orthographic;
  viewer.core.camera_dnear = view_setting->near;
  viewer.core.camera_dfar = view_setting->far;
  viewer.core.camera_view_angle = view_setting->view_angle;
  viewer.core.camera_center = view_setting->camera_center.cast<float>();
  return false;
}

// This callback will run until all view_settinged views have been rendered.
bool ViewerPostDraw(igl::viewer::Viewer &viewer, const Mesh *mesh,
                    const ViewSetting *view_setting) {
  // Allocate temporary buffers.
  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R(window_width,
                                                                 window_height);
  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> G(window_width,
                                                                 window_height);
  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> B(window_width,
                                                                 window_height);
  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> A(window_width,
                                                                 window_height);

  viewer.core.draw_buffer(viewer.data, viewer.opengl, false, R, G, B, A);

  // Post an empty event so igl::viewer::Viewer will continue to pump events
  // and render the next view.
  glfwPostEmptyEvent();
  return false;
}

// Here, the viewer is launched and the views are rendered.
void RunViewer(Mesh &mesh, const ViewSetting *view_setting) {
  // Plot the mesh.
  igl::viewer::Viewer viewer;                       // Create a viewer.
  viewer.data.set_mesh(mesh.vertices, mesh.faces);  // Set mesh data.
  viewer.core.show_lines = false;
  viewer.callback_init =
      std::bind(ViewerInit, std::placeholders::_1, view_setting);
  viewer.callback_pre_draw =
      std::bind(ViewerPreDraw, std::placeholders::_1, &mesh,
                view_setting);  // Bind callback.
  viewer.callback_post_draw =
      std::bind(ViewerPostDraw, std::placeholders::_1, &mesh,
                view_setting);  // Bind callback.
  viewer.launch(true, false);
}

// Usage:
// ./spectral_saliency <model_path>
//  model_path - path to the model file to render [.OFF]
int main(int argc, char *argv[]) {
  int argv_index = 0;

  // Get the file path to load (supports .OFF right now).
  std::string model_path(argv[++argv_index]);

  // Make a mesh struct.
  Mesh mesh;
  mesh.path = model_path;

  // Load a triangular mesh format.
  igl::read_triangle_mesh(mesh.path, mesh.vertices, mesh.faces, mesh.directory,
                          mesh.basename, mesh.extension, mesh.filename);

  // Get the minimum and maximum extents.
  mesh.bounds.lo = mesh.vertices.colwise().minCoeff();
  mesh.bounds.hi = mesh.vertices.colwise().maxCoeff();
  Eigen::Vector3d center = 0.5 * (mesh.bounds.hi + mesh.bounds.lo);

  // Resize mesh.
  double extent = (mesh.bounds.hi - mesh.bounds.lo).norm();
  for (int i = 0; i < mesh.vertices.rows(); ++i) {
    mesh.vertices.row(i) -= center;
    mesh.vertices.row(i) /= extent;
  }
  mesh.center = Eigen::Vector3d(0.0, 0.0, 0.0);
  mesh.bounds.hi /= extent;
  mesh.bounds.lo /= extent;

  int num_scales = 5;
  double scale_base = 0.02 * extent;
  double scales[5] = {1.0 * scale_base, 2.0 * scale_base, 3.0 * scale_base,
                      4.0 * scale_base, 5.0 * scale_base};
  ComputeSaliency(mesh, 1.0, scales, num_scales);

  ViewSetting view_setting =
      ViewSetting(window_width, window_height, Eigen::Vector3d(0.0, 0.0, 3.0),
                  Eigen::Vector3d(0.0, 1.0, 0.0), false, 0.0001, 100, 45.0,
                  Eigen::Vector3d(0.0, 0.0, 0.0));
  RunViewer(mesh, &view_setting);
  return 0;
}
