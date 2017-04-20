#include "common.h"
#include "geometry.h"
#include "geometry_processing.h"
#include "logger.h"
#include "view_setting.h"

#include <cassert>
#include <functional>
#include <string>
#include <vector>

#define IGL_VIEWER_VIEWER_QUIET

#include <igl/cotmatrix.h>
#include <igl/jet.h>
#include <igl/look_at.h>
#include <igl/png/writePNG.h>
#include <igl/qslim.h>
#include <igl/readOFF.h>
#include <igl/read_triangle_mesh.h>
#include <igl/viewer/Viewer.h>
#include <pcl/common/point_operators.h>
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
  LOG(DEBUG) << "RunViewer: #mesh.colors = " << mesh.colors.rows()
             << " #mesh.vertices = " << mesh.vertices.rows() << "\n";
  if (mesh.colors.rows() > 0 && mesh.colors.rows() == mesh.vertices.rows()) {
    viewer.data.set_colors(mesh.colors);
  }
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

  LOG(DEBUG) << "Loading mesh: '" << model_path << "'\n";

  // Load a triangular mesh format.
  igl::read_triangle_mesh(mesh.path, mesh.vertices, mesh.faces, mesh.directory,
                          mesh.basename, mesh.extension, mesh.filename);

  // Decimate.
  int max_faces = 1000;
  Eigen::VectorXi birth_face_indices;
  Eigen::VectorXi birth_vertex_indices;
  Mesh decimated_mesh;
  decimated_mesh.faces.resize(max_faces, 3);
  decimated_mesh.vertices.resize(mesh.vertices.rows(), 3);
  birth_face_indices.resize(max_faces);
  birth_vertex_indices.resize(mesh.vertices.rows());

  igl::qslim(mesh.vertices, mesh.faces, max_faces, decimated_mesh.vertices,
             decimated_mesh.faces, birth_face_indices, birth_vertex_indices);
  mesh.vertices = decimated_mesh.vertices;
  mesh.faces = decimated_mesh.faces;

  PclPointCloud::Ptr input_cloud(new PclPointCloud());

  // Populate the point cloud.
  for (int i = 0; i < mesh.vertices.rows(); ++i)
    input_cloud->push_back(EigenToPclPoint(mesh.vertices.row(i)));

  PclKdtree::Ptr tree(new PclKdtree());
  tree->setInputCloud(input_cloud);

  // Smooth.
  Eigen::MatrixXd smoothed_vertices(mesh.vertices.rows(), mesh.vertices.cols());
  double sigma =
      0.02 *
      (mesh.vertices.colwise().maxCoeff() - mesh.vertices.colwise().minCoeff())
          .norm();
  for (int i = 0; i < mesh.vertices.rows(); ++i) {
    Eigen::VectorXd result;
    ComputeGaussianPoint(mesh, i, tree, sigma, &result);
    smoothed_vertices.row(i) = result;
  }
  int ccw = 0, not_ccw = 0;
  for (int i = 0; i < mesh.faces.rows(); ++i) {
    Eigen::VectorXi face = mesh.faces.row(i);
    Eigen::Vector3d vi = mesh.vertices.row(face(0));
    Eigen::Vector3d vj = mesh.vertices.row(face(1));
    Eigen::Vector3d vk = mesh.vertices.row(face(2));
    Eigen::Vector3d normal = (vj - vi).cross(vk - vi);
    if (IsCCW(vi, vj, vk))
      ++ccw;
    else
      ++not_ccw;
  }
  LOG(DEBUG) << "Encountered " << ccw << " CCW faces and " << not_ccw
             << " non-CCW faces.\n";
  mesh.vertices = smoothed_vertices;

  // Get the minimum and maximum extents.
  mesh.bounds.lo = mesh.vertices.colwise().minCoeff();
  mesh.bounds.hi = mesh.vertices.colwise().maxCoeff();
  Eigen::Vector3d center = 0.5 * (mesh.bounds.hi + mesh.bounds.lo);
  double extent = (mesh.bounds.hi - mesh.bounds.lo).norm();

  LOG(DEBUG) << "Compute stats: bounds.min = " << mesh.bounds.lo
             << " bounds.max = " << mesh.bounds.hi << " extent = " << extent
             << "\n";

  LOG(DEBUG) << "Normalize mesh.\n";

  ViewSetting view_setting =
      ViewSetting(window_width, window_height, Eigen::Vector3d(0.0, 0.0, 3.0),
                  Eigen::Vector3d(0.0, 1.0, 0.0), false, 0.0001, 100, 45.0,
                  Eigen::Vector3d(0.0, 0.0, 0.0));
  RunViewer(mesh, &view_setting);
  return 0;
}
