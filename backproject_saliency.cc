#include "common.h"
#include "geometry.h"
#include "logger.h"
#include "view_setting.h"

#include <functional>
#include <string>

#define IGL_VIEWER_VIEWER_QUIET

#include <igl/embree/EmbreeIntersector.h>
#include <igl/embree/unproject_onto_mesh.h>
#include <igl/look_at.h>
#include <igl/png/writePNG.h>
#include <igl/readOFF.h>
#include <igl/read_triangle_mesh.h>
#include <igl/viewer/Viewer.h>

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
bool ViewerInit(igl::viewer::Viewer &viewer, ViewSettings *view_settings) {
  ViewSetting *view_setting =
      &view_settings->view_setting_list[view_settings->which];
  window_width = view_setting->width;
  window_height = view_setting->height;
  // Set the window size and viewport before drawing begins.
  glfwSetWindowSize(viewer.window, window_width, window_height);
  glViewport(0, 0, window_width, window_height);
  return false;
}

// This is a pre render callback for the Viewr class.
bool ViewerPreDraw(igl::viewer::Viewer &viewer, const Mesh *mesh,
                   const ViewSetting& view_settings) {
  if (view_settings->which >= view_settings->view_setting_list.size())
    return false;
  // If we have something to do, then setup the next render.
  viewer.core.camera_eye = view_setting.eye.cast<float>();
  viewer.core.camera_up = view_setting.up.cast<float>();
  viewer.core.orthographic = view_setting.orthographic;
  viewer.core.camera_dnear = view_setting.near;
  viewer.core.camera_dfar = view_setting.far;
  viewer.core.camera_view_angle = view_setting.view_angle;
  viewer.core.camera_center = view_setting->camera_center.cast<float>();
  return false;
}

// This callback will run until all view_settinged views have been rendered.
bool ViewerPostDraw(igl::viewer::Viewer &viewer, const Mesh *mesh,
                    ViewSettings *view_settings) {
  // If no more views to render, make sure the Viewer class exits.
  if (view_settings->which >= view_settings->view_setting_list.size()) {
    // This tells GLFW to close, the main render loop in Viewer will halt.
    glfwSetWindowShouldClose(viewer.window, true);
    // Push an empty event to make the Viewer class process another event.
    glfwPostEmptyEvent();
    return false;
  }

  // Allocate temporary buffers.
  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R(window_width,
                                                                 window_height);
  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> G(window_width,
                                                                 window_height);
  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> B(window_width,
                                                                 window_height);
  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> A(window_width,
                                                                 window_height);

  // Draw the scene in the buffers.
  ViewSetting *view_setting =
      &view_settings->view_setting_list[view_settings->which];
  viewer.core.draw_buffer(viewer.data, viewer.opengl, false, R, G, B, A);

  // OK, find visible faces.
  igl::embree::EmbreeIntersector intersector;
  int face_id = -1;
  bool hit = false;
  Eigen::Vector3f barcycentric_coordinates;
  Eigen::Vector2f screen_xy;
  intersector.init(mesh->vertices.cast<float>(), mesh->faces, true);

  std::vector<int> hits;
  LOG(DEBUG) << "Runing Embree intersector...\n";
  // For each pixel.
  for (int y = 0; y < view_setting->height; ++y) {
    for (int x = 0; x < view_setting->width; ++x) {
      // Set the screen position.
      screen_xy = Eigen::Vector2f(x, y);
      // Fire a ray and see  if a face gets hit.
      hit = igl::embree::unproject_onto_mesh(
          screen_xy, mesh->faces, viewer.core.view * viewer.core.model,
          viewer.core.proj, viewer.core.viewport, intersector, face_id,
          barcycentric_coordinates);
      if (hit) hits.push_back(face_id);
    }
  }
  LOG(DEBUG) << "Got " << hits.size() << " hits.\n";

  std::string which_str = std::to_string(view_settings->which);
  ++view_settings->which;

  // Post an empty event so igl::viewer::Viewer will continue to pump events
  // and render the next view.
  glfwPostEmptyEvent();
  return false;
}

// Here, the viewer is launched and the views are rendered.
void RunViewer(Mesh &mesh, ViewSettings &view_settings) {
  // Plot the mesh.
  igl::viewer::Viewer viewer;                       // Create a viewer.
  viewer.data.set_mesh(mesh.vertices, mesh.faces);  // Set mesh data.
  viewer.core.show_lines = false;
  viewer.callback_init = std::bind(ViewerInit, _1, &view_settings);
  viewer.callback_pre_draw = std::bind(ViewerPreDraw, _1, &mesh,
                                       &view_settings);  // Bind callback.
  viewer.callback_post_draw = std::bind(ViewerPostDraw, _1, &mesh,
                                        &view_settings);  // Bind callback.
  viewer.launch(true, false);
}

// Usage:
// ./backproject_saliency <model_path> <input_directory>
//  model_path - path to the model file to render [.OFF]
//  input_directory - path to the input directory
//                  - must contain a 'cfg' subdirectory
//                  - which will contain the collection of camera views
//                  - and 'saliency' directory that will contain the
//                  - collectin of saliency maps.
//                  - Each file 'cameraXXX.cfg' in 'cfg/' will correspond to
//                  - a particular saliency map 'saliencyXXX.cfg' in
//                  - 'saliency/'.
int main(int argc, char *argv[]) {
  int argv_index = 0;

  // Get the file path to load (supports .OFF right now).
  std::string model_path(argv[++argv_index]);

  // Get the camera parameters file path.
  std::string input_directory(argv[++argv_index]);

  // Make a mesh struct.
  Mesh mesh;

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

  // Read in the input directory:
  //    - there will be a triple:
  //    - png/outXXX.png
  //    - cfg/cameraXXX.png
  //    - saliency/outXXX.jpg
  //  Read in each triple and get the:
  //    1) camera parameters used to render the image
  //    2) the saliency map
  ViewSettings view_settings;
  LOG(DEBUG) << "Loading render view_settings\n";
  GenerateViewSettings(&mesh, input_directory, &view_settings);
  RunViewer(mesh, view_settings);
  return 0;
}
