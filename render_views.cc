#include "geometry.h"
#include "logger.h"
#include "view_setting.h"

#include <functional>
#include <string>

#define IGL_VIEWER_VIEWER_QUIET

#include <igl/look_at.h>
#include <igl/png/writePNG.h>
#include <igl/read_triangle_mesh.h>
#include <igl/viewer/Viewer.h>


using geometry::BoundingBox;
using geometry::Mesh;
using std::placeholders::_1;
using view_setting::RenderSampleType::kCylinderSample;
using view_setting::RenderSampleType::kIcosahedronSample;
using view_setting::RenderSampleType::kUniformRandomSample;
using view_setting::RenderSampleType;
using view_setting::ViewSetting;
using view_setting::ViewSettings;

int window_width = 256;
int window_height = 256;

std::string output_directory;


// This is the Viewer initialization callback.
bool ViewerInit(igl::viewer::Viewer &viewer) {
  // Set the window size and viewport before drawing begins.
  glfwSetWindowSize(viewer.window, window_width, window_height);
  glViewport(0, 0, window_width, window_height);
  return false;
}

// This is a pre render callback for the Viewr class.
bool ViewerPreDraw(igl::viewer::Viewer &viewer, const Mesh *mesh,
                   ViewSettings *view_settings) {
  if (view_settings->which >= view_settings->view_setting_list.size())
    return false;
  // If we have something to do, then setup the next render.
  ViewSetting *view_setting =
      &view_settings->view_setting_list[view_settings->which];
  viewer.core.camera_eye = view_setting->eye.cast<float>();
  viewer.core.camera_up = view_setting->up.cast<float>();
  viewer.core.camera_dnear = 0.0001f;
  return false;
}

void SaveViewerSettings(const igl::viewer::Viewer &viewer,
                        const std::string &file_name) {
  std::ofstream out(file_name);
  Eigen::IOFormat format(Eigen::FullPrecision, 0, ",", ",");
  out << "width " << window_width << "\n";
  out << "height " << window_height << "\n";
  out << "eye  " << viewer.core.camera_eye.format(format) << "\n";
  out << "up " << viewer.core.camera_up.format(format) << "\n";
  out << "orthographic " << viewer.core.orthographic << "\n";
  out << "near " << viewer.core.camera_dnear << "\n";
  out << "far " << viewer.core.camera_dfar << "\n";
  out << "view_angle " << viewer.core.camera_view_angle << "\n";
  out << "camera_center " << viewer.core.camera_center.format(format) << "\n";
  out.close();
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

  std::string which_str = std::to_string(view_settings->which);
  std::string png_file_path =
      output_directory + "/png/out" + which_str + ".png";
  LOG(DEBUG) << "Saving PNG file to :'" << png_file_path << "\n";
  // Save it to a PNG.
  igl::png::writePNG(R, G, B, A, png_file_path);
  std::string cfg_file_path =
      output_directory + "/cfg/out" + which_str + ".cfg";
  LOG(DEBUG) << "Saving CFG file to :'" << cfg_file_path << "\n";
  // Save the camera settings.
  SaveViewerSettings(viewer, cfg_file_path);
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
  viewer.callback_init = ViewerInit;
  viewer.callback_pre_draw = std::bind(ViewerPreDraw, _1, &mesh,
                                       &view_settings);  // Bind callback.
  viewer.callback_post_draw = std::bind(ViewerPostDraw, _1, &mesh,
                                        &view_settings);  // Bind callback.
  viewer.launch(true, false);
}

// Usage:
// ./render_views <model_path> <output_directory> <sample_type> <num_samples>
//                <width> <height>
//  model_path - path to the model file to render [.OFF]
//  output_directory - path to the directory for rendered views
//  sample_type - type of sampling to use:
//              - 0 : sample the vertices of an icosahedron
//              - 1 : sample the vertices of a cylinder with <num_samples>
//                    samples.
//              - 2 : sample a sphere uniformly at random using <num_samples>
//                    samples.
// num_samples - number of samples to use, only useful for sample_type = 1,2.
// width - output image width
// height - output image height
int main(int argc, char *argv[]) {
  int argv_index = 0;

  // Get the file path to load (supports .OFF right now).
  std::string model_path(argv[++argv_index]);
  LOG(DEBUG) << "model_path =  " << model_path << "\n";

  // Get the output directory.
  output_directory = std::string(argv[++argv_index]);
  LOG(DEBUG) << "output_directory=  " << output_directory << "\n";

  // Get the sample type.
  RenderSampleType sample_type =
      static_cast<RenderSampleType>(std::stoi(std::string(argv[++argv_index])));

  // Get num samples.
  int num_samples = std::stoi(std::string(argv[++argv_index]));

  // Get the window width and height and save it.
  window_width = std::stoi(std::string(argv[++argv_index]));
  window_height = std::stoi(std::string(argv[++argv_index]));

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

  // Setup a render view_settings.
  ViewSettings view_settings;
  GenerateViewSettings(&mesh, sample_type, num_samples, &view_settings);
  RunViewer(mesh, view_settings);
  return 0;
}
