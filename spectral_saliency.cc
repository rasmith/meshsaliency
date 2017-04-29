#include "common.h"
#include "geometry.h"
#include "geometry_processing.h"
#include "logger.h"
#include "view_setting.h"

#include <cassert>
#include <functional>
#include <iostream>
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

std::string output_directory;
bool run_view_sampling = false;

void RgbToLuma(const Eigen::Vector3d &rgb, double &luma) {
  luma = 0.299 * rgb(0) + 0.587 * rgb(1) + 0.1146 * rgb(2);
}

void SaveViewerSettings(const igl::viewer::Viewer &viewer,
                        const std::string &file_name) {
  std::ofstream out(file_name);
  if (!out.good()) {
    LOG(DEBUG) << "Bad file handle.\n";
    exit(0);
  }
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

// This is the Viewer initialization callback.
bool ViewerInit(igl::viewer::Viewer &viewer,
                const ViewSettings *view_settings) {
  LOG(DEBUG) << "ViewerInit\n";
  if (view_settings->which >= view_settings->view_setting_list.size())
    return false;
  const ViewSetting *view_setting =
      &view_settings->view_setting_list[view_settings->which];
  if (!run_view_sampling) {
    window_width = view_setting->width;
    window_height = view_setting->height;
  }
  // LOG(DEBUG) << "view_setting->which = " << view_settings->which << "\n"
  //<< " view_setting->width = " << view_setting->width << "\n"
  //<< " view_setting->height = " << view_setting->height << "\n"
  //<< " window_height = " << window_height << "\n"
  //<< " window_width = " << window_width << "\n";
  // exit(0);
  // Set the window size and viewport before drawing begins.
  glfwSetWindowSize(viewer.window, window_width, window_height);
  glViewport(0, 0, window_width, window_height);
  return false;
}

// This is a pre render callback for the Viewr class.
bool ViewerPreDraw(igl::viewer::Viewer &viewer, const Mesh *mesh,
                   const ViewSettings *view_settings) {
  LOG(DEBUG) << "ViewerPreDraw\n";
  if (view_settings->which >= view_settings->view_setting_list.size())
    return false;
  const ViewSetting *view_setting =
      &view_settings->view_setting_list[view_settings->which];
  viewer.core.camera_eye = view_setting->eye.cast<float>();
  //viewer.core.light_position = viewer.core.camera_eye;
  viewer.core.lighting_factor = 0.0;
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
                    ViewSettings *view_settings) {
  LOG(DEBUG) << "ViewerPostDraw\n";
  if (view_settings->is_sampled) {
    // If no more views to render, make sure the Viewer class exits.
    if (view_settings->which >= view_settings->view_setting_list.size()) {
      // This tells GLFW to close, the main render loop in Viewer will halt.
      glfwSetWindowShouldClose(viewer.window, true);
      // Push an empty event to make the Viewer class process another event.
      glfwPostEmptyEvent();
      return false;
    }
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

  if (view_settings->is_sampled) {
    std::string which_str = std::to_string(view_settings->which);
    std::string png_file_path =
        output_directory + "/sal/out" + which_str + ".png";
    LOG(DEBUG) << "Saving PNG file to :'" << png_file_path << "\n";
    // Save it to a PNG.
    igl::png::writePNG(R, G, B, A, png_file_path);
    //std::string cfg_file_path =
        //output_directory + "/cfg/out" + which_str + ".cfg";
    //LOG(DEBUG) << "Saving CFG file to :'" << cfg_file_path << "\n";
    //// Save the camera settings.
    //SaveViewerSettings(viewer, cfg_file_path);
    ++view_settings->which;

    // Post an empty event so igl::viewer::Viewer will continue to pump events
    // and render the next view.
    glfwPostEmptyEvent();
  }
  return false;
}

// Here, the viewer is launched and the views are rendered.
void RunViewer(Mesh &mesh, ViewSettings *view_settings) {
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
      std::bind(ViewerInit, std::placeholders::_1, view_settings);
  viewer.callback_pre_draw =
      std::bind(ViewerPreDraw, std::placeholders::_1, &mesh,
                view_settings);  // Bind callback.
  viewer.callback_post_draw =
      std::bind(ViewerPostDraw, std::placeholders::_1, &mesh,
                view_settings);  // Bind callback.
  viewer.launch(true, false);
}

// Usage:
// ./spectral_saliency <model_path>
//  model_path - path to the model file to render [.OFF]
int main(int argc, char *argv[]) {
  int argv_index = 0;

  // Get the file path to load (supports .OFF right now).
  std::string model_path(argv[++argv_index]);

  RenderSampleType sample_type;
  int num_samples;

  LOG(DEBUG) << "model_path = " << model_path << "\n";
  if (argc > 2) {
    run_view_sampling = true;

    // Get the output directory.
    output_directory = std::string(argv[++argv_index]);
    // Get the sample type.
    sample_type = static_cast<RenderSampleType>(
        std::stoi(std::string(argv[++argv_index])));

    num_samples = std::stoi(std::string(argv[++argv_index]));

    // Get the window width and height and save it.
    window_width = std::stoi(std::string(argv[++argv_index]));
    window_height = std::stoi(std::string(argv[++argv_index]));
    LOG(DEBUG) << "output_directory=  " << output_directory << "\n"
               << "sample_type = " << sample_type << "\n"
               << "window_height = " << window_height << "\n"
               << "window_width = " << window_width << "\n";
  }

  // Make a mesh struct.
  Mesh mesh;
  mesh.path = model_path;

  LOG(DEBUG) << "Loading mesh: '" << model_path << "'\n";

  // Load a triangular mesh format.
  igl::read_triangle_mesh(mesh.path, mesh.vertices, mesh.faces, mesh.directory,
                          mesh.basename, mesh.extension, mesh.filename);

  // Get the minimum and maximum extents.
  mesh.bounds.lo = mesh.vertices.colwise().minCoeff();
  mesh.bounds.hi = mesh.vertices.colwise().maxCoeff();
  Eigen::Vector3d center = 0.5 * (mesh.bounds.hi + mesh.bounds.lo);
  double extent = (mesh.bounds.hi - mesh.bounds.lo).norm();

  LOG(DEBUG) << "Compute stats: bounds.min = " << mesh.bounds.lo
             << " bounds.max = " << mesh.bounds.hi << " extent = " << extent
             << "\n";

  // Compute the saliency.
  int max_faces = 1000;
  int num_scales = 5;
  double scale_base = 0.002 * extent;
  scale_base *= scale_base;
  LOG(DEBUG) << "scale_base = " << scale_base << "\n";
  double scales[5] = {1.0 * scale_base, 2.0 * scale_base, 3.0 * scale_base,
                      4.0 * scale_base, 5.0 * scale_base};
  Eigen::VectorXd saliency(mesh.vertices.rows());

  ComputeMultiScaleSaliency(mesh, max_faces, scales, num_scales, saliency);
  // LOG(DEBUG) << "Compute jet colors.\n";
  mesh.colors.resize(mesh.vertices.rows(), 3);
  double min_saliency = saliency.minCoeff();
  double max_saliency = saliency.maxCoeff();
  //  igl::jet(saliency, saliency.minCoeff(), saliency.maxCoeff(), mesh.colors);
  for (int i = 0; i < mesh.colors.rows(); ++i) {
    double value = 
        (saliency(i) - min_saliency) / (max_saliency - min_saliency);
    mesh.colors.row(i) = Eigen::Vector3d(value, value, value);
  }
  LOG(DEBUG) << "#mesh.colors() = " << mesh.colors.rows() << "\n";

  LOG(DEBUG) << "Normalize mesh.\n";

  // Resize mesh.
  for (int i = 0; i < mesh.vertices.rows(); ++i) {
    mesh.vertices.row(i) -= center;
    mesh.vertices.row(i) /= extent;
  }
  mesh.center = Eigen::Vector3d(0.0, 0.0, 0.0);
  mesh.bounds.hi /= extent;
  mesh.bounds.lo /= extent;

  ViewSettings view_settings;
  ViewSetting view_setting =
      ViewSetting(window_width, window_height, Eigen::Vector3d(0.0, 0.0, 3.0),
                  Eigen::Vector3d(0.0, 1.0, 0.0), false, 0.0001, 100, 45.0,
                  Eigen::Vector3d(0.0, 0.0, 0.0));
  view_settings.is_sampled = run_view_sampling;
  if (run_view_sampling) {
    // Setup a render view_settings.
    LOG(DEBUG) << "GenerateViewSettings\n";
    GenerateViewSettings(&mesh, sample_type, num_samples, window_width,
                         window_height, &view_settings);
  } else {
    view_settings.view_setting_list.push_back(view_setting);
    view_settings.which = 0;
  }

  RunViewer(mesh, &view_settings);
  return 0;
}
