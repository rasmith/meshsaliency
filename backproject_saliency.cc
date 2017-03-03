#include "common.h"
#include "geometry.h"
#include "logger.h"
#include "view_setting.h"

#include <functional>
#include <string>

#define IGL_VIEWER_VIEWER_QUIET

#include <igl/look_at.h>
#include <igl/png/writePNG.h>
#include <igl/readOFF.h>
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

template <typename ValueType>
void ReadConfigValue(const std::string &input_field_value,
                     ValueType &field_value) {
  std::stringstream stream;
  stream.str(input_field_value);
  stream >> field_value;
}

template <>
void ReadConfigValue<Eigen::Vector3d>(const std::string &input_field_value,
                                      Eigen::Vector3d &field_value) {
  std::stringstream stream;
  stream.str(input_field_value);
  char sep = ',';
  stream >> field_value[0] >> sep >> field_value[1] >> sep >> field_value[2];
}

void SplitString(const std::string &input, const std::string delimiter,
                 std::string &left, std::string &right) {
  int position = input.find(delimiter);
  left = input.substr(0, position);
  right = input.substr(position + delimiter.length(),
                       input.length() - delimiter.length() - position + 1);
}

template <typename ValueType>
void ReadConfigLine(std::ifstream &ifs, const std::string &field_name,
                    ValueType &field_value) {
  std::string input_field_name, input_field_value;
  std::string line;
  std::getline(ifs, line);
  SplitString(line, " ", input_field_name, input_field_value);
  LOG(DEBUG) << "input_field_name = " << input_field_name << "\n";
  LOG(DEBUG) << "field_name= " << field_name << "\n";
  CHECK_TRUE(field_name == input_field_name);
  ReadConfigValue<ValueType>(input_field_value, field_value);
}

void ReadCameraConfig(std::ifstream &camera_file, ViewSetting *view_setting) {
  ReadConfigLine<int>(camera_file, "width", view_setting->width);
  ReadConfigLine<int>(camera_file, "height", view_setting->height);
  ReadConfigLine<Eigen::Vector3d>(camera_file, "eye", view_setting->eye);
  ReadConfigLine<Eigen::Vector3d>(camera_file, "up", view_setting->up);
  ReadConfigLine<bool>(camera_file, "orthographic", view_setting->orthographic);
  ReadConfigLine<double>(camera_file, "near", view_setting->near);
  ReadConfigLine<double>(camera_file, "far", view_setting->far);
  ReadConfigLine<double>(camera_file, "view_angle", view_setting->view_angle);
  ReadConfigLine<Eigen::Vector3d>(camera_file, "camera_center",
                                  view_setting->camera_center);
}

void PrintCameraConfig(std::ostream &out, const ViewSetting &view_setting) {
  Eigen::IOFormat format(Eigen::FullPrecision, 0, ",", ",");
  out << "width " << view_setting.width << "\n";
  out << "height " << view_setting.height << "\n";
  out << "eye  " << view_setting.eye.format(format) << "\n";
  out << "up " << view_setting.up.format(format) << "\n";
  out << "orthographic " << view_setting.orthographic << "\n";
  out << "near " << view_setting.near << "\n";
  out << "far " << view_setting.far << "\n";
  out << "view_angle " << view_setting.view_angle << "\n";
  out << "camera_center " << view_setting.camera_center.format(format) << "\n";
}

// Generate a batch of render view_settings.
// Using the input mesh, there will be n view_settings generated.
void GenerateViewSettings(const Mesh *mesh, const std::string &input_directory,
                          ViewSettings *view_settings) {
  std::string camera_path = input_directory + "/camera0.cfg";
  std::string saliency_path = input_directory + "/saliency0.jpg";
  std::ifstream camera_file(camera_path);
  std::ifstream saliency_file(saliency_path);

  int i = 0;
  while (camera_file.good()) {
    LOG(DEBUG) << "camera file = " << camera_path << "\n";
    ViewSetting view_setting;
    ReadCameraConfig(camera_file, &view_setting);
    PrintCameraConfig(std::cout, view_setting);
    view_settings->view_setting_list.push_back(view_setting);

    camera_file.close();
    saliency_file.close();

    camera_path = input_directory + "/camera" + std::to_string(i) + ".cfg";
    saliency_path = input_directory + "/saliency" + std::to_string(i) + ".cfg";

    camera_file.open(camera_path);
    saliency_file.open(saliency_path);
    ++i;
  }
}

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
  viewer.callback_init = ViewerInit;
  viewer.callback_pre_draw = std::bind(ViewerPreDraw, _1, &mesh,
                                       &view_settings);  // Bind callback.
  viewer.callback_post_draw = std::bind(ViewerPostDraw, _1, &mesh,
                                        &view_settings);  // Bind callback.
  viewer.launch(true, false);
}

// Usage:
// ./backproject_saliency <model_path> <saliency_directory> <output_directory>
//  model_path - path to the model file to render [.OFF]
//  cfg_directory - path to the directory that contains camera parameters
//  saliency_directory - path to the directory of saliency values
int main(int argc, char *argv[]) {
  int argv_index = 0;

  // Get the file path to load (supports .OFF right now).
  std::string model_path(argv[++argv_index]);

  // Get the camera parameters file path.
  std::string input_directory(argv[++argv_index]);

  // Make a mesh struct.
  Mesh mesh;

  // Load a mesh in OFF format.
  igl::readOFF(model_path, mesh.vertices, mesh.faces);

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
