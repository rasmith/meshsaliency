#include "common.h"
#include "geometry.h"
#include "logger.h"
#include "render_request.h"

#include <functional>
#include <string>

#define IGL_VIEWER_VIEWER_QUIET

#include <igl/look_at.h>
#include <igl/png/writePNG.h>
#include <igl/readOFF.h>
#include <igl/viewer/Viewer.h>

using geometry::BoundingBox;
using geometry::Mesh;
using render_request::RenderRequest;
using render_request::RenderRequests;
using render_request::RenderSampleType;
using std::placeholders::_1;

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

template <typename ValueType>
void ReadConfigLine(std::ifstream &ifs, const std::string &field_name,
                    ValueType &field_value) {
  std::string input_field_name, input_field_value;
  ifs >> input_field_name >> input_field_value;
  LOG(DEBUG) << "input_field_name = " << input_field_name;
  LOG(DEBUG) << "field_name= " << field_name;
  CHECK_TRUE(field_name == input_field_name);
  ReadConfigValue<ValueType>(input_field_value, field_value);
}

void ReadCameraConfig(std::ifstream &camera_file, RenderRequest *request) {
  ReadConfigLine<int>(camera_file, "width", request->width);
  ReadConfigLine<int>(camera_file, "height", request->height);
  ReadConfigLine<Eigen::Vector3d>(camera_file, "eye", request->eye);
  ReadConfigLine<Eigen::Vector3d>(camera_file, "up", request->up);
  ReadConfigLine<bool>(camera_file, "orthographic", request->orthographic);
  ReadConfigLine<double>(camera_file, "near", request->near);
  ReadConfigLine<double>(camera_file, "far", request->far);
  ReadConfigLine<double>(camera_file, "view_angle", request->view_angle);
  ReadConfigLine<Eigen::Vector3d>(camera_file, "camera_center",
                                  request->camera_center);
}

void PrintCameraConfig(std::ostream &out, const RenderRequest &request) {
  Eigen::IOFormat format(Eigen::FullPrecision, 0, ",", ",");
  out << "width " << request.width << "\n";
  out << "height " << request.height << "\n";
  out << "eye  " << request.eye.format(format) << "\n";
  out << "up " << request.up.format(format) << "\n";
  out << "orthographic " << request.orthographic << "\n";
  out << "near " << request.near << "\n";
  out << "far " << request.far << "\n";
  out << "view_angle " << request.view_angle << "\n";
  out << "camera_center " << request.camera_center.format(format) << "\n";
}

// Generate a batch of render requests.
// Using the input mesh, there will be n requests generated.
void GenerateRenderRequests(const Mesh *mesh,
                            const std::string &input_directory,
                            RenderRequests *requests) {
  std::string camera_path = input_directory + "/camera0.cfg";
  std::string saliency_path = input_directory + "/saliency0.jpg";
  std::ifstream camera_file(camera_path);
  std::ifstream saliency_file(saliency_path);

  int i = 0;
  while (camera_file.good()) {
    RenderRequest request;
    ReadCameraConfig(camera_file, &request);
    PrintCameraConfig(std::cout, request);
    requests->request_list.push_back(request);

    camera_file.close();
    saliency_file.close();

    camera_path = input_directory + "/camera" + std::to_string(i) + ".cfg";
    saliency_path = input_directory + "/saliency" + std::to_string(i) + ".cfg";

    camera_file.open(camera_path);
    saliency_file.open(saliency_path);
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
                   RenderRequests *requests) {
  if (requests->which >= requests->request_list.size())
    return false;
  // If we have something to do, then setup the next render.
  RenderRequest *request = &requests->request_list[requests->which];
  viewer.core.camera_eye = request->eye.cast<float>();
  viewer.core.camera_up = request->up.cast<float>();
  return false;
}

// This callback will run until all requested views have been rendered.
bool ViewerPostDraw(igl::viewer::Viewer &viewer, const Mesh *mesh,
                    RenderRequests *requests) {
  // If no more views to render, make sure the Viewer class exits.
  if (requests->which >= requests->request_list.size()) {
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
  RenderRequest *request = &requests->request_list[requests->which];
  viewer.core.draw_buffer(viewer.data, viewer.opengl, false, R, G, B, A);

  std::string which_str = std::to_string(requests->which);
  ++requests->which;

  // Post an empty event so igl::viewer::Viewer will continue to pump events
  // and render the next view.
  glfwPostEmptyEvent();
  return false;
}

// Here, the viewer is launched and the views are rendered.
void RunViewer(Mesh &mesh, RenderRequests &render_requests) {
  // Plot the mesh.
  igl::viewer::Viewer viewer;                      // Create a viewer.
  viewer.data.set_mesh(mesh.vertices, mesh.faces); // Set mesh data.
  viewer.core.show_lines = false;
  viewer.callback_init = ViewerInit;
  viewer.callback_pre_draw = std::bind(ViewerPreDraw, _1, &mesh,
                                       &render_requests); // Bind callback.
  viewer.callback_post_draw = std::bind(ViewerPostDraw, _1, &mesh,
                                        &render_requests); // Bind callback.
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
  RenderRequests render_requests;
  LOG(DEBUG) << "Loading render requests\n";
  GenerateRenderRequests(&mesh, input_directory, &render_requests);
  RunViewer(mesh, render_requests);
  return 0;
}
