#include "geometry.h"
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

std::string output_directory;

void gather_samples(std::vector<Eigen::Vector3d>* samples) {}

// Generate a batch of render requests.
// Using the input mesh, there will be n requests generated.
void generate_render_requests(const Mesh* mesh, RenderSampleType sample_type,
                              RenderRequests* requests) {
  // The radius to use.
  double radius = 4.0;

  // Generate samples.
  std::vector<Eigen::Vector3d> samples;
  gather_samples(&samples);

  // Generate render views based on the choice of sampling.
  RenderRequest request;
  requests->request_list.clear();

  for (int i = 0; i < samples.size(); ++i) {
    // Get the eye location.
    request.eye = radius * samples[i];
    // Generate the forward direction.
    request.forward = request.eye.normalized();
    // Generate the up direction
    request.up = Eigen::Vector3d(0.0, 1.0, 0.0);
    // Generate the right direction.
    request.right = request.forward.cross(request.up);
    requests->request_list.push_back(request);
  }
  requests->which = 0;
}

// This is the Viewer initialization callback.
bool viewer_init(igl::viewer::Viewer& viewer) {
  // Set the window size and viewport before drawing begins.
  glfwSetWindowSize(viewer.window, window_width, window_height);
  glViewport(0, 0, window_width, window_height);
  return false;
}

// This is a pre render callback for the Viewr class.
bool viewer_pre_draw(igl::viewer::Viewer& viewer, const Mesh* mesh,
                     RenderRequests* requests) {
  if (requests->which >= requests->request_list.size()) return false;
  // If we have something to do, then setup the next render.
  RenderRequest* request = &requests->request_list[requests->which];
  viewer.core.camera_eye = request->eye.cast<float>();
  viewer.core.camera_up = request->up.cast<float>();
  return false;
}

void save_viewer_settings(const igl::viewer::Viewer& viewer,
                          const std::string& file_name) {
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

// This callback will run until all requested views have been rendered.
bool viewer_post_draw(igl::viewer::Viewer& viewer, const Mesh* mesh,
                      RenderRequests* requests) {
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
  RenderRequest* request = &requests->request_list[requests->which];
  viewer.core.draw_buffer(viewer.data, viewer.opengl, false, R, G, B, A);

  std::string which_str = std::to_string(requests->which);
  // Save it to a PNG.
  igl::png::writePNG(R, G, B, A,
                     output_directory + "/png/out" + which_str + ".png");
  // Save the camera settings.
  save_viewer_settings(viewer,
                       output_directory + "/cfg/camera" + which_str + ".cfg");
  ++requests->which;

  // Post an empty event so igl::viewer::Viewer will continue to pump events
  // and render the next view.
  glfwPostEmptyEvent();
  return false;
}

// Here, the viewer is launched and the views are rendered.
void run_viewer(Mesh& mesh, RenderRequests& render_requests) {
  // Plot the mesh.
  igl::viewer::Viewer viewer;                       // Create a viewer.
  viewer.data.set_mesh(mesh.vertices, mesh.faces);  // Set mesh data.
  viewer.core.show_lines = false;
  viewer.callback_init = viewer_init;
  viewer.callback_pre_draw = std::bind(viewer_pre_draw, _1, &mesh,
                                       &render_requests);  // Bind callback.
  viewer.callback_post_draw = std::bind(viewer_post_draw, _1, &mesh,
                                        &render_requests);  // Bind callback.
  viewer.launch(true, false);
}

// Usage:
// ./backproject_saliency <model_path> <saliency_directory> <output_directory>
//  model_path - path to the model file to render [.OFF]
//  cfg_directory - path to the directory that contains camera parameters
//  saliency_directory - path to the directory of saliency values
int main(int argc, char* argv[]) {
  int argv_index = 0;

  // Get the file path to load (supports .OFF right now).
  std::string model_path(argv[++argv_index]);

  // Get the output directory.
  output_directory = std::string(argv[++argv_index]);

  // Get the sample type.
  RenderSampleType sample_type =
      static_cast<RenderSampleType>(std::stoi(std::string(argv[++argv_index])));

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

  // Setup a render requests.
  RenderRequests render_requests;
  generate_render_requests(&mesh, sample_type, &render_requests);
  run_viewer(mesh, render_requests);
  return 0;
}
