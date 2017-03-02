#include "geometry.h"
#include "render_request.h"

#include <functional>
#include <string>

#define IGL_VIEWER_VIEWER_QUIET

#include <igl/look_at.h>
#include <igl/png/writePNG.h>
#include <igl/readOFF.h>
#include <igl/viewer/Viewer.h>

const double kPi = 3.14159265359;
const double kGoldenRatio = 1.61803398875;

using geometry::BoundingBox;
using geometry::Mesh;
using std::placeholders::_1;

int window_width = 256;
int window_height = 256;

std::string output_directory;

// Each view will have a render request associated with it.
struct RenderRequest {
  Eigen::Vector3d eye;
  Eigen::Vector3d up;
  Eigen::Vector3d right;
  Eigen::Vector3d forward;
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
  kNumRenderSampleTypes
};

// Generate uniform randle samples.
void GenerateUniformRandomSamples(int num_samples,
				  std::vector<Eigen::Vector3d>* samples) {
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
				std::vector<Eigen::Vector3d>* samples) {
  double delta = 2.0 * kPi / num_samples, theta = 0.0;
  samples->clear();
  for (int i = 0; i < num_samples; ++i) {
    samples->push_back(Eigen::Vector3d(cos(theta), 0.0, sin(theta)));
    theta += delta;
  }
}

// Generate samples using the vertices of an icosahedron.
void GenerateIcosahedronSamples(std::vector<Eigen::Vector3d>* samples) {
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

// Generate a batch of render requests.
// Using the input mesh, there will be n requests generated.
void GenerateRenderRequests(const Mesh* mesh, RenderSampleType sample_type,
			    int num_samples, RenderRequests* requests) {
  // The radius to use.
  double radius = 4.0;

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
bool ViewerInit(igl::viewer::Viewer& viewer) {
  // Set the window size and viewport before drawing begins.
  glfwSetWindowSize(viewer.window, window_width, window_height);
  glViewport(0, 0, window_width, window_height);
  return false;
}

// This is a pre render callback for the Viewr class.
bool ViewerPreDraw(igl::viewer::Viewer& viewer, const Mesh* mesh,
		   RenderRequests* requests) {
  if (requests->which >= requests->request_list.size()) return false;
  // If we have something to do, then setup the next render.
  RenderRequest* request = &requests->request_list[requests->which];
  viewer.core.camera_eye = request->eye.cast<float>();
  viewer.core.camera_up = request->up.cast<float>();
  return false;
}

void SaveViewerSettings(const igl::viewer::Viewer& viewer,
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
bool ViewerPostDraw(igl::viewer::Viewer& viewer, const Mesh* mesh,
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
  SaveViewerSettings(viewer,
		     output_directory + "/cfg/camera" + which_str + ".cfg");
  ++requests->which;

  // Post an empty event so igl::viewer::Viewer will continue to pump events
  // and render the next view.
  glfwPostEmptyEvent();
  return false;
}

// Here, the viewer is launched and the views are rendered.
void RunViewer(Mesh& mesh, RenderRequests& render_requests) {
  // Plot the mesh.
  igl::viewer::Viewer viewer;			    // Create a viewer.
  viewer.data.set_mesh(mesh.vertices, mesh.faces);  // Set mesh data.
  viewer.core.show_lines = false;
  viewer.callback_init = ViewerInit;
  viewer.callback_pre_draw = std::bind(ViewerPreDraw, _1, &mesh,
				       &render_requests);  // Bind callback.
  viewer.callback_post_draw = std::bind(ViewerPostDraw, _1, &mesh,
					&render_requests);  // Bind callback.
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
int main(int argc, char* argv[]) {
  int argv_index = 0;

  // Get the file path to load (supports .OFF right now).
  std::string model_path(argv[++argv_index]);

  // Get the output directory.
  output_directory = std::string(argv[++argv_index]);

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
  GenerateRenderRequests(&mesh, sample_type, num_samples, &render_requests);
  RunViewer(mesh, render_requests);
  return 0;
}
