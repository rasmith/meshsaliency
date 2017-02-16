#include <functional>
#include <string>

#define IGL_VIEWER_VIEWER_QUIET

#include <igl/png/writePNG.h>
#include <igl/readOFF.h>
#include <igl/viewer/Viewer.h>

const double kPi = 3.14159265359;
const double kGoldenRatio = 1.61803398875;

using std::placeholders::_1;

int window_width = 256;
int window_height = 256;

std::string output_directory;

// This is for holding axis aligned bounding boxes.
struct BoundingBox {
  Eigen::Vector3d lo;
  Eigen::Vector3d hi;
};

// This is a simple mesh data structure.
struct Mesh {
  Eigen::MatrixXd vertices;
  Eigen::MatrixXi faces;
  BoundingBox bounds;
  Eigen::Vector3d center;
};

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
void generate_uniform_random_samples(int num_samples,
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
void generate_cylindrical_samples(int num_samples,
				  std::vector<Eigen::Vector3d>* samples) {
  double delta = 2.0 * kPi / num_samples, theta = 0.0;
  samples->clear();
  for (int i = 0; i < num_samples; ++i) {
    samples->push_back(Eigen::Vector3d(cos(theta), 0.0, sin(theta)));
    theta += delta;
  }
}

// Generate samples using the vertices of an icosahedron.
void generate_icosahedron_samples(std::vector<Eigen::Vector3d>* samples) {
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
void generate_render_requests(const Mesh* mesh, RenderSampleType sample_type,
			      int num_samples, RenderRequests* requests) {
  // Get the center to use.
  Eigen::Vector3d center = 0.5 * (mesh->bounds.lo + mesh->bounds.hi);
  // Get thr radius to use.
  double radius = 2.5 * (mesh->bounds.hi - mesh->bounds.lo).norm();

  // Generate samples.
  std::vector<Eigen::Vector3d> samples;
  switch (sample_type) {
    case kIcosahedronSample:
      generate_icosahedron_samples(&samples);
      break;
    case kCylinderSample:
      generate_cylindrical_samples(num_samples, &samples);
      break;
    case kUniformRandomSample:
    default:
      generate_uniform_random_samples(num_samples, &samples);
      break;
  }

  // Generate render views based on the choice of sampling.
  RenderRequest request;
  requests->request_list.clear();

  for (int i = 0; i < samples.size(); ++i) {
    // Get the eye location.
    request.eye = center + radius * samples[i];
    // Generate the forward direction.
    request.forward = (center - request.eye).normalized();
    // Generate the up direction
    request.up = Eigen::Vector3d(0.0, 1.0, 0.0);
    request.up = request.up.cross(request.forward).normalized();
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

  // Save it to a PNG.
  igl::png::writePNG(R, G, B, A, output_directory + "/out" +
				     std::to_string(requests->which) + ".png");
  ++requests->which;

  // Post an empty event so igl::viewer::Viewer will continue to pump events
  // and render the next view.
  glfwPostEmptyEvent();
  return false;
}

// Here, the viewer is launched and the views are rendered.
void run_viewer(Mesh& mesh, RenderRequests& render_requests) {
  // Plot the mesh.
  igl::viewer::Viewer viewer;			    // Create a viewer.
  viewer.data.set_mesh(mesh.vertices, mesh.faces);  // Set mesh data.
  viewer.callback_init = viewer_init;
  viewer.callback_pre_draw = std::bind(viewer_pre_draw, _1, &mesh,
				       &render_requests);  // Bind callback.
  viewer.callback_post_draw = std::bind(viewer_post_draw, _1, &mesh,
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

  // Setup a render requests.
  RenderRequests render_requests;
  generate_render_requests(&mesh, sample_type, num_samples, &render_requests);
  run_viewer(mesh, render_requests);
}
