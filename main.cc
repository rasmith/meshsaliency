#include <functional>
#include <string>

#define IGL_VIEWER_VIEWER_QUIET

#include <igl/png/writePNG.h>
#include <igl/readOFF.h>
#include <igl/viewer/Viewer.h>

const double kPi = 3.14159265359;

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

// Generate a batch of render requests.
// Using the input mesh, there will be n requests generated.
void generate_render_requests(const Mesh* mesh, int n,
			      RenderRequests* requests) {
  Eigen::Vector3d center = 0.5 * (mesh->bounds.lo + mesh->bounds.hi);
  double radius = 2.5 * (mesh->bounds.hi - mesh->bounds.lo).norm();
  double u = 0.0, theta = 0.0;
  srand(0);
  RenderRequest request;
  requests->request_list.clear();
  for (int i = 0; i < n; ++i) {
    // Get a random number [-1, 1].
    u = -1.0 + (2.0 * rand()) / RAND_MAX;
    // Get a random number [0, 2 * PI].
    theta = 2.0 * (2.0 * rand()) / RAND_MAX * kPi;
    // Generate the eye location.
    double v = sqrt(1 - u * u);
    request.eye = Eigen::Vector3d(v * cos(theta), v * sin(theta), u);
    request.eye *= radius;
    // Generate the forward direction.
    request.forward = (center - request.eye).normalized();
    // Generate the up direction
    request.up = Eigen::Vector3d(0.0, 1.0, 0.0);
    request.up = request.up.cross(request.forward).normalized();
    // Generate the right direction.
    request.right = request.forward.cross(request.up);
    requests->request_list.push_back(request);
    requests->which = 0;
  }
}

bool viewer_init(igl::viewer::Viewer& viewer) {
  // Set the window size and viewport before drawing begins.
  glfwSetWindowSize(viewer.window, window_width, window_height);
  glViewport(0, 0, window_width, window_height);
  return false;
}

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
  if (requests->which >= requests->request_list.size()) {
    glfwSetWindowShouldClose(viewer.window, true);
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
  // and re-render the next view.
  glfwPostEmptyEvent();
  return false;
}

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

int main(int argc, char* argv[]) {
  // Get the file path to load (supports .OFF right now).
  std::string model_path(argv[1]);

  // Get the output directory.
  output_directory = std::string(argv[2]);

  // Get the window width and height and save it.
  window_width = std::stoi(std::string(argv[3]));
  window_height = std::stoi(std::string(argv[4]));

  // Make a mesh struct.
  Mesh mesh;

  // Load a mesh in OFF format.
  igl::readOFF(model_path, mesh.vertices, mesh.faces);

  // Get the minimum and maximum extents.
  mesh.bounds.lo = mesh.vertices.colwise().minCoeff();
  mesh.bounds.hi = mesh.vertices.colwise().maxCoeff();

  // Setup a render requests.
  RenderRequests render_requests;
  generate_render_requests(&mesh, 25, &render_requests);
  run_viewer(mesh, render_requests);
}
