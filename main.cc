#include <functional>
#include <string>

#include <igl/png/writePNG.h>
#include <igl/readOFF.h>
#include <igl/viewer/Viewer.h>

Eigen::MatrixXd V;
Eigen::MatrixXi F;

using std::placeholders::_1;

struct RenderRequest {
};

struct RenderRequests {
  std::vector<RenderRequest> request_list;
  int current_request;
};

void generate_render_requests(RenderRequests* requests) {
  requests->current_request = 0;
}

bool render_to_png(igl::viewer::Viewer& viewer, RenderRequests* requests) {
  if (requests->current_request < requests->request_list.size()) {
    // Allocate temporary buffers
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R(1280, 800);
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> G(1280, 800);
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> B(1280, 800);
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> A(1280, 800);

    // Draw the scene in the buffers
    viewer.core.draw_buffer(viewer.data, viewer.opengl, false, R, G, B, A);

    // Save it to a PNG
    igl::png::writePNG(R, G, B, A,
		       "out" + std::to_string(requests->current_request) + ".png");
    ++requests->current_request;
  }
  return true;
}

int main(int argc, char* argv[]) {
  std::string model_path(argv[1]);

  // Load a mesh in OFF format.
  igl::readOFF(model_path, V, F);

  // Setup a render requests.
  RenderRequests render_requests;
  generate_render_requests(&render_requests);

  // Plot the mesh.
  igl::viewer::Viewer viewer;
  viewer.callback_post_draw = std::bind(render_to_png, _1, &render_requests);
  viewer.data.set_mesh(V, F);
  viewer.launch();
}
