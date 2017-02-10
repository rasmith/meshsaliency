#include <functional>
#include <string>

#include <igl/png/writePNG.h>
#include <igl/readOFF.h>
#include <igl/viewer/Viewer.h>

Eigen::MatrixXd V;
Eigen::MatrixXi F;

using std::placeholders::_1;

struct RenderRequest {
  unsigned int count;
  unsigned int limit;
};

bool render_to_png(igl::viewer::Viewer& viewer, RenderRequest* request) {
  if (request->count < request->limit) {
    // Allocate temporary buffers
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R(1280, 800);
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> G(1280, 800);
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> B(1280, 800);
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> A(1280, 800);

    // Draw the scene in the buffers
    viewer.core.draw_buffer(viewer.data, viewer.opengl, false, R, G, B, A);

    // Save it to a PNG
    igl::png::writePNG(R, G, B, A,
		       "out" + std::to_string(request->count) + ".png");
    ++request->count;
  }
  return true;
}

int main(int argc, char* argv[]) {
  std::string model_path(argv[1]);

  // Load a mesh in OFF format.
  igl::readOFF(model_path, V, F);

  // Setup a render request.
  RenderRequest render_request = {0, 3};

  // Plot the mesh.
  igl::viewer::Viewer viewer;
  viewer.callback_post_draw = std::bind(render_to_png, _1, &render_request);
  viewer.data.set_mesh(V, F);
  viewer.launch();
}
