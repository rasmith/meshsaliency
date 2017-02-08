#include <string>

#include <igl/png/writePNG.h>
#include <igl/readOFF.h>
#include <igl/viewer/Viewer.h>

Eigen::MatrixXd V;
Eigen::MatrixXi F;

bool render_to_png(igl::viewer::Viewer& viewer) {
  static bool once = false;
  if (!once) {
    // Allocate temporary buffers
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R(1280, 800);
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> G(1280, 800);
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> B(1280, 800);
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> A(1280, 800);

    // Draw the scene in the buffers
    viewer.core.draw_buffer(viewer.data, viewer.opengl, false, R, G, B, A);

    // Save it to a PNG
    igl::png::writePNG(R, G, B, A, "out.png");
    once = true;
  }
  return true;
}

int main(int argc, char* argv[]) {
  std::string model_path(argv[1]);

  // Load a mesh in OFF format
  igl::readOFF(model_path, V, F);

  // Plot the mesh
  igl::viewer::Viewer viewer;
  viewer.callback_post_draw = render_to_png;
  viewer.data.set_mesh(V, F);
  viewer.launch();
}
