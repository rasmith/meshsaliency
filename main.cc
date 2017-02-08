#include <string>

#include <igl/readOFF.h>
#include <igl/viewer/Viewer.h>

Eigen::MatrixXd V;
Eigen::MatrixXi F;

int main(int argc, char *argv[]) {
  std::string model_path(argv[1]);

  // Load a mesh in OFF format
  igl::readOFF(model_path, V, F);

  // Plot the mesh
  igl::viewer::Viewer viewer;
  viewer.data.set_mesh(V, F);
  viewer.launch();
}
