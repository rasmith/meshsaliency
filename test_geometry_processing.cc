#include "common.h"
#include "geometry.h"
#include "geometry_processing.h"
#include "logger.h"
#include "view_setting.h"

#include <cassert>
#include <complex>
#include <functional>
#include <string>
#include <vector>

#define IGL_VIEWER_VIEWER_QUIET

#include <igl/cotmatrix.h>
#include <igl/jet.h>
#include <igl/look_at.h>
#include <igl/png/writePNG.h>
#include <igl/qslim.h>
#include <igl/readOFF.h>
#include <igl/read_triangle_mesh.h>
#include <igl/viewer/Viewer.h>
#include <pcl/common/point_operators.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/point_types.h>

using geometry::BoundingBox;
using geometry::Mesh;
using std::placeholders::_1;
using view_setting::RenderSampleType;
using view_setting::ViewSetting;
using view_setting::ViewSettings;

int window_width = 256;
int window_height = 256;

std::string input_directory;

// This is the Viewer initialization callback.
bool ViewerInit(igl::viewer::Viewer &viewer, const ViewSetting *view_setting) {
  window_width = view_setting->width;
  window_height = view_setting->height;
  // Set the window size and viewport before drawing begins.
  glfwSetWindowSize(viewer.window, window_width, window_height);
  glViewport(0, 0, window_width, window_height);
  return false;
}

// This is a pre render callback for the Viewr class.
bool ViewerPreDraw(igl::viewer::Viewer &viewer, const Mesh *mesh,
		   const ViewSetting *view_setting) {
  viewer.core.camera_eye = view_setting->eye.cast<float>();
  viewer.core.camera_up = view_setting->up.cast<float>();
  viewer.core.orthographic = view_setting->orthographic;
  viewer.core.camera_dnear = view_setting->near;
  viewer.core.camera_dfar = view_setting->far;
  viewer.core.camera_view_angle = view_setting->view_angle;
  viewer.core.camera_center = view_setting->camera_center.cast<float>();
  return false;
}

// This callback will run until all view_settinged views have been rendered.
bool ViewerPostDraw(igl::viewer::Viewer &viewer, const Mesh *mesh,
		    const ViewSetting *view_setting) {
  // Allocate temporary buffers.
  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R(window_width,
								 window_height);
  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> G(window_width,
								 window_height);
  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> B(window_width,
								 window_height);
  Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> A(window_width,
								 window_height);

  viewer.core.draw_buffer(viewer.data, viewer.opengl, false, R, G, B, A);

  // Post an empty event so igl::viewer::Viewer will continue to pump events
  // and render the next view.
  glfwPostEmptyEvent();
  return false;
}

// Here, the viewer is launched and the views are rendered.
void RunViewer(Mesh &mesh, const ViewSetting *view_setting) {
  // Plot the mesh.
  igl::viewer::Viewer viewer;			    // Create a viewer.
  viewer.data.set_mesh(mesh.vertices, mesh.faces);  // Set mesh data.
  LOG(DEBUG) << "RunViewer: #mesh.colors = " << mesh.colors.rows()
	     << " #mesh.vertices = " << mesh.vertices.rows() << "\n";
  if (mesh.colors.rows() > 0 && mesh.colors.rows() == mesh.vertices.rows()) {
    viewer.data.set_colors(mesh.colors);
  }
  viewer.core.show_lines = false;
  viewer.callback_init =
      std::bind(ViewerInit, std::placeholders::_1, view_setting);
  viewer.callback_pre_draw =
      std::bind(ViewerPreDraw, std::placeholders::_1, &mesh,
		view_setting);  // Bind callback.
  viewer.callback_post_draw =
      std::bind(ViewerPostDraw, std::placeholders::_1, &mesh,
		view_setting);  // Bind callback.
  viewer.launch(true, false);
}

// Usage:
// ./spectral_saliency <model_path>
//  model_path - path to the model file to render [.OFF]
int main(int argc, char *argv[]) {
  int argv_index = 0;

  // Get the file path to load (supports .OFF right now).
  std::string model_path(argv[++argv_index]);

  bool use_decimate = false;
  bool use_gaussian = false;
  bool display_spectrum = false;
  bool display_saliency = false;
  bool display_eigenvector = false;
  bool display_irregularity = false;
  bool display_saliency_eigenvector = false;

  for (argv_index = 2; argv_index < argc; ++argv_index) {
    std::string option(argv[argv_index]);
    LOG(DEBUG) << "option = " << option << "\n";
    if (option == "decimate")
      use_decimate = true;
    else if (option == "smooth")
      use_gaussian = true;
    else if (option == "spectrum")
      display_spectrum = true;
    else if (option == "saliency")
      display_saliency = true;
    else if (option == "eigenvector")
      display_eigenvector = true;
    else if (option == "irregularity")
      display_irregularity = true;
    else if (option == "saliency_eigenvector")
      display_saliency_eigenvector = true;
  }

  // Make a mesh struct.
  Mesh mesh;
  mesh.path = model_path;

  LOG(DEBUG) << "Loading mesh: '" << model_path << "'\n";

  // Load a triangular mesh format.
  igl::read_triangle_mesh(mesh.path, mesh.vertices, mesh.faces, mesh.directory,
			  mesh.basename, mesh.extension, mesh.filename);

  if (use_decimate) {
    // Decimate.
    int max_faces = 1000;
    Eigen::VectorXi birth_face_indices;
    Eigen::VectorXi birth_vertex_indices;
    Mesh decimated_mesh;
    decimated_mesh.faces.resize(max_faces, 3);
    decimated_mesh.vertices.resize(mesh.vertices.rows(), 3);
    birth_face_indices.resize(max_faces);
    birth_vertex_indices.resize(mesh.vertices.rows());

    igl::qslim(mesh.vertices, mesh.faces, max_faces, decimated_mesh.vertices,
	       decimated_mesh.faces, birth_face_indices, birth_vertex_indices);
    mesh.vertices = decimated_mesh.vertices;
    mesh.faces = decimated_mesh.faces;
  }

  PclPointCloud::Ptr input_cloud(new PclPointCloud());
  // Populate the point cloud.
  for (int i = 0; i < mesh.vertices.rows(); ++i)
    input_cloud->push_back(EigenToPclPoint(mesh.vertices.row(i)));
  PclKdtree::Ptr tree(new PclKdtree());
  tree->setInputCloud(input_cloud);

  if (use_gaussian) {
    // Smooth.
    Eigen::MatrixXd smoothed_vertices(mesh.vertices.rows(),
				      mesh.vertices.cols());
    double sigma = 0.002 *
		   (mesh.vertices.colwise().maxCoeff() -
		    mesh.vertices.colwise().minCoeff())
		       .norm();
    double scale = 1.0 * sigma * sigma;
    for (int i = 0; i < mesh.vertices.rows(); ++i) {
      Eigen::VectorXd result;
      ComputeGaussianPoint(mesh, i, tree, scale, 2.5 * sqrt(scale), &result);
      smoothed_vertices.row(i) = result;
    }
    mesh.vertices = smoothed_vertices;
  }

  // Get the minimum and maximum extents.
  mesh.bounds.lo = mesh.vertices.colwise().minCoeff();
  mesh.bounds.hi = mesh.vertices.colwise().maxCoeff();
  Eigen::Vector3d center = 0.5 * (mesh.bounds.hi + mesh.bounds.lo);
  double extent = (mesh.bounds.hi - mesh.bounds.lo).norm();

  if (display_spectrum) {
    Eigen::VectorXd spectrum(mesh.vertices.rows());
    Eigen::SelfAdjointEigenSolver<Eigen::SparseMatrix<double>> solver;
    ComputeLogLaplacianSpectrum(mesh.vertices, mesh.faces, solver, spectrum);
    mesh.colors.resize(mesh.vertices.rows(), 3);
    double min_spectrum = spectrum.minCoeff();
    double max_spectrum = spectrum.maxCoeff();
    double avg_spectrum = spectrum.mean();
    LOG(DEBUG) << "min_spectrum = " << min_spectrum
	       << " max_spectrum = " << max_spectrum
	       << " avg_spectrum = " << avg_spectrum << "\n";
    igl::jet(spectrum, spectrum.minCoeff(), spectrum.maxCoeff(), mesh.colors);
  } else if (display_saliency) {
    Eigen::VectorXd saliency(mesh.vertices.rows());
    ComputeMeshSaliency(mesh.vertices, mesh.faces, saliency);
    mesh.colors.resize(mesh.vertices.rows(), 3);
    double min_saliency = saliency.minCoeff();
    double max_saliency = saliency.maxCoeff();
    igl::jet(saliency, saliency.minCoeff(), saliency.maxCoeff(), mesh.colors);
  } else if (display_eigenvector) {
    Eigen::VectorXd spectrum(mesh.vertices.rows());
    Eigen::VectorXd eigenvector(mesh.vertices.rows());
    Eigen::SelfAdjointEigenSolver<Eigen::SparseMatrix<double>> solver;
    ComputeLogLaplacianSpectrum(mesh.vertices, mesh.faces, solver, spectrum);
    mesh.colors.resize(mesh.vertices.rows(), 3);
    eigenvector = solver.eigenvectors().col(0);
    double min_eigenvector = eigenvector.minCoeff();
    double max_eigenvector = eigenvector.maxCoeff();
    double avg_eigenvector = eigenvector.mean();
    LOG(DEBUG) << std::setprecision(16)
	       << "min_eigenvector = " << min_eigenvector
	       << " max_eigenvector = " << max_eigenvector
	       << " avg_eigenvector = " << avg_eigenvector << "\n";
    igl::jet(eigenvector, eigenvector.minCoeff(), eigenvector.maxCoeff(),
	     mesh.colors);
  } else if (display_irregularity) {
    Eigen::VectorXd irregularity(mesh.vertices.rows());
    Eigen::SelfAdjointEigenSolver<Eigen::SparseMatrix<double>> solver;
    ComputeMeshIrregularity(mesh.vertices, mesh.faces, solver, irregularity);
    mesh.colors.resize(mesh.vertices.rows(), 3);
    double min_irregularity = irregularity.minCoeff();
    double max_irregularity = irregularity.maxCoeff();
    double avg_irregularity = irregularity.mean();
    LOG(DEBUG) << "min_irregularity = " << min_irregularity
	       << " max_irregularity = " << max_irregularity
	       << " avg_irregularity = " << avg_irregularity << "\n";
    igl::jet(irregularity, irregularity.minCoeff(), irregularity.maxCoeff(),
	     mesh.colors);
  } else if (display_saliency_eigenvector) {
    Eigen::VectorXd saliency_eigenvector(mesh.vertices.rows());
    Eigen::MatrixXd saliency(mesh.vertices.rows(), mesh.vertices.rows());
    Eigen::EigenSolver<Eigen::MatrixXd> solver;
    ComputeMeshSaliencyMatrix(mesh.vertices, mesh.faces, saliency);
    solver.compute(saliency);
    saliency_eigenvector = solver.eigenvectors().col(0).unaryExpr(
	[](const std::complex<double> &c) -> double { return std::abs(c); });
    mesh.colors.resize(mesh.vertices.rows(), 3);
    double min_saliency_eigenvector = saliency_eigenvector.minCoeff();
    double max_saliency_eigenvector = saliency_eigenvector.maxCoeff();
    double avg_saliency_eigenvector = saliency_eigenvector.mean();
    LOG(DEBUG) << "min_saliency_eigenvector = " << min_saliency_eigenvector
	       << " max_saliency_eigenvector = " << max_saliency_eigenvector
	       << " avg_saliency_eigenvector = " << avg_saliency_eigenvector
	       << "\n";
    igl::jet(saliency_eigenvector, saliency_eigenvector.minCoeff(),
	     saliency_eigenvector.maxCoeff(), mesh.colors);
  }

  LOG(DEBUG) << "Compute stats: bounds.min = " << mesh.bounds.lo
	     << " bounds.max = " << mesh.bounds.hi << " extent = " << extent
	     << "\n";

  LOG(DEBUG) << "Normalize mesh.\n";

  ViewSetting view_setting =
      ViewSetting(window_width, window_height, Eigen::Vector3d(0.0, 0.0, 3.0),
		  Eigen::Vector3d(0.0, 1.0, 0.0), false, 0.0001, 100, 45.0,
		  Eigen::Vector3d(0.0, 0.0, 0.0));
  RunViewer(mesh, &view_setting);
  return 0;
}
