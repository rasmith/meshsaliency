#include "common.h"
#include "geometry.h"
#include "logger.h"
#include "view_setting.h"

#include <functional>
#include <string>
#include <vector>

#define IGL_VIEWER_VIEWER_QUIET

#include <igl/cotmatrix.h>
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

typedef pcl::PointXYZ PclPoint;
typedef pcl::search::KdTree<PclPoint> PclKdtree;
typedef pcl::PointCloud<PclPoint> PclPointCloud;

Eigen::Vector3d PclPointToEigen(const PclPoint &point) {
  return Eigen::Vector3d(point.x, point.y, point.z);
}

PclPoint EigenToPclPoint(const Eigen::Vector3d &point) {
  return PclPoint(point[0], point[1], point[2]);
}

float ComputeGaussian(float x, float sigma) {
  return exp(-(x * x) / (2 * sigma * sigma));
}

void ComputeGaussianPoint(const Mesh &mesh, int i, PclKdtree::Ptr tree,
                          double sigma, Eigen::VectorXd *output) {
  Eigen::Vector3d result;
  std::vector<int> neighbor_indices;
  std::vector<float> neighbor_distances;
  PclPoint input_point = EigenToPclPoint(mesh.vertices.row(i));
  if (pcl::isFinite(input_point) &&
      tree->radiusSearch(i, sigma, neighbor_indices, neighbor_distances)) {
    double total_weight = 0.0;
    for (int k = 0; k < neighbor_indices.size(); ++k) {
      double weight = ComputeGaussian(neighbor_distances[k], sigma);
      result += weight * mesh.vertices.row(neighbor_distances[k]);
      total_weight += weight;
    }
    result /= total_weight;
  }
  *output = result;
}

void ComputeWeightedAdjacency(const Eigen::MatrixXd &vertices,
                              const Eigen::MatrixXi &indices,
                              Eigen::SparseMatrix<double> &weighted_adjacency) {
  weighted_adjacency.resize(vertices.rows(), vertices.rows());
  weighted_adjacency.setZero();
  std::vector<Eigen::Triplet<double>> triples;
  for (int i = 0; i < indices.rows(); ++i) {
    int fi = indices.row(i)(0);
    int fj = indices.row(i)(1);
    int fk = indices.row(i)(2);
    double fi_fj_norm2 = (vertices.row(fi) - vertices.row(fj)).squaredNorm();
    double fi_fk_norm2 = (vertices.row(fi) - vertices.row(fk)).squaredNorm();
    double fj_fk_norm2 = (vertices.row(fj) - vertices.row(fk)).squaredNorm();
    triples.push_back(Eigen::Triplet<double>(fi, fj, fi_fj_norm2));
    triples.push_back(Eigen::Triplet<double>(fj, fi, fi_fj_norm2));
    triples.push_back(Eigen::Triplet<double>(fi, fk, fi_fk_norm2));
    triples.push_back(Eigen::Triplet<double>(fk, fi, fi_fk_norm2));
    triples.push_back(Eigen::Triplet<double>(fj, fk, fj_fk_norm2));
    triples.push_back(Eigen::Triplet<double>(fk, fj, fj_fk_norm2));
  }
  // Compute the adjacency from triplets.
  // NOTE: a lambda is passed to avoid summing on duplicate, the default
  // behavior in Eigen.
  weighted_adjacency.setFromTriplets(
      triples.begin(), triples.end(),
      [](const double &, const double &b) -> double { return b; });
}

void ComputeMeshSaliency(const Eigen::MatrixXd &vertices,
                         const Eigen::MatrixXi &indices,
                         Eigen::VectorXd &saliency) {
  // Get eigensolver ready.
  Eigen::SelfAdjointEigenSolver<Eigen::SparseMatrix<double>> solver;
  // Compute the cotangent matrix.
  Eigen::SparseMatrix<double> cotmatrix;
  // Get the eigenvalues of this matrix.
  igl::cotmatrix(vertices, indices, cotmatrix);
  // Solve.
  solver.compute(-cotmatrix);
  Eigen::VectorXd eigenvalues = solver.eigenvalues();
  Eigen::VectorXd log_laplacian = eigenvalues.unaryExpr(
      [](double x) -> double { return (x > 0.0 ? std::log(x) : x); });
  // Compute the averaging filter, J_n.
  Eigen::VectorXd averaging_filter;
  averaging_filter.setOnes();
  averaging_filter /= vertices.rows();
  // Get average response A.
  Eigen::MatrixXd average = averaging_filter * log_laplacian;
  // Compute the spectral irregularity, R(f).
  Eigen::VectorXd irregularity = (log_laplacian - average).cwiseAbs();
  // Get the R matrix, which is exp(diag(R(f)).  Since this a diagonal
  // matrix, just take the std::exp of its entries.
  Eigen::VectorXd r_diagonal = irregularity.diagonal().unaryExpr(
      [](double x) -> double { return std::exp(x); });
  // Compute weighted adjacency matrix, W.
  Eigen::SparseMatrix<double> weighted_adjacency;
  ComputeWeightedAdjacency(vertices, indices, weighted_adjacency);
  // Compute the saliency S = B*R*B^T * W.
  Eigen::MatrixXd lhs = solver.eigenvectors() * r_diagonal.asDiagonal() *
                        solver.eigenvectors().transpose();
  saliency = (lhs * weighted_adjacency).colwise().sum();
}

void ComputeMultiScaleSaliency(const Mesh &mesh, int max_faces,
                               const double *scales, int num_scales,
                               Eigen::VectorXd &saliency) {
  // Compute scale factor, will use this later.
  double scale_factor = 1.0f;
  Eigen::VectorXd extents;
  Eigen::VectorXd lo = mesh.vertices.colwise().minCoeff();
  Eigen::VectorXd hi = mesh.vertices.colwise().maxCoeff();
  extents = hi - lo;
  scale_factor = 0.02 * extents.norm();

  // Decimate the mesh.
  Eigen::VectorXi birth_face_indices;
  Eigen::VectorXi birth_vertex_indices;
  Mesh decimated_mesh;
  decimated_mesh.faces.resize(max_faces, 3);
  decimated_mesh.vertices.resize(mesh.vertices.rows(), 3);
  birth_face_indices.resize(max_faces);
  birth_vertex_indices.resize(mesh.vertices.rows());

  igl::qslim(mesh.vertices, mesh.faces, max_faces, decimated_mesh.vertices,
             decimated_mesh.faces, birth_face_indices, birth_vertex_indices);

  // Set saliency to zero.
  saliency = Eigen::VectorXd::Zero(decimated_mesh.vertices.rows());

  // Used to compute Gaussian filter.
  PclPointCloud::Ptr input_cloud(new PclPointCloud());
  // Search tree for this.  Really don't want to code.
  PclKdtree::Ptr tree(new PclKdtree());

  // Populate the point cloud.
  for (int i = 0; i < decimated_mesh.vertices.rows(); ++i)
    input_cloud->push_back(EigenToPclPoint(decimated_mesh.vertices.row(i)));

  // Set the input cloud for the kd-tree.
  tree->setInputCloud(input_cloud);

  Eigen::MatrixXd vertices1;  // These vertices represent M(t).
  Eigen::MatrixXd vertices2;  // These vertices represent M(k(i) *t).

  // For each scale compute M(t) and M(k(i)t).
  for (int j = 0; j < num_scales; ++j) {
    // For each point do a search to get its equivalent two points in
    // M(t) and M(k(i)t).
    for (int i = 0; i < decimated_mesh.vertices.rows(); ++i) {
      const PclPoint &input_point = input_cloud->points[i];
      std::vector<int> neighbor_indices;
      std::vector<float> neighbor_distances;
      double sigma = scales[j];
      // Get the first gaussian F(p_i, t)
      Eigen::VectorXd result;
      ComputeGaussianPoint(decimated_mesh, i, tree, sigma, &result);
      vertices1.row(i) = result;
      // Get the second gaussian F(p_i, k(i) * t).
      ComputeGaussianPoint(decimated_mesh, i, tree, sigma, &result);
      vertices2.row(i) = result;
    }
    // Compute the saliency S(i, t).
    Eigen::VectorXd saliency1;
    ComputeMeshSaliency(vertices1, decimated_mesh.faces, saliency1);
    // Compute the saliency S(i, k(i) * t).
    Eigen::VectorXd saliency2;
    ComputeMeshSaliency(vertices2, decimated_mesh.faces, saliency2);
    // Compute S'(i, t) = |S(i, k(i) * t) - S(i, t)|.
    Eigen::VectorXd saliency_t = (saliency2 - saliency1).cwiseAbs();

    // Obtain S(v, t) by method in 3.3.
    // For each vertex v in M, find closest point in simplified decimated_mesh
    // M', and map the saliency of that point to S(v, t).
    for (int j = 0; j < decimated_mesh.vertices.rows(); ++j) {
      const PclPoint &input_point = input_cloud->points[j];
      std::vector<int> neighbor_indices;
      std::vector<float> neighbor_distances;
      tree->nearestKSearch(input_point, 1, neighbor_indices,
                           neighbor_distances);
      // Sum into the current saliency value.
      saliency(j) += saliency_t(neighbor_indices[0]);
    }
  }
}

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
  igl::viewer::Viewer viewer;                       // Create a viewer.
  viewer.data.set_mesh(mesh.vertices, mesh.faces);  // Set mesh data.
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

  // Make a mesh struct.
  Mesh mesh;
  mesh.path = model_path;

  // Load a triangular mesh format.
  igl::read_triangle_mesh(mesh.path, mesh.vertices, mesh.faces, mesh.directory,
                          mesh.basename, mesh.extension, mesh.filename);

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

  Eigen::VectorXd saliency(mesh.vertices.rows());

  int max_vertices = 1000;
  int num_scales = 5;
  double scale_base = 0.02 * extent;
  double scales[5] = {1.0 * scale_base, 2.0 * scale_base, 3.0 * scale_base,
                      4.0 * scale_base, 5.0 * scale_base};
  ComputeMultiScaleSaliency(mesh, max_vertices, scales, num_scales, saliency);

  ViewSetting view_setting =
      ViewSetting(window_width, window_height, Eigen::Vector3d(0.0, 0.0, 3.0),
                  Eigen::Vector3d(0.0, 1.0, 0.0), false, 0.0001, 100, 45.0,
                  Eigen::Vector3d(0.0, 0.0, 0.0));
  RunViewer(mesh, &view_setting);
  return 0;
}
