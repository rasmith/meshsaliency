#include "geometry_processing.h"
#include "common.h"
#include "geometry.h"
#include "logger.h"

#include <igl/cotmatrix.h>
#include <igl/qslim.h>
#include <igl/viewer/Viewer.h>
#include <pcl/common/point_operators.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/point_types.h>


bool IsCCW(const Eigen::Vector3d &a, const Eigen::Vector3d &b,
           const Eigen::Vector3d &c) {
   return (b(0) - a(0))*(c(1) - a(1)) - (b(1) - a(1))*(c(0) - a(0));
}

Eigen::Vector3d PclPointToEigen(const PclPoint &point) {
  return Eigen::Vector3d(point.x, point.y, point.z);
}

PclPoint EigenToPclPoint(const Eigen::Vector3d &point) {
  return PclPoint(point[0], point[1], point[2]);
}

double ComputeAveragePairwiseDistance(const Eigen::MatrixXd &vertices) {
  double result = 0.0;
  int n = vertices.rows();
  for (int i = 0; i < n; ++i)
    for (int j = i + 1; j < n; ++j)
      result += (vertices.row(i) - vertices.row(j)).norm();
  result /= ((n * (n - 1)) / 2);
  return result;
}

float ComputeGaussian(float x, float sigma) {
  return exp(-(x * x) / (2.0 * sigma * sigma));
}

void ComputeGaussianPoint(const geometry::Mesh &mesh, int i,
                          PclKdtree::Ptr tree, double sigma,
                          Eigen::VectorXd *output) {
  Eigen::Vector3d result;
  result.setZero();
  std::vector<int> neighbor_indices;
  std::vector<float> neighbor_distances;
  PclPoint input_point = EigenToPclPoint(mesh.vertices.row(i));
  if (pcl::isFinite(input_point) &&
      tree->radiusSearch(i, 2.0 * sigma, neighbor_indices,
                         neighbor_distances)) {
    double total_weight = 0.0;
    for (int k = 0; k < neighbor_indices.size(); ++k) {
      double weight = ComputeGaussian(neighbor_distances[k], sigma);
      result += weight * mesh.vertices.row(neighbor_indices[k]);
      total_weight += weight;
    }
    result /= total_weight;
    *output = result;
  } else {
    *output = mesh.vertices.row(i);
  }
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
  Eigen::SparseMatrix<double> cotmatrix(vertices.rows(), vertices.rows());
  LOG(DEBUG) << "ComputeMeshSaliency: compute cotangent matrix.\n";
  LOG(DEBUG) << "ComputeMeshSaliency:  #v = " << vertices.rows()
             << " #f = " << indices.rows() << "\n";
  assert(vertices.rows() > 0);
  for (int i = 0; i < indices.rows(); ++i) {
    Eigen::VectorXi face = indices.row(i);
    int fi = face(0);
    int fj = face(1);
    int fk = face(2);
    assert(0 <= fi && fi < vertices.rows());
    assert(0 <= fj && fj < vertices.rows());
    assert(0 <= fk && fk < vertices.rows());
  }
  // Get the eigenvalues of this matrix.
  igl::cotmatrix(vertices, indices, cotmatrix);
  // Solve.
  LOG(DEBUG) << "ComputeMeshSaliency: compute eigenvalues/eigenvectors.\n";
  solver.compute(-cotmatrix);
  Eigen::VectorXd eigenvalues = solver.eigenvalues();
  LOG(DEBUG) << "ComputeMeshSaliency: compute log.\n";
  Eigen::VectorXd log_laplacian = eigenvalues.unaryExpr(
      [](double x) -> double { return (x > 0.0 ? std::log(x) : x); });
  LOG(DEBUG) << "ComputeMeshSaliency: apply averaging filter.\n";
  // Get average response A.
  Eigen::VectorXd average(vertices.rows());
  int filter_size = 9;
  average.setZero();
  for (int i = 0; i < vertices.rows(); ++i) {
    for (int j = -filter_size / 2; j <= filter_size / 2; ++j) {
      int index = i + j;
      average(i) +=
          (index > 0 && index < vertices.rows() ? log_laplacian(index) : 0);
    }
  }
  average /= filter_size;
  LOG(DEBUG) << "ComputeMeshSaliency: compute irregularity.\n";
  // Compute the spectral irregularity, R(f).
  Eigen::VectorXd irregularity = (log_laplacian - average).cwiseAbs();
  // Get the R matrix, which is exp(diag(R(f)).  Since this a diagonal
  // matrix, just take the std::exp of its entries.
  Eigen::VectorXd r_diagonal = irregularity.diagonal().unaryExpr(
      [](double x) -> double { return std::exp(x); });
  // Compute weighted adjacency matrix, W.
  Eigen::SparseMatrix<double> weighted_adjacency;
  LOG(DEBUG) << "ComputeMeshSaliency: compute weighted adjacency.\n";
  ComputeWeightedAdjacency(vertices, indices, weighted_adjacency);
  LOG(DEBUG) << "ComputeMeshSaliency: compute saliency.\n";
  // Compute the saliency S = B*R*B^T * W.
  Eigen::MatrixXd lhs = solver.eigenvectors() * r_diagonal.asDiagonal() *
                        solver.eigenvectors().transpose();
  saliency = (lhs * weighted_adjacency).colwise().sum();
}

void ComputeMultiScaleSaliency(const geometry::Mesh &mesh, int max_faces,
                               const double *scales, int num_scales,
                               Eigen::VectorXd &saliency) {
  // Decimate the mesh.
  Eigen::VectorXi birth_face_indices;
  Eigen::VectorXi birth_vertex_indices;
  geometry::Mesh decimated_mesh;
  decimated_mesh.faces.resize(max_faces, 3);
  decimated_mesh.vertices.resize(mesh.vertices.rows(), 3);
  birth_face_indices.resize(max_faces);
  birth_vertex_indices.resize(mesh.vertices.rows());

  LOG(DEBUG) << "ComputeMultiScaleSaliency: decimate mesh.\n";
  igl::qslim(mesh.vertices, mesh.faces, max_faces, decimated_mesh.vertices,
             decimated_mesh.faces, birth_face_indices, birth_vertex_indices);
  LOG(DEBUG) << "ComputeMultiScaleSaliency: decimate #v = "
             << decimated_mesh.vertices.rows()
             << " #f = " << decimated_mesh.faces.rows() << ".\n";
  // Compute average pairwise distance.
  double average_pairwise =
      ComputeAveragePairwiseDistance(decimated_mesh.vertices);
  // Compute weighted adjacency and degree info.
  Eigen::SparseMatrix<double> weighted_adjacency;
  ComputeWeightedAdjacency(decimated_mesh.vertices, decimated_mesh.faces,
                           weighted_adjacency);
  Eigen::VectorXi degrees(decimated_mesh.vertices.rows());
  for (int i = 0; i < weighted_adjacency.rows(); ++i) {
    degrees(i) = static_cast<int>(
        weighted_adjacency.row(i)
            .unaryExpr([](const double &x) -> double { return 1.0; })
            .sum());
  }
  Eigen::VectorXi scale_factors(decimated_mesh.vertices.rows());
  // Compute k(i) for each vertex in the decimated mesh.
  for (int i = 0; i < decimated_mesh.vertices.rows(); ++i) {
    double denominator =
        weighted_adjacency.row(i)
            .unaryExpr([](const double &x) -> double { return sqrt(x); })
            .sum();
    scale_factors(i) = degrees(i) * average_pairwise / denominator + 1;
  }
  // Set saliency to zero.
  saliency = Eigen::VectorXd::Zero(mesh.vertices.rows());

  LOG(DEBUG) << "ComputeMultiScaleSaliency: create point cloud.\n";
  // Used to compute Gaussian filter.
  PclPointCloud::Ptr input_cloud(new PclPointCloud());

  // Populate the point cloud.
  for (int i = 0; i < decimated_mesh.vertices.rows(); ++i)
    input_cloud->push_back(EigenToPclPoint(decimated_mesh.vertices.row(i)));

  LOG(DEBUG) << "ComputeMultiScaleSaliency: create kd-tree.\n";

  // Set the input cloud for the kd-tree.
  // Search tree for this.  Really don't want to code.
  PclKdtree::Ptr tree(new PclKdtree());
  tree->setInputCloud(input_cloud);

  // These vertices represent M(t).
  Eigen::MatrixXd vertices1(decimated_mesh.vertices.rows(), 3);
  // These vertices represent M(k(i) *t).
  Eigen::MatrixXd vertices2(decimated_mesh.vertices.rows(), 3);

  // For each scale compute M(t) and M(k(i)t).
  for (int j = 0; j < num_scales; ++j) {
    LOG(DEBUG) << "ComputeMultiScaleSaliency: scale j = " << j << "\n";
    // For each point do a search to get its equivalent two points in
    // M(t) and M(k(i)t).
    LOG(DEBUG) << "ComputeMultiScaleSaliency: compute gaussians.\n";
    for (int i = 0; i < decimated_mesh.vertices.rows(); ++i) {
      const PclPoint &input_point = input_cloud->points[i];
      std::vector<int> neighbor_indices;
      std::vector<float> neighbor_distances;
      double sigma = scales[j];
      // Get the first gaussian F(p_i, t)
      Eigen::VectorXd result;
      ComputeGaussianPoint(decimated_mesh, i, tree, sigma, &result);
      // std::cout << "gaussian_point("<<i<<")"<<result << "\n";
      vertices1.row(i) = result;
      // Get the second gaussian F(p_i, k(i) * t).
      ComputeGaussianPoint(decimated_mesh, i, tree, scale_factors(i) * sigma,
                           &result);
      vertices2.row(i) = result;
    }
    LOG(DEBUG)
        << "ComputeMultiScaleSaliency: compute scale space mesh saliency.\n";
    LOG(DEBUG) << "ComputeMultiScaleSaliency: #v1 = " << vertices1.rows()
               << "  #v2 = " << vertices2.rows() << "\n";
    // Compute the saliency S(i, t).
    Eigen::VectorXd saliency1(vertices1.rows());
    ComputeMeshSaliency(vertices1, decimated_mesh.faces, saliency1);
    // Compute the saliency S(i, k(i) * t).
    Eigen::VectorXd saliency2(vertices2.rows());
    ComputeMeshSaliency(vertices2, decimated_mesh.faces, saliency2);
    // Compute S'(i, t) = |S(i, k(i) * t) - S(i, t)|.
    Eigen::VectorXd saliency_t = (saliency2 - saliency1).cwiseAbs();
    LOG(DEBUG) << "ComputeMultiScaleSaliency:  #saliency1 = "
               << saliency1.rows() << " #saliency2 = " << saliency2.rows()
               << "\n";
    LOG(DEBUG) << "ComputeMultiScaleSaliency: reproject saliency.\n";
    LOG(DEBUG) << "ComputeMultiScaleSaliency: #saliency = " << saliency.rows()
               << "\n";

    // Obtain S(v, t) by method in 3.3.
    // For each vertex v in M, find closest point in simplified decimated_mesh
    // M', and map the saliency of that point to S(v, t).
    for (int j = 0; j < mesh.vertices.rows(); ++j) {
      PclPoint input_point = EigenToPclPoint(mesh.vertices.row(j));
      std::vector<int> neighbor_indices;
      std::vector<float> neighbor_distances;
      tree->nearestKSearch(input_point, 1, neighbor_indices,
                           neighbor_distances);
      assert(neighbor_indices[0] >= 0 &&
             neighbor_indices[0] < saliency_t.rows());
      // Sum into the current saliency value from S(v, t) into S'(v).
      saliency(j) += saliency_t(neighbor_indices[0]);
    }
  }
  // Get saliency S(v) = log S'(v).
  for (int j = 0; j < mesh.vertices.rows(); ++j)
    saliency(j) = std::log(saliency(j));
}
