#include "geometry_processing.h"
#include "common.h"
#include "geometry.h"
#include "logger.h"

#include <cmath>
#include <iostream>

#include <igl/AABB.h>
#include <igl/barycentric_coordinates.h>
#include <igl/cotmatrix.h>
#include <igl/qslim.h>
#include <igl/viewer/Viewer.h>
#include <pcl/common/point_operators.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/point_types.h>

bool IsCCW(const Eigen::Vector3d &a, const Eigen::Vector3d &b,
	   const Eigen::Vector3d &c) {
  return (b(0) - a(0)) * (c(1) - a(1)) - (b(1) - a(1)) * (c(0) - a(0));
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

const static double kPi = 3.141592653589793238;

double ComputeGaussian(double x, double scale) {
  double inv = 1.0 / sqrt(2.0 * kPi * scale);
  return inv * exp(-(x * x) / (2.0 * scale));
}

void ComputeSmoothedSaliencyValue(const geometry::Mesh &mesh,
				  const Eigen::VectorXd &saliency, int i,
				  PclKdtree::Ptr tree, double scale,
				  double *output) {
  double result = 0.0;
  std::vector<int> neighbor_indices;
  std::vector<float> neighbor_distances;
  Eigen::VectorXd query = mesh.vertices.row(i);
  PclPoint input_point = EigenToPclPoint(query);
  if (pcl::isFinite(input_point) &&
      tree->radiusSearch(input_point, 2.0 * scale, neighbor_indices,
			 neighbor_distances)) {
    double total_weight = 0.0;
    for (int k = 0; k < neighbor_indices.size(); ++k) {
      Eigen::VectorXd neighbor = mesh.vertices.row(neighbor_indices[k]);
      double distance = (neighbor - query).norm();
      double weight = ComputeGaussian(distance, scale);
      result += weight * saliency(neighbor_indices[k]);
      total_weight += weight;
    }
    result /= total_weight;
    *output = result;
  } else {
    *output = saliency(i);
  }
}

void ComputeGaussianPoint(const geometry::Mesh &mesh, int i,
			  PclKdtree::Ptr tree, double scale, double threshold,
			  Eigen::VectorXd *output) {
  Eigen::Vector3d result;
  result.setZero();
  std::vector<int> neighbor_indices;
  std::vector<float> neighbor_distances;
  Eigen::VectorXd query = mesh.vertices.row(i);
  PclPoint input_point = EigenToPclPoint(query);
  if (pcl::isFinite(input_point) &&
      tree->radiusSearch(input_point, threshold, neighbor_indices,
			 neighbor_distances)) {
    double total_weight = 0.0;
    for (int k = 0; k < neighbor_indices.size(); ++k) {
      Eigen::VectorXd neighbor = mesh.vertices.row(neighbor_indices[k]);
      double distance = (neighbor - query).norm();
      double weight = ComputeGaussian(distance, scale);
      result += weight * mesh.vertices.row(neighbor_indices[k]);
      total_weight += weight;
    }
    result /= total_weight;
    *output = result;
  } else {
    *output = mesh.vertices.row(i);
  }
}

void ComputeGaussianMesh(const geometry::Mesh &mesh, const PclKdtree::Ptr &tree,
			 double scale, Eigen::MatrixXd &smoothed_vertices) {
  smoothed_vertices.resize(mesh.vertices.rows(), mesh.vertices.cols());
  for (int i = 0; i < mesh.vertices.rows(); ++i) {
    double threshold = 2.5 * sqrt(scale);
    Eigen::VectorXd result;
    ComputeGaussianPoint(mesh, i, tree, scale, threshold, &result);
    smoothed_vertices.row(i) = result;
  }
}

void ComputeDynamicGaussianMesh(const geometry::Mesh &mesh,
				const PclKdtree::Ptr &tree, double scale,
				const Eigen::VectorXi &scale_factors,
				Eigen::MatrixXd &smoothed_vertices) {
  smoothed_vertices.resize(mesh.vertices.rows(), mesh.vertices.cols());
  for (int i = 0; i < mesh.vertices.rows(); ++i) {
    double threshold = 2.5 * sqrt(scale_factors(i) * scale);
    Eigen::VectorXd result;
    ComputeGaussianPoint(mesh, i, tree, scale, threshold, &result);
    smoothed_vertices.row(i) = result;
  }
}

void ComputePointSaliency(const geometry::Mesh &mesh,
			  const Eigen::Vector3d &point, int face_index,
			  const Eigen::VectorXd &saliency,
			  double &saliency_value) {
  Eigen::Vector3i face = mesh.faces.row(face_index);
  int fi = face(0);
  int fj = face(1);
  int fk = face(2);
  Eigen::Vector3d vi = mesh.vertices.row(fi);
  Eigen::Vector3d vj = mesh.vertices.row(fj);
  Eigen::Vector3d vk = mesh.vertices.row(fk);
  Eigen::Vector3d coordinates;
  Eigen::Vector3d v0 = vj - vi, v1 = vk - vi, v2 = point - vi;
  double d00 = v0.dot(v0);
  double d01 = v0.dot(v1);
  double d11 = v1.dot(v1);
  double d20 = v2.dot(v0);
  double d21 = v2.dot(v1);
  double denom = d00 * d11 - d01 * d01;
  coordinates(1) = (d11 * d20 - d01 * d21) / denom;
  coordinates(2) = (d00 * d21 - d01 * d20) / denom;
  coordinates(0) = 1.0f - coordinates(1) - coordinates(2);
  saliency_value = 0.0;
  saliency_value += coordinates(0) * saliency(fi);
  saliency_value += coordinates(1) * saliency(fj);
  saliency_value += coordinates(2) * saliency(fk);
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
    double fi_fj_norm2 =
	1.0 / (vertices.row(fi) - vertices.row(fj)).squaredNorm();
    double fi_fk_norm2 =
	1.0 / (vertices.row(fi) - vertices.row(fk)).squaredNorm();
    double fj_fk_norm2 =
	1.0 / (vertices.row(fj) - vertices.row(fk)).squaredNorm();
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

void ComputeLogLaplacianSpectrum(
    const Eigen::MatrixXd &vertices, const Eigen::MatrixXi &indices,
    Eigen::SelfAdjointEigenSolver<Eigen::SparseMatrix<double>> &solver,
    Eigen::VectorXd &log_laplacian_spectrum) {
  // Compute the cotangent matrix.
  Eigen::SparseMatrix<double> cotmatrix(vertices.rows(), vertices.rows());
  // Get the eigenvalues of this matrix.
  igl::cotmatrix(vertices, indices, cotmatrix);
  // Solve.
  solver.compute(-cotmatrix);
  Eigen::VectorXd eigenvalues = solver.eigenvalues();
  log_laplacian_spectrum = eigenvalues.unaryExpr(
      [](double x) -> double { return std::log(std::abs(x)); });
}

void ComputeMeshIrregularity(
    const Eigen::MatrixXd &vertices, const Eigen::MatrixXi &indices,
    Eigen::SelfAdjointEigenSolver<Eigen::SparseMatrix<double>> &solver,
    Eigen::VectorXd &irregularity) {
  Eigen::SparseMatrix<double> cotmatrix(vertices.rows(), vertices.rows());
  Eigen::VectorXd log_laplacian_spectrum(vertices.rows());
  ComputeLogLaplacianSpectrum(vertices, indices, solver,
			      log_laplacian_spectrum);
  // Get average response A.
  Eigen::VectorXd average(vertices.rows());
  int filter_size = 9;
  average.setZero();
  for (int i = 0; i < vertices.rows(); ++i) {
    for (int j = -filter_size / 2; j <= filter_size / 2; ++j) {
      int index = i + j;
      average(i) +=
	  (index >= 0 && index < vertices.rows() ? log_laplacian_spectrum(index)
						 : log_laplacian_spectrum(i));
    }
  }
  average /= static_cast<double>(filter_size);
  // Compute the spectral irregularity, R(f).
  irregularity = (log_laplacian_spectrum - average).cwiseAbs();
}

void ComputeMeshSaliencyMatrix(const Eigen::MatrixXd &vertices,
			       const Eigen::MatrixXi &indices,
			       Eigen::MatrixXd &saliency) {
  Eigen::SelfAdjointEigenSolver<Eigen::SparseMatrix<double>> solver;
  Eigen::VectorXd irregularity(vertices.rows());
  ComputeMeshIrregularity(vertices, indices, solver, irregularity);
  // Get the R matrix, which is exp(diag(R(f)).  Since this a diagonal
  // matrix, just take the std::exp of its entries.
  Eigen::VectorXd r_diagonal =
      irregularity.unaryExpr([](double x) -> double { return std::exp(x); });
  // Compute weighted adjacency matrix, W.
  Eigen::SparseMatrix<double> weighted_adjacency;
  ComputeWeightedAdjacency(vertices, indices, weighted_adjacency);
  // Normalize.
  for (int i = 0; i < vertices.rows(); ++i)
    weighted_adjacency.row(i) /= weighted_adjacency.row(i).norm();
  // Compute the saliency S = B*R*B^T * W.
  Eigen::MatrixXd lhs = solver.eigenvectors() * r_diagonal.asDiagonal() *
			solver.eigenvectors().transpose();
  saliency = (lhs * weighted_adjacency);
}

void ComputeMeshSaliency(const Eigen::MatrixXd &vertices,
			 const Eigen::MatrixXi &indices,
			 Eigen::VectorXd &saliency) {
  Eigen::MatrixXd saliency_matrix(vertices.rows(), vertices.rows());
  ComputeMeshSaliencyMatrix(vertices, indices, saliency_matrix);
  saliency = saliency_matrix.rowwise().sum();
}

void ComputeMultiScaleSaliency(const geometry::Mesh &mesh, int max_faces,
			       const double *scales, int num_scales,
			       Eigen::VectorXd &saliency) {
  saliency.resize(mesh.vertices.rows());
  saliency.setZero();

  // Decimate the mesh.
  Eigen::VectorXi birth_face_indices;
  Eigen::VectorXi birth_vertex_indices;
  geometry::Mesh decimated_mesh;
  decimated_mesh.faces.resize(max_faces, 3);
  decimated_mesh.vertices.resize(mesh.vertices.rows(), 3);
  birth_face_indices.resize(max_faces);
  birth_vertex_indices.resize(mesh.vertices.rows());

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
  for (int i = 0; i < 10; ++i)
    LOG(DEBUG) << "degree(" << i << ")=" << degrees(i) << "\n";

  // Compute k(i) for each vertex in the decimated mesh.
  Eigen::VectorXi scale_factors(decimated_mesh.vertices.rows());
  for (int i = 0; i < decimated_mesh.vertices.rows(); ++i) {
    double denominator =
	weighted_adjacency.row(i)
	    .unaryExpr([](const double &x) -> double { return 1.0 / sqrt(x); })
	    .sum();
    scale_factors(i) =
	static_cast<double>(degrees(i)) * average_pairwise / denominator + 1;
    if (i < 10)
      LOG(DEBUG) << "scale_factors(" << i << ")=" << scale_factors(i) << "\n";
  }

  // Used to compute nearest point.
  igl::AABB<Eigen::MatrixXd, 3> aabb_tree;
  aabb_tree.init(decimated_mesh.vertices, decimated_mesh.faces);

  // Used to compute Gaussian filter.
  PclPointCloud::Ptr input_cloud(new PclPointCloud());

  // Populate the point cloud.
  for (int i = 0; i < decimated_mesh.vertices.rows(); ++i)
    input_cloud->push_back(EigenToPclPoint(decimated_mesh.vertices.row(i)));

  // Set the input cloud for the kd-tree.
  // Search tree for this.  Really don't want to code.
  PclKdtree::Ptr tree(new PclKdtree());
  tree->setInputCloud(input_cloud);

  // These vertices represent M(t).
  Eigen::MatrixXd vertices1(decimated_mesh.vertices.rows(), 3);
  // These vertices represent M(k(i) *t).
  Eigen::MatrixXd vertices2(decimated_mesh.vertices.rows(), 3);

  for (int j = 0; j < num_scales; ++j) {
    double sigma = scales[j];
    ComputeGaussianMesh(decimated_mesh, tree, sigma, vertices1);
    ComputeDynamicGaussianMesh(decimated_mesh, tree, sigma, scale_factors,
			       vertices2);

    // Compute the saliency S(i, t).
    Eigen::VectorXd saliency1(vertices1.rows());
    ComputeMeshSaliency(vertices1, decimated_mesh.faces, saliency1);

    // Compute the saliency S(i, k(i) * t).
    Eigen::VectorXd saliency2(vertices2.rows());
    ComputeMeshSaliency(vertices2, decimated_mesh.faces, saliency2);
    for (int j = 0; j < 10; ++j) {
      LOG(DEBUG) << "vertices1(" << j << ")=" << vertices1(j) << "\n";
      LOG(DEBUG) << "vertices2(" << j << ")=" << vertices2(j) << "\n";
    }

    // Compute S'(i, t) = |S(i, k(i) * t) - S(i, t)|.
    Eigen::VectorXd saliency_t = (saliency2 - saliency1).cwiseAbs();
    for (int j = 0; j < 10; ++j) {
      LOG(DEBUG) << "saliency1(" << j << ")=" << saliency1(j) << "\n";
      LOG(DEBUG) << "saliency2(" << j << ")=" << saliency2(j) << "\n";
    }
    // for (int j = 0; j < mesh.vertices.rows(); ++j) {
    // LOG(DEBUG) << "saliency1("<<j<<")="<<saliency1(j)<<"\n";
    //}

    // Obtain S(v, t) by method in 3.3.
    // For each vertex v in M, find closest point in simplified decimated_mesh
    // M', and map the saliency of that point to S(v, t).
    for (int j = 0; j < mesh.vertices.rows(); ++j) {
      double distance = -1;
      int index = -1;
      Eigen::RowVector3d point;
      Eigen::RowVector3d query = mesh.vertices.row(j);
      distance = aabb_tree.squared_distance(
	  decimated_mesh.vertices, decimated_mesh.faces, query, index, point);
      double saliency_value = -1.0;
      ComputePointSaliency(decimated_mesh, point, index, saliency_t,
			   saliency_value);
      // Sum into the current saliency value from S(v, t) into S'(v).
      saliency(j) += saliency_value;
    }
  }

  // Get saliency S(v) = log S'(v).
  for (int j = 0; j < mesh.vertices.rows(); ++j)
    saliency(j) = std::log(saliency(j));
}
