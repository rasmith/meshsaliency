#pragma once

#include "common.h"
#include "geometry.h"

#include <igl/cotmatrix.h>
#include <pcl/common/point_operators.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZ PclPoint;
typedef pcl::search::KdTree<PclPoint> PclKdtree;
typedef pcl::PointCloud<PclPoint> PclPointCloud;

Eigen::Vector3d PclPointToEigen(const PclPoint &point);

PclPoint EigenToPclPoint(const Eigen::Vector3d &point);

bool IsCCW(const Eigen::Vector3d &a, const Eigen::Vector3d &b,
	   const Eigen::Vector3d &c);

double ComputeAveragePairwiseDistance(const Eigen::MatrixXd &vertices);

double ComputeGaussian(double x, double sigma);

void ComputeGaussianPoint(const geometry::Mesh &mesh, int i,
			  PclKdtree::Ptr tree, double scale, double threshold,
			  Eigen::VectorXd *output);

void ComputeLogLaplacianSpectrum(
    const Eigen::MatrixXd &vertices, const Eigen::MatrixXi &indices,
    Eigen::SelfAdjointEigenSolver<Eigen::SparseMatrix<double>> &solver,
    Eigen::VectorXd &log_laplacian_spectrum);

void ComputeWeightedAdjacency(const Eigen::MatrixXd &vertices,
			      const Eigen::MatrixXi &indices,
			      Eigen::SparseMatrix<double> &weighted_adjacency);

void ComputeMeshSaliency(const Eigen::MatrixXd &vertices,
			 const Eigen::MatrixXi &indices,
			 Eigen::VectorXd &saliency);

void ComputeMultiScaleSaliency(const geometry::Mesh &mesh, int max_faces,
			       const double *scales, int num_scales,
			       Eigen::VectorXd &saliency);
