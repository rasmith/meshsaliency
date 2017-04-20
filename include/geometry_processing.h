#pragma once

#include "common.h"

#include <igl/cotmatrix.h>
#include <pcl/common/point_operators.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZ PclPoint;
typedef pcl::search::KdTree<PclPoint> PclKdtree;
typedef pcl::PointCloud<PclPoint> PclPointCloud;

Eigen::Vector3d PclPointToEigen(const PclPoint &point);

PclPoint EigenToPclPoint(const Eigen::Vector3d &point);

double ComputeAveragePairwiseDistance(const Eigen::MatrixXd &vertices);

float ComputeGaussian(float x, float sigma);

void ComputeGaussianPoint(const geometry::Mesh &mesh, int i, PclKdtree::Ptr tree,
                          double sigma, Eigen::VectorXd *output);

void ComputeWeightedAdjacency(const Eigen::MatrixXd &vertices,
                              const Eigen::MatrixXi &indices,
                              Eigen::SparseMatrix<double> &weighted_adjacency);

void ComputeMeshSaliency(const Eigen::MatrixXd &vertices,
                         const Eigen::MatrixXi &indices,
                         Eigen::VectorXd &saliency);

void ComputeMultiScaleSaliency(const geometry::Mesh &mesh, int max_faces,
                               const double *scales, int num_scales,
                               Eigen::VectorXd &saliency);
