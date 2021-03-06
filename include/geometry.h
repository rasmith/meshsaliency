#pragma once

#include <Eigen/Dense>

namespace geometry {

// This is for holding axis aligned bounding boxes.
struct BoundingBox {
  Eigen::Vector3d lo;
  Eigen::Vector3d hi;
};

// This is a simple mesh data structure.
struct Mesh {
  Eigen::MatrixXd vertices;
  Eigen::MatrixXi faces;
  Eigen::MatrixXd colors;
  BoundingBox bounds;
  Eigen::Vector3d center;
	std::string basename;
	std::string directory;
	std::string extension;
	std::string filename;
	std::string path;
};

} // namespace geometry
