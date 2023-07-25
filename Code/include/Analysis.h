#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
Eigen::Matrix3<double> getRotationMatrix(double theta, const Eigen::Vector3d& axis = Eigen::Vector3d(0, 0, 1));
