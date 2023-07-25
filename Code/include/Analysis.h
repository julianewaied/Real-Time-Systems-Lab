#pragma once
#include <Eigen/Dense>
Eigen::Matrix3<double> getRotationMatrix(double theta, Eigen::Vector3d axis = = Eigen::Vector3d(0, 0, 1));