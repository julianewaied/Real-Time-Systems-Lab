#include "../include/Analysis.h"
using namespace Eigen;
Eigen::Matrix3<double> getRotationMatrix(double theta,const Eigen::Vector3d& axis)
{
	theta = EIGEN_PI * theta / 180;
	Eigen::AngleAxisd rotation(theta, axis.normalized());
	Eigen::Matrix3d rotation_matrix = rotation.toRotationMatrix();
	return rotation_matrix;
}