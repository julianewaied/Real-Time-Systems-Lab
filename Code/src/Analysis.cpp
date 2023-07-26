#include "../include/Analysis.h"
using namespace Eigen;
Eigen::Matrix3<double> Analyzer::getRotationMatrix(double theta,const Eigen::Vector3d& axis)
{
	theta = EIGEN_PI * theta / 180;
	Eigen::AngleAxisd rotation(theta, axis.normalized());
	Eigen::Matrix3d rotation_matrix = rotation.toRotationMatrix();
	return rotation_matrix;
}
const Eigen::Matrix3d& Analyzer::getCameraMatrix() const
{
	return this->CameraMatrix;
}
const Eigen::Matrix3d& Analyzer::buildCameraMatrix()
{
	CameraMatrix(0, 0) = 1 / fx;
	CameraMatrix(1, 1) = 1 / fy;
	CameraMatrix(0, 2) = -cx / fx;
	CameraMatrix(1, 2) = -cy / fy;
	return CameraMatrix;
}


