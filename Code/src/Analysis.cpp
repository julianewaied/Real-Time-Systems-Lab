#include "../include/Analysis.h"
Eigen::Matrix3<double> getRotationMatrix(double theta,Eigen::Vector3d axis)
{
	Eigen::AngleAxis<double> rotation(theta, axis);
}