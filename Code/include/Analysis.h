#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
class Analyzer
{
	double fx, fy;
	int cx, cy;
	Eigen::Matrix3d CameraMatrix;
public:
	Analyzer(double fx = 1, double fy = 1, int cx = 0, int cy = 0) : fx(fx), fy(fy), cx(cx), cy(cy), CameraMatrix(Eigen::Matrix3d::Zero()) {};
	// returns C^-1 where C is the camera matrix.
	static Eigen::Matrix3<double> getRotationMatrix(double theta, const Eigen::Vector3d& axis = Eigen::Vector3d(0, 0, 1));
	const Eigen::Matrix3d& getCameraMatrix() const;
	const Eigen::Matrix3d& buildCameraMatrix();
};
