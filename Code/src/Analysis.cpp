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
	CameraMatrix(2, 2) = 1;
	matrixBuilt = true;
	return CameraMatrix;
}
vector<Eigen::Vector3d> Analyzer::mapNormalizedPoints(const vector<Eigen::Vector2d>& points)
{
	vector<Vector3d> mapped(points.size());
	if (!matrixBuilt) buildCameraMatrix();
	for (int i=0;i<points.size();i++)
		mapped[i] = CameraMatrix * points[i].homogeneous();
	return mapped;
}
vector<Eigen::Vector3d>  Analyzer::mapPoints(const vector<Eigen::Vector2d>& centers, const vector<Eigen::Vector2d>& mv, double dH)
{
	auto normalized = this->mapNormalizedPoints(centers);
	// now calculate d = fy * Delta(H)/Delta(y)
	// assuming constant partial derivative of H.
	vector<double> depths(centers.size());
	for (int i = 0; i < depths.size();i++)
	{
		depths[i] = fy * dH / mv[i](1);
	}
	for (int i =0;i<normalized.size();i++)
	{
		normalized[i] = normalized[i] * depths[i];
	}
	return normalized;
}
