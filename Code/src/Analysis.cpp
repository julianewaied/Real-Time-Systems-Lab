#include "../include/Analysis.h"
using namespace Eigen;
Eigen::Matrix3<double> Analyzer::getRotationMatrix(double theta,const Eigen::Vector3d& axis)
{
	theta = EIGEN_PI * theta / 180;
	Eigen::AngleAxisd rotation(theta, axis.normalized());
	Eigen::Matrix3d rotation_matrix = rotation.toRotationMatrix();
	return rotation_matrix;
}

Eigen::Matrix3d explicitRotation(int angle)
{
	double theta = EIGEN_PI * angle / 180;
	Matrix3d mat = Matrix3d::Zero();
	mat(0, 0) = cos(theta);
	mat(0, 2) = sin(theta);
	mat(2, 0) = -sin(theta);
	mat(2, 2) = cos(theta);
	return mat;
}

void Analyzer::rotatePoints(vector<Eigen::Vector3d>& points, double angle, const Eigen::Vector3d& axis)
{
	auto mat = getRotationMatrix(angle, axis);
	for (int i =0;i<points.size();i++)
	{
		points[i] = mat * points[i];
	}
}

const Eigen::Matrix3d& Analyzer::getCameraMatrix() const
{
	return this->CameraMatrix;
}

const Eigen::Matrix3d& Analyzer::buildCameraMatrix()
{
	if (matrixBuilt) return CameraMatrix;
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
	vector<Vector3d> points;
	// now calculate d = fy * Delta(H)/Delta(y)
	// assuming constant partial derivative of H.
	for (int i =0;i<normalized.size();i++)
	{
		if(mv[i](1) != 0)
			points.push_back(normalized[i] * std::abs(fy * dH / mv[i](1)));
	}
	return points;
}

vector<double> Analyzer::getDepths(const frames& mv, double dH)
{
	vector<double> depths;
	for (int i = 0;i < mv.size();i++)
	{
		if (mv[i](1))
			depths.push_back(fy * dH / std::abs(mv[i](1)));
		else
			depths.push_back(0);
	}
	return depths;
}

