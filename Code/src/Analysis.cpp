#include "../include/Analysis.h"
#include "../include/PointDisplayer.h"
#include <vector>
#define NUM_FRM 24
#define SFILTER (sad<125 && sad>100)

using namespace Eigen;
Eigen::Matrix3<double> Analyzer::getRotationMatrix(double theta,const Eigen::Vector3d& axis)
{
	// convert the angle to radians, then return the rotation matrix retrieved from Eigen
	theta = EIGEN_PI * theta / 180;
	Eigen::AngleAxisd rotation(theta, axis.normalized());
	Eigen::Matrix3d rotation_matrix = rotation.toRotationMatrix();
	return rotation_matrix;
}

Eigen::Matrix3d explicitRotation(double angle)
{
	// used only for y rotations.
	// substitute the values in the matrix and return it.
	double theta = EIGEN_PI * angle / 180;
	Matrix3d mat = Matrix3d::Zero();
	mat(0, 0) = cos(theta);
	mat(0, 2) = sin(theta);
	mat(2, 0) = -sin(theta);
	mat(2, 2) = cos(theta);
	mat(1, 1) = 1;
	return mat;
}

void Analyzer::rotatePoints(vector<Eigen::Vector3d>& points, double angle, const Eigen::Vector3d& axis)
{
	// get rotation matrix, then multiply each point by it
	Eigen::Matrix3d mat;
	Eigen::Vector3d yaxis(0, 1, 0);
	if (axis != yaxis)
		mat = getRotationMatrix(angle, axis);
	else
		mat = explicitRotation(angle);
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
	// substitute the values in each entry, and set the matrix built flag to true
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
	// add coordinate 1 to each 2D point, then multiply it by the camera matrix
	vector<Vector3d> mapped(points.size());
	if (!matrixBuilt) buildCameraMatrix();
	for (int i = 0;i < points.size();i++)
	{
		mapped[i] = CameraMatrix * points[i].homogeneous();
	}
	return mapped;
}

vector<Eigen::Vector3d>  Analyzer::mapPoints(const vector<Eigen::Vector2d>& centers, const vector<Eigen::Vector2d>& mv, double dH,vector<double> SADs)
{
	// get the points C^-1 * c_p
	auto normalized = this->mapNormalizedPoints(centers);
	vector<Vector3d> points;
	// now calculate d = fy * Delta(H)/Delta(y)
	// SFILTER is the filtering policy on SAD.
	// if no SAD array is given, sads.size()=0, thus SFILTER will always be ignored.
	for (int i =0;i<normalized.size();i++)
	{
		double sad = SADs[i];
		if (mv[i](1) != 0 && dH != 0 && (SFILTER||SADs.size()==0))
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

vector<frames> Analyzer::importMV(const string& path)
{
	CSVFile file(path, NUM_FRM);
	file.openFile();
	return file.readFile();
}

vector<Eigen::Vector2d> Analyzer::getCenters()
{
	vector<Eigen::Vector2d> centers;
	for (int i = 0;i < ROWS;i++)
	{
		for (int j = 0; j < COLS;j++)
		{
			centers.push_back(Eigen::Vector2d(16 * i + 8, 16 * j + 8));
		}
	}
	return centers;
}

void Analyzer::continuize(vector<double>& heights)
{
	int i = 1;
	int j = 0;
	while (i < heights.size())
	{
		// move i to the next value
		while (i < heights.size() && heights[i] == heights[j]) i++;
		if (i == heights.size()) break;
		// add the difference to all indices between j and i
		double d = heights[i] - heights[j];
		double diff = d / (i - j);
		for (int k = j + 1; k < i;k++)
		{
			heights[k] = heights[k - 1] + diff;
		}
		// move j to i 
		j = i;
	}
}

void Analyzer::differences(vector<double>& vec)
{
	vector<double> tmp = vec;
	for (int i = 1;i < vec.size();i++)
	{
		vec[i] = tmp[i] - tmp[i - 1];
	}
}

vector<Eigen::Vector3d> Analyzer::extractPoints(string path, string heights_path, int angle)
{
	
	auto motionVectors = importMV(path);
	CSVFile height_file(heights_path, NUM_FRM);
	height_file.openFile();
	auto heights = height_file.readColumn();
	auto centers = getCenters();
	vector<Eigen::Vector3d> points;
	// continuize the heights function.
	continuize(heights);
	differences(heights);
	// map the points of each frame, and add it to the points cloud
	for (int i = 0;i < motionVectors.size();i++)
	{
		vector<Eigen::Vector3d> tmp = this->mapPoints(centers, motionVectors[i], heights[i]);
		points.insert(points.end(), tmp.begin(), tmp.end());
	}
	// rotate by the given angle
	Analyzer::rotatePoints(points, angle);
	return points;
}


