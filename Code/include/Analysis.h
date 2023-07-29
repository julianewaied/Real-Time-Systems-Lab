#pragma once
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <Eigen/Geometry>
using std::vector;
class Analyzer
{
protected:
	double fx, fy;
	int cx, cy;
	bool matrixBuilt;
	Eigen::Matrix3d CameraMatrix;

	
	const Eigen::Matrix3d& getCameraMatrix() const;

	static Eigen::Matrix3<double> getRotationMatrix(double theta, const Eigen::Vector3d& axis = Eigen::Vector3d(0, 0, 1));
	
	// mapping into z-normalized map
	vector<Eigen::Vector3d> mapNormalizedPoints(const vector<Eigen::Vector2d>& points);

public:

	Analyzer(double fx = 1, double fy = 1, int cx = 0, int cy = 0) : fx(fx), fy(fy), cx(cx), cy(cy), CameraMatrix(Eigen::Matrix3d::Zero()),matrixBuilt(false) {};
	
	// returns C^-1 where C is the camera matrix.
	const Eigen::Matrix3d& buildCameraMatrix();
	
	// given a frame and its mv, with height difference, return the full map, without rotation
	// assumes center[i] matches mv[i].
	vector<Eigen::Vector3d>  mapPoints(const vector<Eigen::Vector2d>& centers, const vector<Eigen::Vector2d>& mv, double dH);
};
