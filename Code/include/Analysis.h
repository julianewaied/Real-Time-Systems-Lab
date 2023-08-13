#pragma once
#include "../include/Utility.h"
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <Eigen/Geometry>
#define FILTER (std::abs(s)>250&& d!=0)
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
	
	// returns a list of MV for each frame.
	static vector<frames> importMV(const string& path);

	// returns points of a single video
	vector<Eigen::Vector3d> extractPoints(string path, string heights_path, int angle);
	
	static void rotatePoints(vector<Eigen::Vector3d>& points, double angle, const Eigen::Vector3d& axis = Eigen::Vector3d(0, 1, 0));
	
	vector<double> getDepths(const frames& mv, double dH);

	// given a frame and its mv, with height difference, return the full map, without rotation
	// assumes center[i] matches mv[i].
	vector<Eigen::Vector3d>  mapPoints(const vector<Eigen::Vector2d>& centers, const vector<Eigen::Vector2d>& mv, double dH,vector<double> SADs = vector<double>());
	
	// returns the center of the frame!
	static vector<Eigen::Vector2d> getCenters();

	// fixing sensor inaccuracy
	static void continuize(vector<double>& heights);

	static void differences(vector<double>& vec);
};
