#pragma once
#include "../include/Utility.h"
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <Eigen/Geometry>
#define FILTER ((sads.size()==0 || (100<s && s<125))&& d!=0)
using std::vector;
class Analyzer
{
protected:

	double fx, fy;
	int cx, cy;
	bool matrixBuilt;
	Eigen::Matrix3d CameraMatrix;


	const Eigen::Matrix3d& getCameraMatrix() const;

	// returns the rotation matrix around axis.
	// default value of axis is y-axis
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
	
	// rotates the points around axis with angle
	// default axis: y axis
	static void rotatePoints(vector<Eigen::Vector3d>& points, double angle, const Eigen::Vector3d& axis = Eigen::Vector3d(0, 1, 0));
	
	// returns the depths of the points in the corresponding order
	vector<double> getDepths(const frames& mv, double dH);


	//given a frame and its mv, with height difference, return the full map, without rotation 
	//assumes center[i] matches mv[i].
	//if given SADs, filters SADs by the interval [100,125]
	vector<Eigen::Vector3d>  mapPoints(const vector<Eigen::Vector2d>& centers, const vector<Eigen::Vector2d>& mv, double dH,vector<double> SADs = vector<double>());
	
	// returns the center of the frame!
	static vector<Eigen::Vector2d> getCenters();

	// fixing sensor inaccuracy
	static void continuize(vector<double>& heights);

	// converts heights vector into heights difference vector
	static void differences(vector<double>& vec);
};
