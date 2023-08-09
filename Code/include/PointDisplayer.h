#pragma once
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>
#include <vector>
#include <string>
#include "../include/Utility.h"
using std::vector;
using std::string;
using cv::Point2i;
class PointDisplayer {
	const int CIRCLE_RADIUS = 2;
	const int HEIGHT = 700;
	const int WIDTH = 700;
	string window_name;
	
	void fitPoints(vector<Point2i>& points) const;
	void displayPoint(const Point2i& point, cv::Mat img) const;

public:
	PointDisplayer(string& window_name);

	void display(const vector<Eigen::Vector2d>& points) const;
	void display(const vector<cv::Point2i>& points) const;

	void topDownView(const vector<Eigen::Vector3d>& points) const;


	// builds top-down view of all the files and plots it
	static int BuildTDView(vector<string> mvFiles, vector<string> heightFiles);

	// builds depth map of one file and plots it
	static void BuildDepthMap(const string& path, const string& videoPath, vector<vector<double>> sads);

};