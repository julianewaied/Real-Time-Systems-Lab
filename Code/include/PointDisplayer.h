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
	const int HEIGHT = 300;
	const int WIDTH = 300;
	string window_name;
	
	void fitPoints(vector<Point2i>& points) const;
	void displayPoint(const Point2i& point, cv::Mat img) const;

public:
	PointDisplayer(string& window_name);

	void display(const vector<Eigen::Vector2d>& points) const;
	void display(const vector<cv::Point2i>& points) const;

	void topDownView(const vector<Eigen::Vector3d>& points) const;

	// takes in a video path and centers depth and plot the video with the rectangles.
	void showDepthMap(const string& path, const vector<double>& depths, const vector<frames> centers);

};