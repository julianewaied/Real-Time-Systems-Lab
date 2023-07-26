#pragma once
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <string>
using std::vector;
using std::string;
using cv::Point2i;

class PointDisplayer {
	const int RECTANGLE_SIZE = 20;
	string window_name;
	
	const cv::Mat& image;
	cv::Mat display_image;
	
	const vector<Point2i>& points;

	void displayPoint(Point2i &point) const;

public:
	PointDisplayer(string& window_name, const cv::Mat& image, vector<Point2i>& points);

	void display() const;


};