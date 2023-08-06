#pragma once
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>
#include <vector>
#include <string>
using std::vector;
using std::string;
using cv::Point2i;
class PointDisplayer {
	const int CIRCLE_RADIUS = 2;
	const int HEIGHT = 700;
	const int WIDTH = 700;
	const int RECTANGLE_SIZE = 3;
	const double Z_NORMAL = 200;
	const double MIN= -90;
	const double DIFF = 170;
	string window_name;

	void fitPoints(vector<Point2i>& points) const;
	void displayPoint(const Point2i& point, cv::Mat img) const;
	void displayRect(const Point2i& point, cv::Mat &img,const cv::Scalar &color= cv::Scalar(0, 0, 255)) const;
	void makeWindow(const cv::Mat &img) const;
public:
	PointDisplayer(string& window_name);

	void depthImage(cv::Mat img, const vector<Eigen::Vector2d>& points, const vector<double>& depths, const vector<Eigen::Vector2d>& mvs) const;
	void display(const vector<Eigen::Vector2d>& points) const;
	void display(const vector<cv::Point2i>& points) const;
	//will be deleted
	void depthMap(const vector<Eigen::Vector3d>& points,cv::Mat img) const;
	
	void topDownView(const vector<Eigen::Vector3d>& points) const;

};