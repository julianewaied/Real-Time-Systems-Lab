#include "../include/PointDisplayer.h"
#include <iostream>
PointDisplayer::PointDisplayer(string& window_name):
	window_name(window_name) {}

void PointDisplayer::fitPoints(vector<Point2i>& points) const
{
	int maxX, maxY;
	int minX, minY;
	maxX = minX = points[0].x;
	maxY = minY = points[0].y;
	for (auto p : points)
	{
		maxX = std::max(p.x, maxX);
		maxY = std::max(p.y, maxY);
		minY = std::min(p.y, minY);
		minX = std::min(p.x, minX);
	}
	int cx = (maxX + minX) / 2;
	int cy = (maxY + minY) / 2;
	int dx = std::abs(maxX - minX);
	int dy = std::abs(maxY - minY);
	dy = dy > 0 ? dy : abs(maxY)*2;
	dx = dx > 0 ? dx : abs(maxX)*2;
	// not final at all!!!!
	for (int i = 0;i < points.size();i++)
	{
		points[i].x = (WIDTH * 0.8)  * (points[i].x -minX) / dx;
		points[i].y = (HEIGHT * 0.8) * (points[i].y -minY) / dy;
	}
}

void PointDisplayer::display(const vector<Eigen::Vector2d>& points) const {
	vector<cv::Point2i> points2d(points.size());

	for (int i = 0;i < points.size();i++)
	{
		points2d[i].x = points[i](0);
		points2d[i].y = points[i](1);
	}
	display(points2d);
}

inline void PointDisplayer::displayPoint(const Point2i& point,cv::Mat img) const{
	cv::circle(img, point, CIRCLE_RADIUS, cv::Scalar(0, 0, 255),2);
}

void PointDisplayer::display(const vector<cv::Point2i>& points) const
{
	auto fit = points;
	fitPoints(fit);
	
	cv::Mat img(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));

	for (auto point : fit) {
		displayPoint(point, img);
	}

	makeWindow(img);
}
void PointDisplayer::makeWindow(const cv::Mat& img) const {
	cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
	cv::imshow(window_name, img);
	cv::waitKey(0);
}


void PointDisplayer::displayRect(const Point2i& point, cv::Mat &img,const cv::Scalar &color) const {
	cv::rectangle(img, point + cv::Point(-RECTANGLE_SIZE, -RECTANGLE_SIZE),
		point + cv::Point(RECTANGLE_SIZE, RECTANGLE_SIZE), color, cv::FILLED);
}

void PointDisplayer::depthMap(const vector<Eigen::Vector3d>& points, cv::Mat img) const{

	vector<cv::Point2i> points2d(points.size());
	for (int i = 0; i < points.size(); i++)
	{
		points2d[i].x = points[i](0);
		points2d[i].y = points[i](1);
	}
	
	fitPoints(points2d);
	for (int i = 0; i < points2d.size(); i++) {
		//display the point
		double r_value = 255 - ((points[i](2) / Z_NORMAL) * 255);
		displayRect(points2d[i], img, cv::Scalar(r_value, 0, 0));
	}

	makeWindow(img);
}

void PointDisplayer::topDownView(const vector<Eigen::Vector3d>& points) const
{
	vector<cv::Point2i> points2d(points.size());
	for(int i=0;i<points.size();i++)
	{
		points2d[i].x = points[i](0);
		points2d[i].y = points[i](2);
	}
	display(points2d);
}


void PointDisplayer::depthImage(cv::Mat img, const vector<Eigen::Vector2d>& points, const vector<double>& depths) const {
	vector<cv::Point2i> pixels(points.size());
	for (int i = 0; i < points.size(); i++)
	{
		pixels[i].x = points[i](0);
		pixels[i].y = points[i](1);
	}

	for (int i = 0; i < pixels.size(); i++) {
		//display the point
		double r_value = 255 - ((depths[i]/ Z_NORMAL) * 255);
		displayRect(pixels[i], img, cv::Scalar(r_value, 0, 0));
	}

	makeWindow(img);
}