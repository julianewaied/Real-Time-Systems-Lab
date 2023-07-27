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
	std::cout << "x between : " << minX << " " << maxX << std::endl;
	int cx = (maxX + minX) / 2;
	int cy = (maxY + minY) / 2;
	int dx = std::abs(maxX - minX);
	int dy = std::abs(maxY - minY);
	for (int i = 0;i < points.size();i++)
	{
		points[i].x = (WIDTH - 10)  * (points[i].x - cx) / dx;
		points[i].y = (HEIGHT - 10) * (points[i].y - cy) / dy;
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
	cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
	cv::Mat img(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));

	for (auto point : fit) {
		displayPoint(point, img);
	}

	cv::imshow(window_name, img);
	cv::waitKey(0);
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