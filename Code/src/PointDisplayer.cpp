#include "../include/PointDisplayer.h"
#include <iostream>
PointDisplayer::PointDisplayer(string& window_name):
	window_name(window_name) {}

//void PointDisplayer::fitPoints(vector<Point2i>& points) const
//{
//	int maxX, maxY;
//	int minX, minY;
//	maxX = minX = points[0].x;
//	maxY = minY = points[0].y;
//	for (auto p : points)
//	{
//		maxX = std::max(p.x, maxX);
//		maxY = std::max(p.y, maxY);
//		minY = std::min(p.y, minY);
//		minX = std::min(p.x, minX);
//	}
//	int cx = (maxX + minX) / 2;
//	int cy = (maxY + minY) / 2;
//	int dx = std::abs(maxX - minX);
//	int dy = std::abs(maxY - minY);
//	dy = dy > 0 ? dy : abs(maxY)*2;
//	dx = dx > 0 ? dx : abs(maxX)*2;
//	// not final at all!!!!
//	for (int i = 0;i < points.size();i++)
//	{
//		points[i].x = (WIDTH * 0.8)  * (points[i].x -minX) / dx;
//		points[i].y = (HEIGHT * 0.8) * (points[i].y -minY) / dy;
//	}
//}

void PointDisplayer::fitPoints(std::vector<cv::Point2i>& points) const {
		int numPoints = points.size();
		if (numPoints == 0) {
			return; // No points to fit
		}

		// Find the bounding box of the points
		cv::Rect boundingBox = cv::boundingRect(points);

		// Calculate the scale factors to fit the points into the screen
		double scaleX = (boundingBox.width == 0) ? 1.0 : (static_cast<double>(WIDTH) / boundingBox.width)/2;
		double scaleY = (boundingBox.height == 0) ? 1.0 : (static_cast<double>(HEIGHT) / boundingBox.height)/2;
		double scaleFactor = std::min(scaleX, scaleY);

		// Calculate the translation to center the points
		int offsetX = (WIDTH - static_cast<int>(scaleFactor * boundingBox.width)) / 2;
		int offsetY = (HEIGHT - static_cast<int>(scaleFactor * boundingBox.height)) / 2;

		// Resize and center each point
		for (cv::Point2i& point : points) {
			point.x = static_cast<int>((point.x - boundingBox.x) * scaleFactor + offsetX);
			point.y = static_cast<int>((point.y - boundingBox.y) * scaleFactor + offsetY);
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