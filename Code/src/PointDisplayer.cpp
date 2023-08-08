#include "../include/PointDisplayer.h"
#include <iostream>
#include <map>
PointDisplayer::PointDisplayer(string& window_name):
	window_name(window_name) {}


void PointDisplayer::fitPoints(std::vector<cv::Point2i>& points) const {
		int numPoints = points.size();
		if (numPoints == 0) {
			return; // No points to fit
		}

		// Find the bounding box of the points
		cv::Rect boundingBox = cv::boundingRect(points);

		// Calculate the scale factors to fit the points into the screen
		double scaleX = (boundingBox.width == 0) ? 1.0 : (static_cast<double>(WIDTH) / boundingBox.width)/1.05;
		double scaleY = (boundingBox.height == 0) ? 1.0 : (static_cast<double>(HEIGHT) / boundingBox.height)/1.05;
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

double calculateMean(const std::vector<double>& depths) {
	double sum = 0.0;
	for (const double& depth : depths) {
		sum += depth;
	}
	return sum / depths.size();
}

double calculateStandardDeviation(const std::vector<double>& depths, double mean) {
	double variance = 0.0;
	for (const double& depth : depths) {
		variance += std::pow(depth - mean, 2);
	}
	variance /= depths.size();
	return std::sqrt(variance);
}

void PointDisplayer::showDepthMap(const vector<Eigen::Vector3d>& points) const 
{
	vector<double> depths;
	double maxd = points[0](2), mind = points[0](2);
	for (auto p : points)
	{
		maxd = std::max(maxd, p(2));
		mind = std::min(mind, p(2));
	}
	double d = maxd - mind;
	for (auto p : points)
	{
		depths.push_back((p(2) - mind) / d);
	}
	vector<Point2i> fit;
	for (int i = 0;i < points.size();i++)
	{
		fit.push_back(Point2i(points[i](0), points[i](1)));
	}
	fitPoints(fit);
	cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
	cv::Mat img(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));

	for (int i = 0;i < fit.size();i++) {
		Point2i& point = fit[i];
		cv::circle(img, point, 1, cv::Scalar(255 , 255 * depths[i], 255 * depths[i]), 2);
	}
	cv::imshow(window_name, img);
	cv::waitKey(0);

}