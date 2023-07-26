#include "PointDisplayer.h"

PointDisplayer::PointDisplayer(string& window_name, const cv::Mat& image, vector<Point2i>& points) :
	window_name(window_name), display_image(image),image(image), points(points) {}

void PointDisplayer::display() const {
	//display_image(image);
	cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);


	for (Point2i point :points){
		displayPoint(point);
	}

	cv::imshow(window_name, display_image);
	cv::waitKey(0);
}

void PointDisplayer::displayPoint(Point2i& point) const{
	Point2i p1 = point - Point2i(RECTANGLE_SIZE, RECTANGLE_SIZE);
	Point2i p2 = point + Point2i(RECTANGLE_SIZE, RECTANGLE_SIZE);

	cv::rectangle(display_image, p1, p2, cv::Scalar(0, 0, 255),cv::FILLED);
}