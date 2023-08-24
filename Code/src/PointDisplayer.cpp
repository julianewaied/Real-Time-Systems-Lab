#include "../include/PointDisplayer.h"
#include <iostream>
#include "../include/Analysis.h"
#include <map>
#define NUM_FRM 24
#define NORM(x,y) (x*x+y*y)
#define FILTER_RADIUS 5E3
// camera properties for convenience.
const double cx = 319.7108;
const double cy = 231.1376;
const double fx = 506.2113;
const double fy = 505.1260;


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
	// convert all vectors into cv::Point2i, then use the display function for cv::Point2i
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
	// step 1: build a new empty window
	auto fit = points;
	fitPoints(fit);
	cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
	cv::Mat img(HEIGHT, WIDTH, CV_8UC3, cv::Scalar(255, 255, 255));

	// step 2: plot each point on the image
	for (auto point : fit) {
		displayPoint(point, img);
	}

	// step 3: plot the image 
	cv::imshow(window_name, img);
	cv::waitKey(0);
}

void PointDisplayer::topDownView(const vector<Eigen::Vector3d>& points) const
{
	// convert each (x,y,z) point into (x,z) cv::Point2i, then use the display function.
	vector<cv::Point2i> points2d;
	for (int i = 0;i < points.size();i++)
	{
		if(NORM(points[i](0),points[i](2))< FILTER_RADIUS)
			points2d.push_back(Point2i(points[i](0), points[i](2)));
	}
	display(points2d);
}

int PointDisplayer::BuildTDView(vector<string> mvFiles, vector<string> heightFiles)
{
	if (mvFiles.size() != heightFiles.size()) throw "Invalid sizes in BuildTDView";
	vector<Eigen::Vector3d> points;
	Analyzer a(fx, fy, cx, cy);
	// proccess points
	for (int i = 0;i < mvFiles.size();i++)
	{
		auto tmp = a.extractPoints(mvFiles[i], heightFiles[i], 60 * i);
		std::cout << "Processing Angle : " << 60 * i << std::endl;
		points.insert(points.end(), tmp.begin(), tmp.end());
	}
	// show topdown view of the points
	string window_name = "Room Map";
	PointDisplayer displayer(window_name);
	displayer.topDownView(points);
	return 0;
}

void PointDisplayer::BuildDepthMap(const string& path, const string& videoPath, vector<vector<double>> sads)
{
	const int frame_num = 5;
	const int num_vid = 4;
	// get all required data
	auto motionVectors = Analyzer::importMV(path);
	auto centers = Analyzer::getCenters();
	string window_name = "Depth Map";
	vector<cv::Mat> frms(NUM_FRM);
	static cv::VideoCapture cap(videoPath);
	// read the video
	if (!cap.isOpened()) {
		std::cout << "Error opening video file." << std::endl;
		exit(-1);
	}
	for (int i = 0;i < NUM_FRM && cap.isOpened();i++)
	{
		cap >> frms[i];
	}
	cap.release();
	// build a window
	cv::namedWindow(window_name);
	int k = 0;
	while(true)
	{
		// k is the number of frame we are showing
		double maxy, miny;
		auto& mvs = motionVectors[k];
		auto& sad = sads[k];
		vector<double> depths;
		for (auto mv : mvs) depths.push_back(mv(1));
		maxy = miny = depths[0];
		// finds the maximum and the minimum depths, limited to SAD filtering
		for (int i = 0;i < depths.size();i++)
		{
			double d = depths[i];
			double s;
			if (sads.size())
				s = sad[i];
			else
				s = 0;
			if (FILTER)
			{
				maxy = std::max(maxy, d);
				miny = std::min(miny, d);
			}
		}

		cv::Mat resizedFrame;
		cv::resize(frms[k], resizedFrame, cv::Size(), 0.5, 0.5); // Resize image to fit

		// for each center, plot a rectangle with a color proportional to its depth multiplicative inverse (dy)
		// applying the SAD filtering 
		for (int i = 0;i < ROWS;i += 2)
		{
			for (int j = 0; j < COLS; j += 2)
			{
				int ij = i * COLS + j;
				double dy = (depths[ij] - miny) / (maxy - miny);
				cv::Point p1(8 * i, 8 * j), p2(8 * i + 8, 8 * j + 8);
				int s = sads.size()?sad[ij]:0;
				double d = dy;
				if (FILTER)
					cv::rectangle(resizedFrame, p1, p2, cv::Scalar(dy * 255, dy * 255 / 2, dy * 255 / 2), cv::FILLED);
			}
		}

		// show the frame with the rectangles
		cv::imshow(window_name, resizedFrame);
		int key = 'X';
		// wait for keys 'S' and 'W'
		while (key != 'S' && key != 'W')
		{
			key = toupper(cv::waitKey(0));
		}
		// if we got key 'W', we should progress in the frames
		if (key == 'W') k = std::min(k + 1, static_cast<int>(motionVectors.size()) - 1);
		// otherwise, we should go back in frames
		else if (key == 'S') k = std::max(k - 1, 0);
	}
	cv::waitKey(0);
	cv::destroyAllWindows();

}
