#include "../include/Analysis.h"
#include "../include/Utility.h"
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

int Run() {
    // Sample STL vector of 3D Eigen vectors
    std::vector<Eigen::Vector3d> points3d;
    points3d.push_back(Eigen::Vector3d(100.0, 2.0, 300.0));
    points3d.push_back(Eigen::Vector3d(100.0, 5.0, 60.0));
    points3d.push_back(Eigen::Vector3d(170.0, 8.0, 190.0));

    // Extract (x(0), x(2)) components from each 3D Eigen vector
    std::vector<cv::Point> points2d;
    for (const auto& point3d : points3d) {
        cv::Point point2d(static_cast<int>(point3d[0]), static_cast<int>(point3d[2]));
        points2d.push_back(point2d);
    }

    // Create an OpenCV window to display the points
    cv::namedWindow("Points", cv::WINDOW_AUTOSIZE);

    // Create an empty black image
    cv::Mat image = cv::Mat::ones(500, 500, CV_8UC3)*255;

    // Draw the points on the image
    for (const auto& point2d : points2d) {
        cv::circle(image, point2d, 5, cv::Scalar(0, 255, 0), -1);
    }

    // Show the image with points
    cv::imshow("Points", image);

    // Wait for a key press and then close the window
    cv::waitKey(0);
    cv::destroyAllWindows();

    return 0;
}