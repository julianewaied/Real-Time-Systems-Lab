#include "../include/Analysis.h"
#include "../include/Utility.h"
#include "../include/PointDisplayer.h"
#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#define NUM_FRM 199
double fx = 319.7108;
double fy = 231.1376;
double cx = 506.2113;
double cy = 505.1260;
const string path = "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/motion_data_rise.csv";

// returns a list of MV for each frame.
vector<frames> importMV(const string& path)
{
    CSVFile file(path,NUM_FRM);
    file.openFile();
    return file.readFile();
}

// returns the center of the frame!
vector<Eigen::Vector2d> getCenters()
{
    vector<Eigen::Vector2d> centers;
    for (int i = 0;i < ROWS;i++)
    {
        for (int j = 0; j < COLS;j++)
        {
            centers.push_back(Eigen::Vector2d(16 * i + 8, 16 * j + 8));
        }
    }
    return centers;
}
int Run() {
    auto motionVectors = importMV(path);
    auto centers = getCenters();
    Analyzer analyzer(fx, fy, cx, cy);
    vector<Eigen::Vector3d> points = analyzer.mapPoints(centers,motionVectors[6], 1);
    string window_name = "Room Map";
    PointDisplayer displayer(window_name);
    displayer.topDownView(points);
    return 0;
}