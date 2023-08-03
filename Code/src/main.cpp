#include "../include/Analysis.h"
#include "../include/Utility.h"
#include "../include/PointDisplayer.h"
#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#define NUM_FRM 199
double cx = 319.7108;
double cy = 231.1376;
double fx = 506.2113;
double fy = 505.1260;
const string path = "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/close/close.csv";
const string heights_path = "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/close/tello_heights.csv";
using std::cout;
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

void continuize(vector<double>& heights)
{
    int i = 1;
    int j = 0;
    while (i < heights.size())
    {
        while (i < heights.size() && heights[i] == heights[j]) i++;
        if (i == heights.size()) break;
        double d = heights[i] - heights[j];
        double diff = d / (i - j);
        for (int k = j + 1; k < i;k++)
        {
            heights[k] = heights[k - 1] + diff;
        }
        j = i;
    }
}
void differences(vector<double>& vec)
{
    vector<double> tmp = vec;
    for (int i = 1;i < vec.size();i++)
    {
        vec[i] = tmp[i] - tmp[i - 1];
    }
}
int Run() {
    auto motionVectors = importMV(path);
    CSVFile height_file(heights_path,NUM_FRM);
    height_file.openFile();
    auto heights = height_file.readColumn();
    auto centers = getCenters();
    Analyzer analyzer(fx, fy, cx, cy);
    vector<Eigen::Vector3d> points;
    // continuize the heights function.
    continuize(heights);
    differences(heights);
    for (int i=0;i<motionVectors.size();i++)
    {
        vector<Eigen::Vector3d> tmp = analyzer.mapPoints(centers, motionVectors[i], heights[i]);
        points.insert(points.end(), tmp.begin(), tmp.end());
    }
    string window_name = "Room Map";
    PointDisplayer displayer(window_name);
    displayer.topDownView(points);
    return 0;
}