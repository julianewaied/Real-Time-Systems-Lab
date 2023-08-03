#include "../include/Analysis.h"
#include "../include/Utility.h"
#include "../include/PointDisplayer.h"
#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#define NUM_FRM 24
const double cx = 319.7108;
const double cy = 231.1376;
const double fx = 506.2113;
const double fy = 505.1260;
using std::cout;
using std::set;
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

vector<Eigen::Vector3d> extractPoints(string path, string heights_path,int angle)
{
    auto motionVectors = importMV(path);
    CSVFile height_file(heights_path, NUM_FRM);
    height_file.openFile();
    auto heights = height_file.readColumn();
    auto centers = getCenters();
    Analyzer analyzer(fx, fy, cx, cy);
    vector<Eigen::Vector3d> points;
    // continuize the heights function.
    continuize(heights);
    differences(heights);
    for (int i = 0;i < motionVectors.size();i++)
    {
        vector<Eigen::Vector3d> tmp = analyzer.mapPoints(centers, motionVectors[i], heights[i]);
        points.insert(points.end(), tmp.begin(), tmp.end());
    }
    Analyzer::rotatePoints(points,angle);
    return points;
}
void showTD(vector<Eigen::Vector3d> points)
{
    string window_name = "Room Map";
    PointDisplayer displayer(window_name);
    displayer.topDownView(points);
}
int BuildTDView(vector<string> mvFiles, vector<string> heightFiles) 
{

    if (mvFiles.size() != heightFiles.size()) throw "Invalid sizes in BuildTDView";
    vector<Eigen::Vector3d> points;
    for (int i = 0;i < mvFiles.size();i++)
    {
        auto tmp = extractPoints(mvFiles[i], heightFiles[i], 60*i);
        std::cout << "Processing Angle : " << 60 * i << std::endl;
        points.insert(points.end(), tmp.begin(), tmp.end());
    }
    showTD(points);
    return 0;
}
//int BuildDepthMap()
//{
//    auto motionVectors = importMV(path);
//    CSVFile height_file(heights_path, NUM_FRM);
//    height_file.openFile();
//    auto heights = height_file.readColumn();
//    auto centers = getCenters();
//    Analyzer analyzer(fx, fy, cx, cy);
//    vector<Eigen::Vector3d> points;
//    // continuize the heights function.
//    continuize(heights);
//    differences(heights);
//    // get the depths of all the centers. if 0 don't draw the center
//    vector<vector<double>> depths;
//    for (int i = 0;i < motionVectors.size();i++)
//    {
//        depths.push_back(analyzer.getDepths(motionVectors[i], heights[i]));
//    }
//}
void Testing()
{
    vector<Eigen::Vector2d> motionVectors {
        Eigen::Vector2d(0,1),
        Eigen::Vector2d(0,1),
        Eigen::Vector2d(0,2),
        Eigen::Vector2d(0,4)
    };
    double height = 2;
    vector<Eigen::Vector2d> centers{
        Eigen::Vector2d(1,0),
        Eigen::Vector2d(0,1),
        Eigen::Vector2d(1,1),
        Eigen::Vector2d(0,0)
    };
    // (0,-2,2), (-2,0,2) ,  (0,0,1) , (-0.5,-0.5,0.5)
    Analyzer analyzer(1, 1, 1, 1);
    /*continuize(heights);
    differences(heights);*/
    vector<Eigen::Vector3d> points = analyzer.mapPoints(centers, motionVectors, height);
    //Analyzer::rotatePoints(points, 90);
    for (auto point : points)
    {
        std::cout << point.transpose() << std::endl;
    }
    showTD(points);
}
int Run()
{
    vector<string> heights{
        "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/vertical rotation/heights csv/tello_heights_rise0.csv",
        "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/vertical rotation/heights csv/tello_heights_fall0.csv",
        "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/vertical rotation/heights csv/tello_heights_rise1.csv",
        "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/vertical rotation/heights csv/tello_heights_fall1.csv",
        "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/vertical rotation/heights csv/tello_heights_rise2.csv",
        "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/vertical rotation/heights csv/tello_heights_fall2.csv"
    };
    vector<string> mvs{
        "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/vertical rotation/csv/rise0.csv",
        "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/vertical rotation/csv/fall0.csv",
        "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/vertical rotation/csv/rise1.csv",
        "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/vertical rotation/csv/fall1.csv",
        "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/vertical rotation/csv/rise2.csv",
        "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/vertical rotation/csv/fall2.csv"
    };
    //BuildTDView(mvs,heights);
    Testing();
    return 0;
}