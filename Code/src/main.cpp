#include "../include/Analysis.h"
#include "../include/Utility.h"
#include "../include/PointDisplayer.h"
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
using std::cout;
using std::map;
using std::pair;
#define NUM_FRM 24
const double cx = 319.7108;
const double cy = 231.1376;
const double fx = 506.2113;
const double fy = 505.1260;
// file paths
vector<string> heights{
        "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/vertical rotation/heights csv/tello_heights_rise0.csv",
        "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/vertical rotation/heights csv/tello_heights_fall0.csv",
        "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/vertical rotation/heights csv/tello_heights_rise1.csv",
        "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/vertical rotation/heights csv/tello_heights_fall1.csv",
        "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/vertical rotation/heights csv/tello_heights_rise2.csv",
        "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/vertical rotation/heights csv/tello_heights_fall2.csv"
};
vector<string> mvs_paths{
    "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/vertical rotation/csv/rise0.csv",
    "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/vertical rotation/csv/fall0.csv",
    "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/vertical rotation/csv/rise1.csv",
    "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/vertical rotation/csv/fall1.csv",
    "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/vertical rotation/csv/rise2.csv",
    "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/vertical rotation/csv/fall2.csv"
};


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
    for (int i = 0;i < vec.size()-1;i++)
    {
        vec[i] = tmp[i+1] - tmp[i];
    }
    vec.pop_back();
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

void BuildDepthMap(const string& path,const string& heights_path)
{
    auto points = extractPoints(path, heights_path, 0);
    string name = "Depth Map";
    PointDisplayer displayer(name);
    displayer.showDepthMap(points);
}

void Testing()
{
    
}

void countFile(const string& path)
{
    auto mvs = importMV(path);
    map<double, int> all;
    for (int i = 0;i < mvs.size();i++)
    {
        frames& frm = mvs[i];
        for (int j = 0;j < frm.size();j++)
        {
            all[frm[j](1)]++;
        }
    }
    for (auto it = all.begin(); it != all.end(); ++it) {
        const double key = it->first;
        //std::cout << key << " : " << it->second << std::endl;
    }

}

int Run()
{
    string path = mvs_paths[0];
    string heights_path = heights[0];
    auto motionVectors = importMV(path);
    CSVFile height_file(heights_path, NUM_FRM);
    height_file.openFile();
    auto heights = height_file.readColumn();
    auto centers = getCenters();
    string window_name = "Depth Map";
    const int ROWS = 104;
    const int COLS = 77;
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
    cv::Mat img(ROWS * 16, COLS * 16, CV_8UC3, cv::Scalar(255, 255, 255));
    for(int i=0;i<ROWS;i++)
    {
        for (int j = 0;j < COLS;j++)
        {
            cv::Point p1(16 * i, 16 * j), p2(16 * i + 16, 16*j + 16);
            cv::rectangle(img, p1, p2, cv::Scalar(255, 0, 0), 10, cv::FILLED);
        }
    }
    cv::imshow(window_name, img);
    cv::waitKey(0);
    return 0;
}