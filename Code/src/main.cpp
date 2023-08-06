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
const string path = "C:/Users/Malek/Dropbox/My PC (DESKTOP-CUL8BRV)/Documents/GitHub/Real-Time-Systems-Lab/Code/Data/close/close.csv";
const string heights_path = "C:/Users/Malek/Dropbox/My PC (DESKTOP-CUL8BRV)/Documents/GitHub/Real-Time-Systems-Lab/Code/Data/close/tello_heights.csv";
const string VIDEO_PATH = "C:/Users/Malek/Dropbox/My PC (DESKTOP-CUL8BRV)/Documents/GitHub/Real-Time-Systems-Lab/Code/Data/far/rotrise.h264";
//const string VIDEO_PATH = "C:/Users/Malek/Dropbox/My PC(DESKTOP - CUL8BRV)/Documents/GitHub/Real - Time - Systems - Lab/Code/src/video.mp4";
//const string VIDEO_PATH = "C:/Users/Malek/Videos/Captures/video.mp4";
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
            centers.push_back(Eigen::Vector2d(16*j+8 , 16 *i+8 ));
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

    cv::VideoCapture cap;
    //string inputName = cv::parser.get<string>("@filename");
    
    if (!cap.open(VIDEO_PATH)) {
        std::cout << "already\n";
    }
    cv::Mat img;

    for (int i=0;i<motionVectors.size();i++)
    {
        vector<Eigen::Vector3d> tmp = analyzer.mapPoints(centers, motionVectors[i], heights[i]);
        points.insert(points.end(), tmp.begin(), tmp.end());

        vector<double> depths;
        //depth vector    
        for (int j = 0; j < tmp.size(); j++) {
            depths.push_back(tmp[j](2));
        }
        
        //pixels that werent removed aka motionvector not zero
        vector<Eigen::Vector2d> pixels;
        for (int j = 0; j < centers.size(); j++) {
            
            if (motionVectors[i][j](1)!=0) {
                pixels.push_back(centers[j]);
            }
        }

        //get image   
        if (!cap.read(img)) {
            cout << "its fucked";
            break;
        }
        

        string window_name = "Room Map";
        PointDisplayer displayer(window_name);

        

        displayer.depthImage(img, pixels, depths);


    }

    //string window_name = "Room Map";
    //PointDisplayer displayer(window_name);
    //displayer.topDownView(points);
    //displayer.depthMap(points, cv::Mat(700, 700, CV_8UC3, cv::Scalar(255, 255, 255)));
    //cv::Mat img(700, 700, CV_8UC3, cv::Scalar(255, 255, 255));
    
    //displayer.depthImage(img, vector<Eigen::Vector2d>({ Eigen::Vector2d(5,5),Eigen::Vector2d(800,800) }), vector<double>({100,10}));

    return 0;
}