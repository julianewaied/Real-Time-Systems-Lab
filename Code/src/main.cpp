#include "../include/Analysis.h"
#include "../include/Utility.h"
#include "../include/PointDisplayer.h"
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#define NUM_FRM 24
using std::cout;
using std::map;
using std::pair;

const double cx = 319.7108;
const double cy = 231.1376;
const double fx = 506.2113;
const double fy = 505.1260;
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
        std::cout << key << " : " << it->second << std::endl;
    }

}

int Run()
{
    //BuildTDView(mvs_paths, heights);
    const int frame_num = 5;
    const int num_vid = 4;
    auto motionVectors = importMV(mvs_paths[4]);
    auto centers = getCenters();
    string window_name = "Depth Map";
    const int ROWS = 103;
    const int COLS = 77;
    frames& mvs = motionVectors[frame_num];
    double maxy, miny;
    maxy = miny = mvs[0](1);
    for (auto mv : mvs)
    {
        maxy = std::max(maxy, mv(1));
        //avoid y = 0
        if(mv(1)) 
            miny = std::min(miny, mv(1));
    }






    // Replace "your_video_path" with the actual path to your H.264 video file
    std::string videoPath = R"(C:\Users\WIN10PRO\Desktop\My Stuff\University\BSC\Y3\RT systems\Real-Time-Systems-Lab\Code\Data\vertical rotation\h264\rise0.h264)";

    cv::VideoCapture cap(videoPath);

    if (!cap.isOpened()) {
        std::cout << "Error opening video file." << std::endl;
        return -1;
    }

    int frameNumber = frame_num;

    cap.set(cv::CAP_PROP_POS_FRAMES, frameNumber);

    cv::Mat frame;
    cap.read(frame);
    cv::Mat resizedFrame;
    cv::resize(frame, resizedFrame, cv::Size(), 0.5, 0.5); // Resize to half the dimensions

    for (int i = 0;i < ROWS;i+=2)
    {
        for (int j = 0; j < COLS; j+=2)
        {
            int ij = i * ROWS + j;
            double dy = (mvs[ij](1) - miny) / (maxy - miny);
            double dx = mvs[ij](0);
            cv::Point p1(8 * i+1, 8 * j+1), p2(8 * i + 8-1, 8 * j + 8-1);
            if(dy>=0 && std::abs(dx)<10)
                cv::rectangle(resizedFrame, p1, p2, cv::Scalar(dy * 255, dy * 255, dy * 255), cv::FILLED);
        }
    }


    cv::imshow("Frame", resizedFrame);
    cv::waitKey(0);

    cap.release();
    cv::destroyAllWindows();

    return 0;
}