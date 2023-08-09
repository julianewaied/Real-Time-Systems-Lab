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

// counts the number of instances of each y value and plots out counters
void countFile(const string& path)
{
    auto mvs = Analyzer::importMV(path);
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

// writes points to obj file to be projected in 3D Builder
void writeOBJ(const vector<string>& mvFiles, const vector<string>& heightFiles, const string& outputPath)
{
    std::ofstream out;
    out.open(outputPath);
    if (!out.is_open())
        exit(1);
    vector<Eigen::Vector3d> points;
    Analyzer a(fx, fy, cx, cy);
    for (int i = 0;i < mvFiles.size();i++)
    {
        auto tmp = a.extractPoints(mvFiles[i], heightFiles[i], 60 * i);
        std::cout << "Processing Angle : " << 60 * i << std::endl;
        points.insert(points.end(), tmp.begin(), tmp.end());
    }
    int line = 1;
    vector<string> faces;
    for (auto point : points)
    {
        string s = "f ";
        for (int k = 0;k < 3;k++) {
            int A = 20;
            out << "v " << point(0) + ((k%3)/2)*A << " " << point(2) + (((k+1)%3)/2)*A << " " << point(1) / 10 + A*((k+2)%3)/2 << std::endl;
            s = s + std::to_string(line++) + " ";
        }
        faces.push_back(s+"\n");
        //if (line >=10) break;
    }
    for (auto f : faces)
    {
        out << f;
    }
}

int Run()
{
    PointDisplayer::BuildTDView(mvs_paths, heights);
    int i = 0;
    static std::string videoPath = R"(C:\Users\WIN10PRO\Desktop\My Stuff\University\BSC\Y3\RT systems\Real-Time-Systems-Lab\Code\Data\vertical rotation\h264\rise0.h264)";
    //BuildDepthMap(mvs_paths[i], videoPath);
    //CSVFile file(mvs_paths[i], NUM_FRM);
    //file.openFile();
    //vector<vector<double>> SADs = file.getSAD();
    //BuildDepthMap(mvs_paths[i], videoPath,SADs);
    string output = R"(C:\Users\WIN10PRO\Desktop\test.obj)";
    //writeOBJ(mvs_paths, heights, output);
    
    return 0;
}