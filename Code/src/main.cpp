#include "../include/Analysis.h"
#include "../include/Utility.h"
#include "../include/PointDisplayer.h"
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#define NORM(x,y) (x*x+y*y)
#define FILTER_RADIUS 3E4
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
        if (true)
        {
            // import all data for file
            auto motionVectors = a.importMV(mvFiles[i]);
            CSVFile height_file(heightFiles[i], NUM_FRM);
            height_file.openFile();
            auto heights = height_file.readColumn();
            auto centers = a.getCenters();
            CSVFile sads_file(mvFiles[i], NUM_FRM);
            sads_file.openFile();
            auto sads = sads_file.getSAD();
            // dH is a backup for calculating depths
            vector<double> dh;
            for (auto h : heights)
            {
                dh.push_back(h);
            }
            vector<Eigen::Vector3d> tmp;
            // continuize the heights function.
            a.continuize(dh);
            //a.differences(dh);
            // for each frame in the video
            for (int k = 1;k < motionVectors.size();k++)
            {
                // get the mapped points then add the heights, and add it to the cloud
                vector<Eigen::Vector3d> tmp = a.mapPoints(centers, motionVectors[k], dh[k]-dh[k-1], sads[k]);
                for (auto v : tmp)
                {
                    v(1) += heights[k];
                }
                Analyzer::rotatePoints(tmp, 60*i);
                for (int j = 0;j < tmp.size();j++)
                {
                    if (NORM(tmp[j](0), tmp[j](2)) < FILTER_RADIUS)
                        points.push_back(tmp[j]);
                }
            }
            std::cout << "Processing Angle : " << 60 * i << std::endl;
        }
    }
        // write to obj file
        int line = 1;
        vector<string> faces;
        for (auto point : points)
        {
            string s = "f ";
            for (int k = 0;k < 3;k++) {
                int A = 1;
                out << "v " << point(0) + ((k % 3) / 2) * A << " " << point(2) + (((k + 1) % 3) / 2) * A << " " << point(1) / 10 + A * ((k + 2) % 3) / 2 << std::endl;
                s = s + std::to_string(line++) + " ";
            }
            faces.push_back(s + "\n");
        }
        for (auto f : faces)
        {
            out << f;
        }
        
}

int Run()
{
    // testing
    //PointDisplayer::BuildTDView(mvs_paths, heights);
    int i = 5;
    static std::string videoPath = R"(C:\Users\WIN10PRO\Desktop\My Stuff\University\BSC\Y3\RT systems\Real-Time-Systems-Lab\Code\Data\vertical rotation\h264\fall2.h264)";
    //PointDisplayer::BuildDepthMap(mvs_paths[i], videoPath);
    CSVFile file(mvs_paths[i], NUM_FRM);
    file.openFile();
    vector<vector<double>> SADs = file.getSAD();
    PointDisplayer::BuildDepthMap(mvs_paths[i], videoPath,SADs);
    // printing 3D image
    //string output = R"(C:\Users\WIN10PRO\Desktop\test.obj)";
    //writeOBJ(mvs_paths, heights, output);
    
    return 0;
}