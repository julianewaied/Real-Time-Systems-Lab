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
#define COLOR_FILTER 100
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
        
        // import all data for file
        auto motionVectors = a.importMV(mvFiles[i]);
        CSVFile height_file(heightFiles[i], NUM_FRM);
        height_file.openFile();
        auto heights = height_file.readColumn();
        auto centers = a.getCenters();
        CSVFile sads_file(mvFiles[i], NUM_FRM);
        sads_file.openFile();
        auto sads = sads_file.getSAD();
        vector<cv::Mat> pics;
        // cont_heights is a backup for calculating depths
        vector<double> cont_heights;
        for (auto h : heights)
        {
            cont_heights.push_back(h);
        }
        vector<Eigen::Vector3d> tmp;
        // continuize the heights function.
        a.continuize(cont_heights);
        // for each frame in the video
        for (int k = 1;k < motionVectors.size();k++)
        {
            // get the mapped points then add the heights, and add it to the cloud
            vector<Eigen::Vector3d> tmp = a.mapPoints(centers, motionVectors[k], cont_heights[k]-cont_heights[k-1], sads[k]);
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

vector<cv::Mat> extractFrames(const string& path)
{
    vector<cv::Mat> frms;
    cv::VideoCapture cap(path);
    if (!cap.isOpened()) {
        std::cout << "Error opening video file." << std::endl;
        exit(-1);
    }
    for (int i = 0;i < NUM_FRM && cap.isOpened();i++)
    {
        cv::Mat tmp;
        cap >> tmp;
        frms.push_back(tmp);
    }
    cap.release();
    return frms;
}
vector<int8_t> processImage(cv::Mat image)
{
    vector<int8_t> flags;
    // step 1: average each macro-block
    vector<vector<cv::Vec3i>> colAvg;
    for (int i = 0;i < image.rows;i+=16)
        colAvg.push_back(vector<cv::Vec3i>(image.cols/16));
    for (int i = 0;i < image.rows;i+=16)
    {
        for (int j = 0;j < image.cols;j+=16)
        {
            cv::Vec3i avg(0, 0, 0);
            // k moves between diagonals
            for (int k = 0; k < 16;k++)
            {
                if (i + k < image.rows && j + k < image.cols)
                {
                    cv::Vec3i x = image.at<cv::Vec3i>(cv::Point(i + k, j + k));
                    avg = avg + x;
                }
            }
            avg = (1 / 16) * avg;
            colAvg[i][j] = avg;
        }
    }
    // step 2: calculate the distances between all adjacent macro-blocks
    for (int i = 0;i < colAvg.size();i++)
        for (int j = 0; j < colAvg[0].size();j++)
        {
            long long sum = 0;
            if (i > 0)
            {
                sum += cv::norm(colAvg[i][j] - colAvg[i - 1][j]);
                if (j > 0)
                {
                    sum += cv::norm(colAvg[i][j] - colAvg[i - 1][j - 1]);
                    sum += cv::norm(colAvg[i][j] - colAvg[i][j - 1]);
                }
                if (j < colAvg[0].size() - 1)
                    sum += cv::norm(colAvg[i][j] - colAvg[i - 1][j + 1]);
            }
            if (i < colAvg.size() - 1)
            {
                sum += cv::norm(colAvg[i][j] - colAvg[i + 1][j]);
                if (j > 0)
                    sum += cv::norm(colAvg[i][j] - colAvg[i + 1][j - 1]);
                if (j < colAvg[0].size() - 1)
                {
                    sum += cv::norm(colAvg[i][j] - colAvg[i + 1][j + 1]);
                    sum += cv::norm(colAvg[i][j] - colAvg[i][j + 1]);
                }
            }
            // step 3: for every macro block with distances sum<f, write 0 (which will cancel the motion vector)
            if (sum < COLOR_FILTER)
                flags.push_back(0);
            else
                flags.push_back(1);
        }
    return flags;
}
// writes points to obj file to be projected in 3D Builder with filtering
void writeFilteredOBJ(const vector<string>& mvFiles, const vector<string>& heightFiles, const vector<string>& vidFiles,const string& outputPath)
{
    std::ofstream out;
    out.open(outputPath);
    if (!out.is_open())
        exit(1);
    vector<Eigen::Vector3d> points;
    Analyzer a(fx, fy, cx, cy);
    for (int i = 0;i < mvFiles.size();i++)
    {

        // import all data for file
        auto motionVectors = a.importMV(mvFiles[i]);
        vector<cv::Mat> images = extractFrames(vidFiles[i]);
        CSVFile height_file(heightFiles[i], NUM_FRM);
        height_file.openFile();
        auto heights = height_file.readColumn();
        auto centers = a.getCenters();
        CSVFile sads_file(mvFiles[i], NUM_FRM);
        sads_file.openFile();
        auto sads = sads_file.getSAD();
        // cont_heights is a backup for calculating depths
        vector<double> cont_heights;
        for (auto h : heights)
        {
            cont_heights.push_back(h);
        }
        vector<Eigen::Vector3d> tmp;
        // continuize the heights function.
        a.continuize(cont_heights);
        // for each frame in the video
        for (int k = 1;k < motionVectors.size();k++)
        {
            // get the mapped points then add the heights, and add it to the cloud
            vector<int8_t> filter_images = processImage(images[k]);
            vector<Eigen::Vector3d> tmp = a.mapPoints(centers, motionVectors[k], cont_heights[k] - cont_heights[k - 1], sads[k]);
            for (auto v : tmp)
            {
                v(1) += heights[k];
            }
            Analyzer::rotatePoints(tmp, 60 * i);
            for (int j = 0;j < tmp.size();j++)
            {
                if (NORM(tmp[j](0), tmp[j](2)) < FILTER_RADIUS)
                    points.push_back(tmp[j]);
            }
        }
        std::cout << "Processing Angle : " << 60 * i << std::endl;

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