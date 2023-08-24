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
#define COLOR_FILTER 200
#define PI 3.14159
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
vector<string> videos{
    "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/vertical rotation/h264/rise0.h264",
    "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/vertical rotation/h264/fall0.h264",
    "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/vertical rotation/h264/rise1.h264",
    "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/vertical rotation/h264/fall1.h264",
    "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/vertical rotation/h264/rise2.h264",
    "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/vertical rotation/h264/fall2.h264"
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

vector<Eigen::Vector3d> medianFilter(const vector<Eigen::Vector3d>& points)
{
    vector<Eigen::Vector3d> ps;
    // step 1: convert to (x,y,z) to (theta,r,y) in radians
    for (auto p : points)
    {
        double r = std::sqrt(std::pow(p(0), 2) + std::pow(p(2), 2));
        double theta = std::atan(p(2) / p(0));
        // since arctan is not a bijection over 360 degrees.
        if (theta * p(2) < 0)
            theta = theta + PI;
        if (theta < 0)
            theta = theta + 2 * PI;
        ps.push_back(Eigen::Vector3d(theta, r, p(1)));
    }
    // step 2: sort into buckets
    vector<vector<Eigen::Vector3d>> buckets(360);
    for (auto p : ps)
    {
        int index = (p(0) / PI) * 180;
        if (index == 360) index = 359;
        buckets[index].push_back(p);
    }
    // step 3: for each bucket, take all important points
}

// writes points to obj file to be projected in 3D Builder
void writeOBJMedian(const vector<string>& mvFiles, const vector<string>& heightFiles, const string& outputPath)
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
    points = medianFilter(points);
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
    uint8_t* pixels = static_cast<uint8_t*>(image.data);
    int cn = image.channels();
    vector<int8_t> flags;
    // step 1: average each macro-block
    vector<vector<cv::Vec3d>> colAvg;
    for (int i = 0;i < image.rows-16;i+=16)
    {
        colAvg.push_back(vector<cv::Vec3d>());
        for (int j = 0;j < image.cols-16;j+=16)
        {
            cv::Vec3d avg(0, 0, 0);
            // k moves between diagonals
            for (int k = 0; k < 16;k++)
            {
                if (i + k < image.rows && j + k < image.cols)
                {

                    cv::Vec3d x;
                    x(0) = pixels[i * cn * image.cols + j * cn + 0];
                    x(1) = pixels[i * cn * image.cols + j * cn + 1];
                    x(2) = pixels[i * cn * image.cols + j * cn + 2];
                    avg = avg + x;
                }
            }
            avg = (1.0 / 16) * avg;
            colAvg[i/16].push_back(avg);
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
                    sum += std::pow((cv::norm(colAvg[i][j] - colAvg[i - 1][j - 1])),2);
                    sum += std::pow(cv::norm(colAvg[i][j] - colAvg[i][j - 1]), 2);
                }
                if (j < colAvg[0].size() - 1)
                    sum += std::pow(cv::norm(colAvg[i][j] - colAvg[i - 1][j + 1]),2);
            }
            if (i < colAvg.size() - 1)
            {
                sum += std::pow(cv::norm(colAvg[i][j] - colAvg[i + 1][j]),2);
                if (j > 0)
                    sum += std::pow(cv::norm(colAvg[i][j] - colAvg[i + 1][j - 1]),2);
                if (j < colAvg[0].size() - 1)
                {
                    sum += std::pow(cv::norm(colAvg[i][j] - colAvg[i + 1][j + 1]),2);
                    sum += std::pow(cv::norm(colAvg[i][j] - colAvg[i][j + 1]),2);
                }
            }
            // step 3: for every macro block with distances sum<f, write 0 (which will cancel the motion vector)
            if (sum < COLOR_FILTER)
            {
                flags.push_back(0);
            }
            else
                flags.push_back(1);
        }
    return flags;
}

frames filterMVs(const frames& motionVectors, const vector<int8_t>& filters)
{
    frames mvs;
    for (int i = 0;i < motionVectors.size();i++)
    {
        if (filters[i])
            mvs.push_back(motionVectors[i]);
        else
            mvs.push_back(MotionVector(0, 0));
    }
    return mvs;
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
            motionVectors[k] = filterMVs(motionVectors[k], filter_images);
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
            int A = 1.5;
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
    //int i = 5;
    //static std::string videoPath = R"(C:\Users\WIN10PRO\Desktop\My Stuff\University\BSC\Y3\RT systems\Real-Time-Systems-Lab\Code\Data\vertical rotation\h264\fall2.h264)";
    //PointDisplayer::BuildDepthMap(mvs_paths[i], videoPath);
    //CSVFile file(mvs_paths[i], NUM_FRM);
    //file.openFile();
    //vector<vector<double>> SADs = file.getSAD();
    //PointDisplayer::BuildDepthMap(mvs_paths[i], videoPath,SADs);
    // printing 3D image
    //writeOBJ(mvs_paths, heights, output);
    string output = R"(C:\Users\WIN10PRO\Desktop\test.obj)";
    writeFilteredOBJ(mvs_paths, heights, videos, output);
    return 0;
}