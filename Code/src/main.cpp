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

// camera properties
const double cx = 319.7108;
const double cy = 231.1376;
const double fx = 506.2113;
const double fy = 505.1260;

// data paths
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

// counts the number of instances of each dy value and plots out counters
void countFile(const string& path)
{
    auto mvs = Analyzer::importMV(path);
    map<double, int> all;
    // insert all dy's into a map with dy as the key
    for (int i = 0;i < mvs.size();i++)
    {
        frames& frm = mvs[i];
        for (int j = 0;j < frm.size();j++)
        {
            all[frm[j](1)]++;
        }
    }
    // output all counters of each key
    for (auto it = all.begin(); it != all.end(); ++it) {
        const double key = it->first;
        std::cout << key << " : " << it->second << std::endl;
    }

}

// takes each radial slice and returns the 0.05-0.2 closest points to the origin
vector<Eigen::Vector3d> medianFilter(const vector<Eigen::Vector3d>& points)
{
    const double lowPrecentage = 0.05;
    const double hightPrecentage = 0.2;
    vector<Eigen::Vector3d> ps;
    vector<Eigen::Vector3d> filtered;
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
        if(p(2)<=50&& p(2)>=-50)
        buckets[index].push_back(p);
    }
    // step 3: for each bucket, take all important points
    for (auto b : buckets)
    {
        std::sort(b.begin(), b.end(), [](const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs)
            {
                return lhs(1) < rhs(1);
            });
        for (int i = lowPrecentage * b.size();i < hightPrecentage * b.size();i++)
        {
            double x = b[i](1) * std::cos(b[i](0));
            double z = b[i](1) * std::sin(b[i](0));
            filtered.push_back(Eigen::Vector3d(x,b[i](2),z));
        }
    }
    return filtered;
}

// writes points to obj file to be projected in 3D Builder (after Median Compressing Filtering)
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
            // match the height (y coordinate)
            for (auto v : tmp)
            {
                v(1) += heights[k];
            }
            Analyzer::rotatePoints(tmp, 60*i);
            // filter by radius, just for covnenience
            for (int j = 0;j < tmp.size();j++)
            {
                if (NORM(tmp[j](0), tmp[j](2)) < FILTER_RADIUS)
                    points.push_back(tmp[j]);
            }
        }
        std::cout << "Processing Angle : " << 60 * i << std::endl;
        
    }
    // filtering by radius of each radial slice
    points = medianFilter(points);
    // write to obj file
    int line = 1;
    vector<string> faces;
    // for each point, writes 3 vertices in the obj file. A is the "size" of the face corrsponding to the point
    for (auto point : points)
    {
        string s = "f ";
        for (int k = 0;k < 3;k++) {
            int A = 1;
            out << "v " << point(0) + ((k % 3) / 2) * A << " " << point(2) + (((k + 1) % 3) / 2) * A << " " << point(1) / 10 + A * ((k + 2) % 3) / 2 << std::endl;
            s = s + std::to_string(line++) + " ";
        }
        // keeping the faces for the end of the obj file
        faces.push_back(s + "\n");
    }
    // writing out the faces according to what we have already written
    for (auto f : faces)
    {
        out << f;
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
            vector<Eigen::Vector3d> tmp = a.mapPoints(centers, motionVectors[k], cont_heights[k] - cont_heights[k - 1], sads[k]);
            // matching heights
            for (auto v : tmp)
            {
                v(1) += heights[k];
            }
            Analyzer::rotatePoints(tmp, 60 * i);
            // radius filtering for convenience only
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
        // for each point writes 3 vertices around it. A is the "size" of the corresponding triangle
        for (int k = 0;k < 3;k++) {
            int A = 1;
            out << "v " << point(0) + ((k % 3) / 2) * A << " " << point(2) + (((k + 1) % 3) / 2) * A << " " << point(1) / 10 + A * ((k + 2) % 3) / 2 << std::endl;
            s = s + std::to_string(line++) + " ";
        }
        faces.push_back(s + "\n");
    }
    // writing out the faces
    for (auto f : faces)
    {
        out << f;
    }

}

// writes points to csv file to be processed later by other programs 
void writeCSV(const vector<string>& mvFiles, const vector<string>& heightFiles, const string& outputPath)
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
            points.insert(points.begin(), tmp.begin(),tmp.end());
        }
        std::cout << "Processing Angle : " << 60 * i << std::endl;

    }
    cout << points.size();
    // write to csv file
    for (auto point : points)
    {
        out << point(0) << ", " << point(1) << ", " << point(2) << std::endl;
    }
}

// given a path to an h264 video, returns the frames in cv::Mat type.
vector<cv::Mat> extractFrames(const string& path)
{
    vector<cv::Mat> frms;
    cv::VideoCapture cap(path);
    // check the file is opened
    if (!cap.isOpened()) {
        std::cout << "Error opening video file." << std::endl;
        exit(-1);
    }
    // read the frames frame by frame into the frames vector, then return the frames.
    for (int i = 0;i < NUM_FRM && cap.isOpened();i++)
    {
        cv::Mat tmp;
        cap >> tmp;
        frms.push_back(tmp);
    }
    cap.release();
    return frms;
}

// given an image, returns the flags of the Color Average Filtering Algorithm
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

// given the flags, throws away motion vectors corresponding to flags 0 (to be thrown away).
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
// writes points to obj file to be projected in 3D Builder with Average Color Filtering
void writeOBJColor(const vector<string>& mvFiles, const vector<string>& heightFiles, const vector<string>& vidFiles,const string& outputPath)
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
            // filter the motion vectors, get the mapped points then add the heights, and add it to the cloud
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
    // options are: "write obj", "show depth map", "write obj median", "write obj color"
    string function = "write obj";
    // set numVideo to a number between 0 and 5, used in building depthMap.
    int numVideo = 3;

    // set the output to the path of the obj file, and csv output to the csv output file
    string csvoutput = R"(C:\Users\WIN10PRO\Desktop\points.csv)";
    string output = R"(C:\Users\WIN10PRO\Desktop\test.obj)";

    // running the function matching the requirement 
    if (function == string("top down"))
        PointDisplayer::BuildTDView(mvs_paths, heights);
    else if (function == string("write csv"))
        writeCSV(mvs_paths, heights, csvoutput);
    else if (function == string("show depth map"))
    {
        CSVFile file(mvs_paths[numVideo], NUM_FRM);
        file.openFile();
        vector<vector<double>> SADs = file.getSAD();
        PointDisplayer::BuildDepthMap(mvs_paths[numVideo], videos[numVideo], SADs);
    }
    else if (function == string("write obj"))
        writeOBJ(mvs_paths, heights, output);
    else if (function == string("write obj median"))
        writeOBJMedian(mvs_paths, heights, output);
    else if (function == string("write obj color"))
        writeOBJColor(mvs_paths, heights, videos , output);
    else
    {
        cout << "unknown option!";
        exit(1);
    }
    return 0;
}