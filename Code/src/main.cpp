#include <iostream>
#include <Eigen/Dense>
<<<<<<< Updated upstream
#include "../include/Analysis.h"
#include "../include/Utility.h"
using namespace Eigen;
const char* path = "C:/Users/WIN10PRO/Desktop//test.csv";
int Run()
{
    CSVFile file(path);
    file.openFile();
    vector<matrix> v = file.readFile(0, 1);
    std::cout << v[0][0][1] << std::endl;
    file.setPath("None");
    file.closeFile();
=======
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
#include "../include/Utility.h"
const char* path = "C:/Users/Malek/Pictures/logo-Stockfish-468x468.png";
int Run()
{
    //// just a driver code
    //CSVFile file(path);
    //std::cout << "Hello World!\n";
    //file.openFile();
    //vector<matrix> v = file.readFile(0, 1);
    //std::cout << std::get<1>(v[0][0][0])<<std::endl;
    //file.setPath("None");
    //file.closeFile();
    //cv::Mat img = cv::imread("C:/Users/Malek/Pictures/logo-Stockfish-468x468.png");
    //namedWindow("Test", WINDOW_AUTOSIZE);
    //imshow("Test", img);
    //cv::waitKey(0);
    //cv::destroyAllWindows();

    Mat image = imread(path);

    vector<Point2i> points = { Point2i(100,100),Point2i(200,200),Point2i(300,300),Point2i(400,400) };

    

    return 0;
>>>>>>> Stashed changes
}