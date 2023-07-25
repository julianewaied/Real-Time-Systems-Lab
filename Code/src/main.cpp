#include <iostream>
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
using namespace cv;
#include "../include/Utility.h"
const char* path = "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/src/test3.csv";
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
    cv::Mat img = cv::imread("C:/Users/WIN10PRO/Pictures/fish.png");
    namedWindow("Test", WINDOW_AUTOSIZE);
    imshow("Test", img);
    cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}