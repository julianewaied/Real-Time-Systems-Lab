#include <iostream>
#include <Eigen/Dense>
#include "../include/Analysis.h"
#include "../include/Utility.h"
<<<<<<< Updated upstream
using namespace Eigen;
const char* path = "C:/Users/WIN10PRO/Desktop//test.csv";
int Run()
=======
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
const string path = "C:/Users/Malek/Dropbox/My PC (DESKTOP-CUL8BRV)/Documents/GitHub/Real-Time-Systems-Lab/Code/Data/motion_data_rise.csv";

// returns a list of MV for each frame.
vector<frames> importMV(const string& path)
>>>>>>> Stashed changes
{
    CSVFile file(path);
    file.openFile();
    vector<matrix> v = file.readFile(0, 1);
    std::cout << v[0][0][1] << std::endl;
    file.setPath("None");
    file.closeFile();
}