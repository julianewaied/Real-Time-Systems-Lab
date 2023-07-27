#include "../include/Analysis.h"
#include "../include/Utility.h"
#include "../include/PointDisplayer.h"
#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#define NUM_FRM 199
const string path = "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/Data/motion_data_rise.csv";
// returns a list of MV for each frame.
vector<frames> importMV(const string& path)
{
    CSVFile file(path,NUM_FRM);
    file.openFile();
    return file.readFile();
}
int Run() {
    auto data = importMV(path);

    for (auto mv : data[6])
    {
        std::cout << mv.transpose() << std::endl;
    }

    return 0;
}