#include <iostream>
#include <Eigen/Dense>
#include "../include/Analysis.h"
#include "../include/Utility.h"
using namespace Eigen;
using namespace std;
const char* path = "C:/Users/WIN10PRO/Desktop//test.csv";
int Run()
{
    CSVFile file(path);
    Analyzer a(1, 1, 2, 2);
    a.buildCameraMatrix();
    cout << a.getCameraMatrix() << endl;
    cout << Analyzer::getRotationMatrix(90, Eigen::Vector3d(0, 0, 1));
}