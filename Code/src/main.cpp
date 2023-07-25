#include <iostream>
#include <opencv2/core.hpp>
using namespace cv;
#include "../include/Utility.h"
const char* path = "C:/Users/WIN10PRO/Desktop//test.csv";
int Run()
{
    // just a driver code
    CSVFile file(path);
    std::cout << "Hello World!\n";
    file.openFile();
    vector<matrix> v = file.readFile(0, 1);
    std::cout << v[0][0][1] << std::endl;
    file.setPath("None");
    file.closeFile();
    
}