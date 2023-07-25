#include <iostream>
#include <Eigen/Dense>
#include "../include/Utility.h"
const char* path = "C:/Users/WIN10PRO/Desktop//test.csv";
int Run()
{
    CSVFile file(path);
    file.openFile();
    vector<matrix> v = file.readFile(0, 1);
    std::cout << v[0][0][1] << std::endl;
    file.setPath("None");
    file.closeFile();
}