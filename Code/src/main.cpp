#include <iostream>
#include <Eigen/Dense>
#include "../include/Utility.h"
const char* path = "C:/Users/WIN10PRO/Desktop/My Stuff/University/BSC/Y3/RT systems/Real-Time-Systems-Lab/Code/src/test3.csv";
int Run()
{
    // just a driver code
    CSVFile file(path);
    std::cout << "Hello World!\n";
    file.openFile();
    vector<matrix> v = file.readFile(0, 1);
    std::cout << std::get<1>(v[0][0][0])<<std::endl;
    file.setPath("None");
    file.closeFile();
    return 0;
}