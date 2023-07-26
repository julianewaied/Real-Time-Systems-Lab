#include "../include/Analysis.h"
#include "../include/Utility.h"
#include "../include/PointDisplayer.h"
#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

int Run() {
    vector<Eigen::Vector2d> v;
    for (int i = 0;i < 200;i+=20)
    {
        v.push_back(Eigen::Vector2d(i, 2 * i));
    }
    string test = "testing";
    PointDisplayer displayer(test);
    displayer.display(v);
    return 0;
}