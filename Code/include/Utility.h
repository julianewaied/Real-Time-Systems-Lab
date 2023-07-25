#pragma once
#include <string>
#include <vector>
#include <tuple>
#include <fstream>
#include <Eigen/Dense>
#include <opencv2/core/types.hpp>
#define NUM_FRM 100
using std::string;
using std::vector;
// MotionVector matrix dimensions!
const int rows = 30;
const int cols = 41;
typedef cv::Point3i MotionVector;
typedef vector<MotionVector> matrow;
typedef vector<matrow> matrix;
class CSVFile
{
	string path;
	std::fstream filp;
	MotionVector readNext();

public:
	CSVFile(const std::string& path) : path(path) {};

	CSVFile() : path("None") {};

	~CSVFile();

	void openFile();

	void closeFile();

	void setPath(const std::string& path);

	vector<matrix> readFile(int startPosition = 0, int length = NUM_FRM);
};