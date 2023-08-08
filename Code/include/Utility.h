#pragma once
#include <string>
#include <vector>
#include <fstream>
#include <Eigen/Dense>
using std::string;
using std::vector;
// MotionVector matrix dimensions!
const int ROWS = 102;
const int COLS = 77;
typedef Eigen::Vector2d MotionVector;
typedef vector<MotionVector> frames;

class CSVFile
{
	int lines;
	string path;
	std::ifstream filp;
	MotionVector readNext2();
	double readNext1();

public:
	CSVFile(const std::string& path, int lines) : path(path), lines(lines) {};

	CSVFile() : path("None") {};

	~CSVFile();

	void openFile();

	void closeFile();

	void setPath(const std::string& path);

	vector<frames> readFile(int length = -1);

	vector<double> readColumn(int length = -1);

};