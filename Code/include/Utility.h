#pragma once
#include <string>
#include <vector>
#include <tuple>
#include <fstream>
#define NUM_FRM 100
using std::string;
using std::vector;
using std::tuple;
// MotionVector matrix dimensions!
const int rows = 30;
const int cols = 41;
typedef tuple<int, int, int> MotionVector;
typedef vector<MotionVector> matrow;
typedef vector<matrow> matrix;
class CSVFile
{
	string path;
	std::fstream filp;
	tuple<int,int,int> readNext();

public:
	CSVFile(const std::string& path) : path(path) {};

	CSVFile() : path("None") {};

	~CSVFile();

	void openFile();

	void closeFile();

	void setPath(const std::string& path);

	vector<matrix> readFile(int startPosition = 0, int length = NUM_FRM);
};