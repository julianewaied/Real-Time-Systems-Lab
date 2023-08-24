#pragma once
#include <string>
#include <vector>
#include <fstream>
#include <Eigen/Dense>
using std::string;
using std::vector;
// MotionVector matrix dimensions!
const int ROWS = 104;
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
	double readNextSAD();

public:
	CSVFile(const std::string& path, int lines) : path(path), lines(lines) {};

	CSVFile() : path("None"), lines(0) {};

	~CSVFile();

	// opens the file of the given path
	void openFile();

	// closes the file of the given path
	void closeFile();

	// closes the current file if opened, and changes to another path
	void setPath(const std::string& path);

	// reads the first two columns from the csv file (dx,dy)
	// in default case, reads all the file. otherwise, reads from the required index to the required length
	vector<frames> readFile(int length = -1);

	// reads the first column from the csv file (heights)
	// in default case, reads all the file. otherwise, reads from the required index to the required length

	vector<double> readColumn(int length = -1);

	// reads the third column from the csv file (SAD)
	// in default case, reads all the file. otherwise, reads from the required index to the required length

	vector<vector<double>> getSAD(int length = -1);

};