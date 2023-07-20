#pragma once
#include <string>
#include <vector>
#include <tuple>
#include <fstream>
using std::string;
using std::vector;
using std::tuple;
class CSVFile
{
	string path;
	std::fstream filp;
public:
	CSVFile(const std::string& path) : path(path) {};
	CSVFile() : path("None") {};
	void openFile();
	bool setPath(const std::string& path);
	vector<tuple<int, int, int>> readFile(int startPosition = -1, int length = 0) const;
	vector<tuple<int, int, int>> readAll() const;
};