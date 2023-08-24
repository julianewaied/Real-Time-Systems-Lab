#include "../include/Utility.h"
#include <iostream>
#include <sstream>

void CSVFile::openFile()
{
	if (this->path == "None") throw "Reading a file before assigment";
	this->filp = std::ifstream(this->path, std::ios::in);
	if (!filp.is_open()) {
		std::cout << "Failed to open the file\n";
		exit(1);
	}
}

CSVFile::~CSVFile()
{
	this->closeFile();
}

void CSVFile::closeFile()
{
	this->filp.close();
}

void CSVFile::setPath(const std::string& path)
{
	if (filp.is_open()) filp.close();
	this->path = path;
}

vector<frames> CSVFile::readFile(int length)
{
	vector<frames> all;
	if (length == -1) length = lines;
	for (int k = 0;k < length;k++)
	{
		// read matrix i
		frames frm;
		for (int i = 0;i < ROWS;i++)
			for(int j=0;j<COLS;j++)
				frm.push_back(readNext2());
		all.push_back(frm);
	}
	return all;
}

vector<vector<double>> CSVFile::getSAD(int length)
{
	vector<vector<double>> all;
	if (length == -1) length = lines;
	for (int i = 0;i < length;i++)
	{
		vector<double> v;
		for(int j = 0; j<ROWS; j++)
			for(int k=0;k<COLS;k++)
				v.push_back(readNextSAD());
		all.push_back(v);
	}
	return all;
}

double CSVFile::readNextSAD()
{
	double a = 0.0; // Initialize to a default value in case there's an error
	double b = 0.0;
	double c = 0.0;
	char ch = ',';
	std::string line;
	if (!std::getline(filp, line)) {
		std::cout << "Problem reading a line from the file!\n";
	}
	else {
		// read the 3 coordinates, return the third
		std::istringstream stream(line);
		stream >> a >>ch>> b >> ch >> c;
		if (stream.fail()) {
			std::cout << "Error converting to double: Invalid data format in line.\n";
			a = 0.0; // Set to a default value
		}
	}
	return c;
}

MotionVector CSVFile::readNext2()
{
	int a, b, c;
	char comma;
	std::string line;
	std::getline(this->filp, line);
	std::istringstream stream(line);
	// read 2 coordinates from the file, then return them as a MotionVector
	stream >> a >> comma >> b;
	MotionVector vec = MotionVector(a,b);
	return vec;
}

double CSVFile::readNext1()
{
	double a = 0.0; // Initialize to a default value in case there's an error
	std::string line;
	if (!std::getline(filp, line)) {
		std::cout << "Problem reading a line from the file!\n";
	}
	else {
		std::istringstream stream(line);
		stream >> a;
		if (stream.fail()) {
			std::cout << "Error converting to double: Invalid data format in line.\n";
			a = 0.0; // Set to a default value
		}
	}
	return a;
}

vector<double> CSVFile::readColumn(int length)
{
	vector<double> heights;
	if (length == -1) length = lines;
	// read each line into the heights vector, then return the vector.
	for (int i = 0;i < length;i++)
	{
		heights.push_back(readNext1());
	}
	return heights;
}
