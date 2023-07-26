#include "../include/Utility.h"
#include <iostream>
#include <sstream>

void CSVFile::openFile()
{
	//if (this->path == "None") throw "Reading a file before assigment";
	this->filp = std::fstream(this->path, std::ios::in);
	//if (!filp.is_open()) throw "Failed to open the File";
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
// in default case, reads all the file. otherwise, reads from the required index to the required length
// matrix will be replaced with eigen matrix later on.
vector<matrix> CSVFile::readFile(int startPosition, int length)
{
	vector<matrix> all;
	
	for (int i = startPosition;i < length;i++)
	{
		// read matrix i
		matrix mvmat;
		for (int i = 0;i < rows;i++)
		{
			mvmat.push_back(matrow());
			for (int j = 0;j < cols;j++)
			{
				mvmat[i].push_back(readNext());
			}
		}
		all.push_back(mvmat);
	}
	return all;
}

MotionVector CSVFile::readNext()
{
	int a, b, c;
	char comma;
	std::string line;
	std::getline(this->filp, line);
	std::istringstream stream(line);

	stream >> a >> comma >> b >> comma >> c;
	MotionVector vec = MotionVector(a,b,c);
	return vec;
}