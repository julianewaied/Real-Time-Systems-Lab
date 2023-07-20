#include "../include/Utility.h"
void CSVFile::openFile()
{
	if (this->path == "None") throw "Reading a file before assigment";
	this->filp = std::fstream(this->path, std::ios::in);
	if (!filp.is_open()) throw "Failed to open the File";
}