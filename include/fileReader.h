#ifndef FILEREADER_H
#define FILEREADER_H

#include <iostream>
#include <fstream>
#include <string>

using namespace std;
class fileReader
{
public:
	fileReader(string input_file);
	~fileReader();

	// PARAMETERS

	// Simulation Settings
	unsigned int ntrials;
	unsigned int nMCs;
	int solver_type;
	int numWps;
	unsigned int seed;

	// General Path Planning Algorithm Settings
	double clearance;
	unsigned int iters_limit;

	// Map Settings
	string boundaries_in_file;
	string latitude0;
	string longitude0;
	double height0;
	bool is3D;
	double minCylRadius;
	double maxCylRadius;
	double minCylHeight;
	double maxCylHeight;
	double minFlyHeight;
	double maxFlyHeight;
	double waypoint_clearance;
	unsigned int nCyli;

	// Output Files
	string file_extension;
	string performance_file_name;
	string boundaries_out_file;
	string cylinders_out_file;
	string primary_wps_out_file;
	string tree_file;
	string path_file;
private:

};
#endif