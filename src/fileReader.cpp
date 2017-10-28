#include "./../include/fileReader.h"

using namespace std;
fileReader::fileReader(string input_file)
{
	ifstream fin;
	fin.open(input_file.c_str());
	string line_string, variable_name, variable_value;
	size_t found, found2;
	while (getline(fin,line_string))
	{
		// Find out if there is a comment # and delete everything after
		found = line_string.find_first_of("#");
		if (found !=string::npos)
			line_string.resize(found);

		// Delete all of the spaces
		found = line_string.find_first_of(" \t");
		while (found !=string::npos)
		{
			line_string.erase(found, 1);
			found = line_string.find_first_of(" \t");
		}

		if (line_string.length() > 0)
		{
			// Parse the name and value
			variable_name = line_string;

			found = line_string.find_first_of("=");
			variable_name.resize(found);
			found2 = line_string.find_first_of(";");
			variable_value = line_string.substr(found+1, found2 - found-1);

			if (variable_name == "ntrials")
				ntrials = stoul(variable_value);
			else if (variable_name == "nMCs")
				nMCs = stoul(variable_value);
			else if (variable_name == "solver_type")
				solver_type = stoi(variable_value);
			else if (variable_name == "numWps")
				numWps = stoi(variable_value);
			else if (variable_name == "seed")
				seed = stoul(variable_value);
			else if (variable_name == "turn_radius")
				turn_radius = stod(variable_value);
			else if (variable_name == "clearance")
				clearance = stod(variable_value);
			else if (variable_name == "iters_limit")
				iters_limit = stoul(variable_value);
			else if (variable_name == "boundaries_in_file")
				boundaries_in_file = variable_value;
			else if (variable_name == "latitude0")
				latitude0 = variable_value;
			else if (variable_name == "longitude0")
				longitude0 = variable_value;
			else if (variable_name == "height0")
				height0 = stod(variable_value);
			else if (variable_name == "is3D")
				is3D = (("true" == variable_value) ? true : false);
			else if (variable_name == "minCylRadius")
				minCylRadius = stod(variable_value);
			else if (variable_name == "maxCylRadius")
				maxCylRadius = stod(variable_value);
			else if (variable_name == "minCylHeight")
				minCylHeight = stod(variable_value);
			else if (variable_name == "maxCylHeight")
				maxCylHeight = stod(variable_value);
			else if (variable_name == "minFlyHeight")
				minFlyHeight = stod(variable_value);
			else if (variable_name == "maxFlyHeight")
				maxFlyHeight = stod(variable_value);
			else if (variable_name == "waypoint_clearance")
				waypoint_clearance = stod(variable_value);
			else if (variable_name == "nCyli")
				nCyli = stoul (variable_value);
			else if (variable_name == "file_extension")
				file_extension = variable_value;
			else if (variable_name == "performance_file_name")
				performance_file_name = variable_value;
			else if (variable_name == "boundaries_out_file")
				boundaries_out_file = variable_value;
			else if (variable_name == "cylinders_out_file")
				cylinders_out_file = variable_value;
			else if (variable_name == "primary_wps_out_file")
				primary_wps_out_file = variable_value;
			else if (variable_name == "tree_file")
				tree_file = variable_value;
			else if (variable_name == "path_file")
				path_file = variable_value;
			else if (variable_name == "special_pfile")
				special_pfile = variable_value;
		}
	}
	fin.close();
	performance_file_name = performance_file_name + file_extension;
	boundaries_out_file = boundaries_out_file     + file_extension;
	cylinders_out_file = cylinders_out_file       + file_extension;
	primary_wps_out_file = primary_wps_out_file   + file_extension;
	path_file = path_file						  + file_extension;
	special_pfile = special_pfile				  + file_extension;
}
fileReader::~fileReader()
{
}
