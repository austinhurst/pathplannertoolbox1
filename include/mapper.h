/*	DESCRIPTION:
*	This is the header for the mapper class.
*
*/
#pragma once
#include <fstream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>

#include "./../include/map_s.h"
#include "./../include/randGen.h"

using namespace std;
class mapper
{
public:
	// Functions
	mapper(unsigned int seed);				// default constructor (uses the competition boundaries) with random obstacles.
	~mapper();								// virtual deconstructor
	void fprint_map();						// Prints the details about the map to different files.

	// Members
	map_s map;								// The map struct. This is where all of the important information about the created map.

private:
	// Functions
	bool flyZoneCheck(const double NED[]);	// Returns true if it is within boundaries and not on an obstacle, returns false otherwise
	void fprint_boundaries();				// Prints the boundaries of the map
	void fprintf_cylinders();				// Prints the cylinders that were developed

	// Members
	ifstream boundaries_in_file;			// The file that recieves boundaries
	ifstream cylinders_in_file;				// The file that recieves where cylinders are.
	randGen rg;								// This is the random generator
	vector<vector<double> > lineMinMax;		// (N x 4) vector containing the (min x, max x, min y, max y) for each boundary line
	vector<vector<double> > line_Mandb;		// (N x 2) vector that contains the slope and intercept of the line (m, b) from N = m*E + b ... not sure about E = constant lines yet.
	unsigned int nBPts;						// Number of Boundary Points
	unsigned int nCyli;						// Number of Cylinder Obstacles
	double maxNorth;						// Calculated maximum North Coordinate within boundaries
	double minNorth;						// Calculated minimum North Coordinate within boundaries
	double maxEast;							// Calculated maximum East  Coordinate within boundaries
	double minEast;							// Calculated minimum East  Coordinate within boundaries
	double minCylRadius;					// Minimum Cylinder Radius in meters
	double maxCylRadius;					// Maximum Cylinder Radius in meters
	double minCylHeight;					// Minimum Cylinder Height in meters
	double maxCylHeight;					// Maximum Cylinder Height in meters
};