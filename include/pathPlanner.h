#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <string>
#include <fstream>

#include "./../include/map_s.h"
#include "./../include/randGen.h"
#include "./../include/fileReader.h"

using namespace std;
class pathPlanner
{
public:
	pathPlanner();										// Constructor, sets up some simulation parameters
	virtual ~pathPlanner();								//	Deconstructor, frees vector memory
	virtual void solve_static();						// Virtual Function (the child class, algorithm should handel it) Solves for the static path
	virtual void fprint_static_solution();				// Virtual Function prints all of the run data (tree, final path etc.)
	double total_path_length;							// Total path length
	int total_nWPS;										// Total number of waypoints
private:
	bool lineAndPoint(NED_s ls, NED_s le, double MinMax[], double Mandb[], NED_s p, double r);	// Function called by the LINE flyZoneCheck(NED,NED,radius)
protected:
	fileReader *input_file;								// address of the input file
	vector<vector<NED_s> > all_wps;						// final path waypoints,
	vector<double> path_distances;						// Distances for the final path.
	map_s map;											// This is the terrain map that contains the boundary and obstacle information (static)
	void setup_flyZoneCheck();							// Function that does calculations on the boundary lines in preparation to flyZoneCheck()
	bool flyZoneCheck(const NED_s ps, const NED_s pe, const double r);	// Checks to see if a LINE is at least radius away from an obstacle.
	bool flyZoneCheck(const NED_s NED, const double r);	// Checks to see if a POINT is at least radius away from an obstacle.
	void compute_performance();							// After an algorithm runs this computes some basic performance stats.
	unsigned int nBPts;									// Number of boundary points
	vector<vector<double> > lineMinMax;					// (N x 4) vector containing the (min N, max N, min E, max E) for each boundary line
	vector<vector<double> > line_Mandb;					// (N x 4) vector that contains the slope and intercept of the line (m, b, (-1/m), (m + 1/m)) from N = m*E + b ... not sure about E = constant lines yet.
	randGen rg;											// Here is the random generator for the algorithm
	void ppSetup();										// This sets up some preliminary things like the below doubles
	double maxNorth;									// Maximum North coordinate inside the boundaries
	double minNorth;									// Minimum North coordinate inside the boundaries
	double maxEast;										// Maximum East  coordinate inside the boundaries
	double minEast;										// Minimum East  coordinate inside the boundaries
	double minFlyHeight;								// Minimum Fly Height (positive value)
	double maxFlyHeight;								// Maximum Fly Height (positive value)
	double clearance;									// The minimum clearance that the path will have away from any obstacles
	bool is3D;											// If the simulation is in 3D or 2D
};
#endif