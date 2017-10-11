/*	DESCRIPTION:
 *	This is a header that defines a struct that contains all of the static
 *	information about the competition environment. It contains the boundaries
 *	and the static obstacles (cylinders).
 *
 */

//#pragma once
#ifndef MAP_H
#define MAP_H

#include <vector>

using namespace std;

struct map_s
{
	vector<vector<double> > boundary_pts;		// (N x 2) Contains the North and East position in meters. (No Repeats)
	vector<vector<double> > cylinders;			// (N x 4) North, East, Radius, Height
};

#endif