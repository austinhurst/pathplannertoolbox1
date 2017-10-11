/*	This project is intended to provide a test bed for path planning algorithms.
 *	The promising algorithms will be implemented into the AUVSI ROSplane source.
 *	
 *	The goal of the path planner is to take in primary waypoints and a map and 
 *	then develop a "good" set of waypoints/path. The goals are to stay within 
 *	boundaries, avoid static obstacles and avoid moving obstacles, and of course
 *	hit the primary waypoints.
 *	
 *	REVISION HISTORY:
 *		10-10-17	:	Author:	Austin Hurst
 *						Notes :	Initial Commit, Basic Structure
 *
 */

#include <iostream>

#include "./../include/mapper.h"
#include "./../include/randgen.h"

using namespace std;

int main()
{
	// Create the mapper object
	mapper myWorld(123456789);  // Inputs: seed for random generator
	myWorld.fprint_map();
	//system("pause");
	return 0;
}