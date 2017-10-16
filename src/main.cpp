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
 *		10-16-17	:	Author: Austin Hurst
 *						Notes : First path planning algorithm, RRT
 *
 */

#include <iostream>
#include <ctime>

#include "./../include/mapper.h"
#include "./../include/randGen.h"
#include "./../include/pathPlanner.h"
#include "./../include/simpleRRT.h"

using namespace std;

int main()
{


	// Create the mapper and solver objects
	mapper myWorld(7543);						// Inputs: seed for random generator, max = 4,294,967,295 min = 0
	pathPlanner *solver;

	// Choose which Algorithm
	solver = new simpleRRT(myWorld.map, 30221); //  Inputs: map, and seed for random generator, max = 4,294,967,295 min = 0

	// Solve
	clock_t timer = clock();					// Start the timer
	solver->solve_static();						// SOLVE!
	timer = clock() - timer;					// End the timer

	// Output results
	myWorld.fprint_map();
	solver->fprint_static_solution();
	printf("Execution time: %f  seconds\n", (float)timer / CLOCKS_PER_SEC);

	// Clean up
	delete solver;
	


	//system("pause");
	return 0;
}