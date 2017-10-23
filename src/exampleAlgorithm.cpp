/*	The purpose of this file is to give a template for developing new
 *	path planning algorithms. It is meant to copied and pasted into a
 *	new file. All of the important aspects of interfacing with the rest
 *	of the program are included here. A few other helpful things that may
 *	not be used in your specific algorithm are included here as well.
 *	Only add lines to this file if it is to aid in the creation of new
 *	algorithms.
 */

#include "./../include/exampleAlgorithm.h"

exampleAlgorithm::exampleAlgorithm(map_s map_in, unsigned int seed, fileReader *input_file_in, exampleAlgorithm_input alg_input_in)		// Setup the object
{
	// Keep these in your constructor... most of them give necessary stuff to the parent class
	input_file = input_file_in;
	alg_input = alg_input_in;
	map = map_in;											// Get a copy of the terrain map
	randGen rg_in(seed);									// Make a random generator object that is seeded This may not be used, but probably will be
	rg = rg_in;												// Copy that random generator into the class.
	ppSetup();												// default stuff for every algorithm that needs to be called after it recieves the map.
}
exampleAlgorithm::~exampleAlgorithm()
{
	// PLEASE DO NOT CAUSE MEMORY LEAKS
	// delete any new, allocated memory, (pointers, trees, etc.)
	// This function will be called by the main before a new simulation happens. It is called by the delete solver line.
}
void exampleAlgorithm::solve_static()								// This function solves for a path in between the waypoinnts
{
	// Solve to each Primary Waypoint

	// For every Primary Waypoint
	for (unsigned int i = 0; i < map.wps.size() - 1; i++)
	{
		// Make sure that you reach the next primary waypoint in this for loop
		// The bulk (if not all) of your algorithm will go here.
		// Please implement checks so that your algorithm does not get hung on impoissible maps
		// The mapper class doesn't check to see if the map it develops is possible to solve for a given clearance level.
		// The variable input_file->iters_limit is a variable set in the simulation_parameters.txt that can be used to control how many times the algorithm
		// tries to reach teh next waypoint before giving up or decreasing the clearance level.

		// Once you have found the next waypoint be sure to do the following things:
		// path_distances.push_back(x);			// Put the distance that it took to get from the last spot to this newly reached waypoint into this vector
		// all_wps.push_back(wps_to_PrimaryWP);	// Put all of the waypoints generated into this vector (including the Primary Waypoints. Notice it is a vector of vectors.
		// wps_to_PrimaryWP.clear();			// Remember to clear the vector you pushed to all_wps
	}
	// Once all of the waypoints have been reached end by calculating the performance
	compute_performance();						// Call this function (in the pathPlanner class). If you kept up with the path_distances and all_wps 2d vectors this function will be happy
}
void exampleAlgorithm::fprint_static_solution()						// Print the solution to the static solver
{
	// Print the files you need to plot in MATLAB
	// The toolbox all ready has files for plotting trees and the final paths
	// The tree plotter just calls a plot(X,Y) on the data you print to the file.
	// If you want to print the tree correctly recursively go down the tree and print each node. Remeber to print each node again on the way back up!
}