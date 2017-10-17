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
 *		10-17-17	:	Author: Austin Hurst
 *						Notes : First path planning algorithm, RRT
 *
 */
#include <iostream>
#include <ctime>
#include <chrono>
#include <vector>
#include <fstream>

#include "./../include/mapper.h"
#include "./../include/randGen.h"
#include "./../include/pathPlanner.h"
#include "./../include/simpleRRT.h"
#include "./../include/fileReader.h"

using namespace std;

int main()
{
	// ***************************************************************************************
	//********************************** SETUP ***********************************************
	// Bring in the Default Parameter file
	fileReader input_file("./input_files/simulation_parameters.txt");

	// Setup the number of runs
	unsigned int ntrials = input_file.ntrials;						// Total number of trials
	unsigned int nMCs    = input_file.nMCs;							// Total number of Monte Carlo runs

	// Set up the Random seeds
	unsigned int seed = input_file.seed;
	// seed = 52613221;
	// seed = std::chrono::system_clock::now().time_since_epoch().count();
	randGen rg(seed);												// seed: max = 4,294,967,295 min = 0
	vector<unsigned int> map_seeds = rg.UINTv(nMCs);
	vector<unsigned int> alg_seeds = rg.UINTv(nMCs);

	// Output File
	ofstream performance_file;
	performance_file.open(input_file.performance_file_name.c_str());

	// Create the relevant input structs for the algorithms
	simpleRRT_input rrt;

	// ***************************************************************************************
	//******************************** SIMULATE **********************************************
	cout << "**************************************" << endl;
	for (unsigned int i = 0; i < ntrials; i++)
	{
		// Change parameters here according to the trial number
		int solver_type = input_file.solver_type;
		rrt.D = 45;

		// Record Containers
		double avgTime(0);
		double avgDist(0);
		double avgNWPS(0);

		// Perform the Monte Carlo Runs
		for (unsigned int j = 0; j < nMCs; j++)
		{
			// Create the mapper and solver objects
			mapper myWorld(map_seeds[j], &input_file);				// Inputs: seed for random generator

			// Choose which Algorithm
			pathPlanner *solver;
			switch (solver_type)
			{
			case 1:
				solver = new simpleRRT(myWorld.map, alg_seeds[j], &input_file, rrt);	//  Inputs: map, and seed for random generator
				break;
			default:
				solver = new simpleRRT(myWorld.map, alg_seeds[j], &input_file, rrt);	// needs a default solver to make the compiler happy.
				break;
			}

			// Solve
			clock_t timer = clock();								// Start the timer
			solver->solve_static();									// SOLVE!
			timer = clock() - timer;								// End the timer

			// Record the monte carlo run performance
			avgTime += (float)timer / CLOCKS_PER_SEC;
			avgDist += solver->total_path_length;
			avgNWPS += solver->total_nWPS;

			if (nMCs == 1 && ntrials == 1) // Don't print the tree or map if there are multiple simulations
			{
				myWorld.fprint_map();
				solver->fprint_static_solution();
			}

			// Clean up
			delete solver;
		}

		// Output results
		cout << "Trial Number :\t" << i << endl;
		cout << "Monte Carlos :\t" <<  nMCs << endl;
		cout << "Total time   :\t" << avgTime << "\tseconds" << endl;
		cout << "Average time :\t" << avgTime / nMCs << "\tseconds" << endl;
		cout << "Mean Distance:\t" << avgDist / nMCs << "\tmeters" << endl;
		cout << "Mean # of WPS:\t" << avgNWPS / nMCs << endl << endl;
		cout << "**************************************\n" << endl;
		performance_file << i << "\t" << nMCs <<"\t" << avgTime << "\t" << avgTime / nMCs << "\t" << avgDist / nMCs << "\t" << avgNWPS / nMCs << endl;
	}
	performance_file.close();

	system("pause");
	return 0;
}