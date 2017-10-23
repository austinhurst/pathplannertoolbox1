/*	The purpose of this file is to give a template for developing new
*	path planning algorithms. It is meant to copied and pasted into a
*	new file. All of the important aspects of interfacing with the rest
*	of the program are included here. A few other helpful things that may
*	not be used in your specific algorithm are included here as well.
*	Only add lines to this file if it is to aid in the creation of new
*	algorithms.
*/

// Here are some other important notes about adding more algorithms:
// Remember to add #include "./../include/exampleAlgorithm.h" to main.cpp
// The exampleAlgorithm should also be included into the switch case in the main file. This is where
// it is potentially created. Follow the format all ready there, update the simulation_parameters.txt to indicate
// which case number it is to select your algorithm
//
// Instructions for adding it to the Makefile
// Write it in under the OBJS variable
// OBJS = $(OBJDIR)/main.o $(OBJDIR)/mapper.o $(OBJDIR)/randGen.o $(OBJDIR)/fileReader.o $(OBJDIR)/pathPlanner.o \
// $(OBJDIR)/simpleRRT.o $(OBJDUR)/exampleAlgorithm.o
// Add it to the dependencies of the main.o.
// HEADMAIN = $(HDIR)/mapper.h $(HDIR)/randGen.h $(HDIR)/fileReader.h $(HDIR)/simpleRRT.h $(HDIR)/exampleAlgorithm.h
// Create an instruction for building this algorithm's .o file
//$(OBJDIR)/exampleAlgorithm.o : $(SRCDIR)/exampleAlgorithm.cpp $(HDIR)/exampleAlgorithm.h $(HDIR)/pathPlanner.h
//		$(CC) $(CXXFLAGS) - o$(OBJDIR)/exampleAlgorithm.o $(SRCDIR)/exampleAlgorithm.cpp

#ifndef EXAMPLEALGORITHM_H	// Put file gaurds on your header
#define EXAMPLEALGORITHM_H	// They prevent compiler problems

#include "./../include/pathPlanner.h"

// Input Options
struct exampleAlgorithm_input
{
	// This struct hold all of the options that your algorithm has.
	// For example:
	// int numSolutions;		// This could be a the number of paths the algorithm finds before selecting the optimum
	exampleAlgorithm_input()	// This block defines a constructor, put in default values here
	{
		// For example:
		// numSolutions = 1;
	}
};

// Class Definition
class exampleAlgorithm : public pathPlanner
{
public:
	// This is the constructor
	exampleAlgorithm(map_s map_in, unsigned int seed, fileReader *input_file, exampleAlgorithm_input alg_options);
	// The deconstructor
	~exampleAlgorithm();

	void solve_static();				// This is a virtual function that solves for the static path
	void fprint_static_solution();		// This function prints out the files that are used for matlab to plor your algorithm

private:
	exampleAlgorithm_input alg_input;	// Save the input struct to use in functions other than the constructor.
	// Put the members and functions for your algorithm here.
	// Understand the pathPlanner class members and functions to understand what is all ready there to be used in your algorithm
	// flyZoneCheck(NED_s,NED_s,double) checks if the line connecting the 2 points stays at least a certain distance away from the obstacles.
	// That distance may be different for your application, but it will likely be the variable 'clearance' from the pathPlanner class.
};
#endif