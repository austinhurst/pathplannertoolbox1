#ifndef SIMPLERRT_H
#define SIMPLERRT_H

#include <stack>

#include "./../include/pathPlanner.h"

// Input Options
struct simpleRRT_input
{
	// input options
	double D;					// This is the distance between each node
	bool uniform2P;				// If this is true the distance between each node is uniform between 0 and the distance to P
	bool gaussianD;				// If this is true the distance between each node is gaussian mean = D, std = gaussianSTD
	double gaussianSTD;			// The standard deviation if the gaussianD is set to true
	bool connect_to_end;		// If true the RRT will check each node to see if it can connect to the end
	simpleRRT_input()			// Struct constructor, pust the default values here.
	{
		D = 50;
		uniform2P = false;
		gaussianD = false;
		gaussianSTD = 15;
		connect_to_end = true;
	}
};

// CLass Definition
class simpleRRT : public pathPlanner										// Inherit the base class, pathPlanner
{
public:
	simpleRRT(map_s map_in, unsigned int seed, fileReader *input_file, simpleRRT_input RRT_options);		// Constructor - input the terrain map and the random generator seed
	~simpleRRT();															// Deconstructor - deletes the tree
	void solve_static();													// Solves the static path
	void fprint_static_solution();											// Prints out the tree, the final solution and the performance measures
private:
	simpleRRT_input alg_input;												// This contains all of the options for simpleRRT
	struct node																// This is the node struct for each spot on the tree
	{
		NED_s NED;															// North, East Down of the node position
		vector<node*> children;												// Vector of all of the children nodes
		node* parent;														// Pointer to the parent of this node
		double distance;													// Distance from this node to its parent
	};
	double D;																// If used, this is the distance the algorithm uses between each node
	node* find_closest_node(node* nin, NED_s P, node* minNode, double* minD)// This recursive function return the closes node to the input point P, for some reason it wouldn't go in the cpp...
	{// nin is the node to measure, P is the point, minNode is the closes found node so far, minD is where to store the minimum distance
		// Recursion
		double distance;													// distance to the point P
		for (unsigned int i = 0; i < nin->children.size(); i++)				// For all of the children figure out their distances
		{
			distance = sqrt(pow(P.N - nin->children[i]->NED.N, 2) + pow(P.E - nin->children[i]->NED.E, 2) + pow(P.D - nin->children[i]->NED.D, 2));	// Calculate the distance to the node
			if (distance < *minD)											// If we found a better distance, update it
			{
				minNode = nin->children[i];									// reset the minNode
				*minD = distance;											// reset the minimum distance
			}
			minNode = find_closest_node(nin->children[i], P, minNode, minD);// Recursion for each child
		}
		return minNode;														// Return the closest node
	}
	void delete_tree();														// Delete the entire tree
	void delete_node(node*);												// Recursively delete the nodes
	vector<node*> root_ptrs;												// Vector of all roots, each element is the start of the tree to reach the next primary waypoint
	void print_tree(node* ptr, ofstream& file);								// Print the tree in a way that MATLAB can easily graph this tree with plot()
	node *closest_node;														// This is a variable that is used to find the closest node - if it is in here there are no memory leaks.
};
#endif