#include "./../include/simpleRRT.h"

simpleRRT::simpleRRT(map_s map_in, unsigned int seed)		// Setup the object
{
	D = 50;													// Distance between each simpleRRT waypoint
	map = map_in;											// Get a copy of the terrain map
	randGen rg_in(seed);									// Make a random generator object that is seeded
	rg = rg_in;												// Copy that random generator into the class.
	ppSetup();												// default stuff for every algorithm that needs to be called after it recieves the map.
}
simpleRRT::~simpleRRT()
{
	delete_tree();											// Delete all of those tree pointer nodes
	vector<node*>().swap(root_ptrs);						// Free the memory of the vector.
}
void simpleRRT::solve_static()								// This function solves for a path in between the waypoinnts (2 Dimensional)
{
	// Solve to each Primary Waypoint
	bool reached_next_wp, found_feasible_link;
	NED_s P;

	// For every Primary Waypoint develop a tree and a find a path
	for (unsigned int i = 0; i < map.wps.size()-1; i++)
	{
		reached_next_wp = false;							// Flag to see if you have reached the next waypoint
		node *root = new node;								// Starting position of the tree (and the waypoint beginning)
		root->NED = map.wps[i];
		root->parent = NULL;								// No parent
		root_ptrs.push_back(root);

		double theta;

		// Keep adding to the tree until you have found a solution
		while (reached_next_wp == false)
		{
			node *vpos = new node;							// vpos is the next node to add to the tree
			found_feasible_link = false;

			// Once you found a node to add to the tree that doesn't intersect with an obstacle, add it to the tree
			while (found_feasible_link == false)
			{
				// Generate random P until it is within boundaries and not on an obstacle.
				P.N = rg.randLin()*(maxNorth - minNorth) + minNorth;
				P.E = rg.randLin()*(maxEast - minEast) + minEast;
				P.D = 0; // Simple... 2D
				while (flyZoneCheck(P, clearance) == false)
				{
					P.N = rg.randLin()*(maxNorth - minNorth) + minNorth;
					P.E = rg.randLin()*(maxEast - minEast) + minEast;
					P.D = 0; // Simple... 2D
				}
				double distance = sqrt(pow(P.N - root->NED.N, 2) + pow(P.E - root->NED.E, 2));

				// Find the closest node to the point P;
				closest_node = find_closest_node(root, P, root, &distance);
				theta = atan2(P.N - closest_node->NED.N, P.E - closest_node->NED.E);

				// Go a distance D along the line from closest node to P to find the next node position vpos
				vpos->NED.N = (closest_node->NED.N) + sin(theta)*D;
				vpos->NED.E = (closest_node->NED.E) + cos(theta)*D;
				vpos->NED.D = (closest_node->NED.D);
				
				// If this path is good move on.
				if (flyZoneCheck(closest_node->NED, vpos->NED, clearance))
					found_feasible_link = true;
			}

			// add the new node to the tree
			closest_node->children.push_back(vpos);
			vpos->parent = closest_node;

			// Check to see if it is possible to go from this newly added node to the next primary waypoint
			if (flyZoneCheck(vpos->NED, map.wps[i + 1], clearance))
			{

				// We can go to the next waypoint!
				reached_next_wp = true;								// Set the flag
				node *final_node = new node;
				final_node->NED = map.wps[i + 1];
				vpos->children.push_back(final_node);
				final_node->parent = vpos;

				// populate all wps by working back up the tree
				stack<NED_s> wpstack;
				node *current_node = final_node;
				while (current_node != root)
				{
					wpstack.push(current_node->NED);
					current_node = current_node->parent;
				}
				wpstack.push(root->NED);

				// Now put all of the waypoints into the all_wps vector
				vector<NED_s> wps_to_PrimaryWP;
				while (!wpstack.empty())
				{
					wps_to_PrimaryWP.push_back(wpstack.top());
					wpstack.pop();
				}
				all_wps.push_back(wps_to_PrimaryWP);
				wps_to_PrimaryWP.clear();
			}
		}
	}
}
void simpleRRT::delete_tree()
{
	for (unsigned int i = 0; i < root_ptrs.size(); i++)		// Delete every tree generated
	{
		delete_node(root_ptrs[i]);
	}
}
void simpleRRT::delete_node(node* pn)						// Recursively delete every node
{
	for (unsigned int i = 0; i < pn->children.size();i++)
	{
		delete_node(pn->children[i]);
	}
	pn->children.clear();
	delete pn;
}
void simpleRRT::fprint_static_solution()						// Print the solution to the static solver
{

	// Maybe get smart about what to print depending on how many Monte Carlo runs there are.
	// Print the trees (each waypoint gets its own tree file
	string basefilename = "./graphing_files/output_tree_";
	string extension = ".txt";
	for (unsigned int i = 0; i < root_ptrs.size(); i++)
	{
		string filename = basefilename + to_string(i) + extension;
		ofstream tree_file;
		tree_file.open(filename.c_str());
		print_tree(root_ptrs[i], tree_file);
		tree_file.close();
	}

	// Print the final path
	string pathfilename = "./graphing_files/output_path.txt";
	ofstream path_file;
	path_file.open(pathfilename.c_str());
	for (unsigned int i = 0; i < all_wps.size(); i++)
		for (unsigned int j = 0; j < all_wps[i].size(); j++)
			path_file << all_wps[i][j].N << "\t" << all_wps[i][j].E << "\t" << all_wps[i][j].D << endl;
}
void simpleRRT::print_tree(node* ptr, ofstream& file)
{
	// Recursive function to print the tree in a way that MATLAB can easily plot it with plot();
	// Basically it prints the coordinates of each node and then when it works itself back up the tree it also prints the coordinate again.
	for (unsigned int i = 0; i < ptr->children.size(); i++)
	{
		file << (ptr->NED.N) << "\t" << (ptr->NED.E) << "\t" << (ptr->NED.D) << endl;
		print_tree(ptr->children[i], file);
	}
	file << (ptr->NED.N) << "\t" << (ptr->NED.E) << "\t" << (ptr->NED.D) << endl;
}