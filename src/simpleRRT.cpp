#include "./../include/simpleRRT.h"

simpleRRT::simpleRRT(map_s map_in, unsigned int seed, fileReader *input_file_in, simpleRRT_input alg_input_in)		// Setup the object
{
	input_file = input_file_in;
	alg_input = alg_input_in;
	D = alg_input.D;										// Distance between each simpleRRT waypoint
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
	// Different Algorithm Settings
	// Can vary the D, the distance between each node			alg_input.D
	// Can gaussian the D, distance between nodes are gaussian	alg_input.gaussianD, alg_input.gaussianSTD
	// Can uniform to the point, connect to end or not.			alg_input.connect_to_end
	
	// Solve to each Primary Waypoint
	bool reached_next_wp, found_feasible_link;
	NED_s P;

	// For every Primary Waypoint develop a tree and a find a path
	for (unsigned int i = 0; i < map.wps.size()-1; i++)
	{
		// Set up some initial things for this waypoint to waypoint tree
		reached_next_wp = false;							// Flag to see if you have reached the next waypoint
		node *root = new node;								// Starting position of the tree (and the waypoint beginning)
		root->NED = map.wps[i];
		root->NED.D = 0;									// 0 for simple 2d
		root->parent = NULL;								// No parent
		root->distance = 0.0;								// 0 distance.
		root_ptrs.push_back(root);
		node *second2last = root;							// This will be set as the second to last waypoint
		double theta;
		double clearanceP = clearance;

		// Check to see if it is possible to go stright to the next path
		if (alg_input.connect_to_end && flyZoneCheck(root->NED, map.wps[i + 1], clearance))
			reached_next_wp = true;								// Set the flag
		if (!alg_input.connect_to_end && sqrt(pow(map.wps[i + 1].N - root->NED.N, 2) + pow(map.wps[i + 1].E - root->NED.E, 2) + pow(map.wps[i + 1].D - root->NED.D, 2)) < D && flyZoneCheck(root->NED, map.wps[i + 1], clearance))
			reached_next_wp = true;								// Set the flag

		// Keep adding to the tree until you have found a solution
		unsigned int added_nodes = 0;							// Keep a record of how many nodes are added so that the algorithm doesn't get stuck.
		while (reached_next_wp == false)
		{
			node *vpos = new node;								// vpos is the next node to add to the tree
			found_feasible_link = false;

			// Once you found a node to add to the tree that doesn't intersect with an obstacle, add it to the tree
			while (found_feasible_link == false)
			{
				// Generate random P until it is within boundaries and not on an obstacle.
				P.N = rg.randLin()*(maxNorth - minNorth) + minNorth;
				P.E = rg.randLin()*(maxEast - minEast) + minEast;
				P.D = 0; // Simple... 2D
				unsigned int p_count(0);
				while (flyZoneCheck(P, clearanceP) == false)
				{
					P.N = rg.randLin()*(maxNorth - minNorth) + minNorth;
					P.E = rg.randLin()*(maxEast - minEast) + minEast;
					P.D = 0; // Simple... 2D
					p_count++;
					if (p_count >= iters_limit / 2)
					{
						clearanceP = clearanceP / 2.0;
						p_count = p_count*7.0 / 8.0;
					}
				}
				double distance = sqrt(pow(P.N - root->NED.N, 2) + pow(P.E - root->NED.E, 2) + pow(P.D - root->NED.D, 2));

				// Find the closest node to the point P;
				closest_node = find_closest_node(root, P, root, &distance);
				theta = atan2(P.N - closest_node->NED.N, P.E - closest_node->NED.E);

				// Go a distance D along the line from closest node to P to find the next node position vpos
				if (alg_input.uniform2P)
					D = rg.randLin()*distance;
				else if (alg_input.gaussianD)
					D = rg.randNor(distance,alg_input.gaussianSTD);
				vpos->NED.N = (closest_node->NED.N) + sin(theta)*D;
				vpos->NED.E = (closest_node->NED.E) + cos(theta)*D;
				vpos->NED.D = 0; // Simple... 2D
				
				// If this path is good move on.
				// Check to see if the straight line path is good.
				if (flyZoneCheck(closest_node->NED, vpos->NED, clearance))
					found_feasible_link = true;
				// Check to see if for any 2 paths formed together there is a good fillet.
					
			}

			// add the new node to the tree
			closest_node->children.push_back(vpos);
			vpos->parent = closest_node;
			vpos->distance = sqrt(pow(closest_node->NED.N - vpos->NED.N, 2) + pow(closest_node->NED.E - vpos->NED.E, 2) + pow(closest_node->NED.D - vpos->NED.D, 2));
			added_nodes++;
			if (added_nodes == iters_limit/2 || added_nodes == floor(iters_limit*3.0/4.0) || added_nodes == floor(iters_limit*7.0 / 8.0))
			{
				cout << "DECREASING THE CLEARANCE LEVEL" << endl;
				clearance = clearance / 2.0;
				cout << "CLEARANCE:\t" << clearance << endl;
			}
			if (added_nodes >= iters_limit)
			{
				cout << "WARNING -- ALGORITHM FAILED TO CONVERGE\nRESULTS WILL VIOLATE AN OBSTACLE" << endl;
				reached_next_wp = true;
				second2last = vpos;
			}
			// Check to see if it is possible to go from this newly added node to the next primary waypoint
			if (alg_input.connect_to_end && flyZoneCheck(vpos->NED, map.wps[i + 1], clearance))
			{
				reached_next_wp = true;								// Set the flag
				second2last = vpos;
			}								// Set the flag
			if (!alg_input.connect_to_end && sqrt(pow(map.wps[i + 1].N - vpos->NED.N, 2) + pow(map.wps[i + 1].E - vpos->NED.E, 2) + pow(map.wps[i + 1].D - vpos->NED.D, 2)) < D && flyZoneCheck(vpos->NED, map.wps[i + 1], clearance))
			{
				reached_next_wp = true;								// Set the flag
				second2last = vpos;
			}								// Set the flag
		}
		// We can go to the next waypoint!
		// The following code wraps up the algorithm.
		double pdistance = 0;								// This is a distance that cummulates into the total path distance (from one waypoint to another)
		node *final_node = new node;
		final_node->NED = map.wps[i + 1];
		second2last->children.push_back(final_node);
		final_node->parent = second2last;
		final_node->distance = sqrt(pow(final_node->NED.N - second2last->NED.N, 2) + pow(final_node->NED.E - second2last->NED.E, 2) + pow(final_node->NED.D - second2last->NED.D, 2));

		// populate all wps by working back up the tree
		stack<NED_s> wpstack;
		node *current_node = final_node;
		while (current_node != root)
		{
			wpstack.push(current_node->NED);
			pdistance += current_node->distance;
			current_node = current_node->parent;
		}
		wpstack.push(root->NED);
		path_distances.push_back(pdistance);

		// Now put all of the waypoints into the all_wps vector
		vector<NED_s> wps_to_PrimaryWP;
		while (!wpstack.empty())
		{
			wps_to_PrimaryWP.push_back(wpstack.top());
			wpstack.pop();
		}
		all_wps.push_back(wps_to_PrimaryWP);
		wps_to_PrimaryWP.clear();

		// Smooth Out the path (Algorithm 11 in the UAV book)
		vector<NED_s> path_smoothed;
		double smooth_distances = 0;
		unsigned int i_node = 0;
		unsigned int j_node = 1;
		path_smoothed.push_back(all_wps[i][0]);
		while (j_node < all_wps[i].size())
		{
			if (all_wps[i].size() >= j_node + 2 && flyZoneCheck(path_smoothed[i_node], all_wps[i][j_node + 1], clearance) == false)
			{
				path_smoothed.push_back(all_wps[i][j_node]);
				i_node++;
				smooth_distances += sqrt(pow(path_smoothed[i_node].N - path_smoothed[i_node -1].N,2.) + pow(path_smoothed[i_node].E - path_smoothed[i_node - 1].E,2) + pow(path_smoothed[i_node].D - path_smoothed[i_node - 1].D,2));
			}
			j_node++;
		}
		path_distances[i] = smooth_distances;
		all_wps[i].swap(path_smoothed);
	}
	// Add the final waypoint to the waypoint list.
	all_wps[map.wps.size() - 2].push_back(map.wps[map.wps.size() - 1]);
	compute_performance();									// Do this after you finish the algorithm so that we can get the right performance values.
	


	//*********************************************************************************************
	// We still need to do some improvements.

	// (all ready complete) get the algorithm to connect point to point
	// Second Task. Kinematically possible route.
	//		cleaning up unneseccary waypoints (mostly done)
	//		which type of path? Straight, fillet, dubins
	//		possibly add some waypoints around the primary waypoints to help "guide" the algorithms into and out of waypoints
	//		make sure it won't collide anywhere
	// Third task get it to work in 3 dimensions
	//		Climb and descend rates
	//		look over obstacles in efficient way
	// Fourth Task
	//		Get all of the above to work with moving targets (maybe not until december...)
	// Refinement stage
	//		Possibly get the algorithm to compute multiple paths to get the optimal solution.
	//**********************************************************************************************
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
	// Print the trees (each waypoint gets its own tree file
	string basefilename = input_file->tree_file;
	string extension = input_file->file_extension;
	for (unsigned int i = 0; i < root_ptrs.size(); i++)
	{
		string filename = basefilename + to_string(i) + extension;
		ofstream tree_file;
		tree_file.open(filename.c_str());
		print_tree(root_ptrs[i], tree_file);
		tree_file.close();
	}

	// Print the final path
	string pathfilename = input_file->path_file;
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