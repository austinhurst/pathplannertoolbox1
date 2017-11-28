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
	initialize_tree();
	NED_s second2last_post_smoothed = root_ptrs[0]->NED;
	for (unsigned int i = 0; i < map.wps.size()-1; i++)
	{
		cout << "MAIN LOOP: " << i << endl;
		path_clearance = input_file->clearance;
		node *second2last = root_ptrs[i];					// This will be set as the second to last waypoint
		double distance_in, fillet_angle;
		bool direct_shot = false;
		bool direct_connect = direct_connection(i, &second2last_post_smoothed, &distance_in, &fillet_angle, &direct_shot);		//******* IMPORTANT
		if (direct_connect)
			second2last = closest_node;
		develop_tree(i, direct_connect,second2last,&second2last_post_smoothed,&distance_in,&fillet_angle);						//******* IMPORTANT
		smoother(direct_connect, i, &distance_in, &fillet_angle, &second2last_post_smoothed, direct_shot);						//******* IMPORTANT
		calc_path_distance(i);
	}
	all_wps[map.wps.size() - 2].push_back(map.wps[map.wps.size() - 1]);	// Add the final waypoint to the waypoint list.
	compute_performance();
}
bool simpleRRT::check_direct_fan(NED_s second_wp, NED_s primary_wp, NED_s coming_from, node* next_root, NED_s* cea_out, double* din, double* anglin)
{
	bool found_at_least_1_good_path = false;
	double R = sqrt(pow(second_wp.N - primary_wp.N,2) + pow(second_wp.E - primary_wp.E,2) + pow(second_wp.D - primary_wp.D,2));

	double approach_angle = atan2(primary_wp.N - coming_from.N, primary_wp.E - coming_from.E) + 3.141592653;
	double approach_angleDEGREES = approach_angle*180.0 / 3.141592653;
	double beta, lambda, Q, phi, theta, zeta, gamma, d;
	NED_s cpa, cea, lea, fake_wp;

	// What should alpha be?
	double leave_angle = atan2(second_wp.N - primary_wp.N, second_wp.E - primary_wp.E);
	double leave_angleDEGREES = leave_angle*180.0 / 3.141592653;
	double alpha = atan2(second_wp.N - primary_wp.N, second_wp.E - primary_wp.E) - approach_angle;
	//double alphaDEGREES = alpha*180.0 / 3.141592653;
	while (alpha < -3.141592653)
		alpha = alpha + 2 * 3.141592653;
	while (alpha > 3.141592653)
		alpha = alpha - 2 * 3.141592653;

	bool positive_angle = true;
	if (alpha < 0.0)
	{
		positive_angle = false;
		alpha = -1.0*alpha;
	}
	if (2.0*input_file->turn_radius / R > 1.0 || 2.0*input_file->turn_radius / R < -1.0)
		return false;
	double minAngle = asin(2.0*input_file->turn_radius / R) + 8*3.141592653/180.0;
	if (alpha < minAngle || (alpha > 3.141592653- minAngle && alpha < 3.141592653 + minAngle))
		return false;

	beta = 3.141592653 / 2 - alpha;
	lambda = 3.141592653 - 2 * beta;
	Q = sqrt(R*(R - input_file->turn_radius*sin(lambda) / sin(beta)) + input_file->turn_radius*input_file->turn_radius);
	phi = 3.141592653 - asin(R*sin(beta) / Q);
	theta = acos(input_file->turn_radius / Q);
	zeta = (2 * 3.141592653 - phi - theta) / 2.0;
	gamma = 3.141592653 - 2 * zeta;
	d = input_file->turn_radius / tan(gamma / 2.0);

	fake_wp.N = primary_wp.N - d*sin(approach_angle);
	fake_wp.E = primary_wp.E - d*cos(approach_angle);
	fake_wp.D = primary_wp.D;

	// What should it be on the positive or on the negative side?
	if (positive_angle)
	{
		// Check the positive side
		cpa.N = primary_wp.N + input_file->turn_radius*cos(approach_angle);
		cpa.E = primary_wp.E - input_file->turn_radius*sin(approach_angle);
		cpa.D = primary_wp.D;

		cea.N = fake_wp.N + d*sin(gamma + approach_angle);
		cea.E = fake_wp.E + d*cos(gamma + approach_angle);
		cea.D = primary_wp.D;

		lea.N = primary_wp.N + R*sin(approach_angle + alpha);
		lea.E = primary_wp.E + R*cos(approach_angle + alpha);
		lea.D = primary_wp.D;

		if (flyZoneCheck(primary_wp, cea, input_file->turn_radius, cpa, clearance, false))
			if (flyZoneCheck(cea, lea, clearance))
			{
				// Looks like things are going to work out for this maneuver!
				found_at_least_1_good_path = true;
				node *fake_child = new node;
				//node *normal_gchild = new node;
				fake_child->NED = fake_wp;
				fake_child->available_dist = 0;
				fake_child->parent = next_root;
				fake_child->distance = 2.0*zeta*input_file->turn_radius;
				fake_child->path_type = 1;
				next_root->children.push_back(fake_child);
			}
	}
	else // Check the negative side
	{
		cpa.N = primary_wp.N - input_file->turn_radius*cos(approach_angle);
		cpa.E = primary_wp.E + input_file->turn_radius*sin(approach_angle);
		cpa.D = primary_wp.D;

		cea.N = fake_wp.N + d*sin(-gamma + approach_angle);
		cea.E = fake_wp.E + d*cos(-gamma + approach_angle);
		cea.D = primary_wp.D;

		lea.N = primary_wp.N + R*sin(approach_angle - alpha);
		lea.E = primary_wp.E + R*cos(approach_angle - alpha);
		lea.D = primary_wp.D;

		if (flyZoneCheck(primary_wp, cea, input_file->turn_radius, cpa, clearance, false))
			if (flyZoneCheck(cea, lea, clearance))
			{
				// Looks like things are going to work out for this maneuver!
				found_at_least_1_good_path = true;
				node *fake_child = new node;
				node *normal_gchild = new node;
				fake_child->NED = fake_wp;
				fake_child->available_dist = 0;
				fake_child->parent = next_root;
				fake_child->distance = 2.0*zeta*input_file->turn_radius;
				fake_child->path_type = 1;
				next_root->children.push_back(fake_child);
			}
	}
	*cea_out = cea;
	*din = sqrt(pow(fake_wp.N - cea.N, 2) + pow(fake_wp.E - cea.E, 2) + pow(fake_wp.D - cea.D, 2));
	*anglin = 2.0*zeta;
	return found_at_least_1_good_path;
}
bool simpleRRT::check_create_fan(NED_s primary_wp, NED_s coming_from, node* next_root)
{
	bool found_at_least_1_good_path = false;
	// Make sure that it is possible to go to the next waypoint

	double alpha = 3.141592653 / 4.0;			// Start with 45.0 degrees
	double R = 3 * input_file->turn_radius;
	int num_circle_trials = 10;				// Will try num_circle_trials on one side and num_circle_trials on the other side.
	double dalpha = (3.141592653 - alpha) / num_circle_trials;

	double approach_angle = atan2(primary_wp.N - coming_from.N, primary_wp.E - coming_from.E) + 3.141592653;
	double beta, lambda, Q, phi, theta, zeta, gamma, d;
	NED_s cpa, cea, lea, fake_wp;
	for (int j = 0; j < num_circle_trials; j++)
	{
		alpha = alpha + dalpha;
		beta = 3.141592653 / 2 - alpha;
		lambda = 3.141592653 - 2 * beta;
		Q = sqrt(R*(R - input_file->turn_radius*sin(lambda) / sin(beta)) + input_file->turn_radius*input_file->turn_radius);
		phi = 3.141592653 - asin(R*sin(beta) / Q);
		theta = acos(input_file->turn_radius / Q);
		zeta = (2 * 3.141592653 - phi - theta) / 2.0;
		gamma = 3.141592653 - 2 * zeta;
		d = input_file->turn_radius / tan(gamma / 2.0);

		// Check the positive side
		fake_wp.N = primary_wp.N - d*sin(approach_angle);
		fake_wp.E = primary_wp.E - d*cos(approach_angle);
		fake_wp.D = primary_wp.D;

		cpa.N = primary_wp.N + input_file->turn_radius*cos(approach_angle);
		cpa.E = primary_wp.E - input_file->turn_radius*sin(approach_angle);
		cpa.D = primary_wp.D;

		cea.N = fake_wp.N + d*sin(gamma + approach_angle);
		cea.E = fake_wp.E + d*cos(gamma + approach_angle);
		cea.D = primary_wp.D;

		lea.N = primary_wp.N + R*sin(approach_angle + alpha);
		lea.E = primary_wp.E + R*cos(approach_angle + alpha);
		lea.D = primary_wp.D;

		if (flyZoneCheck(primary_wp, cea, input_file->turn_radius, cpa, clearance, false))
			if (flyZoneCheck(cea, lea, clearance))
			{
				// Looks like things are going to work out for this maneuver!
				found_at_least_1_good_path = true;
				node *fake_child = new node;
				node *normal_gchild = new node;
				fake_child->NED = fake_wp;
				fake_child->available_dist = 0;
				fake_child->parent = next_root;
				fake_child->distance = 2.0*zeta*input_file->turn_radius;
				fake_child->path_type = 1;
				next_root->children.push_back(fake_child);

				normal_gchild->NED = lea;
				normal_gchild->available_dist = sqrt(R*(R - input_file->turn_radius*sin(lambda) / sin(beta)));
				normal_gchild->parent = fake_child;
				normal_gchild->distance = normal_gchild->available_dist;
				normal_gchild->path_type = 1;
				fake_child->children.push_back(normal_gchild);
			}
		// Check the negative side
		cpa.N = primary_wp.N - input_file->turn_radius*cos(approach_angle);
		cpa.E = primary_wp.E + input_file->turn_radius*sin(approach_angle);
		cpa.D = primary_wp.D;

		cea.N = fake_wp.N + d*sin(-gamma + approach_angle);
		cea.E = fake_wp.E + d*cos(-gamma + approach_angle);
		cea.D = primary_wp.D;

		lea.N = primary_wp.N + R*sin(approach_angle - alpha);
		lea.E = primary_wp.E + R*cos(approach_angle - alpha);
		lea.D = primary_wp.D;

		if (flyZoneCheck(primary_wp, cea, input_file->turn_radius, cpa, clearance, false))
			if (flyZoneCheck(cea, lea, clearance))
			{
				// Looks like things are going to work out for this maneuver!
				found_at_least_1_good_path = true;
				node *fake_child = new node;
				node *normal_gchild = new node;
				fake_child->NED = fake_wp;
				fake_child->available_dist = 0;
				fake_child->parent = next_root;
				fake_child->distance = 2.0*zeta*input_file->turn_radius;
				fake_child->path_type = 1;
				next_root->children.push_back(fake_child);

				normal_gchild->NED = lea;
				normal_gchild->available_dist = sqrt(R*(R - input_file->turn_radius*sin(lambda) / sin(beta)));
				normal_gchild->parent = fake_child;
				normal_gchild->distance = normal_gchild->available_dist;
				normal_gchild->path_type = 1;
				fake_child->children.push_back(normal_gchild);
			}
	}
	return found_at_least_1_good_path;
}
bool simpleRRT::check_fillet(NED_s par, NED_s mid, NED_s nex, double avail_dis, double* din, double* cangle)
{
	bool found_feasible_link = true;
	// Calculate the fillet and check to see if it is a good fit.
	// a dot b = ||A|| * ||B|| * cos(theta)
	double a_dot_b = (par.E - mid.E)*(nex.E - mid.E) + (par.N - mid.N)*(nex.N - mid.N) + (par.D - mid.D)*(nex.D - mid.D);
	double A = sqrt(pow(par.E - mid.E, 2) + pow(par.N - mid.N, 2) + pow(par.D - mid.D, 2));
	double B = sqrt(pow(nex.N - mid.N, 2) + pow(nex.E - mid.E, 2) + pow(nex.D - mid.D, 2));
	double Fangle = acos((a_dot_b) / (A*B));
	double turn_radius = input_file->turn_radius;
	double distance_in = turn_radius / tan(Fangle / 2.0);// Notice this equation was written incorrectly in the UAV book //sqrt(turn_radius*turn_radius / sin(Fangle / 2.0) / sin(Fangle / 2.0) - turn_radius*turn_radius);
	if (distance_in > avail_dis || distance_in > sqrt(pow(mid.N - nex.N, 2) + pow(mid.E - nex.E, 2) + pow(mid.D - nex.D, 2)))
		found_feasible_link = false;
	else
	{
		NED_s ps, pe, cp;
		double theta = atan2(nex.N - mid.N, nex.E - mid.E);
		pe.N = (mid.N) + sin(theta)*distance_in;
		pe.E = (mid.E) + cos(theta)*distance_in;
		pe.D = mid.D;
		double gamma = atan2(par.N - mid.N, par.E - mid.E);
		ps.N = (mid.N) + sin(gamma)*distance_in;
		ps.E = (mid.E) + cos(gamma)*distance_in;
		ps.D = mid.D;
		// Find out whether it is going to the right (cw) or going to the left (ccw)
		// Use the cross product to see if it is cw or ccw
		bool ccw;
		double cross_product = ((mid.E - ps.E)*(pe.N - mid.N) - (mid.N - ps.N)*(pe.E - mid.E));
		if (cross_product < 0)
			ccw = false;
		else
			ccw = true;
		if (ccw)
		{
			cp.N = (mid.N) + sin(gamma - Fangle / 2.0)*turn_radius / sin(Fangle / 2.0);
			cp.E = (mid.E) + cos(gamma - Fangle / 2.0)*turn_radius / sin(Fangle / 2.0);
		}
		else
		{
			cp.N = (mid.N) + sin(gamma + Fangle / 2.0)*turn_radius / sin(Fangle / 2.0);
			cp.E = (mid.E) + cos(gamma + Fangle / 2.0)*turn_radius / sin(Fangle / 2.0);
		}
		cp.D = mid.D;
		if (flyZoneCheck(ps, pe, turn_radius, cp, clearance, ccw) == false)
			found_feasible_link = false;
	}
	*din = distance_in;
	*cangle = 2.0*atan(distance_in / turn_radius);
	return found_feasible_link;
}
void simpleRRT::initialize_tree()
{
	// Set up all of the roots
	for (unsigned int i = 0; i < map.wps.size() - 1; i++)
	{
		node *root_in = new node;							// Starting position of the tree (and the waypoint beginning)
		root_in->NED = map.wps[i];
		root_in->parent = NULL;								// No parent
		root_in->distance = 0.0;							// 0 distance.
		root_in->available_dist = 0.0;						// No available distance, (No parent assumption)
		root_in->path_type = 0;								// straight lines for now at the primary waypoints.
		root_ptrs.push_back(root_in);
	}
}
bool simpleRRT::direct_connection(unsigned int i, NED_s* second2last_post_smoothed, double* distance_in, double* fillet_angle, bool* direct_shot)
{
	// Variables created for this chuck of code in a function
	bool reached_next_wp = false;
	NED_s P;
	node* root = root_ptrs[i];
	double distance;
	
	// Make sure that the clearance level is right. Mostly this little bit checks to make sure that there is appropriate room to get waypoints close to the floor or ceiling.
	clearance = input_file->clearance;
	clearance = (-map.wps[i].D   - input_file->minFlyHeight < clearance) ? -map.wps[i].D   - input_file->minFlyHeight : clearance;
	clearance = ( map.wps[i].D   + input_file->maxFlyHeight < clearance) ?  map.wps[i].D   + input_file->maxFlyHeight : clearance;
	clearance = (-map.wps[i+1].D - input_file->minFlyHeight < clearance) ? -map.wps[i+1].D - input_file->minFlyHeight : clearance;
	clearance = ( map.wps[i+1].D + input_file->maxFlyHeight < clearance) ?  map.wps[i+1].D + input_file->maxFlyHeight : clearance;
	if (clearance <= 0)
		cout << "ERROR. CLEARANCE IS 0 OR LESS THAN 0." << endl;

	// Check to see if it is possible to go straight to the next path
	cout << "\tChecking direct Connect" << endl;
	NED_s coming_from;
	if (i == 0)
	{
		coming_from = map.wps[i];
		reached_next_wp = flyZoneCheck(root->NED, map.wps[i + 1], clearance);
		closest_node = root;
		if (reached_next_wp)
			reached_next_wp = check_slope(root->NED, map.wps[i + 1]);
	}
	else
	{
		P = map.wps[i + 1];
		distance = 9999999999999999999999.0; // Some crazy big number to start out with, maybe look into HUGE_VAL or Inf - worried about embedded implementation.
		double distance_gchild;
		bool found_clean_path = false;
		// Check to see if you can make one turn radius and then go to the next waypoint.
		NED_s cea;
		if (i <= map.wps.size() - 1 && check_direct_fan(map.wps[i + 1], root->NED, *second2last_post_smoothed, root, &cea, distance_in, fillet_angle))
		{
			if (check_slope(root->children[root->children.size() - 1]->NED, map.wps[i + 1]))
			{
				closest_node = root->children[root->children.size() - 1];
				found_clean_path = true;
				*direct_shot = true;
				coming_from = closest_node->NED;
				reached_next_wp = true;
				closest_node->available_dist = 0.0;
				distance = sqrt(pow(root->NED.N - closest_node->NED.N, 2) + pow(root->NED.E - closest_node->NED.E, 2) + pow(root->NED.D - closest_node->NED.D, 2));
				closest_node->distance = distance;
			}
		}
		else
		{
			for (unsigned int j = 0; j < root->children.size(); j++)
				for (unsigned int k = 0; k < root->children[j]->children.size(); k++)
				{
					distance_gchild = sqrt(pow(P.N - root->children[j]->children[k]->NED.N, 2) + pow(P.E - root->children[j]->children[k]->NED.E, 2) + pow(P.D - root->children[j]->children[k]->NED.D, 2));
					closest_node_gchild = find_closest_node(root->children[j]->children[k], P, root->children[j]->children[k], &distance_gchild);
					if (distance_gchild < distance)
					{
						closest_node = closest_node_gchild;
						distance = distance_gchild;
					}
				}
			coming_from = closest_node->NED;
			reached_next_wp = flyZoneCheck(coming_from, map.wps[i + 1], clearance) && check_slope(coming_from, map.wps[i + 1]);
			if (reached_next_wp)
				reached_next_wp = check_fillet(closest_node->parent->NED, closest_node->NED, map.wps[i + 1], closest_node->available_dist, distance_in, fillet_angle);
		}
	}
	if (reached_next_wp == true && i < map.wps.size() - 2) // This if statement handles setting waypoints to fly out of the primary waypoints.
		if (check_create_fan(map.wps[i + 1], coming_from, root_ptrs[i + 1]) == false)
			reached_next_wp = false;
	return reached_next_wp;
}
void simpleRRT::develop_tree(unsigned int i, bool reached_next_wp, node* second2last, NED_s* second2last_post_smoothed, double* distance_in, double* fillet_angle)
{
	// Variables needed to create for this function
	double distance;
	NED_s P;
	node* root = root_ptrs[i];
	double clearanceP = clearance;
	int clearance_drops = 0;
	double added_nodes = 0.0;								// Keep a record of how many nodes are added so that the algorithm doesn't get stuck.
	cout << "\tDeveloping branches" << endl;
	while (reached_next_wp == false)
	{
		node *vpos = new node;								// vpos is the next node to add to the tree
		bool found_feasible_link = false;

		// Once you found a node to add to the tree that doesn't intersect with an obstacle, add it to the tree
		unsigned int p_count(0);
		while (found_feasible_link == false)
		{
			// Generate random P until it is within boundaries and not on an obstacle.
			P.N = rg.randLin()*(maxNorth - minNorth) + minNorth;
			P.E = rg.randLin()*(maxEast - minEast) + minEast;
			P.D = map.wps[i + 1].D;
			while (flyZoneCheck(P, clearanceP) == false)
			{
				P.N = rg.randLin()*(maxNorth - minNorth) + minNorth;
				P.E = rg.randLin()*(maxEast - minEast) + minEast;
				P.D = map.wps[i + 1].D;
			}
			p_count++;
			if (p_count == floor(iters_limit / 2.0) || p_count == floor(iters_limit*3.0 / 4.0) || p_count == floor(iters_limit*7.0 / 8.0))
			{
				clearanceP = clearanceP / 6.0;
				cout << "DECREASING THE CLEARANCE LEVEL" << endl;
				clearance = clearance / 2.0;
				cout << "CLEARANCE:\t" << clearance << endl;
				path_clearance = (clearance < path_clearance) ? clearance : path_clearance;
			}
			distance = sqrt(pow(P.N - root->NED.N, 2) + pow(P.E - root->NED.E, 2) + pow(P.D - root->NED.D, 2));

			// Find the closest node to the point P, if you are not trying to get to waypoint 1 (which is the second waypoint), then don't accept the root or the children of the root.
			if (i == 0)
				closest_node = find_closest_node(root, P, root, &distance);
			else
			{
				distance = 9999999999999999999999.0; // Some crazy big number to start out with - maybe look into HUGE_VAL or Inf - worried about embedded implementation.
				double distance_gchild;
				for (unsigned int j = 0; j < root->children.size(); j++)
					for (unsigned int k = 0; k < root->children[j]->children.size(); k++)
					{
						distance_gchild = sqrt(pow(P.N - root->children[j]->children[k]->NED.N, 2) + pow(P.E - root->children[j]->children[k]->NED.E, 2) + pow(P.D - root->children[j]->children[k]->NED.D, 2));
						closest_node_gchild = find_closest_node(root->children[j]->children[k], P, root->children[j]->children[k], &distance_gchild);
						if (distance_gchild < distance)
						{
							closest_node = closest_node_gchild;
							distance = distance_gchild;
						}
					}
			}
			double theta = atan2(P.N - closest_node->NED.N, P.E - closest_node->NED.E);

			// Go a distance D along the line from closest node to P to find the next node position vpos
			if (alg_input.uniform2P)
				D = rg.randLin()*distance;
			else if (alg_input.gaussianD)
				D = rg.randNor(distance, alg_input.gaussianSTD);
			vpos->NED.N = (closest_node->NED.N) + sin(theta)*D;
			vpos->NED.E = (closest_node->NED.E) + cos(theta)*D;

			if (map.wps[i + 1].D > closest_node->NED.D - tan(input_file->climb_angle)*D && map.wps[i + 1].D < closest_node->NED.D + tan(input_file->descend_angle)*D)
				vpos->NED.D = map.wps[i + 1].D;
			else if (map.wps[i + 1].D > closest_node->NED.D)
				vpos->NED.D = closest_node->NED.D + tan(input_file->descend_angle)*D;
			else
				vpos->NED.D = closest_node->NED.D - tan(input_file->descend_angle)*D;

			// If this path is good move on.
			// Check to see if the straight line path is good.
			if (flyZoneCheck(closest_node->NED, vpos->NED, clearance))
			{
				found_feasible_link = true;
				vpos->parent = closest_node;
				if (alg_input.path_type == 1 && root != vpos->parent)								// If the path type is fillets, check to see if the fillet is possible.
					found_feasible_link = check_fillet(closest_node->parent->NED, closest_node->NED, vpos->NED, closest_node->available_dist, distance_in, fillet_angle);
			}
		}
		// add the new node to the tree
		closest_node->children.push_back(vpos);
		vpos->distance = sqrt(pow(closest_node->NED.N - vpos->NED.N, 2) + pow(closest_node->NED.E - vpos->NED.E, 2) + pow(closest_node->NED.D - vpos->NED.D, 2));
		if (alg_input.path_type == 1 && vpos->parent != root)
		{
			// Adjust the vpos->distance to account for the turning radius ( a little bit smaller.)
			vpos->available_dist = vpos->distance - *distance_in;
			vpos->distance = vpos->distance - 2.0*(*distance_in) + (*fillet_angle)*input_file->turn_radius;
		}
		else
			vpos->available_dist = vpos->distance;

		// Make provisions so that the algorithm doesn't hang
		added_nodes++;
		if (added_nodes == floor(iters_limit / 2.0) || added_nodes == floor(iters_limit*3.0 / 4.0) || added_nodes == floor(iters_limit*7.0 / 8.0))
		{
			cout << "DECREASING THE CLEARANCE LEVEL" << endl;
			clearance = clearance / (2.0*++clearance_drops);
			cout << "CLEARANCE:\t" << clearance << endl;
			path_clearance = (clearance < path_clearance) ? clearance : path_clearance;
		}
		if (added_nodes >= iters_limit)
		{
			cout << "WARNING -- ALGORITHM FAILED TO CONVERGE\nRESULTS WILL VIOLATE AN OBSTACLE" << endl;
			reached_next_wp = true;
			second2last = vpos;
		}

		// Check to see if it is possible to go from this newly added node to the next primary waypoint
		if (alg_input.connect_to_end && flyZoneCheck(vpos->NED, map.wps[i + 1], clearance) && check_slope(vpos->NED, map.wps[i + 1]))
		{
			reached_next_wp = true;								// Set the flag
			second2last = vpos;
			if (alg_input.path_type == 1 && root != vpos)								// If the path type is fillets, check to see if the fillet is possible.
				reached_next_wp = check_fillet(vpos->parent->NED, vpos->NED, map.wps[i + 1], vpos->available_dist, distance_in, fillet_angle);
		}								// Set the flag
		if (!alg_input.connect_to_end && sqrt(pow(map.wps[i + 1].N - vpos->NED.N, 2) + pow(map.wps[i + 1].E - vpos->NED.E, 2) + pow(map.wps[i + 1].D - vpos->NED.D, 2)) < D && flyZoneCheck(vpos->NED, map.wps[i + 1], clearance) && check_slope(vpos->NED, map.wps[i + 1]))
		{
			reached_next_wp = true;								// Set the flag
			second2last = vpos;
			if (alg_input.path_type == 1 && root != vpos)								// If the path type is fillets, check to see if the fillet is possible.
				reached_next_wp = check_fillet(vpos->parent->NED, vpos->NED, map.wps[i + 1], vpos->available_dist, distance_in, fillet_angle);
		}
		if (reached_next_wp == true && i < map.wps.size() - 2) // This if statement handles setting waypoints to get out of the primary waypoints.
		{
			if (check_create_fan(map.wps[i + 1], vpos->NED, root_ptrs[i + 1]) == false)
				reached_next_wp = false;
		}
	}
	// We can go to the next waypoint!
	// The following code wraps up the algorithm.
	clearance = input_file->clearance;									// Bump up the clearance level again.
	node *final_node = new node;
	final_node->NED = map.wps[i + 1];
	second2last->children.push_back(final_node);
	*second2last_post_smoothed = second2last->NED;
	final_node->parent = second2last;
	final_node->distance = sqrt(pow(final_node->NED.N - second2last->NED.N, 2) + pow(final_node->NED.E - second2last->NED.E, 2) + pow(final_node->NED.D - second2last->NED.D, 2));
	if (alg_input.path_type == 1 && final_node->parent != root)
		final_node->distance = final_node->distance - 2.0*(*distance_in) + (*fillet_angle)*input_file->turn_radius;
	// populate all wps by working back up the tree
	stack<node*> wpstack;
	node *current_node = final_node;
	while (current_node != root)
	{
		wpstack.push(current_node);
		current_node = current_node->parent;
	}
	wpstack.push(root);

	// Now put all of the waypoints into the all_wps vector
	vector<NED_s> wps_to_PrimaryWP;
	while (!wpstack.empty())
	{
		wps_to_PrimaryWP.push_back(wpstack.top()->NED);
		wpstack.pop();
	}
	all_wps.push_back(wps_to_PrimaryWP);
	wps_to_PrimaryWP.clear();
}
void simpleRRT::smoother(bool skip_smoother, unsigned int i, double* distance_in, double* fillet_angle, NED_s* second2last_post_smoothed, bool direct_shot)
{
	node* root = root_ptrs[i];
	//skip_smoother = true; // Uncommmenting this line turns the smoother off. It can be helpful for debugging.

	// Smooth Out the path (Algorithm 11 in the UAV book)
	// Check somewhere in this function to see if it possible to get a more smooth exit out of waypoints
	if (skip_smoother == false)
	{
		cout << "\tSmoothing Path" << endl;
		vector<NED_s> path_smoothed;
		vector<double> available_ds;
		unsigned int i_node = 0;	// i_node is which index of the SMOOTHED PATH you are on.
		unsigned int j_node = 1;	// j_node is which index of the ROUGH PATH you are on.
		double line_Distance;
		path_smoothed.push_back(all_wps[i][0]);

		if (i != 0)
		{
			path_smoothed.push_back(all_wps[i][1]);
			i_node++;
			j_node++;
			line_Distance = sqrt(pow(path_smoothed[i_node].N - path_smoothed[i_node - 1].N, 2.) + pow(path_smoothed[i_node].E - path_smoothed[i_node - 1].E, 2) + pow(path_smoothed[i_node].D - path_smoothed[i_node - 1].D, 2));
			available_ds.push_back(line_Distance - *distance_in);

			path_smoothed.push_back(all_wps[i][2]);
			i_node++;
			j_node++;
			line_Distance = sqrt(pow(path_smoothed[i_node].N - path_smoothed[i_node - 1].N, 2.) + pow(path_smoothed[i_node].E - path_smoothed[i_node - 1].E, 2) + pow(path_smoothed[i_node].D - path_smoothed[i_node - 1].D, 2));
			available_ds.push_back(line_Distance - *distance_in);
		}
		while (j_node < all_wps[i].size())
		{
			bool bad_path_flag = false;
			if ((alg_input.path_type == 0 || i_node == 0) && all_wps[i].size() >= j_node + 2 && (flyZoneCheck(path_smoothed[i_node], all_wps[i][j_node + 1], path_clearance) == false || check_max_slope(path_smoothed[i_node], all_wps[i][j_node + 1]) == false))
				bad_path_flag = true;
			else if (alg_input.path_type == 1 && all_wps[i].size() >= j_node + 2 && (flyZoneCheck(path_smoothed[i_node], all_wps[i][j_node + 1], path_clearance) == false || check_max_slope(path_smoothed[i_node], all_wps[i][j_node + 1]) == false))
				bad_path_flag = true;
			if (alg_input.path_type == 1 && all_wps[i].size() >= j_node + 2 && i_node >= 1 && check_fillet(path_smoothed[i_node - 1], path_smoothed[i_node], all_wps[i][j_node + 1], available_ds[i_node - 1], distance_in, fillet_angle) == false)
				bad_path_flag = true;
			if (i != 0 && alg_input.path_type == 1 && j_node == all_wps[i].size() - 3 && bad_path_flag == false)
			{
				double temp_available_ds = sqrt(pow(path_smoothed[i_node].N - all_wps[i][j_node + 2].N, 2) + pow(path_smoothed[i_node].E - all_wps[i][j_node + 2].E, 2) + pow(path_smoothed[i_node].D - all_wps[i][j_node + 2].D, 2)) - *distance_in;
				if (check_fillet(path_smoothed[i_node], all_wps[i][j_node + 1], all_wps[i][j_node + 2], temp_available_ds, distance_in, fillet_angle) == false)
				{
					bad_path_flag = true;
					// If this is true then really you should check the next one back.
					// Now we are working backwards to find a good path.
					while (bad_path_flag)
					{
						j_node--;
						temp_available_ds = sqrt(pow(path_smoothed[i_node].N - all_wps[i][j_node + 1].N, 2) + pow(path_smoothed[i_node].E - all_wps[i][j_node + 1].E, 2) + pow(path_smoothed[i_node].D - all_wps[i][j_node + 1].D, 2)) - *distance_in;
						if (flyZoneCheck(path_smoothed[i_node], all_wps[i][j_node + 1], path_clearance))
							if (check_fillet(path_smoothed[i_node - 1], path_smoothed[i_node], all_wps[i][j_node + 1], temp_available_ds, distance_in, fillet_angle))
								bad_path_flag = false;
					}
					bad_path_flag = true;
				}
			}
			if (bad_path_flag) // Try adding the second to last node every time. Avoids problems with smoothing out the last fillet.
			{
				path_smoothed.push_back(all_wps[i][j_node]);
				i_node++;
				line_Distance = sqrt(pow(path_smoothed[i_node].N - path_smoothed[i_node - 1].N, 2.) + pow(path_smoothed[i_node].E - path_smoothed[i_node - 1].E, 2) + pow(path_smoothed[i_node].D - path_smoothed[i_node - 1].D, 2));
				available_ds.push_back(line_Distance - *distance_in);
			}
			j_node++;
		}
		// Check to see if the fan can be created better
		// If so, change path_smoothed[1] and delete path_smoothed[2]
		NED_s cea;
		if (i > 0 && i <= map.wps.size() - 1 && direct_shot == false && path_smoothed.size() >= 4 && check_direct_fan(path_smoothed[3], root->NED, all_wps[i - 1][all_wps[i - 1].size() - 1], root, &cea, distance_in, fillet_angle))
		{
			if (path_smoothed.size() >= 4 && flyZoneCheck(cea, path_smoothed[3], path_clearance))
			{
				path_smoothed[1] = root->children[root->children.size() - 1]->NED;
				path_smoothed.erase(path_smoothed.begin() + 2);
				cout << "SMOOTHED THE FAN" << endl;
			}
		}

		all_wps[i].swap(path_smoothed);
		*second2last_post_smoothed = all_wps[i][all_wps[i].size() - 1];
	}
	else
		all_wps[i].erase(all_wps[i].begin() + all_wps[i].size() - 1);
}
bool simpleRRT::check_slope(NED_s beg, NED_s en)
{
	double slope = atan2(-1.0*(en.D - beg.D), sqrt(pow(beg.N - en.N, 2) + pow(beg.E - en.E, 2)));
	if (slope < -1.0*input_file->descend_angle || slope > input_file->climb_angle)
		return false;
	return true;
}
bool simpleRRT::check_max_slope(NED_s beg, NED_s en)
{
	double slope = atan2(-1.0*(en.D - beg.D), sqrt(pow(beg.N - en.N, 2) + pow(beg.E - en.E, 2)));
	if (slope < -1.0*input_file->max_descend_angle || slope > input_file->max_climb_angle)
		return false;
	return true;
}
void simpleRRT::calc_path_distance(unsigned int i)
{
	// Determine the true path distance. (This kind of covers up some mistakes above...)
	double final_distance = 0;
	for (unsigned j = 0; j < all_wps[i].size() - 1; j++)
	{
		final_distance += sqrt(pow(all_wps[i][j].N - all_wps[i][j + 1].N, 2) + pow(all_wps[i][j].E - all_wps[i][j + 1].E, 2) + pow(all_wps[i][j].D - all_wps[i][j + 1].D, 2));
	}
	final_distance += sqrt(pow(all_wps[i][all_wps[i].size() - 1].N - map.wps[i + 1].N, 2) + pow(all_wps[i][all_wps[i].size() - 1].E - map.wps[i + 1].E, 2) + pow(all_wps[i][all_wps[i].size() - 1].D - map.wps[i + 1].D, 2));
	path_distances.push_back(final_distance);
	cout << "PATH DISTANCE: " << path_distances[i] << endl;
}
void simpleRRT::delete_tree()
{
	for (unsigned int i = 0; i < root_ptrs.size(); i++)		// Delete every tree generated
		delete_node(root_ptrs[i]);
}
void simpleRRT::delete_node(node* pn)						// Recursively delete every node
{
	for (unsigned int i = 0; i < pn->children.size();i++)
		delete_node(pn->children[i]);
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
	path_file.close();

	// Print A special file just for MATLAB plotting functions
	string spfilename = input_file->special_pfile;
	ofstream spath_file;
	spath_file.open(spfilename.c_str());
	spath_file << input_file->turn_radius << endl;
	spath_file.close();
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