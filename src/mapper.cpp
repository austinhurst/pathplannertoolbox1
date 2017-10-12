/*	DESCRIPTION:
 *	This is the cpp for the mapper class.
 *	The mapper is seeded with an unsigned int. Every time the mapper is seeded with the
 *	same seed it will produce the same exact map! (This may not be true if using different
 *	compilers etc).
 *	The functions (will) include loading in boundaries, randomly generating 
 *	static obstacles (cylinders), generating .world files, and possibly
 *	generating moving obstacles.
 */

#include "./../include/mapper.h"

using namespace std;
mapper::mapper(unsigned int seed)
{
	// Create the random generator
	randGen rg_in(seed);
	rg = rg_in;

	// Some settings for Generating the Map
	waypoint_clearance = 15; // (m) This is the minimum clearance that each waypoint has with other obstacles... Just to make things reasonable.
	is3D = false;			 // This make the board 3D, which pretty much just means the cylinders have a specific height and waypoints can be above them.

	// Set up some competition constants
	minCylRadius = 9.144; // 9.144 m = 30 ft.
	maxCylRadius = 91.44; // 91.44 m = 300 ft.
	minCylHeight = 9.144; // 9.144 m = 30 ft.
	maxCylHeight = 228.6; // 228.6 m = 750 ft.
	minFlyHeight = 30.48; // 30.48 m = 100 ft. // This still needs to add in the take off altitude
	maxFlyHeight = 228.6; // 228.6 m = 750 ft. // This still needs to add in the take off altitude

	// Pull in the competition boundaries
	boundaries_in_file.open("./input_files/competition_boundaries.txt");
	if (!boundaries_in_file)
		cerr << "Could not open the boundaries file." << endl;
	NED_s boundary_point;
	bool setFirstValues = true;
	while (boundaries_in_file.eof() == false)
	{
		boundaries_in_file >> boundary_point.N >> boundary_point.E;
		if (setFirstValues == false)
		{
			maxNorth = (boundary_point.N > maxNorth) ? boundary_point.N : maxNorth; // if new N is greater than maxN, set maxN = new N
			minNorth = (boundary_point.N < minNorth) ? boundary_point.N : minNorth;
			maxEast  = (boundary_point.E > maxEast ) ? boundary_point.E : maxEast ;
			minEast  = (boundary_point.E < minEast ) ? boundary_point.E : minEast ;
		}
		else
		{
			maxNorth = boundary_point.N;
			minNorth = boundary_point.N;
			maxEast  = boundary_point.E;
			minEast  = boundary_point.E;
			setFirstValues = false;
		}

		map.boundary_pts.push_back(boundary_point);	// This line puts the boundary points into the map member.
	}
	boundaries_in_file.close();

	// Set up flyZoneCheck()
	//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv These lines are used to prep the flyZoneCheck() algorithm
	vector<double> NminNmaxEminEmax;				// Yeah, this is a riduculous name...
	vector<double> mb;								// Vector of slope and intercepts
	nBPts = map.boundary_pts.size();				// Number of Boundary Points
	double m, b, w, m_w;
	for (unsigned int i = 0; i < nBPts; i++)		// Loop through all points
	{
		// Find the min and max of North and East coordinates on the line connecting two points.
		NminNmaxEminEmax.push_back(min(map.boundary_pts[i].N, map.boundary_pts[(i + 1) % nBPts].N));
		NminNmaxEminEmax.push_back(max(map.boundary_pts[i].N, map.boundary_pts[(i + 1) % nBPts].N));
		NminNmaxEminEmax.push_back(min(map.boundary_pts[i].E, map.boundary_pts[(i + 1) % nBPts].E));
		NminNmaxEminEmax.push_back(max(map.boundary_pts[i].E, map.boundary_pts[(i + 1) % nBPts].E));
		lineMinMax.push_back(NminNmaxEminEmax);
		NminNmaxEminEmax.clear();
		// Find the slope and intercept
		m = (map.boundary_pts[(i + 1) % nBPts].N - map.boundary_pts[i].N) / (map.boundary_pts[(i + 1) % nBPts].E - map.boundary_pts[i].E);
		b = -m*map.boundary_pts[i].E + map.boundary_pts[i].N;
		w = (-1.0 / m);
		m_w = m - w;
		mb.push_back(m);
		mb.push_back(b);
		mb.push_back(w);
		mb.push_back(m_w);
		line_Mandb.push_back(mb);
		mb.clear();
	}
	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ These lines are used to set up the flyZoneCheck() algorithm.

	// Randomly Generate 10 Cylinders
	// radius between 30 ft and 300 ft, height between 30 ft and 750 ft
	// can be up to 10 cylinders
	nCyli = 10;
	cyl_s cyl;
	for (unsigned int i = 0; i < nCyli; i++)
	{
		cyl.N = rg.randLin()*(maxNorth - minNorth) + minNorth;
		cyl.E = rg.randLin()*(maxEast  - minEast ) + minEast ;
		cyl.R = rg.randLin()*(maxCylRadius - minCylRadius) + minCylRadius;
		while (flyZoneCheck(cyl) == false)
		{
			cyl.N = rg.randLin()*(maxNorth - minNorth) + minNorth;
			cyl.E = rg.randLin()*(maxEast - minEast) + minEast;
			cyl.R = rg.randLin()*(maxCylRadius - minCylRadius) + minCylRadius;
		}
		if (is3D)
			cyl.H = rg.randLin()*(maxCylHeight - minCylHeight) + minCylHeight;
		else
			cyl.H = maxFlyHeight;
		map.cylinders.push_back(cyl);
	}


	int numWps = 3;
	NED_s wp;
	// Randomly generate some waypoints
	for (int i = 0; i < numWps; i++)
	{
		wp.N = rg.randLin()*(maxNorth - minNorth) + minNorth;
		wp.E = rg.randLin()*(maxEast - minEast) + minEast;
		wp.D = (rg.randLin()*(maxFlyHeight - minFlyHeight) + minFlyHeight)*-1.0; // Put the MSL into down (make it negative)
		while (flyZoneCheck(wp,waypoint_clearance) == false)
		{
			wp.N = rg.randLin()*(maxNorth - minNorth) + minNorth;
			wp.E = rg.randLin()*(maxEast - minEast) + minEast;
			wp.D = rg.randLin()*(maxFlyHeight - minFlyHeight) + maxFlyHeight;
		}
		map.wps.push_back(wp);
	}
}
mapper::~mapper()
{
}
void mapper::fprint_map()
{
	fprint_boundaries();
	fprint_cylinders();
	fprint_primaryWPS();
}
void mapper::fprint_boundaries()				// Prints the boundaries of the map
{
	ofstream boundaries_out_file;
	boundaries_out_file.open("./graphing_files/output_boundaries.txt");
	for (unsigned int i = 0; i < map.boundary_pts.size(); i++)
		boundaries_out_file << map.boundary_pts[i].N << "\t" << map.boundary_pts[i].E << "\n";
	boundaries_out_file.close();
}
void mapper::fprint_cylinders()				// Prints the cylinders that were developed
{
	ofstream cylinders_out_file;
	cylinders_out_file.open("./graphing_files/output_cylinders.txt");
	for (unsigned int i = 0; i < map.cylinders.size(); i++)
		cylinders_out_file << map.cylinders[i].N << "\t" << map.cylinders[i].E << "\t" << map.cylinders[i].R << "\t" << map.cylinders[i].H << "\n";
	cylinders_out_file.close();
}
void mapper::fprint_primaryWPS()
{
	ofstream primary_wps_out_file;
	primary_wps_out_file.open("./graphing_files/output_primary_wps.txt");
	for (unsigned int i = 0; i < map.wps.size(); i++)
		primary_wps_out_file << map.wps[i].N << "\t" << map.wps[i].E << "\t" << map.wps[i].D << "\n";
	primary_wps_out_file.close();
}
bool mapper::flyZoneCheck(const cyl_s cyl)			// Returns true if it is within boundaries and not on an obstacle, returns false otherwise
{
	NED_s NED;
	NED.N = cyl.N;
	NED.E = cyl.E;
	NED.D = 0;
	double radius = cyl.R;
	return  flyZoneCheckMASTER(NED, radius);
}
bool mapper::flyZoneCheck(const NED_s NED, const double radius)			// Returns true if it is within boundaries and not on an obstacle, returns false otherwise
{
	return flyZoneCheckMASTER(NED, radius);
}

bool mapper::flyZoneCheckMASTER(const NED_s NED, const double radius)
{
	// First, Check Within the Boundaries
	bool withinBoundaries;
	// Look at the Point in Polygon Algorithm
	// Focus on rays South.
	vector<bool> lineConcerns;
	int crossed_lines = 0;							// This is a counter of the number of lines that the point is NORTH of.
	double bt, Ei, Ni, de1, de2, shortest_distance;
	for (unsigned int i = 0; i < nBPts; i++)
	{
		// Find out if the line is either North or South of the line
		if (NED.E >= lineMinMax[i][2] && NED.E < lineMinMax[i][3]) // Only one equal sign solves both the above/ below a vertice problem and the vertical line problem
		{
			if (NED.N > line_Mandb[i][0] * NED.E + line_Mandb[i][1])
				crossed_lines++;
			else if (NED.N == line_Mandb[i][0] * NED.E + line_Mandb[i][1])	// On the rare chance that the point is ON the line
				return false;
		}
		// Check to see if it is too close to the boundary lines
		if (NED.E >= lineMinMax[i][2] - radius && NED.E < lineMinMax[i][3] + radius && NED.N >= lineMinMax[i][0] - radius && NED.N < lineMinMax[i][1] + radius)
		{
			bt = NED.N - line_Mandb[i][2] * NED.E;
			Ei = (bt - line_Mandb[i][1]) / line_Mandb[i][3];
			Ni = line_Mandb[i][2] * Ei + bt;
			// 3 cases first point, second point, or on the line.
			// If the intersection is on the line, dl is the shortest distance
			// Otherwise it is one of the endpoints.
			if (Ni > lineMinMax[i][0] && Ni < lineMinMax[i][1] && Ei > lineMinMax[i][2] && Ei < lineMinMax[i][3])
				shortest_distance = sqrt(pow(Ni - NED.N, 2) + pow(Ei - NED.E, 2));
			else
			{
				de1 = sqrt(pow(map.boundary_pts[i].N - NED.N, 2) + pow(map.boundary_pts[i].E - NED.E, 2));
				de2 = sqrt(pow(map.boundary_pts[(i + 1) % nBPts].N - NED.N, 2) + pow(map.boundary_pts[(i + 1) % nBPts].E - NED.E, 2));
				shortest_distance = min(de1, de2);
			}
			if (shortest_distance < radius)
				return false;
		}
	}
	withinBoundaries = crossed_lines % 2; // If it crosses an even number of boundaries it is NOT inside, if it crosses an odd number it IS inside
	if (withinBoundaries == false)
		return false;

	// Second, Check for Cylinders
	// Check if the point falls into the volume of the cylinder
	bool avoidsCylinders = true;
	for (unsigned int i = 0; i < map.cylinders.size(); i++)
		if (sqrt(pow(NED.N - map.cylinders[i].N, 2) + pow(NED.E - map.cylinders[i].E, 2)) < map.cylinders[i].R + radius && -NED.D - radius < map.cylinders[i].H)
			return false;
	return true; // The coordinate is in the safe zone if it got to here!
}