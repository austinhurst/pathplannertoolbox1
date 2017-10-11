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

	// Set up some competition constants
	minCylRadius = 9.144; // 9.144 m = 30 ft.
	maxCylRadius = 91.44; // 91.44 m = 300 ft.
	minCylHeight = 9.144; // 9.144 m = 30 ft.
	maxCylHeight = 228.6; // 228.6 m = 750 ft.

	// Pull in the competition boundaries
	boundaries_in_file.open("./input_files/competition_boundaries.txt");
	if (!boundaries_in_file)
		cerr << "Could not open the boundaries file." << std::endl;
	vector<double> boundary_point;
	boundary_point.push_back(0);
	boundary_point.push_back(0);
	bool setFirstValues = true;
	while (boundaries_in_file.eof() == false)
	{
		boundaries_in_file >> boundary_point[0] >> boundary_point[1];
		if (setFirstValues == false)
		{
			maxNorth = (boundary_point[0] > maxNorth) ? boundary_point[0] : maxNorth; // if new N is greater than maxN, set maxN = new N
			minNorth = (boundary_point[0] < minNorth) ? boundary_point[0] : minNorth;
			maxEast  = (boundary_point[1] > maxEast ) ? boundary_point[1] : maxEast ;
			minEast  = (boundary_point[1] < minEast ) ? boundary_point[1] : minEast ;
		}
		else
		{
			maxNorth = boundary_point[0];
			minNorth = boundary_point[0];
			maxEast  = boundary_point[1];
			minEast  = boundary_point[1];
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
	double m, b;
	for (unsigned int i = 0; i < nBPts; i++)		// Loop through all points
	{
		// Find the min and max of North and East coordinates on the line connecting two points.
		NminNmaxEminEmax.push_back(min(map.boundary_pts[i][0], map.boundary_pts[(i + 1) % nBPts][0]));
		NminNmaxEminEmax.push_back(max(map.boundary_pts[i][0], map.boundary_pts[(i + 1) % nBPts][0]));
		NminNmaxEminEmax.push_back(min(map.boundary_pts[i][1], map.boundary_pts[(i + 1) % nBPts][1]));
		NminNmaxEminEmax.push_back(max(map.boundary_pts[i][1], map.boundary_pts[(i + 1) % nBPts][1]));
		lineMinMax.push_back(NminNmaxEminEmax);
		NminNmaxEminEmax.clear();
		// Find the slope and intercept
		m = (map.boundary_pts[(i + 1) % nBPts][0] - map.boundary_pts[i][0]) / (map.boundary_pts[(i + 1) % nBPts][1] - map.boundary_pts[i][1]);
		b = -m*map.boundary_pts[i][1] + map.boundary_pts[i][0];
		mb.push_back(m);
		mb.push_back(b);
		line_Mandb.push_back(mb);
		mb.clear();
	}
	//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ These lines are used to set up the flyZoneCheck() algorithm.

	// Randomly Generate 10 Cylinders
	// radius between 30 ft and 300 ft, height between 30 ft and 750 ft
	// can be up to 10 cylinders
	nCyli = 10;
	vector<double> cyl;
	double N, E, r, h; // North, East, Radius, Height
	double NED[3];
	NED[2] = 0.0;
	for (unsigned int i = 0; i < nCyli; i++)
	{
		N = rg.randLin()*(maxNorth - minNorth) + minNorth;
		E = rg.randLin()*(maxEast  - minEast ) + minEast ;
		NED[0] = N;
		NED[1] = E;
		while (flyZoneCheck(NED) == false)
		{
			N = rg.randLin()*(maxNorth - minNorth) + minNorth;
			E = rg.randLin()*(maxEast - minEast) + minEast;
			NED[0] = N;
			NED[1] = E;
		}
		r = rg.randLin()*(maxCylRadius - minCylRadius) + minCylRadius;
		h = rg.randLin()*(maxCylHeight - minCylHeight) + minCylHeight;
		cyl.push_back(N);
		cyl.push_back(E);
		cyl.push_back(r);
		cyl.push_back(h);
		map.cylinders.push_back(cyl);
		cyl.clear();
	}
	NED[0] = 0.0;
}
mapper::~mapper()
{
}
void mapper::fprint_map()
{
	fprint_boundaries();
	fprintf_cylinders();
}
void mapper::fprint_boundaries()				// Prints the boundaries of the map
{
	ofstream boundaries_out_file;
	boundaries_out_file.open("./graphing_files/output_boundaries.txt");
	for (unsigned int i = 0; i < map.boundary_pts.size(); i++)
		boundaries_out_file << map.boundary_pts[i][0] << "\t" << map.boundary_pts[i][1] << "\n";
	boundaries_out_file.close();
}
void mapper::fprintf_cylinders()				// Prints the cylinders that were developed
{
	ofstream cylinders_out_file;
	cylinders_out_file.open("./graphing_files/output_cylinders.txt");
	for (unsigned int i = 0; i < map.cylinders.size(); i++)
		cylinders_out_file << map.cylinders[i][0] << "\t" << map.cylinders[i][1] << "\t" << map.cylinders[i][2] << "\t" << map.cylinders[i][3] << "\n";
	cylinders_out_file.close();
}
bool mapper::flyZoneCheck(const double NED[])		// Returns true if it is within boundaries and not on an obstacle, returns false otherwise
{
	// First, Check Within the Boundaries
	bool withinBoundaries;
	// Look at the Point in Polygon Algorithm
	// Focus on rays North.
	vector<bool> lineConcerns;
	int crossed_lines = 0;							// This is a counter of the number of lines that the point is NORTH of.
	for (unsigned int i = 0; i < nBPts; i++)
	{
		// Find out if the line is either North or South of the line
		if (NED[1] >= lineMinMax[i][2] && NED[1] < lineMinMax[i][3]) // Only one equal sign solves both the above/ below a vertice problem and the vertical line problem
		{
			if (NED[0] > line_Mandb[i][0] * NED[1] + line_Mandb[i][1])
				crossed_lines++;
			else if (NED[0] == line_Mandb[i][0] * NED[1] + line_Mandb[i][1])	// On the rare chance that the point is ON the line
				return false;
		}
	}
	withinBoundaries = crossed_lines % 2; // If it crosses an even number of boundaries it is NOT inside, if it crosses an odd number it IS inside

	// Second, Check for Cylinders
	// Check if the point falls into the volume of the cylinder
	bool avoidsCylinders = true;
	for (unsigned int i = 0; i < map.cylinders.size(); i++)
		if (sqrt(pow(NED[0] - map.cylinders[i][0], 2) + pow(NED[0] - map.cylinders[i][0], 2)) < map.cylinders[i][2] && NED[2] < map.cylinders[i][3])
			return false;
	return (withinBoundaries && avoidsCylinders);
}