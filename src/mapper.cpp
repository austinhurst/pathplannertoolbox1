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
	double m, b;
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
		cyl.H = rg.randLin()*(maxCylHeight - minCylHeight) + minCylHeight;
		map.cylinders.push_back(cyl);
	}
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
		boundaries_out_file << map.boundary_pts[i].N << "\t" << map.boundary_pts[i].E << "\n";
	boundaries_out_file.close();
}
void mapper::fprintf_cylinders()				// Prints the cylinders that were developed
{
	ofstream cylinders_out_file;
	cylinders_out_file.open("./graphing_files/output_cylinders.txt");
	for (unsigned int i = 0; i < map.cylinders.size(); i++)
		cylinders_out_file << map.cylinders[i].N << "\t" << map.cylinders[i].E << "\t" << map.cylinders[i].R << "\t" << map.cylinders[i].H << "\n";
	cylinders_out_file.close();
}
bool mapper::flyZoneCheck(const NED_s NED)			// Returns true if it is within boundaries and not on an obstacle, returns false otherwise
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
		if (NED.E >= lineMinMax[i][2] && NED.E < lineMinMax[i][3]) // Only one equal sign solves both the above/ below a vertice problem and the vertical line problem
		{
			if (NED.N > line_Mandb[i][0] * NED.E + line_Mandb[i][1])
				crossed_lines++;
			else if (NED.N == line_Mandb[i][0] * NED.E + line_Mandb[i][1])	// On the rare chance that the point is ON the line
				return false;
		}
	}
	withinBoundaries = crossed_lines % 2; // If it crosses an even number of boundaries it is NOT inside, if it crosses an odd number it IS inside

	// Second, Check for Cylinders
	// Check if the point falls into the volume of the cylinder
	bool avoidsCylinders = true;
	for (unsigned int i = 0; i < map.cylinders.size(); i++)
		if (sqrt(pow(NED.N- map.cylinders[i].N, 2) + pow(NED.E - map.cylinders[i].E, 2)) < map.cylinders[i].R && -NED.D < map.cylinders[i].H)
			return false;
	return (withinBoundaries && avoidsCylinders);
}
bool mapper::flyZoneCheck(const cyl_s cyl)			// Returns true if it is within boundaries and not on an obstacle, returns false otherwise
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
		if (cyl.E >= lineMinMax[i][2] && cyl.E < lineMinMax[i][3]) // Only one equal sign solves both the above/ below a vertice problem and the vertical line problem
		{
			if (cyl.N > line_Mandb[i][0] * cyl.E + line_Mandb[i][1])
				crossed_lines++;
			else if (cyl.N == line_Mandb[i][0] * cyl.E + line_Mandb[i][1])	// On the rare chance that the point is ON the line
				return false;
		}
	}
	withinBoundaries = crossed_lines % 2; // If it crosses an even number of boundaries it is NOT inside, if it crosses an odd number it IS inside


	// Alright guys, this is kind of a hack:
	// The next little bit figures out if the cylinder outer radius extends outside of a border.
	// It isn't perfect... it actually seems like kind of a tough problem that doesn't warrant a ton of time
	// The quick fix to this is to check only 4 (or as many as you want...) points on the cylinders outer surface, north, south, west, east
	// The hard way would be to calculate the nearest point to the line. Which isn't that bad except for there 
	// are difficulties next to sharp points in the boundaries.

	NED_s ned_cyl_edge;
	ned_cyl_edge.D = 0;
	double theta;
	int nCheckPoints = 4;	// Increasing this number increases the number of calculations...
	for (int i = 0; i < nCheckPoints; i++)
	{
		theta = 2*3.141592653589793*i / nCheckPoints;
		ned_cyl_edge.N = cyl.N + cyl.R*cos(theta);
		ned_cyl_edge.E = cyl.E + cyl.R*sin(theta);
		if (flyZoneCheck(ned_cyl_edge) == false)
			return false;
	}
	// Second, Check for Cylinders
	// Check if the point falls into the volume of the cylinder
	bool avoidsCylinders = true;
	for (unsigned int i = 0; i < map.cylinders.size(); i++)
		if (sqrt(pow(cyl.N - map.cylinders[i].N, 2) + pow(cyl.E - map.cylinders[i].E, 2)) < map.cylinders[i].R + cyl.R)
			return false;
	return (withinBoundaries && avoidsCylinders);
}