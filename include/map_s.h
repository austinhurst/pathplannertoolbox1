/*	DESCRIPTION:
 *	This is a header that defines a struct that contains all of the static
 *	information about the competition environment. It contains the boundaries
 *	and the static obstacles (cylinders).
 *
 */
#ifndef MAP_H
#define MAP_H
#include <vector>

using namespace std;
struct NED_s
{
	double N;						// North (m)
	double E;						// East  (m)
	double D;						// Down  (m), Remember up is negative!
};
struct cyl_s
{
	double N;						// North  (m)
	double E;						// East   (m)
	double R;						// Radius (m)
	double H;						// Height (m)
};
struct map_s
{
	vector<NED_s> boundary_pts;		// Contains all boundary points, no repeats
	vector<cyl_s> cylinders;		// Contains all cylinders
};

#endif