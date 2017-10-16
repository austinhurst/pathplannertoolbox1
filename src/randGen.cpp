/*	DESCRIPTION:
 *	This is the cpp for the randGen class. It generates random numbers
 *	by the mersenne twister - make sure to seed the generator.
 *
 */
#include "./../include/randGen.h"

using namespace std;
randGen::randGen(unsigned int seed_in)	// Seed the Random Generator for reproducibility
{
	seed = seed_in;
	mt19937 mt(seed_in);				// Create and seed an mt19937 object in the constructor
	mersenneTwister = mt;				// Copy the created object into the private member of the class
}
randGen::randGen()						// This empty function is needed so that randGen can be a member of a class.
{
}
randGen::~randGen()						// Deconstructor
{
}
double randGen::randLin()				// This public function returns a random number from 0 to 1, uniform distribution
{
	uniform_real_distribution<double> UniDis(0.0, 1.0);
	return UniDis(mersenneTwister);
}