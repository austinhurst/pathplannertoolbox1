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
double randGen::randNor(double mu, double std)				// This public function returns a random number from the unit normal distribution
{
	normal_distribution<> gaussian(mu, std);
	return 	gaussian(mersenneTwister);
}
vector<unsigned int> randGen::UINTv(unsigned int len)
{
	vector<unsigned int> uints;			// This function returns a vector of unsigned ints
	for (unsigned int i = 0; i < len; i++)
	{
		uints.push_back(mersenneTwister());
	}
	return uints;
}