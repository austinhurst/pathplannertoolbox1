#pragma once
#include <random>

using namespace std;
class randGen
{
public:
	randGen(unsigned int seed_in);	// Use this contructor - give it a seed
	randGen();						// Default contructor - NO SEED. Needed to allow randGen to be a member of a class
	~randGen();						// Deconstructor
	double randLin();				// This public function returns a random number from 0 to 1.
private:
	unsigned int seed;				// Stores the seed - might be unnecessary.
	mt19937 mersenneTwister;		// Here is the psuedo random generator object - The mersenne twister.
};