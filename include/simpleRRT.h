#ifndef SIMPLERRT_H
#define SIMPLERRT_H



#include "pathPlanner.h"
#include "./../include/map_s.h"
#include "./../include/randGen.h"



class simpleRRT :
	public pathPlanner
{
public:
	simpleRRT();
	~simpleRRT();
	virtual void solve_static(map_s map);
};

#endif