#pragma once
#include <mrpt/base.h>
#include "SharedResource.h"
#include <mrpt/synch/CCriticalSection.h>
#include <mrpt/system/threads.h>


class PathDirector
{
private:
	enum StateType {ExecutingAntiClockwise90Deg,ExecutingClockwise90Deg,ExecutingAntiClockwise180Deg,ExecutingForward,ReadyForPathPlanning};

	mrpt::slam::COccupancyGridMap2DPtr GridMap;
	int xmax;
	int ymax;
	int ** map;
	int InitializationValue;
	int ** DTarray;
	double AbsolutePosition[3];
	StateType State;
	char Direction;
	double RefrenceForStepping[3];

	void initialize(int xg,int yg);
	void forwardscan(int ** cspace,int xg,int yg);
	void reversescan(int ** cspace,int xg,int yg);
	char DTPP(int xstart,int ystart,int xgoal,int ygoal);
	void InitializeInterfaces();
public:
	PathDirector();
	~PathDirector(void);

	void UpdateDirection(SharedResource*Resource);
};

