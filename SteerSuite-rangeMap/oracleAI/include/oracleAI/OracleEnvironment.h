//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//



#ifndef __PHASE_DECIMATION_SPACETIME_ASTAR_H__
#define __PHASE_DECIMATION_SPACETIME_ASTAR_H__

#include "astar/Environment.h"
#include "SteerLib.h"
#include "oracleAI/OracleAgent.h"


class OracleEnvironment : public Environment
{
public:
	OracleEnvironment( OracleAgent * newAgent );
	~OracleEnvironment(void);
	float getHeuristic(int start, int target) const;
	void getSuccessors(int nodeId, int lastNodeId, vector<Successor> & result) const;
	bool isValidNodeId(int nodeId) const;

protected:
	inline bool _addFootstepIfValid(float stepDuration, float parabolaOrientationPhi, bool phiIsIdeal, float desiredVelocity, const Footstep & previousStep, FootStateEnum nextState, vector<Successor> & result) const;
	OracleAgent * agent;

public:
	mutable unsigned int _numNodesExpanded;
	mutable unsigned int _totalNodesExpanded;
};


#endif
