//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//



#ifndef __HYBRID_AGENT__
#define __HYBRID_AGENT__

/// @file HybridAgent.h
/// @brief Declares the HybridAgent class.

#include <queue>
#include "SteerLib.h"
// #include "HybridAgent.h"
#include "HybridAIModule.h"

/**
 * @brief An example agent with very basic AI, that is part of the HybridAI plugin.
 *
 * This agent performs extremely Hybrid AI using forces and Euler integration, simply
 * steering towards static goals without avoiding any other agents or static obstacles.
 * Agents that are "selected" in the GUI will have some simple annotations that
 * show how the spatial database and engine interface can be used.
 *
 * This class is instantiated when the engine calls HybridAIModule::createAgent().
 *
 */
class HybridAgent : public SteerLib::AgentInterface
{
public:
	HybridAgent();
	~HybridAgent();
	void reset(const SteerLib::AgentInitialConditions & initialConditions, SteerLib::EngineInterface * engineInfo);
	void updateAI(float timeStamp, float dt, unsigned int frameNumber);
	void disable();
	void draw();

	bool enabled() const { return _enabled; }
	Util::Point position() const { return _position; }
	Util::Vector forward() const { return _forward; }
	Util::Vector velocity() const { return this->_velocity; }
	float radius() const { return _radius; }
	const SteerLib::AgentGoalInfo & currentGoal() const { return _goalQueue.front(); }
	size_t id() const { return _id;}
	const std::queue<SteerLib::AgentGoalInfo> & agentGoals() const { return _goalQueue; }
	void addGoal(const SteerLib::AgentGoalInfo & newGoal) { throw Util::GenericException("addGoals() not implemented yet for HybridAgent"); }
	void clearGoals() { throw Util::GenericException("clearGoals() not implemented yet for HybridAgent"); }

	void insertAgentNeighbor(const SteerLib::AgentInterface *agent, float &rangeSq) { throw Util::GenericException("insertAgentNeighbor not implemented yet for BenchmarkAgent"); }
	void setParameters(SteerLib::Behaviour behave)
	{
		throw Util::GenericException("setParameters() not implemented yet for this Agent");
	}

	/// @name The SteerLib::SpatialDatabaseItemInterface
	/// @brief These functions are required so that the agent can be used by the SteerLib::GridDatabase2D spatial database;
	/// The Util namespace helper functions do the job nicely for basic circular agents.
	//@{
	bool intersects(const Util::Ray &r, float &t) { return Util::rayIntersectsCircle2D(_position, _radius, r, t); }
	bool overlaps(const Util::Point & p, float radius) { return Util::circleOverlapsCircle2D( _position, _radius, p, radius); }
	float computePenetration(const Util::Point & p, float radius) { return Util::computeCircleCirclePenetration2D( _position, _radius, p, radius); }
	//@}


protected:
	/// Updates position, velocity, and orientation of the agent, given the force and dt time step.
	void _doEulerStep(const Util::Vector & steeringDecisionForce, float dt);
	void _switchSteeringMethod( SteerLib::AgentInitialConditions ai);

	virtual SteerLib::EngineInterface * getSimulationEngine();

	SteerLib::AgentInterface * _agentInterface;

	unsigned int _switchLength;
	enum AgentStates
	{
		PPR,
		ORCA,
		FOOTSTEP,
		CC
	};

	unsigned int _agentState;

private:
	// SteerLib::AgentInitialConditions getAgentConditions(SteerLib::AgentInterface * agentInterface);
};

#endif
