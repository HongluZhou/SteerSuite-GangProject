//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//
// Parts extracted from SocialForcesAgent.h located in socialForcesAI folder
//

#ifndef __PredictiveAvoidance_AGENT__
#define __PredictiveAvoidance_AGENT__

/// @file PredictiveAvoidanceAgent.h
/// @brief Declares the SimpleAgent class.

#include <queue>
#include <list>
#include "SteerLib.h"

#include "PredictiveAvoidance_Parameters.h"


/**
 * @brief Predictive  Agent stuff
 *
 *
 */


class PredictiveAvoidanceAgent : public SteerLib::AgentInterface
{
public:
	PredictiveAvoidanceAgent();
	~PredictiveAvoidanceAgent();
	void reset(const SteerLib::AgentInitialConditions & initialConditions, SteerLib::EngineInterface * engineInfo);
	void updateAI(float timeStamp, float dt, unsigned int frameNumber);
	void disable();
	void draw();

	bool enabled() const { return _enabled; }
	Util::Point position() const { return _position; }
	Util::Vector forward() const { return _forward; }
	Util::Vector velocity() const {return _velocity; }
	float radius() const { return _radius; }
	const SteerLib::AgentGoalInfo & currentGoal() const { return _goalQueue.front(); }
	size_t id() const { return id_;}
	const std::queue<SteerLib::AgentGoalInfo> & agentGoals() const { return _goalQueue; }
	void addGoal(const SteerLib::AgentGoalInfo & newGoal) { throw Util::GenericException("addGoals() not implemented yet for PredictiveAvoidanceAgent"); }
	void clearGoals() { throw Util::GenericException("clearGoals() not implemented yet for PredictiveAvoidanceAgent"); }
	void setParameters(SteerLib::Behaviour behave);
	/// @name The SteerLib::SpatialDatabaseItemInterface
	/// @brief These functions are required so that the agent can be used by the SteerLib::SpatialDataBaseInterface spatial database;
	/// The Util namespace helper functions do the job nicely for basic circular agents.
	//@{
	bool intersects(const Util::Ray &r, float &t) { return Util::rayIntersectsCircle2D(_position, _radius, r, t); }
	bool overlaps(const Util::Point & p, float radius) { return Util::circleOverlapsCircle2D( _position, _radius, p, radius); }
	float computePenetration(const Util::Point & p, float radius) { return Util::computeCircleCirclePenetration2D( _position, _radius, p, radius); }
	//@}

protected:
	/// Updates position, velocity, and orientation of the agent, given the force and dt time step.
	// void _doEulerStep(const Util::Vector & steeringDecisionForce, float dt);

	PredictiveAvoidanceParameters _PredictiveAvoidanceParams;

	virtual SteerLib::EngineInterface * getSimulationEngine();

	void update(float timeStamp, float dt, unsigned int frameNumber);


	size_t id_;
	SteerLib::ModuleInterface * rvoModule;

	SteerLib::EngineInterface * _gEngine;

private:
	Util::Vector calcPAMForces(float dt, Util::Vector goalVector);

	Util::Vector calcWallNormal(SteerLib::ObstacleInterface* obs);
	std::pair<Util::Point, Util::Point> calcWallPointsFromNormal(SteerLib::ObstacleInterface* obs, Util::Vector normal);
	Util::Vector calcObsNormal(SteerLib::ObstacleInterface* obs);
	std::multimap<float, SteerLib::AgentInterface*> calcSetOfCollidingAgents(Util::Vector desiredV);
	float rayIntersectsDisc(const Util::Point& Pa, const Util::Point & Pb, const Util::Vector & v, float radius);
	std::set<SteerLib::SpatialDatabaseItemPtr> collectObjectsInVisualField();

	friend class PredictiveAvoidanceAIModule;

};


#endif
