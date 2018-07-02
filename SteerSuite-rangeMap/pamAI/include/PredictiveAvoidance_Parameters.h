//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//
// Parts extracted from SocialForces_Parameters.h located in socialForcesAI folder
//

#ifndef PredictiveAvoidance_PARAMETERS_H_
#define PredictiveAvoidance_PARAMETERS_H_

// #include "testcaseio/Behaviour.h"


#define KSI 0.5f;
#define NEIGHBOUR_DISTANCE 10.f
#define MAX_NEIGHBOURS 3
#define MAX_ACCELERATION 20.f
#define MAX_SPEED 2.f
#define PREFERRED_SPEED 1.3f
#define RADIUS 0.5f
#define GOAL_RADIUS 1.f
#define TIME_HORIZON 4.f
#define AGENT_DISTANCE .1f
#define WALL_DISTANCE .1f
#define WALL_STEEPNESS 2.f
#define AGENT_STRENGTH 1.f
#define W_FACTOR .8f
#define D_MID 4.f
#define FOV 200.f

#define _EPSILON 0.001f		// A custom epsilon value
#define INFTY 9e9f 			// Imitation of infinity

#define MASS 1
// #define WAYPOINT_THRESHOLD_MULTIPLIER 2.5
// #define GOAL_THRESHOLD_MULTIPLIER 10.5
#define WAYPOINT_THRESHOLD_MULTIPLIER 1
#define GOAL_THRESHOLD_MULTIPLIER 2.5

#define USE_PLANNING 1
// #define DRAW_ANNOTATIONS 1
#define USE_CIRCLES 1
// #define _DEBUG_ 1
namespace PredictiveAvoidanceGlobals {

	struct PhaseProfilers {
		Util::PerformanceProfiler aiProfiler;
		Util::PerformanceProfiler drawProfiler;
		Util::PerformanceProfiler longTermPhaseProfiler;
		Util::PerformanceProfiler midTermPhaseProfiler;
		Util::PerformanceProfiler shortTermPhaseProfiler;
		Util::PerformanceProfiler perceptivePhaseProfiler;
		Util::PerformanceProfiler predictivePhaseProfiler;
		Util::PerformanceProfiler reactivePhaseProfiler;
		Util::PerformanceProfiler steeringPhaseProfiler;
	};


	extern SteerLib::EngineInterface * gEngineInfo;
	extern SteerLib::SpatialDataBaseInterface * gSpatialDatabase;
	extern unsigned int gLongTermPlanningPhaseInterval;
	extern unsigned int gMidTermPlanningPhaseInterval;
	extern unsigned int gShortTermPlanningPhaseInterval;
	extern unsigned int gPredictivePhaseInterval;
	extern unsigned int gReactivePhaseInterval;
	extern unsigned int gPerceptivePhaseInterval;
	extern bool gUseDynamicPhaseScheduling;
	extern bool gShowStats;
	extern bool gShowAllStats;
	extern bool dont_plan;


	// Adding a bunch of parameters so they can be changed via input
	extern float pam_preferred_speed;
	extern float pam_max_acceleration;
	extern float pam_max_speed;
	extern float pam_fov;
	extern float pam_goal_radius;
	extern float pam_ksi;
	extern float pam_neighbour_distance;
	extern int pam_max_neighbours;
	extern float pam_time_horizon;
	extern float pam_agent_distance;
	extern float pam_wall_distance;
	extern float pam_d_mid;				
	extern float pam_d_max;				
	extern float pam_d_min;				
	extern float pam_personal_space;	
	extern float pam_agent_strength;	
	extern float pam_wall_steepness;	
	extern float pam_w_factor;			

	extern PhaseProfilers * gPhaseProfilers;
}


class PredictiveAvoidanceParameters
{
public:
	// Adding a bunch of parameters so they can be changed via input
	float pam_preferred_speed;		// Agent's preferred speed
	float pam_max_acceleration;		// Agent's max acceleratoin
	float pam_max_speed;			// Agent's max speed
	float pam_fov;					// Agent's field of view
	float pam_goal_radius;			// How close to goal before sim stop // Not needed since we handle this internally?
	float pam_ksi;					// Relaxation time
	float pam_neighbour_distance;	// neighbour distance for NN queries
	int pam_max_neighbours;			// max number of neighbours to take into account
	float pam_time_horizon;			// Time horizon for predictions
	float pam_agent_distance;		// Safe distance from other agents
	float pam_wall_distance;		// Safe doistance from walls
	float pam_d_mid;				// Predictive force interval
	float pam_d_max;				// Set in init
	float pam_d_min;				// Set in init
	float pam_personal_space;		// Set in init
	float pam_agent_strength;		// Strength of predictive force
	float pam_wall_steepness;		// Wall potential steepness
	float pam_w_factor;				// Scalling factor for force accumulation

	void setParameters(SteerLib::Behaviour behavior)
	{
		// std::cout << "Setting parameters from behaviour" << std::endl;
		int i;
		for ( i = 0; i < behavior.getParameters().size(); i++)
		{
			std::string p_key = behavior.getParameters().at(i).key;
			std::stringstream value(behavior.getParameters().at(i).value);

			// std::cout << "key: " << p_key << ", value: " << value << std::endl;
			
			if (p_key == "pam_preferred_speed")
			{
				value >> pam_preferred_speed;
			}
			else if (p_key == "pam_max_acceleration")
			{
				value >> pam_max_acceleration;
			}
			else if (p_key == "pam_max_speed")
			{
				value >> pam_max_speed;
			}
			else if (p_key == "pam_fov")
			{
				value >> pam_fov;
			}
			else if (p_key == "pam_ksi")
			{
				value >> pam_ksi;
			}
			else if (p_key == "pam_neighbour_distance")
			{
				value >> pam_neighbour_distance;
			}
			else if (p_key == "pam_max_neighbours")
			{
				value >> pam_max_neighbours;
			}
			else if (p_key == "pam_time_horizon")
			{
				value >> pam_time_horizon;
			}
			else if (p_key == "pam_agent_distance")
			{
				value >> pam_agent_distance;
			}
			else if (p_key == "pam_wall_distance")
			{
				value >> pam_wall_distance;
			}
			else if (p_key == "pam_d_mid")
			{
				value >> pam_d_mid;
			}
			else if (p_key == "pam_agent_strength")
			{
				value >> pam_agent_strength;
			}
			else if (p_key == "pam_wall_steepness")
			{
				value >> pam_wall_steepness;
			}
			else if (p_key == "pam_w_factor")
			{
				value >> pam_w_factor;
			}
		}
		
	}
};

inline std::ostream &operator<<(std::ostream & out, const PredictiveAvoidanceParameters & p)
{ // methods used here must be const
	out << "pam_preferred_speed: " << p.pam_preferred_speed << std::endl;
	out << "pam_max_acceleration: " << p.pam_max_acceleration << std::endl;
	out << "pam_max_speed: " << p.pam_max_speed << std::endl;
	out << "pam_fov: " << p.pam_fov << std::endl;
	out << "pam_goal_radius: " << p.pam_goal_radius << std::endl;
	out << "pam_ksi: " << p.pam_ksi << std::endl;
	out << "pam_neighbour_distance: " << p.pam_neighbour_distance << std::endl;
	out << "pam_time_horizon: " << p.pam_time_horizon << std::endl;
	out << "pam_agent_distance: " << p.pam_agent_distance << std::endl;
	out << "pam_wall_distance: " << p.pam_wall_distance << std::endl;
	out << "pam_d_mid: " << p.pam_d_mid;
	out << "pam_agent_strength: " << p.pam_agent_strength;
	out << "pam_wall_steepness: " << p.pam_wall_steepness;
	out << "pam_w_factor: " << p.pam_w_factor;

	return out;
}



#endif /* PredictiveAvoidance_PARAMETERS_H_ */
