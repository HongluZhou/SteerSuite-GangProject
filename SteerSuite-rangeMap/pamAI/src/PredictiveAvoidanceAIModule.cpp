//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//
// Extracted from SocialForcesAIModule.cpp located in socialForcesAI folder
//  

/// @file PredictiveAvoidanceAIModule.cpp
/// @brief Implements the PredictiveAvoidanceAIModule plugin.


#include "SimulationPlugin.h"
#include "PredictiveAvoidanceAIModule.h"
#include "PredictiveAvoidanceAgent.h"

#include "LogObject.h"
#include "LogManager.h"


// globally accessible to the simpleAI plugin
// SteerLib::EngineInterface * gEngine;
// SteerLib::SpatialDataBaseInterface * gSpatialDatabase;

namespace PredictiveAvoidanceGlobals
{

	// SteerLib::EngineInterface * gEngineInfo;
	// SteerLib::SpatialDataBaseInterface * gSpatialDatabase;
	unsigned int gLongTermPlanningPhaseInterval;
	unsigned int gMidTermPlanningPhaseInterval;
	unsigned int gShortTermPlanningPhaseInterval;
	unsigned int gPredictivePhaseInterval;
	unsigned int gReactivePhaseInterval;
	unsigned int gPerceptivePhaseInterval;
	bool gUseDynamicPhaseScheduling;
	bool gShowStats;
	bool gShowAllStats;
	bool dont_plan;


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


	PhaseProfilers * gPhaseProfilers;
}

using namespace PredictiveAvoidanceGlobals;

PLUGIN_API SteerLib::ModuleInterface * createModule()
{
	return new PredictiveAvoidanceAIModule;
}

PLUGIN_API void destroyModule( SteerLib::ModuleInterface*  module )
{
	delete module;
}


void PredictiveAvoidanceAIModule::init( const SteerLib::OptionDictionary & options, SteerLib::EngineInterface * engineInfo )
{
	// gEngine = engineInfo;
	// gSpatialDatabase = engineInfo->getSpatialDatabase();
	_gEngine = engineInfo;		
	_data = "";

	gUseDynamicPhaseScheduling = false;
	gShowStats = false;
	logStats = false;
	gShowAllStats = false;
	logFilename = "pamAI.log";
	dont_plan = false;

// SETTING THESE TO DEFAULTS
	pam_preferred_speed = PREFERRED_SPEED;		// Agent's preferred speed
	pam_max_acceleration = MAX_ACCELERATION;		// Agent's max acceleratoin
	pam_max_speed = MAX_SPEED;			// Agent's max speed
	pam_fov = FOV;					// Agent's field of view
	pam_goal_radius = GOAL_RADIUS;			// How close to goal before sim stop // Not needed since we handle this internally?
	pam_ksi = KSI;					// Relaxation time
	pam_neighbour_distance = NEIGHBOUR_DISTANCE;	// neighbour distance for NN queries
	pam_max_neighbours = MAX_NEIGHBOURS;			// max number of neighbours to take into account
	pam_time_horizon = TIME_HORIZON;			// Time horizon for predictions
	pam_agent_distance = AGENT_DISTANCE;		// Safe distance from other agents
	pam_wall_distance = WALL_DISTANCE;		// Safe doistance from walls
	pam_d_mid = D_MID;				// Predictive force interval
	pam_agent_strength = AGENT_STRENGTH;		// Strength of predictive force
	pam_wall_steepness = WALL_STEEPNESS;		// Wall potential steepness
	pam_w_factor = W_FACTOR;				// Scalling factor for force accumulation


	SteerLib::OptionDictionary::const_iterator optionIter;
	for (optionIter = options.begin(); optionIter != options.end(); ++optionIter) {
		std::stringstream value((*optionIter).second);
		// std::cout << "option " << (*optionIter).first << " value " << value.str() << std::endl;
		if ((*optionIter).first == "")
		{
			value >> gLongTermPlanningPhaseInterval;
		}
		else if ((*optionIter).first == "pam_preferred_speed")
		{
			value >> pam_preferred_speed;
		}
		else if ((*optionIter).first == "pam_max_acceleration")
		{
			value >> pam_max_acceleration;
		}
		else if ((*optionIter).first == "pam_max_speed")
		{
			value >> pam_max_speed;
		}
		else if ((*optionIter).first == "pam_fov")
		{
			value >> pam_fov;
		}
		else if ((*optionIter).first == "pam_ksi")
		{
			value >> pam_ksi;
		}
		else if ((*optionIter).first == "pam_neighbour_distance")
		{
			value >> pam_neighbour_distance;
		}
		else if ((*optionIter).first == "pam_max_neighbours")
		{
			value >> pam_max_neighbours;
		}
		else if ((*optionIter).first == "pam_time_horizon")
		{
			value >> pam_time_horizon;
		}
		else if ((*optionIter).first == "pam_agent_distance")
		{
			value >> pam_agent_distance;
		}
		else if ((*optionIter).first == "pam_wall_distance")
		{
			value >> pam_wall_distance;
		}
		else if ((*optionIter).first == "pam_d_mid")
		{
			value >> pam_d_mid;
		}
		else if ((*optionIter).first == "pam_agent_strength")
		{
			value >> pam_agent_strength;
		}
		else if ((*optionIter).first == "pam_wall_steepness")
		{
			value >> pam_wall_steepness;
		}
		else if ((*optionIter).first == "pam_w_factor")
		{
			value >> pam_w_factor;
		}
		else if ((*optionIter).first == "ailogFileName")
		{
			logFilename = value.str();
			logStats = true;
		}
		else if ((*optionIter).first == "logAIStats")
		{
			logStats = true;
		}
		else if ((*optionIter).first == "stats")
		{
			gShowStats = Util::getBoolFromString(value.str());
		}
		else if ((*optionIter).first == "allstats")
		{
			gShowAllStats = Util::getBoolFromString(value.str());
		}
		else if ((*optionIter).first == "dont_plan")
		{
			dont_plan = Util::getBoolFromString(value.str());
		}
		else
		{
			// throw Util::GenericException("unrecognized option \"" + Util::toString((*optionIter).first) + "\" given to PPR AI module.");
		}
	}

	

	_rvoLogger = LogManager::getInstance()->createLogger(logFilename,LoggerType::BASIC_WRITE);

	_rvoLogger->addDataField("number_of_times_executed",DataType::LongLong );
	_rvoLogger->addDataField("total_ticks_accumulated",DataType::LongLong );
	_rvoLogger->addDataField("shortest_execution",DataType::LongLong );
	_rvoLogger->addDataField("longest_execution",DataType::LongLong );
	_rvoLogger->addDataField("fastest_execution", DataType::Float);
	_rvoLogger->addDataField("slowest_execution", DataType::Float);
	_rvoLogger->addDataField("average_time_per_call", DataType::Float);
	_rvoLogger->addDataField("total_time_of_all_calls", DataType::Float);
	_rvoLogger->addDataField("tick_frequency", DataType::Float);

	if( logStats )
		{
		// LETS TRY TO WRITE THE LABELS OF EACH FIELD
		std::stringstream labelStream;
		unsigned int i;
		for (i=0; i < _rvoLogger->getNumberOfFields() - 1; i++)
			labelStream << _rvoLogger->getFieldName(i) << " ";
		labelStream << _rvoLogger->getFieldName(i);
		_data = labelStream.str() + "\n";

		_rvoLogger->writeData(labelStream.str());

	}
}

void PredictiveAvoidanceAIModule::initializeSimulation()
{
	//
	// initialize the performance profilers
	//
	gPhaseProfilers = new PhaseProfilers;
	gPhaseProfilers->aiProfiler.reset();
	gPhaseProfilers->longTermPhaseProfiler.reset();
	gPhaseProfilers->midTermPhaseProfiler.reset();
	gPhaseProfilers->shortTermPhaseProfiler.reset();
	gPhaseProfilers->perceptivePhaseProfiler.reset();
	gPhaseProfilers->predictivePhaseProfiler.reset();
	gPhaseProfilers->reactivePhaseProfiler.reset();
	gPhaseProfilers->steeringPhaseProfiler.reset();

}

void PredictiveAvoidanceAIModule::finish()
{
	// nothing to do here
}

void PredictiveAvoidanceAIModule::preprocessSimulation()
{


}

void PredictiveAvoidanceAIModule::preprocessFrame(float timeStamp, float dt, unsigned int frameNumber)
{
	if ( frameNumber == 1)
	{
		// Adding in this extra one because it seemed sometimes agents would forget about obstacles.

	}
	if ( !agents_.empty() )
	{

	}
}

void PredictiveAvoidanceAIModule::postprocessFrame(float timeStamp, float dt, unsigned int frameNumber)
{
	// do nothing for now
	int i = 0;
	i = i + i;
}
SteerLib::AgentInterface * PredictiveAvoidanceAIModule::createAgent()
{
	PredictiveAvoidanceAgent * agent = new PredictiveAvoidanceAgent;
	agent->rvoModule = this;
	agent->id_ = agents_.size();
	agents_.push_back(agent);
	agent->_gEngine = this->_gEngine;
	return agent;
}

void PredictiveAvoidanceAIModule::destroyAgent( SteerLib::AgentInterface * agent )
{
	// TODO this is going to be a memory leak for now.
	delete agent;
}

void PredictiveAvoidanceAIModule::cleanupSimulation()
{
	agents_.clear();

		LogObject rvoLogObject;

		rvoLogObject.addLogData(gPhaseProfilers->aiProfiler.getNumTimesExecuted());
		rvoLogObject.addLogData(gPhaseProfilers->aiProfiler.getTotalTicksAccumulated());
		rvoLogObject.addLogData(gPhaseProfilers->aiProfiler.getMinTicks());
		rvoLogObject.addLogData(gPhaseProfilers->aiProfiler.getMaxTicks());
		rvoLogObject.addLogData(gPhaseProfilers->aiProfiler.getMinExecutionTimeMills());
		rvoLogObject.addLogData(gPhaseProfilers->aiProfiler.getMaxExecutionTimeMills());
		rvoLogObject.addLogData(gPhaseProfilers->aiProfiler.getAverageExecutionTimeMills());
		rvoLogObject.addLogData(gPhaseProfilers->aiProfiler.getTotalTime());
		rvoLogObject.addLogData(gPhaseProfilers->aiProfiler.getTickFrequency());

		_rvoLogger->writeLogObject(rvoLogObject);
		_data = _data + _rvoLogger->logObjectToString(rvoLogObject);
		_logData.push_back(rvoLogObject.copy());

		// cleanup profileing metrics for next simulation/scenario
		gPhaseProfilers->aiProfiler.reset();
		gPhaseProfilers->longTermPhaseProfiler.reset();
		gPhaseProfilers->midTermPhaseProfiler.reset();
		gPhaseProfilers->shortTermPhaseProfiler.reset();
		gPhaseProfilers->perceptivePhaseProfiler.reset();
		gPhaseProfilers->predictivePhaseProfiler.reset();
		gPhaseProfilers->reactivePhaseProfiler.reset();
		gPhaseProfilers->steeringPhaseProfiler.reset();
	if ( logStats )
	{
		_rvoLogger->writeLogObject(rvoLogObject);
	}

}
