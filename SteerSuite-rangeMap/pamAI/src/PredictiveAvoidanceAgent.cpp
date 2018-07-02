//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//
// Parts extracted from SocialForcesAgent.cpp located in socialForcesAI folder
//
// Parts of code for functions:
//        Util::Vector calcEvasionForce(float timeStamp, float dt);
//        float rayIntersectsDisc(const Util::Point& Pa, const Util::Point & Pb, const Util::Vector & v, float radius);
//        std::multimap<float, SteerLib::AgentInterface*> calcSetOfCollidingAgents(Util::Vector desiredV);
//
//  ... extracted from I. Karamouzas' implimentation:
//  https://sites.google.com/site/ikaramouzas/pam

#include "PredictiveAvoidanceAgent.h"
#include "PredictiveAvoidanceAIModule.h"
#include "PredictiveAvoidance_Parameters.h"

/// @file PredictiveAvoidanceAgent.cpp
/// @brief Implements the PredictiveAvoidanceAgent class.

#undef min
#undef max

#define AGENT_MASS 1.0f

using namespace Util;
using namespace PredictiveAvoidanceGlobals;
using namespace SteerLib;

// #define _DEBUG_ENTROPY 1

/*
 * constructor
 * borrowed from SocialFrocesAgent.cpp
 */
PredictiveAvoidanceAgent::PredictiveAvoidanceAgent()
{
    _PredictiveAvoidanceParams.pam_preferred_speed = pam_preferred_speed;
    _PredictiveAvoidanceParams.pam_max_acceleration = pam_max_acceleration;
    _PredictiveAvoidanceParams.pam_max_speed = pam_max_speed;
    _PredictiveAvoidanceParams.pam_fov = pam_fov;
    _PredictiveAvoidanceParams.pam_ksi = pam_ksi;
    _PredictiveAvoidanceParams.pam_neighbour_distance = pam_neighbour_distance;
    _PredictiveAvoidanceParams.pam_max_neighbours = pam_max_neighbours;
    _PredictiveAvoidanceParams.pam_time_horizon = pam_time_horizon;
    _PredictiveAvoidanceParams.pam_agent_distance = pam_agent_distance;
    _PredictiveAvoidanceParams.pam_wall_distance = pam_wall_distance;
    _PredictiveAvoidanceParams.pam_d_mid = pam_d_mid;
    _PredictiveAvoidanceParams.pam_agent_strength = pam_agent_strength;
    _PredictiveAvoidanceParams.pam_wall_steepness = pam_wall_steepness;
    _PredictiveAvoidanceParams.pam_w_factor = pam_w_factor;
    
    _enabled = false;
}

/*
 * destructor
 * borrowed from SocialFrocesAgent.cpp
 */
PredictiveAvoidanceAgent::~PredictiveAvoidanceAgent()
{
    // std::cout << this << " is being deleted" << std::endl;
    /*
    if (this->enabled())
    {
        Util::AxisAlignedBox bounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.0f, _position.z-_radius, _position.z+_radius);
        // getSimulationEngine()->getSpatialDatabase()->removeObject( this, bounds);
    }*/
    // std::cout << "Someone is removing an agent " << std::endl;
}

/*
 * gets the simulation engine
 */
SteerLib::EngineInterface * PredictiveAvoidanceAgent::getSimulationEngine()
{
    return _gEngine;
}

/*
 * borrowed from SocialFrocesAgent.cpp
 */
void PredictiveAvoidanceAgent::setParameters(Behaviour behave)
{
    this->_PredictiveAvoidanceParams.setParameters(behave);
}

/*
 * borrowed from SocialFrocesAgent.cpp
 */
void PredictiveAvoidanceAgent::disable()
{
    // DO nothing for now
    // if we tried to disable a second time, most likely we accidentally ignored that it was disabled, and should catch that error.
    // std::cout << "this agent is being disabled " << this << std::endl;
    assert(_enabled==true);
    
    
    //  1. remove from database
    AxisAlignedBox b = AxisAlignedBox(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
    getSimulationEngine()->getSpatialDatabase()->removeObject(dynamic_cast<SpatialDatabaseItemPtr>(this), b);
    
    //  2. set enabled = false
    _enabled = false;
    
}

/*
 * borrowed from SocialFrocesAgent.cpp
 */
void PredictiveAvoidanceAgent::reset(const SteerLib::AgentInitialConditions & initialConditions, SteerLib::EngineInterface * engineInfo)
{
    // compute the "old" bounding box of the agent before it is reset.  its OK that it will be invalid if the agent was previously disabled
    // because the value is not used in that case.
    // std::cout << "resetting agent " << this << std::endl;
    _waypoints.clear();
    _midTermPath.clear();

    Util::AxisAlignedBox oldBounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.5f, _position.z-_radius, _position.z+_radius);


    // initialize the agent based on the initial conditions
    /*
    position_ = Vector2(initialConditions.position.x, initialConditions.position.z);
    radius_ = initialConditions.radius;
    velocity_ = normalize(Vector2(initialConditions.direction.x, initialConditions.direction.z));
    velocity_ = velocity_ * initialConditions.speed;
*/
    // initialize the agent based on the initial conditions
    _position = initialConditions.position;
    _forward = normalize(initialConditions.direction);
    _radius = initialConditions.radius;
    _velocity = initialConditions.speed * _forward;
    // std::cout << "inital colour of agent " << initialConditions.color << std::endl;
    if ( initialConditions.colorSet == true )
    {
        this->_color = initialConditions.color;
    }
    else
    {
        this->_color = Util::gBlue;
    }

    // compute the "new" bounding box of the agent
    Util::AxisAlignedBox newBounds(_position.x-_radius, _position.x+_radius, 0.0f, 0.5f, _position.z-_radius, _position.z+_radius);

    if (!_enabled) {
        // if the agent was not enabled, then it does not already exist in the database, so add it.
        // std::cout
        getSimulationEngine()->getSpatialDatabase()->addObject( dynamic_cast<SpatialDatabaseItemPtr>(this), newBounds);
    }
    else {
        // if the agent was enabled, then the agent already existed in the database, so update it instead of adding it.
        // std::cout << "new position is " << _position << std::endl;
        // std::cout << "new bounds are " << newBounds << std::endl;
        // std::cout << "reset update " << this << std::endl;
        getSimulationEngine()->getSpatialDatabase()->updateObject( dynamic_cast<SpatialDatabaseItemPtr>(this), oldBounds, newBounds);
        // engineInfo->getSpatialDatabase()->updateObject( this, oldBounds, newBounds);
    }

    _enabled = true;

    if (initialConditions.goals.size() == 0)
    {
        throw Util::GenericException("No goals were specified!\n");
    }

    while (!_goalQueue.empty())
    {
        _goalQueue.pop();
    }

    // iterate over the sequence of goals specified by the initial conditions.
    for (unsigned int i=0; i<initialConditions.goals.size(); i++) {
        if (initialConditions.goals[i].goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET ||
                initialConditions.goals[i].goalType == GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL)
        {

            if (initialConditions.goals[i].targetIsRandom)
            {
                // if the goal is random, we must randomly generate the goal.
                // std::cout << "assigning random goal" << std::endl;
                SteerLib::AgentGoalInfo _goal;
                _goal.targetLocation = getSimulationEngine()->getSpatialDatabase()->randomPositionWithoutCollisions(1.0f, true);
                _goalQueue.push(_goal);
                _currentGoal.targetLocation = _goal.targetLocation;
            }
            else
            {
                _goalQueue.push(initialConditions.goals[i]);
            }
            // std::cout << "Setting target behaviour: " << std::endl;
            this->setParameters(initialConditions.goals[i].targetBehaviour);
        }
        else {
            throw Util::GenericException("Unsupported goal type; SocialForcesAgent only supports GOAL_TYPE_SEEK_STATIC_TARGET and GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL.");
        }
    }

    runLongTermPlanning(_goalQueue.front().targetLocation, dont_plan);

    // std::cout << "first waypoint: " << _waypoints.front() << " agents position: " << position() << std::endl;
    /*
     * Must make sure that _waypoints.front() != position(). If they are equal the agent will crash.
     * And that _waypoints is not empty
     */
    Util::Vector goalDirection;
    if ( !_midTermPath.empty() )
    {
        this->updateLocalTarget();
        goalDirection = normalize( this->_currentLocalTarget - position());
    }
    else
    {
        goalDirection = normalize( _goalQueue.front().targetLocation - position());
    }


    // _velocity = _prefVelocity;
#ifdef _DEBUG_ENTROPY
    std::cout << "goal direction is: " << goalDirection << " prefvelocity is: " << prefVelocity_ <<
            " and current velocity is: " << velocity_ << std::endl;
#endif


    // std::cout << "Parameter spec: " << _SocialForcesParams << std::endl;
    // _gEngine->addAgent(this, rvoModule);
    assert(_forward.length()!=0.0f);
    assert(_goalQueue.size() != 0);
    assert(_radius != 0.0f);
}


/*
 * borrowed from SocialFrocesAgent.cpp
 */
std::pair<float, Util::Point> minimum_distance(Util::Point l1, Util::Point l2, Util::Point p)
{
    // Return minimum distance between line segment vw and point p
    float lSq = (l1 - l2).lengthSquared();  // i.e. |l2-l1|^2 -  avoid a sqrt
    if (lSq == 0.0)
    return std::make_pair((p - l2).length(),l1 );   // l1 == l2 case
    // Consider the line extending the segment, parameterized as l1 + t (l2 - l1).
    // We find projection of point p onto the line.
    // It falls where t = [(p-l1) . (l2-l1)] / |l2-l1|^2
    const float t = dot(p - l1, l2 - l1) / lSq;
    if (t < 0.0)
    {
        return std::make_pair((p - l1).length(), l1);       // Beyond the 'l1' end of the segment
    }
    else if (t > 1.0)
    {
        return std::make_pair((p - l2).length(), l2);  // Beyond the 'l2' end of the segment
    }
    const Util::Point projection = l1 + t * (l2 - l1);  // Projection falls on the segment
    return std::make_pair((p - projection).length(), projection) ;
}


/*
 * The calcPAMForces function
 * 
 */
Util::Vector PredictiveAvoidanceAgent::calcPAMForces(float dt, Util::Vector goalVector)
{
    // CURRENT GOAL
    //SteerLib::AgentGoalInfo goalInfo = _goalQueue.front();

    

    // PREFERRED VELOCITY
    Util::Vector _preferredVelocity = goalVector;
    _preferredVelocity *= _PredictiveAvoidanceParams.pam_preferred_speed / _preferredVelocity.length();


    // DRIVING FORCE
    Util::Vector _drivingForce = (_preferredVelocity - velocity()) / _PredictiveAvoidanceParams.pam_ksi;

    const float SAFE = _PredictiveAvoidanceParams.pam_wall_distance * _PredictiveAvoidanceParams.pam_wall_distance;
    bool collision = false;


    // STATIC OBSTACLES
    std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
    getSimulationEngine()->getSpatialDatabase()->getItemsInRange(_neighbors,
    _position.x-(this->_radius + _PredictiveAvoidanceParams.pam_neighbour_distance),
    _position.x+(this->_radius + _PredictiveAvoidanceParams.pam_neighbour_distance),
    _position.z-(this->_radius + _PredictiveAvoidanceParams.pam_neighbour_distance),
    _position.z+(this->_radius + _PredictiveAvoidanceParams.pam_neighbour_distance),
    dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

    SteerLib::ObstacleInterface * tmp_ob;
    
    for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbour = _neighbors.begin();  neighbour != _neighbors.end();  neighbour++)
    {
        if ( !(*neighbour)->isAgent() )
        {
            tmp_ob = dynamic_cast<SteerLib::ObstacleInterface *>(*neighbour);
        }
        else
        {
            continue;
        }

        Util::Vector wall_normal = calcWallNormal( tmp_ob );
        std::pair<Util::Point,Util::Point> line = calcWallPointsFromNormal(tmp_ob, wall_normal);
        std::pair<float, Util::Point> min_stuff = minimum_distance(line.first, line.second, position());


        Util::Vector n_w = _position - min_stuff.second;
        float d_w = n_w.length() * n_w.length();
        if (d_w < SAFE)
        {
            d_w = sqrtf(d_w);
            if (d_w > 0)
                n_w /= d_w;

            const float distMinRadius = (d_w - _radius) < _EPSILON? _EPSILON : d_w - _radius;
            //if (d_w < _radius) collision = true;
            _drivingForce += (_PredictiveAvoidanceParams.pam_wall_distance - d_w)/powf(distMinRadius, _PredictiveAvoidanceParams.pam_wall_steepness) * n_w;  
        }
    } 


    // DESIRED VELOCITY
    Util::Vector _desiredVelocity = velocity() + _drivingForce * dt;
    float desiredSpeed = _desiredVelocity.length();


    // NN NEIGHBOUR COLLISION SET    
    float _cosFov = cosf( (0.5f * M_PI * _PredictiveAvoidanceParams.pam_fov) / 180.f);

    std::multimap<float, SteerLib::AgentInterface*> _collidingNeighbors;
    
    //iterate over items in agent's visual field
    for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbor = _neighbors.begin(); neighbor != _neighbors.end(); ++neighbor) {
        
        // ignore items that are not AI agents.
        if (!(*neighbor)->isAgent())
            continue;
        
        SteerLib::AgentInterface * otherGuy = static_cast<SteerLib::AgentInterface *>(*neighbor);
        
        // ignore self
        if (this == otherGuy) continue;
        
        // ignore pedestrians who are not enabled
        if (!otherGuy->enabled())
            continue;
        
        const float combinedRadius = pam_personal_space + otherGuy->radius();
        const Util::Vector w = otherGuy->position() - position(); // how far away are the two agents currently
        if (w.lengthSquared() < combinedRadius * combinedRadius) { //collision!
            if (!collision) {
                collision = true;
                _collidingNeighbors.clear();
            }
            _collidingNeighbors.insert(std::make_pair(.0f, otherGuy)); // insert immediate collision t_collision=0 to set
        }
        else {
            
            Util::Vector relDir = normalize(w);

            if ( ( relDir * normalize(_velocity) ) < _cosFov ) continue;
            
            // compute collision time...
            const float t_collision = rayIntersectsDisc(position(), otherGuy->position(), _desiredVelocity - otherGuy->velocity(), combinedRadius); 
            if ( t_collision < _PredictiveAvoidanceParams.pam_time_horizon)
            {
                
                if ((int)_collidingNeighbors.size() < _PredictiveAvoidanceParams.pam_max_neighbours) //set is not full, insert
                {
                    _collidingNeighbors.insert(std::make_pair(t_collision, otherGuy));
                }
                else if (t_collision < (--_collidingNeighbors.end())->first)    //set is full, add a new character only if the collision is more imminent and remove other agent from set
                {
                    _collidingNeighbors.erase(--_collidingNeighbors.end());
                    _collidingNeighbors.insert(std::make_pair(t_collision, otherGuy));
                }
                
            }
        }
    }


    // PREDICTION        
    int count=0;
    
    // iterate over colliding neighbors
    std::multimap<float,SteerLib::AgentInterface*>::iterator it_end = _collidingNeighbors.end();
    for (std::multimap<float, SteerLib::AgentInterface*>::iterator it = _collidingNeighbors.begin(); it != it_end; ++it)
    {
        
        const SteerLib::AgentInterface * const otherGuy = it->second;
        float t_collision = it->first; //time of collision
        
        // calculate future collision positions
        Util::Vector forceDir = position() + _desiredVelocity * t_collision - otherGuy->position() - otherGuy->velocity() * t_collision; // pointing away from collision position
        float forceDist = forceDir.length();
        if (forceDist > 0)
            forceDir /= forceDist;
        
        float collisionDist = std::max(forceDist - radius() - otherGuy->radius(), .0f); // distance between their cylindrical bodies at the time of collision
        float D = std::max(desiredSpeed * t_collision + collisionDist, _EPSILON); // D = input to evasive force magnitude piecewise function
        
        // calulate the magnitude of the evasive force
        float mag;
        if ( D < _PredictiveAvoidanceParams.pam_d_min ) {
            mag = _PredictiveAvoidanceParams.pam_agent_strength * _PredictiveAvoidanceParams.pam_d_min / D;
            } else if ( D < _PredictiveAvoidanceParams.pam_d_mid ) {
            mag = _PredictiveAvoidanceParams.pam_agent_strength;
            } else if ( D < _PredictiveAvoidanceParams.pam_d_max ) {
            mag = _PredictiveAvoidanceParams.pam_agent_strength * (_PredictiveAvoidanceParams.pam_d_max - D)/(_PredictiveAvoidanceParams.pam_d_max - _PredictiveAvoidanceParams.pam_d_mid);
            } else {
            continue;   // magnitude is zero
        }
        mag *= pow((collision?1.f:_PredictiveAvoidanceParams.pam_w_factor), count++); //minimize magnitude of force according to its weight
        _drivingForce += mag * forceDir; //weighted sum of individual evasive forces
    }
        
    //add a noise force to the total evasive force in order to prevent agents from deadlocking due to symmatry
    float angle = std::rand() * 2.0f * M_PI / RAND_MAX;
    float dist = std::rand() * 0.001f / RAND_MAX; // keep noise force small
    _drivingForce += dist*Vector(cos(angle), sin(angle), 0);
    
    // return total evasive force
    return _drivingForce;
        
}


/*
 *
 * The rayIntersectsDisc function computes the time that the ray will intersect the disc
 * In the case that a past intersection occured, it returns INFTY
 * In the case that no intersection occurs, it returns INFTY as well
 * The INFTY parameter definition can be found in the PredictiveAvoidance_Parameters.h file
 * in the include folder in pamAI
 *
 */
float PredictiveAvoidanceAgent::rayIntersectsDisc(const Util::Point& Pa, const Util::Point & Pb, const Util::Vector & v, float radius)
{
    float t;
    Util::Vector w = Pb - Pa;
    float a = ((v.x*v.x) + (v.y*v.y) + (v.z*v.z));//v*v;
    float b = ((w.x*v.x) + (w.y*v.y) + (w.z*v.z));//w*v;
    float c = ((w.x*w.x) + (w.y*w.y) + (w.z*w.z)) - (radius*radius);//w*w - radius*radius;
    float discr = b*b - a*c;
    if (discr > 0.0f)
    {
        t = (b - sqrtf(discr)) / a;
        if (t < 0)
            t = INFTY;
    }
    else
        t = INFTY;
    
    return t;
}

/*
 *
 * The collectObjectsInVisualField function returns the set of items in the agent's 
 * visual field and is limited by the QUERY_RADIUS value.
 * the QUERY_RADIUS parameter definition can be found in the PredictiveAvoidance_Parameters.h file
 * in the include folder in pamAI  
 *
 */
std::set<SteerLib::SpatialDatabaseItemPtr> PredictiveAvoidanceAgent::collectObjectsInVisualField()
{
    
    std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
    _neighbors.clear();
    
    getSimulationEngine()->getSpatialDatabase()->getItemsInVisualField(_neighbors, _position.x - _PredictiveAvoidanceParams.pam_neighbour_distance, _position.x + _PredictiveAvoidanceParams.pam_neighbour_distance,
    _position.z- _PredictiveAvoidanceParams.pam_neighbour_distance, _position.z + _PredictiveAvoidanceParams.pam_neighbour_distance, dynamic_cast<SpatialDatabaseItemPtr>(this),
    position(), forward(), (float)(_PredictiveAvoidanceParams.pam_neighbour_distance * _PredictiveAvoidanceParams.pam_neighbour_distance));
    
    return _neighbors;
    
}


/*
 * borrowed from SocialFrocesAgent.cpp
 */
std::pair<Util::Point, Util::Point> PredictiveAvoidanceAgent::calcWallPointsFromNormal(SteerLib::ObstacleInterface* obs, Util::Vector normal)
{
    Util::AxisAlignedBox box = obs->getBounds();
    if ( normal.z == 1)
    {
        return std::make_pair(Util::Point(box.xmin,0,box.zmax), Util::Point(box.xmax,0,box.zmax));
        // Ended here;
    }
    else if ( normal.z == -1 )
    {
        return std::make_pair(Util::Point(box.xmin,0,box.zmin), Util::Point(box.xmax,0,box.zmin));
    }
    else if ( normal.x == 1)
    {
        return std::make_pair(Util::Point(box.xmax,0,box.zmin), Util::Point(box.xmax,0,box.zmax));
    }
    else // normal.x == -1
    {
        return std::make_pair(Util::Point(box.xmin,0,box.zmin), Util::Point(box.xmin,0,box.zmax));
    }
}

/**
* Basically What side of the obstacle is the agent on use that as the normal
* DOES NOT SUPPORT non-axis-aligned boxes
*
*
*                           /
*                           /
*                       a     /
*                          /
*                      _
*             a        | |       a
*                      -
*                   /
*                  /   a
*                 /
*                /
*
*
*/
Util::Vector PredictiveAvoidanceAgent::calcWallNormal(SteerLib::ObstacleInterface* obs)
{
    Util::AxisAlignedBox box = obs->getBounds();
    if ( position().x > box.xmax )
    {
        if ( position().z > box.zmax)
        {
            if ( abs(position().z - box.zmax ) >
            abs( position().x - box.xmax) )
            {
                return Util::Vector(0, 0, 1);
            }
            else
            {
                return Util::Vector(1, 0, 0);
            }
            
        }
        else if ( position().z < box.zmin )
        {
            if ( abs(position().z - box.zmin ) >
            abs( position().x - box.xmax) )
            {
                return Util::Vector(0, 0, -1);
            }
            else
            {
                return Util::Vector(1, 0, 0);
            }
            
        }
        else
        { // in between zmin and zmax
            return Util::Vector(1, 0, 0);
        }
        
    }
    else if ( position().x < box.xmin )
    {
        if ( position().z > box.zmax )
        {
            if ( abs(position().z - box.zmax ) >
            abs( position().x - box.xmin) )
            {
                return Util::Vector(0, 0, 1);
            }
            else
            {
                return Util::Vector(-1, 0, 0);
            }
            
        }
        else if ( position().z < box.zmin )
        {
            if ( abs(position().z - box.zmin ) >
            abs( position().x - box.xmin) )
            {
                return Util::Vector(0, 0, -1);
            }
            else
            {
                return Util::Vector(-1, 0, 0);
            }
            
        }
        else
        { // in between zmin and zmax
            return Util::Vector(-1, 0, 0);
        }
    }
    else // between xmin and xmax
    {
        if ( position().z > box.zmax )
        {
            return Util::Vector(0, 0, 1);
        }
        else if ( position().z < box.zmin)
        {
            return Util::Vector(0, 0, -1);
        }
        else
        { // What do we do if the agent is inside the wall?? Lazy Normal
            return calcObsNormal( obs );
        }
    }
    
}

/**
* Treats Obstacles as a circle and calculates normal
*/
Util::Vector PredictiveAvoidanceAgent::calcObsNormal(SteerLib::ObstacleInterface* obs)
{
    Util::AxisAlignedBox box = obs->getBounds();
    Util::Point obs_centre = Util::Point((box.xmax+box.xmin)/2, (box.ymax+box.ymin)/2,
    (box.zmax+box.zmin)/2);
    return normalize(position() - obs_centre);
}

/*
 * borrowed from SocialFrocesAgent.cpp
 */
void PredictiveAvoidanceAgent::updateAI(float timeStamp, float dt, unsigned int frameNumber)
{
    if(gShowStats)
        Util::AutomaticFunctionProfiler profileThisFunction( &PredictiveAvoidanceGlobals::gPhaseProfilers->aiProfiler );

    if (!enabled()) 
    {
        return;
    }

    // Store old bounds
    Util::AxisAlignedBox oldBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);

    if (!this->hasLineOfSightTo(_currentLocalTarget))
    {
        runLongTermPlanning(this->_goalQueue.front().targetLocation, false);
        this->updateMidTermPath();
        this->updateLocalTarget();
    }

    SteerLib::AgentGoalInfo goalInfo = _goalQueue.front();
    Util::Vector goalVector;
    if ( ! _midTermPath.empty() && !this->hasLineOfSightTo(goalInfo.targetLocation) )
    {
        if (reachedLocalTarget())
        {
            this->updateMidTermPath();
            this->updateLocalTarget();
        }

        
    }
    else if (this->hasLineOfSightTo(goalInfo.targetLocation))
    {
        _currentLocalTarget = goalInfo.targetLocation;
    }

    goalVector = normalize(_currentLocalTarget - position());


// COMPUTE NEXT MOVE

    Util::Vector acceleration = calcPAMForces(dt, goalVector);
    acceleration = clamp(acceleration, _PredictiveAvoidanceParams.pam_max_acceleration);
    _velocity = _velocity + acceleration * dt;  
    _velocity = clamp(_velocity, _PredictiveAvoidanceParams.pam_max_speed);
    _velocity.y=0.0f;
    _position = position() + _velocity * dt;


    // A grid database update should always be done right after the new position of the agent is calculated
    /*
    * Or when the agent is removed for example its true location will not reflect its location in the grid database.
    * Not only that but this error will appear random depending on how well the agent lines up with the grid database
    * boundaries when removed.
    */
    // std::cout << "Updating agent" << this->id() << " at " << this->position() << std::endl;
    Util::AxisAlignedBox newBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
    getSimulationEngine()->getSpatialDatabase()->updateObject( this, oldBounds, newBounds);
    
    /*
    * Now do the conversion from PredictiveAvoidanceAgent into the SteerSuite coordinates
    */

    goalVector = normalize(goalVector);
    
    if ((goalInfo.targetLocation - position()).length() < radius()*GOAL_THRESHOLD_MULTIPLIER ||
            (goalInfo.goalType == GOAL_TYPE_AXIS_ALIGNED_BOX_GOAL &&
                    Util::boxOverlapsCircle2D(goalInfo.targetRegion.xmin, goalInfo.targetRegion.xmax,
                            goalInfo.targetRegion.zmin, goalInfo.targetRegion.zmax, this->position(), this->radius())))
    {
        _goalQueue.pop();
        // std::cout << "Made it to a goal" << std::endl;
        if (_goalQueue.size() != 0)
        {
            // in this case, there are still more goals, so start steering to the next goal.
            goalVector = _goalQueue.front().targetLocation - _position;
            //_prefVelocity = Util::Vector(goalVector.x, 0.0f, goalVector.z);
        }
        else
        {
            // in this case, there are no more goals, so disable the agent and remove it from the spatial database.
            disable();
            return;
        }
    }
    

}

/*
 * borrowed from SocialFrocesAgent.cpp
 */
void PredictiveAvoidanceAgent::draw()
{
    #ifdef ENABLE_GUI
    AgentInterface::draw();
    // if the agent is selected, do some annotations just for demonstration
    if (getSimulationEngine()->isAgentSelected(this))
    {
        // draw goal
        DrawLib::drawFlag( _currentLocalTarget, gBlack, 4.0 );
    }
    #ifdef DRAW_COLLISIONS
    std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
    getSimulationEngine()->getSpatialDatabase()->getItemsInRange(_neighbors, _position.x-(this->_radius * 3), _position.x+(this->_radius * 3),
    _position.z-(this->_radius * 3), _position.z+(this->_radius * 3), dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));
    
    for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbor = _neighbors.begin();  neighbor != _neighbors.end();  neighbor++)
    {
        if ( (*neighbor)->isAgent() && (*neighbor)->computePenetration(this->position(), this->_radius) > 0.00001f)
        {
            Util::DrawLib::drawStar(
            this->position()
            +
            (
            (
            dynamic_cast<AgentInterface*>(*neighbor)->position()
            -
            this->position()
            )
            /2), Util::Vector(1,0,0), 0.8f, gRed);
            
        }
    }
    #endif
    #ifdef DRAW_HISTORIES
    __oldPositions.push_back(position());
    int points = 0;
    float mostPoints = 100.0f;
    while ( __oldPositions.size() > mostPoints )
    {
        __oldPositions.pop_front();
    }
    for (int q = __oldPositions.size()-1 ; q > 0 && __oldPositions.size() > 1; q--)
    {
        DrawLib::drawLineAlpha(__oldPositions.at(q), __oldPositions.at(q-1),gBlack, q/(float)__oldPositions.size());
    }
    
    #endif
    
    #ifdef DRAW_ANNOTATIONS
    
    for (int i=0; ( _waypoints.size() > 1 ) && (i < (_waypoints.size() - 1)); i++)
    {
        if ( _gEngine->isAgentSelected(this) )
        {
            DrawLib::drawLine(_waypoints.at(i), _waypoints.at(i+1), gYellow);
        }
        else
        {
            //DrawLib::drawLine(_waypoints.at(i), _waypoints.at(i+1), gBlack);
        }
    }
    
    for (int i=0; i < (_waypoints.size()); i++)
    {
        DrawLib::drawStar(_waypoints.at(i), Util::Vector(1,0,0), 0.34f, gBlack);
    }
    
    for (int i=0; ( _midTermPath.size() > 1 ) && (i < (_midTermPath.size() - 1)); i++)
    {
        if ( _gEngine->isAgentSelected(this) )
        {
            DrawLib::drawLine(_midTermPath.at(i), _midTermPath.at(i+1), gMagenta);
        }
        else
        {
            // DrawLib::drawLine(_midTermPath.at(i), _midTermPath.at(i+1), gCyan);
        }
    }
    
    DrawLib::drawLine(position(), this->_currentLocalTarget, gGray10);
    DrawLib::drawStar(this->_currentLocalTarget+Util::Vector(0,0.001,0), Util::Vector(1,0,0), 0.24f, gGray10);
    
    /*
    // draw normals and closest points on walls
    std::set<SteerLib::ObstacleInterface * > tmp_obs = gEngine->getObstacles();
    
    for (std::set<SteerLib::ObstacleInterface * >::iterator tmp_o = tmp_obs.begin();  tmp_o != tmp_obs.end();  tmp_o++)
    {
        Util::Vector normal = calcWallNormal( *tmp_o );
        std::pair<Util::Point,Util::Point> line = calcWallPointsFromNormal(* tmp_o, normal);
        Util::Point midpoint = Util::Point((line.first.x+line.second.x)/2, ((line.first.y+line.second.y)/2)+1,
        (line.first.z+line.second.z)/2);
        DrawLib::drawLine(midpoint, midpoint+normal, gGreen);
        
        // Draw the closes point as well
        std::pair<float, Util::Point> min_stuff = minimum_distance(line.first, line.second, position());
        DrawLib::drawStar(min_stuff.second, Util::Vector(1,0,0), 0.34f, gGreen);
    }
    */
    
    #endif
    
    #endif
}