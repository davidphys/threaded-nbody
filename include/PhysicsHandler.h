#ifndef PHYSICSHANDLER_H
#define PHYSICSHANDLER_H
#include "glm/glm.hpp"
#include <vector>
#include "phystructs.h"
#include "easytime.h"
#include "QuadNode.h"
#include "GridHandler.h"



/** PhysicsHandler
The PhysicsHandler class can take in a bunch of point masses (added individually through the addMass function),
calculate their interaction forces (based on a hut-tree estimation, actual force depends on whatever is in
the QuadNode::calculateForces function), and integrate them/step time based on the update function.
*/
class PhysicsHandler
{
	double G;
	double collK;
	double collDampening;
	PhysicsHandlerTiming phytime;
	EasyTimer timer;
public:
	std::vector<PointMass> masses;


	/**creates a physics handler. G is the gravitational constant in GMm/r^2, collision_spring_constant is the k in F=-kx (so,
        there's a hookean spring collision force), collision_damping is a force which acts against velocity, in the direction of
        the axis of collision. the constant passed in is the dampening force, which is the constant of proportionality between the
        relative velocity along the axis collision, and the supplied force.*/
	PhysicsHandler(double G, double collision_spring_constant, double collision_dampening);
	~PhysicsHandler();

    ///Changes to the center of mass frame of the system    
	void zeroMomentumAndCM();

    ///Adds a mass to the system. The spring force only acts inside the particle's radius.
	int addMass(glm::dvec2 pos, double mass, double radius, glm::dvec2 velocity);

    ///Calculates forces and adds them to PointMass::springforce and PointMass::gravforce
	void calcSpringForces();
	void calcGravForces();

	///integrates the particles accelerations
	void update(double timestep);

    ///returns timing info
	const PhysicsHandlerTiming getTiming() const;


	///Returns the sum of momentum (mass1*vel1+...) of the entire system.
	glm::dvec2 getMomentum() const;
	///Returns the sum of momentum (mass1*vel1+...) of the entire system.
	double getMass() const;
	double getAngularMomentum() const;


    //Helper functions
	glm::dvec2 getMassPosSum() const;
	double recursiveSumMass(int from, int to, int minimum) const;
    glm::dvec2 recursiveSumMomentum(int from, int to, int minimum) const;
    glm::dvec2 recursiveSumPositionMass(int from, int to, int minimum) const;
	double recursiveAngularMomentum(int from, int to, int minimum) const;

	///gets the position of a particular mass
	glm::dvec2 getMassPos(int num) const;
};


#endif // PHYSICSHANDLER_H

