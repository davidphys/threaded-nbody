#ifndef PHYSICSHANDLER_H
#define PHYSICSHANDLER_H
#include "glm/glm.hpp"
#include <vector>
#include <iostream>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include "easytime.h"
/** PointMass
dumb structure that stores the data of a gravitating particle.
*/
struct PointMass
{

	glm::dvec2 position;
	glm::dvec2 velocity;
	glm::dvec2 gravforce;
	glm::dvec2 springforce;
	double mass;
	double radius;
};
/** QuadNode
The QuadNode class takes in PointMass* pointers and creates a quadtree data
structure with exactly one PointMass* per node.

"tl" and "br" are the top-left and bottom-right location vectors of the quadtree node, respectively.

addPointMass puts a given pointmass into the quadtree, possibly creating new subnodes in the process.

calculateForces recursively calls any substructure, and modifies the passed in PointMass by calculating/
adding to the PointMass::force vector based on positions/what-not.
*/

class QuadNode
{
	glm::dvec2 tl; //top-left
	glm::dvec2 br; //bottom-right
	QuadNode *subnodes[2][2]; //array of subnodes. Either all are null or none are null.
	PointMass *val; //The pointmass stored in the array. Possibly null.
	glm::dvec2 masspos; //The sum of the masses in this node and/or any subnodes, times the positions. (mass1*x1+mass2*x2+mass3*x3+...)
	double mass; //the sum of masses (mass1+mass2+...)
	//(center of mass = masspos/mass )


	void createSubNodes(); //fills the subnodes array.
public:
    ///Creates a new QuadNode. tl is top-left, br is bottom-right. Adding PointMasses outside these bounds will cause a crash!
	QuadNode(glm::dvec2 tl, glm::dvec2 br);
	~QuadNode();
	///Adds a point mass to the quadtree.
	void addPointMass(PointMass *arg);
	/**Uses Barnes-Hut calculation to approximate the gravitational (and spring) forces on a passed in particle, according to the
	passed in physical constants (see PhysicsHandler's constructor)*/
	void calculateForces(PointMass *arg, const double& G);
};

/** GridHandler
 * takes in a bunch of Point Masses and calculates short-distance forces, such as spring-like interactions,
 * by using a hash table and a grid system, where only interactions between masses in adjacent cells are
 * calculated.
 *
 * */
class GridHandler
{
	std::unordered_map<std::pair<int,int>,std::vector<PointMass*>,boost::hash<std::pair<int,int>>> *gridmap;
	double multiplier;
	int expectedParticles;
public:
	GridHandler(double positionMultiplier, int expectedParticles);
	~GridHandler();
	void clearPoints();
	void addPointMass(PointMass *arg);
	void calculateForces(PointMass *arg, const double& K, const double& damping);
};


struct PhysicsHandlerTiming {
	double quadCreate;
	double quadPhy;
	double quadDelete;
	double hashCreate;
	double hashPhy;
	double hashDelete;
	double timestepping;
	PhysicsHandlerTiming() : 
		quadCreate(0),quadPhy(0),quadDelete(0),hashCreate(0),hashPhy(0),hashDelete(0),timestepping(0){
	}
};

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

	const PhysicsHandlerTiming getTiming() const;
	/**creates a physics handler. G is the gravitational constant in GMm/r^2, collision_spring_constant is the k in F=-kx (so,
        there's a hookean spring collision force), collision_damping is a force which acts against velocity, in the direction of
        the axis of collision. the constant passed in is the dampening force, which is the constant of proportionality between the
        relative velocity along the axis collision, and the supplied force.*/
	PhysicsHandler(double G, double collision_spring_constant, double collision_dampening);
	~PhysicsHandler();

	///Returns the sum of momentum (mass1*vel1+...) of the entire system.
	glm::dvec2 getMomentum();
	///Returns the sum of momentum (mass1*vel1+...) of the entire system.
	double getMass();
	double getAngularMomentum();

	glm::dvec2 getMassPosSum();
	void zeroMomentumAndCM();
	double recursiveSumMass(int from, int to, int minimum);
    glm::dvec2 recursiveSumMomentum(int from, int to, int minimum);
    glm::dvec2 recursiveSumPositionMass(int from, int to, int minimum);
	double recursiveAngularMomentum(int from, int to, int minimum);

    ///Adds a mass to the system. The spring force only acts inside the particle's radius.
	int addMass(glm::dvec2 pos, double mass, double radius, glm::dvec2 velocity);


	void calcSpringForces();
	void calcGravForces();

	///integrates the particles accelerations
	void update(double timestep);

	///gets the position of a particular mass
	glm::dvec2 getMassPos(int num);
};


#endif // PHYSICSHANDLER_H

