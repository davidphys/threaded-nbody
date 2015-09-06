#ifndef PHYSTRUCTS_H
#define PHYSTRUCTS_H
#include "glm/glm.hpp"
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


struct PhysicsHandlerThreadedTiming {
    double maxThreadTime;
    double minThreadTime;
    double realTime;
	double timestepping;
	PhysicsHandlerThreadedTiming() : 
		maxThreadTime(0),minThreadTime(0),realTime(0),timestepping(0){
	}
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
#endif
