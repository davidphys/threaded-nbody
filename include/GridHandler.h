#ifndef GRIDHANDLER_H 
#define GRIDHANDLER_H

#include "phystructs.h"
#include <vector>
#include <iostream>
#include <unordered_map>
#include <boost/functional/hash.hpp>

/** GridHandler
 * takes in a bunch of Point Masses and calculates short-distance forces, such as spring-like interactions,
 * by using a hash table and a grid system, where only interactions between masses in adjacent cells are
 * calculated.
 *
 * */
typedef std::unordered_map<std::pair<int,int>,std::vector<PointMass*>,boost::hash<std::pair<int,int>>> gridmaptype;
class GridHandler
{
	gridmaptype *gridmap;
	double multiplier;
	int expectedParticles;
public:
	GridHandler(double positionMultiplier, int expectedParticles);
	~GridHandler();
	void clearPoints();
	void addPointMass(PointMass *arg);
	void calculateForces(PointMass *arg, const double& K, const double& damping) const;
};
#endif
