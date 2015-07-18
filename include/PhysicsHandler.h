#ifndef PHYSICSHANDLER_H
#define PHYSICSHANDLER_H
#include "glm/glm.hpp"
#include <vector>
#include "phystructs.h"
#include "easytime.h"
#include "QuadNode.h"
#include "GridHandler.h"
#include "SDL2/SDL_thread.h"

class PhysicsThreadDispatcher;

struct PhysicsThreadData{
    SDL_Thread *thread;
    PhysicsThreadDispatcher *parent;
    TimeType timestarted;
    TimeType timeended;
    int particleStartRange;
    int particleEndRange;
};

int PhysicsThreadFunction(void *data);
//This class should handle all of the needed multithreading.
class PhysicsThreadDispatcher{
    TimeType present; //access using getPresent(). Set using setPresent().
    SDL_sem *presentlock;

    std::vector<PhysicsThreadData> threads;
    SDL_sem *threadsrunning;

    QuadNode *quad;
    GridHandler *grid;

    std::vector<PointMass> masses;

    int nthreads;

    void destructQuadGrid();
    void destructThreads();
    void fillGrid();
    void fillQuad();

    public:
    PhysicsThreadDispatcher(const std::vector<PointMass>& masses,int nthreads);
    ~PhysicsThreadDispatcher();

    void dispatch();

    TimeType getPresent();
    void setPresent(TimeType t);

    bool running();

    //The following two functions are only safe if "running" returns false.
    std::vector<double> getTiming();
    const std::vector<PointMass>& getMasses();

    friend int PhysicsThreadFunction (void *data);
};

class PhysicsHandlerThreaded{
    double G;
    double collK;
    double collDampening;
    PhysicsHandlerThreadedTiming phytime;
    EasyTimer timer;

    std::vector<PointMass> masses;

    PhysicsThreadDispatcher *phythreadhandler;

	glm::dvec2 getMassPosSum() const;
	double recursiveSumMass(int from, int to, int minimum) const;
    glm::dvec2 recursiveSumMomentum(int from, int to, int minimum) const;
    glm::dvec2 recursiveSumPositionMass(int from, int to, int minimum) const;
	double recursiveAngularMomentum(int from, int to, int minimum) const;

public:
    PhysicsHandlerThreaded(double G, double collision_spring_constant,double collision_dampening);
    ~PhysicsHandlerThreaded();

    //Starts the update threads.
	void update(double timestep);
    bool updating();

    //these two functions should only be called if updating() is false. 
    //They are thread safe, but their actions will be overwritten once the update threads are done. 
    void zeroMomentumAndCM();
	void addMass(glm::dvec2 pos, double mass, double radius, glm::dvec2 velocity);

	PhysicsHandlerThreadedTiming getTiming() const;

	glm::dvec2 getMomentum() const;
	double getMass() const;
	double getAngularMomentum() const;

    const PointMass& getPointMass(int i) const; 
    int numParticles() const;
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


	/**creates a physics handler. G is the gravitational constant in GMm/r^2, collision_spring_constant is the k in F=-kx (so,
        there's a hookean spring collision force), collision_damping is a force which acts against velocity, in the direction of
        the axis of collision. the constant passed in is the dampening force, which is the constant of proportionality between the
        relative velocity along the axis collision, and the supplied force.*/
	PhysicsHandler(double G, double collision_spring_constant, double collision_dampening);
	~PhysicsHandler();


	void zeroMomentumAndCM();






    ///Adds a mass to the system. The spring force only acts inside the particle's radius.
	int addMass(glm::dvec2 pos, double mass, double radius, glm::dvec2 velocity);


	void calcSpringForces();
	void calcGravForces();

	///integrates the particles accelerations
	void update(double timestep);

	const PhysicsHandlerTiming getTiming() const;


	///Returns the sum of momentum (mass1*vel1+...) of the entire system.
	glm::dvec2 getMomentum() const;
	///Returns the sum of momentum (mass1*vel1+...) of the entire system.
	double getMass() const;
	double getAngularMomentum() const;



	glm::dvec2 getMassPosSum() const;
	double recursiveSumMass(int from, int to, int minimum) const;
    glm::dvec2 recursiveSumMomentum(int from, int to, int minimum) const;
    glm::dvec2 recursiveSumPositionMass(int from, int to, int minimum) const;
	double recursiveAngularMomentum(int from, int to, int minimum) const;

	///gets the position of a particular mass
	glm::dvec2 getMassPos(int num) const;
};


#endif // PHYSICSHANDLER_H

