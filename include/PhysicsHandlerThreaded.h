#ifndef PHYSICSHANDLERTHREADED_H
#define PHYSICSHANDLERTHREADED_H
#include "glm/glm.hpp"
#include <vector>
#include "phystructs.h"
#include "easytime.h"
#include "QuadNode.h"
#include "GridHandler.h"
#include "SDL2/SDL_thread.h"

/** There are a few main tasks that need threading:
    -   filling the QuadNode/GridHandler, which is done on a single thread separate from the graphics/interaction
    -   calculating the forces using the filled-in data structures. This is done on as many threads as possible.

Both of these tasks are handled by the PhysicsThreadDispatcher function, which dispatches the threads
that run the functions "quadGridFillerThreadFunction" and "physicsThreadFunction". 
 */


class PhysicsThreadDispatcher;

struct PhysicsThreadData{
    SDL_Thread *thread;
    PhysicsThreadDispatcher *parent;
    TimeType timestarted;
    TimeType timeended;
    int particleStartRange;
    int particleEndRange;
};

struct QuadGridFillerThreadData{
    SDL_Thread *thread;
    PhysicsThreadDispatcher *parent;
};

int quadGridFillerThreadFunction(void* data);
int physicsThreadFunction(void* data);

//This class should handle all of the needed multithreading.
class PhysicsThreadDispatcher{
    double G;
    double collK;
    double collDampening;
    TimeType present; //access using getPresent(). Set using setPresent().
    SDL_sem *presentlock;

    std::vector<PhysicsThreadData> threads;
    SDL_sem *threadsrunning;

    QuadGridFillerThreadData quadgridthread;

    QuadNode *quad;
    GridHandler *grid;

    std::vector<PointMass> masses;

    int nthreads;

    SDL_Thread *fillthread(); 

    void destructQuadGrid();
    void destructThreads();
    void fillGrid();
    void fillQuad();

    public:
    PhysicsThreadDispatcher(double G, double collK, double collDampening,const std::vector<PointMass>& masses,int nthreads);
    ~PhysicsThreadDispatcher();

    /* Mode 0: quadgrids need filling.
     * Mode 1: quadgrids currently updating.
     * Mode 2: physics needs running.
     * Mode 3: physics currently updating.
     * Mode 4: physics done.
     * */
    int getMode();



    void dispatch();
    void dispatchQuadGrid();

    TimeType getPresent();
    void setPresent(TimeType t);

    bool running();

    //The following two functions are only safe if "running" returns false.
    std::vector<double> getTiming();
    const std::vector<PointMass>& getMasses();

    friend int physicsThreadFunction(void *data);
    friend int quadGridFillerThreadFunction(void *data);
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
    void replaceMassList(std::vector<PointMass> massesNew);

	PhysicsHandlerThreadedTiming getTiming() const;

	glm::dvec2 getMomentum() const;
	double getMass() const;
	double getAngularMomentum() const;

    const PointMass& getPointMass(int i) const; 
    const std::vector<PointMass>& getPointMassList() const;
    int numParticles() const;
};
#endif //PHYSICSHANDLERTHREADED_H
