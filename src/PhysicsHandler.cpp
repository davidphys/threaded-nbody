#include <iostream>
#include "PhysicsHandler.h"
#include "GridHandler.h"
#include "QuadNode.h"


/******************************************************************************* 
 *                           PhysicsThreadDispatcher                           *
 *******************************************************************************/
///////////////Friend function

int physicsThreadFunction(void *data) {
    PhysicsThreadData *thread=(PhysicsThreadData *)data;
    PhysicsThreadDispatcher *parent=thread->parent;

    for(size_t i=thread->particleStartRange;i<thread->particleEndRange;i++){
        PointMass *p=&(parent->masses.at(i));
        p->gravforce=glm::dvec2(0,0);
        parent->quad->calculateForces(p,parent->G);
        p->springforce=glm::dvec2(0,0);
        parent->grid->calculateForces(p,parent->collK,parent->collDampening);
    }


    thread->timeended=parent->getPresent();
    SDL_SemWait(parent->threadsrunning);
    return 0;
}

///////////////private
void PhysicsThreadDispatcher::fillGrid(){
    if(grid!=nullptr){
        std::cerr<<"Error: fillGrid called when PhysicsThreadDispatcher grid already filled"<<std::endl;
        return;
    }

    grid=new GridHandler(0.6/masses[0].radius,masses.size());
    for(size_t n=0;n<masses.size();n++)
        grid->addPointMass(&masses.at(n));
}
void PhysicsThreadDispatcher::fillQuad(){
    if(quad!=nullptr){
        std::cerr<<"Error: fillQuad called when PhysicsThreadDispatcher quad already filled"<<std::endl;
        return;
    }

	double left=masses.at(0).position.x;
	double right=masses.at(0).position.x;
	double top=masses.at(0).position.y;
	double bottom=masses.at(0).position.y;
	for(size_t n=0;n<masses.size();n++)
	{
		if(masses.at(n).position.x<left)
			left=masses.at(n).position.x;

		if(masses.at(n).position.y>top)
			top=masses.at(n).position.y;

		if(masses.at(n).position.x>right)
			right=masses.at(n).position.x;

		if(masses.at(n).position.y<bottom)
			bottom=masses.at(n).position.y;
	}

    quad=new QuadNode(glm::dvec2(left,top),glm::dvec2(right,bottom));

    for(size_t n=0;n<masses.size();n++)
        quad->addPointMass(&masses.at(n));
}
void PhysicsThreadDispatcher::destructQuadGrid(){
    if(grid!=nullptr)
        delete grid;
    grid=nullptr;
    if(quad!=nullptr)
        delete quad;
    quad=nullptr;
}
void PhysicsThreadDispatcher::destructThreads(){
    for(int i=0;i<threads.size();i++){
        if(threads[i].thread!=nullptr){
            int ret=0;
            SDL_WaitThread(threads[i].thread,&ret);
            threads[i].thread=nullptr;
        }
    }
    threads=std::vector<PhysicsThreadData>();

    if(presentlock!=nullptr)
        SDL_DestroySemaphore(presentlock);
    presentlock=nullptr;
    if(threadsrunning!=nullptr)
        SDL_DestroySemaphore(threadsrunning);
    threadsrunning=nullptr;
}
///////////////public
PhysicsThreadDispatcher::PhysicsThreadDispatcher(double G, double collK, double collDampening,const std::vector<PointMass>& masses,int nthreads)
    : G(G),collK(collK),collDampening(collDampening), present(),presentlock(nullptr), threads(), threadsrunning(nullptr), quad(nullptr), grid(nullptr),
    masses(masses), nthreads(nthreads) { 
    if(masses.size()==0)
        std::cerr<<"Error: PhysicsThreadDispatcher created with an empty masses array"<<std::endl;
}
PhysicsThreadDispatcher::~PhysicsThreadDispatcher() {
    destructThreads();
    destructQuadGrid();
}

void PhysicsThreadDispatcher::dispatch(){
    if(running()){
        std::cerr<<"Error: PhysicsThreadDispatcher::dispatch called when already dispatched"<<std::endl;
        return;
    }
    
    //Safely ensure no threads are running and all the semaphores are uninitialized.
    destructThreads();
    destructQuadGrid();

    //Construct the needed data structures;
    fillGrid();
    fillQuad();

    presentlock=SDL_CreateSemaphore(1);
    threadsrunning=SDL_CreateSemaphore(nthreads);
    double stepsize=double(masses.size())/nthreads;
    for(int i=0;i<nthreads;i++){
        PhysicsThreadData newthread;
        newthread.parent=this;
        newthread.particleStartRange=int(stepsize*i);
        newthread.particleEndRange=int(stepsize*(i+1));
        threads.push_back(newthread);
    }

    for(int i=0;i<nthreads;i++){
        threads[i].timestarted=getPresent();
        threads[i].thread=SDL_CreateThread(physicsThreadFunction,"GravityWorker",(void *)(&(threads[i])));
    }
}


TimeType PhysicsThreadDispatcher::getPresent(){
    if(presentlock!=nullptr)
        SDL_SemWait(presentlock);
    TimeType t2=present;
    if(presentlock!=nullptr)
        SDL_SemPost(presentlock);
    return t2;
}
void PhysicsThreadDispatcher::setPresent(TimeType t){
    if(presentlock!=nullptr)
        SDL_SemWait(presentlock);
    present=t;
    if(presentlock!=nullptr)
        SDL_SemPost(presentlock);
}


bool PhysicsThreadDispatcher::running(){
    return (threadsrunning!=nullptr)&&(SDL_SemValue(threadsrunning)>0);
}


//The following two functions are only safe if "running" returns false.
std::vector<double> PhysicsThreadDispatcher::getTiming(){
    std::vector<double> times;
    for(auto& t:threads)
    {
        times.push_back((DurationType(t.timeended-t.timestarted)).count());
    }
    return times;
}
const std::vector<PointMass>& PhysicsThreadDispatcher::getMasses(){return masses;}


/******************************************************************************* 
 *                           PhysicsHandlerThreaded                            *
 *******************************************************************************/

//////////private:

glm::dvec2 PhysicsHandlerThreaded::getMassPosSum() const {
	return recursiveSumPositionMass(0,masses.size(),50);
}
double PhysicsHandlerThreaded::recursiveSumMass(int from, int to, int minimum) const{
    if(from>to)
        return 0;
	if(to-from<minimum){
        double sum=0;
        for(int i=from;i<to;i++){
            sum+=masses.at(i).mass;
        }
        return sum;
	} else {
        return recursiveSumMass(from,(from+to)/2,minimum)+recursiveSumMass((from+to)/2,to,minimum);
	}
}
glm::dvec2 PhysicsHandlerThreaded::recursiveSumMomentum(int from, int to, int minimum) const{
    if(from>to)
        return glm::dvec2(0,0);
	if(to-from<=minimum){
        glm::dvec2 sum=glm::dvec2(0,0);
        for(int i=from;i<to;i++){
            sum+=masses.at(i).velocity*masses.at(i).mass;
        }
        return sum;
	} else {
        return recursiveSumMomentum(from,(from+to)/2,minimum)+recursiveSumMomentum((from+to)/2,to,minimum);
	}
}
glm::dvec2 PhysicsHandlerThreaded::recursiveSumPositionMass(int from, int to, int minimum) const{
    if(from>to)
        return glm::dvec2(0,0);
	if(to-from<minimum){
        glm::dvec2 sum=glm::dvec2(0,0);
        for(int i=from;i<to;i++){
            sum+=masses.at(i).position*masses.at(i).mass;
        }
        return sum;
	} else {
        return recursiveSumPositionMass(from,(from+to)/2,minimum)+recursiveSumPositionMass((from+to)/2,to,minimum);
	}
}
double PhysicsHandlerThreaded::recursiveAngularMomentum(int from, int to, int minimum) const{
    if(from>to)
        return 0;
	if(to-from<minimum){
		double sum=0;
        for(int i=from;i<to;i++){
			//m*(x*vy-y*vx)
			const PointMass &p=masses.at(i);
            sum+=masses.at(i).mass*(p.position.x*p.velocity.y-p.velocity.x*p.position.y);
        }
        return sum;
	} else {
        return recursiveAngularMomentum(from,(from+to)/2,minimum)+recursiveAngularMomentum((from+to)/2,to,minimum);
	}
}

//////////public:

PhysicsHandlerThreaded::PhysicsHandlerThreaded(double G, double collision_spring_constant,double collision_dampening) :
    G(G),collK(collision_spring_constant),collDampening(collision_dampening),
    phytime(),timer(), masses(),phythreadhandler(nullptr)
{}
PhysicsHandlerThreaded::~PhysicsHandlerThreaded(){
    if(phythreadhandler!=nullptr)
        delete phythreadhandler;
}


void PhysicsHandlerThreaded::update(double timestep){
    //Update the multithreaded physics updater's low-resolution but thread-safe clock
    if(phythreadhandler!=nullptr)
        phythreadhandler->setPresent(easytime::getPresent());

    //Do nothing if the computation threads are running
    if(updating())
        return;
    
    //If the computation has finished but the thread handler has not been cleaned up yet,
    //clean up/finish the computation.
    if(phythreadhandler!=nullptr){

        std::cout<<"Finish Computation timing: "<<std::endl;
        EasyTimer timer2;//timer2
        timer2.tick();//timer2

        masses=phythreadhandler->getMasses();
        std::vector<double> threadtimes=phythreadhandler->getTiming();

        double timingsum=0;
        for(size_t i=0;i<threadtimes.size();i++){
            timingsum+=threadtimes[i];
        }
        std::cout<<"\tTotal Thread time: "<<timingsum<<std::endl;



        delete phythreadhandler;
        std::cout<<"\tThread Dispatcher Deletion: "<<timer2.tick()<<std::endl; //timer2
        phythreadhandler=nullptr;
        phytime.realTime=timer.tick(); //timer shouldn't have ticked since the update called when phythreadhandler was created.
        

        //Integration step [done on the single thread] 
        timer.tick();
        for(size_t n=0;n<masses.size();n++)
        {
            PointMass &m=masses.at(n);
            m.position+=(m.velocity+.5*timestep*(m.springforce+m.gravforce)/m.mass)*timestep;
            m.velocity+=timestep*(m.springforce+m.gravforce)/m.mass;
        }
        int dist=5000;

        for(size_t n=0;n<masses.size();n++){
            PointMass &m=masses.at(n);
            if(m.position.x*m.position.x+m.position.y*m.position.y>dist*dist)
            {    masses.erase(masses.begin()+n);
            n--;}

        }
        phytime.timestepping=timer.tick();
        std::cout<<"\tparticle integration: "<<timer2.tick()<<std::endl; //timer2
    } else { //In this case, we need to start a new computation.
        phythreadhandler=new PhysicsThreadDispatcher(G, collK, collDampening,masses,8);
        timer.tick();
        phythreadhandler->setPresent(easytime::getPresent());
        phythreadhandler->dispatch();
    }
    
}
bool PhysicsHandlerThreaded::updating(){
    return (phythreadhandler!=nullptr)&&(phythreadhandler->running());
}


void PhysicsHandlerThreaded::zeroMomentumAndCM() {
    double mass=recursiveSumMass(0,masses.size(),50);
    glm::dvec2 posdiff=recursiveSumPositionMass(0,masses.size(),50)*1.0/mass;
    glm::dvec2 momentumdiff=recursiveSumMomentum(0,masses.size(),50)*1.0/mass;
	for(size_t n=0;n<masses.size();n++)
	{
		masses.at(n).velocity-=momentumdiff;
		masses.at(n).position-=posdiff;
	}
}
void PhysicsHandlerThreaded::addMass(glm::dvec2 pos, double mass, double radius, glm::dvec2 velocity) {
	PointMass newmass;
	newmass.position=pos;
	newmass.velocity=velocity;
	newmass.gravforce=glm::dvec2(0,0);
	newmass.springforce=glm::dvec2(0,0);
	newmass.mass=mass;
	newmass.radius=radius;
	masses.push_back(newmass);
}


PhysicsHandlerThreadedTiming PhysicsHandlerThreaded::getTiming() const{
	return phytime;
}


glm::dvec2 PhysicsHandlerThreaded::getMomentum() const
{
	return recursiveSumMomentum(0,masses.size(),50);
}
double PhysicsHandlerThreaded::getMass() const{
	return recursiveSumMass(0,masses.size(),50);
}
double PhysicsHandlerThreaded::getAngularMomentum() const{
	return recursiveAngularMomentum(0,masses.size(),50);
}


const PointMass& PhysicsHandlerThreaded::getPointMass(int i) const {
    return masses.at(i);
}
int PhysicsHandlerThreaded::numParticles() const {
    return masses.size();
}





//single threaded
/******************************************************************************* 
 *                                 PhysicsHandler                              *
 *******************************************************************************/




PhysicsHandler::PhysicsHandler(double G, double collision_spring_constant, double collision_dampening) :
G(G),collK(collision_spring_constant),collDampening(collision_dampening),phytime(PhysicsHandlerTiming()),timer(EasyTimer())
{
	masses=std::vector<PointMass>();
}
PhysicsHandler::~PhysicsHandler()
{
}
glm::dvec2 PhysicsHandler::getMomentum() const
{
	/*glm::dvec2 ms(0,0);
	for(size_t n=0;n<masses.size();n++)
	{
		ms+=masses.at(n).velocity*masses.at(n).mass;
	}
	return ms;*/
	return recursiveSumMomentum(0,masses.size(),50);
}
glm::dvec2 PhysicsHandler::getMassPosSum() const
{
	/*glm::dvec2 ms(0,0);
	for(size_t n=0;n<masses.size();n++)
	{
		ms+=masses.at(n).velocity*masses.at(n).mass;
	}
	return ms;*/
	return recursiveSumPositionMass(0,masses.size(),50);
}
double PhysicsHandler::getMass() const{
	return recursiveSumMass(0,masses.size(),50);
}
double PhysicsHandler::getAngularMomentum() const{
	return recursiveAngularMomentum(0,masses.size(),50);
}


double PhysicsHandler::recursiveSumMass(int from, int to, int minimum) const{
    if(from>to)
        return 0;
	if(to-from<minimum){
        double sum=0;
        for(int i=from;i<to;i++){
            sum+=masses.at(i).mass;
        }
        return sum;
	} else {
        return recursiveSumMass(from,(from+to)/2,minimum)+recursiveSumMass((from+to)/2,to,minimum);
	}
}
glm::dvec2 PhysicsHandler::recursiveSumMomentum(int from, int to, int minimum) const{
    if(from>to)
        return glm::dvec2(0,0);
	if(to-from<=minimum){
        glm::dvec2 sum=glm::dvec2(0,0);
        for(int i=from;i<to;i++){
            sum+=masses.at(i).velocity*masses.at(i).mass;
        }
        return sum;
	} else {
        return recursiveSumMomentum(from,(from+to)/2,minimum)+recursiveSumMomentum((from+to)/2,to,minimum);
	}
}
glm::dvec2 PhysicsHandler::recursiveSumPositionMass(int from, int to, int minimum) const{
    if(from>to)
        return glm::dvec2(0,0);
	if(to-from<minimum){
        glm::dvec2 sum=glm::dvec2(0,0);
        for(int i=from;i<to;i++){
            sum+=masses.at(i).position*masses.at(i).mass;
        }
        return sum;
	} else {
        return recursiveSumPositionMass(from,(from+to)/2,minimum)+recursiveSumPositionMass((from+to)/2,to,minimum);
	}
}
double PhysicsHandler::recursiveAngularMomentum(int from, int to, int minimum) const{
    if(from>to)
        return 0;
	if(to-from<minimum){
		double sum=0;
        for(int i=from;i<to;i++){
			//m*(x*vy-y*vx)
			const PointMass &p=masses.at(i);
            sum+=masses.at(i).mass*(p.position.x*p.velocity.y-p.velocity.x*p.position.y);
        }
        return sum;
	} else {
        return recursiveAngularMomentum(from,(from+to)/2,minimum)+recursiveAngularMomentum((from+to)/2,to,minimum);
	}
}




void PhysicsHandler::zeroMomentumAndCM()
{
    double mass=recursiveSumMass(0,masses.size(),50);
    glm::dvec2 posdiff=recursiveSumPositionMass(0,masses.size(),50)*1.0/mass;
    glm::dvec2 momentumdiff=recursiveSumMomentum(0,masses.size(),50)*1.0/mass;
	for(size_t n=0;n<masses.size();n++)
	{
		masses.at(n).velocity-=momentumdiff;
		masses.at(n).position-=posdiff;
	}
}
int PhysicsHandler::addMass(glm::dvec2 pos, double mass, double radius, glm::dvec2 velocity)
{
	PointMass newmass;
	newmass.position=pos;
	newmass.velocity=velocity;
	newmass.gravforce=glm::dvec2(0,0);
	newmass.springforce=glm::dvec2(0,0);
	newmass.mass=mass;
	newmass.radius=radius;
	masses.push_back(newmass);
	return masses.size()-1;
}

const PhysicsHandlerTiming PhysicsHandler::getTiming() const{
	return phytime;
}
void PhysicsHandler::calcSpringForces()
{
	if(masses.size()<=0)
		return;

	{
		timer.tick(); //Ready timer

		GridHandler grid=GridHandler(0.6/masses[0].radius,masses.size());
		for(size_t n=0;n<masses.size();n++)
			grid.addPointMass(&masses.at(n));

		phytime.hashCreate=timer.tick(); //Record elapsed time for building the grid

		for(size_t n=0;n<masses.size();n++){
			masses.at(n).springforce=glm::dvec2(0,0);
			grid.calculateForces(&masses.at(n),collK,collDampening);
		}

		phytime.hashPhy=timer.tick(); //Record elapsed time for calculating the physics

	}
	phytime.hashDelete=timer.tick(); //Record elapsed time for deleting the grid

}
void PhysicsHandler::calcGravForces()
{
	if(masses.size()<=0)
		return;
	double left=masses.at(0).position.x;
	double right=masses.at(0).position.x;
	double top=masses.at(0).position.y;
	double bottom=masses.at(0).position.y;
	for(size_t n=0;n<masses.size();n++)
	{
		if(masses.at(n).position.x<left)
			left=masses.at(n).position.x;

		if(masses.at(n).position.y>top)
			top=masses.at(n).position.y;

		if(masses.at(n).position.x>right)
			right=masses.at(n).position.x;

		if(masses.at(n).position.y<bottom)
			bottom=masses.at(n).position.y;
	}

	
	timer.tick();
	{
		QuadNode huttree=QuadNode(glm::dvec2(left,top),glm::dvec2(right,bottom));

		for(size_t n=0;n<masses.size();n++)
			huttree.addPointMass(&masses.at(n));

		phytime.quadCreate=timer.tick();

		for(size_t n=0;n<masses.size();n++){
			masses.at(n).gravforce=glm::dvec2(0,0);
			huttree.calculateForces(&masses.at(n),G);
		}

		phytime.quadPhy=timer.tick();
	}
	phytime.quadDelete=timer.tick();
}

void PhysicsHandler::update(double timestep)
{

	timer.tick();
	for(size_t n=0;n<masses.size();n++)
	{
		PointMass &m=masses.at(n);
		m.position+=(m.velocity+.5*timestep*(m.springforce+m.gravforce)/m.mass)*timestep;
		m.velocity+=timestep*(m.springforce+m.gravforce)/m.mass;
	}
	int dist=5000;

	for(size_t n=0;n<masses.size();n++){
		PointMass &m=masses.at(n);
		if(m.position.x*m.position.x+m.position.y*m.position.y>dist*dist)
        {    masses.erase(masses.begin()+n);
        n--;}

	}
	phytime.timestepping=timer.tick();
}




glm::dvec2 PhysicsHandler::getMassPos(int num) const{return masses.at(num).position;}
