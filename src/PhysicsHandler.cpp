#include <iostream>
#include "PhysicsHandler.h"


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
