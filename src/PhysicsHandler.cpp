#include "PhysicsHandler.h"
#include <iostream>
/*class QuadNode
{
	glm::dvec2 tl;
	glm::dvec2 br;
	QuadNode *subnodes[2][2];
	PointMass *val;
public:
	QuadNode(glm::dvec2 tl, glm::dvec2 br);
	~QuadNode();
	void addPointMass(PointMass arg);
	void calculateForces(PointMass *arg);
};*/


void QuadNode::calculateForces(PointMass *arg, const double& G)
{

	if(subnodes[0][0]!=NULL)
	{
		/*

		glm::dvec2 pt = masspos/mass;
		register double diffw = tl.x - br.x; //(tl.x-br.x)*(tl.x-br.x)
		if (diffw >= (abs(pt.x-arg->position.x)+abs(pt.y-arg->position.y))*0.9) */
		glm::dvec2 pt=masspos/mass; 
		glm::dvec2 diff=pt-arg->position;
		double d=diff.x*diff.x+diff.y*diff.y;
		register double diffw=(tl.x-br.x);
		if (diffw*diffw >= d*.7) 
		{
			for (int n = 0; n<2; n++)
				for (int m = 0; m<2; m++)
					subnodes[m][n]->calculateForces(arg, G);
		} else
		{
			glm::dvec2 diff = pt - arg->position;
			double d = diff.x*diff.x + diff.y*diff.y;
			double force = G*arg->mass*mass / d;
			d = sqrt(d);
			if (d>0.5)
				arg->gravforce += diff*force / d;
		}
	} 
	else if(val!=NULL)
	{
		glm::dvec2 diff=val->position-arg->position;
		double d=diff.x*diff.x+diff.y*diff.y;
		double force=G*arg->mass*val->mass/d;
		d=sqrt(d);
		if(d>0.5)
			arg->gravforce+=diff*force/d;
	}
}
QuadNode::QuadNode(glm::dvec2 tl, glm::dvec2 br) : tl(tl),br(br),val(NULL),masspos(0,0),mass(0)
{
	for(int n=0;n<2;n++)
		for(int m=0;m<2;m++)
			subnodes[n][m]=NULL;
}
QuadNode::~QuadNode()
{

	for(int n=0;n<2;n++)
		for(int m=0;m<2;m++)
			if(subnodes[n][m])
				delete subnodes[n][m];
}
void QuadNode::createSubNodes()
{
	glm::dvec2 mid=(tl+br)*.5;
	glm::dvec2 midleft=glm::dvec2(tl.x,mid.y);
	glm::dvec2 midtop=glm::dvec2(mid.x,tl.y);
	glm::dvec2 midright=glm::dvec2(br.x,mid.y);
	glm::dvec2 midbottom=glm::dvec2(mid.x,br.y);

	subnodes[0][0]=new QuadNode(tl,mid);
	subnodes[0][1]=new QuadNode(midtop,midright);
	subnodes[1][0]=new QuadNode(midleft,midbottom);
	subnodes[1][1]=new QuadNode(mid,br);
}
void QuadNode::addPointMass(PointMass *arg)
{
	if(val==NULL)
	{	if(subnodes[0][0]==NULL)//no current mass & no subnodes
		{
			val=arg;
		} else { //has subnodes
			masspos+=arg->position*arg->mass;
			mass+=arg->mass;
			glm::dvec2 mid=(tl+br)*.5;
			int x=arg->position.x>mid.x;
			int y=arg->position.y<mid.y;
			subnodes[y][x]->addPointMass(arg);
		}
	}
	else {
		if(val->position==arg->position)
			return;
		PointMass *valtemp=val;
		val=NULL;
		createSubNodes();
		masspos=glm::dvec2(0,0);
		mass=0;
		addPointMass(valtemp);
		addPointMass(arg);
	}
}



typedef std::unordered_map<std::pair<int,int>,std::vector<PointMass*>,boost::hash<std::pair<int,int>>> GridMapType ;
typedef std::unordered_map<std::pair<int,int>,std::vector<PointMass*>,boost::hash<std::pair<int,int>>>::iterator GridMapIterator ;

//	std::unordered_map<std::pair<int,int>,std::vector<PointMass*>> *gridmap;
//	double multiplier;
GridHandler::GridHandler(double positionMultiplier,int expectedParticles) :
	gridmap(NULL),
	multiplier(positionMultiplier),
	expectedParticles(expectedParticles) {


}
GridHandler::~GridHandler(){
	clearPoints();
}
void GridHandler::clearPoints(){
	if(gridmap){
		delete gridmap;
		gridmap=NULL;
	}
}

void GridHandler::addPointMass(PointMass *arg) {
	if(!gridmap)
		gridmap=new GridMapType(expectedParticles);

	//if the map is instantiated, insert an element with a key of type std::pair<int,int>, representing integer location coordinates.
	//(the particle's (x,y) position is mapped to (floor(x*c),floor(y*c)).
	//If no list has been created to store particles at the grid location, then
	std::pair<int,int> key=std::make_pair((int)floor(arg->position.x*multiplier),(int)floor(arg->position.y*multiplier));
	GridMapIterator f=gridmap->find(key);
	if(f==gridmap->end()){
		gridmap->insert(std::make_pair(key,std::vector<PointMass*>(1,arg)));
	} else {
		f->second.push_back(arg);
	}
}
void GridHandler::calculateForces(PointMass *arg, const double& K, const double& damping) {
	int x0=(int)floor(arg->position.x*multiplier);
	int y0=(int)floor(arg->position.y*multiplier);

	//iterate over the 9x9 grid around the argument's position
	for(int x=-1;x<=1;x++){
		for(int y=-1;y<=1;y++){
			GridMapIterator f=gridmap->find(std::make_pair(x0+x,y0+y));
			if(f!=gridmap->end()){
				//if there's a vector, iterate over it and calculate each resulting force.
				for(size_t i=0; i<f->second.size(); i++) {
					PointMass *val=f->second.at(i);

					glm::dvec2 diff=val->position-arg->position;
					double d=diff.x*diff.x+diff.y*diff.y;
					if(d<.00001) continue;

					d=sqrt(d);
					diff=diff*1.0/d; //normalize difference

					double overlap=arg->radius+val->radius-d;
					if(overlap>0) //collision
					{
						double force=-K*overlap; //spring force

						//the dot product gives the relative velocity in the direction from arg to val.
						glm::dvec2 relvel=val->velocity-arg->velocity;
						force+=damping*glm::dot(diff,relvel);

						arg->springforce+=diff*force;
					}
				}
			}
		}
	}
}



PhysicsHandler::PhysicsHandler(double G, double collision_spring_constant, double collision_dampening) :
G(G),collK(collision_spring_constant),collDampening(collision_dampening),phytime(PhysicsHandlerTiming()),timer(EasyTimer())
{
	masses=std::vector<PointMass>();
}
PhysicsHandler::~PhysicsHandler()
{
}
glm::dvec2 PhysicsHandler::getMomentum()
{
	/*glm::dvec2 ms(0,0);
	for(size_t n=0;n<masses.size();n++)
	{
		ms+=masses.at(n).velocity*masses.at(n).mass;
	}
	return ms;*/
	return recursiveSumMomentum(0,masses.size(),50);
}
glm::dvec2 PhysicsHandler::getMassPosSum()
{
	/*glm::dvec2 ms(0,0);
	for(size_t n=0;n<masses.size();n++)
	{
		ms+=masses.at(n).velocity*masses.at(n).mass;
	}
	return ms;*/
	return recursiveSumPositionMass(0,masses.size(),50);
}
double PhysicsHandler::getMass() {
	return recursiveSumMass(0,masses.size(),50);
}
double PhysicsHandler::getAngularMomentum(){
	return recursiveAngularMomentum(0,masses.size(),50);
}


double PhysicsHandler::recursiveSumMass(int from, int to, int minimum){
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
glm::dvec2 PhysicsHandler::recursiveSumMomentum(int from, int to, int minimum){
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
glm::dvec2 PhysicsHandler::recursiveSumPositionMass(int from, int to, int minimum){
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
double PhysicsHandler::recursiveAngularMomentum(int from, int to, int minimum){
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
	    /*if(n==0){
	        std::cout<<"Position: "<<m.position.x<<", "<<m.position.y<<std::endl;
	        std::cout<<"Velocity: "<<m.velocity.x<<", "<<m.velocity.y<<std::endl;
	        std::cout<<"Acceleration: "<<m.force.x<<", "<<m.force.y<<std::endl;
	        std::cout<<"radius: "<<m.radius<<std::endl;
	        std::cout<<"mass: "<<m.mass<<std::endl<<std::endl;
	    }*/
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




glm::dvec2 PhysicsHandler::getMassPos(int num){return masses.at(num).position;}
