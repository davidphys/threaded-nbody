

#include "GridHandler.h"
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
		gridmap=new gridmaptype(expectedParticles);

	//if the map is instantiated, insert an element with a key of type std::pair<int,int>, representing integer location coordinates.
	//(the particle's (x,y) position is mapped to (floor(x*c),floor(y*c)).
	//If no list has been created to store particles at the grid location, then
	std::pair<int,int> key=std::make_pair((int)floor(arg->position.x*multiplier),(int)floor(arg->position.y*multiplier));
	auto f=gridmap->find(key);
	if(f==gridmap->end()){
		gridmap->insert(std::make_pair(key,std::vector<PointMass*>(1,arg)));
	} else {
		f->second.push_back(arg);
	}
}
void GridHandler::calculateForces(PointMass *arg, const double& K, const double& damping) const{
	int x0=(int)floor(arg->position.x*multiplier);
	int y0=(int)floor(arg->position.y*multiplier);

	//iterate over the 9x9 grid around the argument's position
	for(int x=-1;x<=1;x++){
		for(int y=-1;y<=1;y++){
			auto f=gridmap->find(std::make_pair(x0+x,y0+y));
			if(f!=gridmap->end()){
				//if there's a vector, iterate over it and calculate each resulting force.
				for(size_t i=0; i<f->second.size(); i++) {
					PointMass *val=f->second[i];

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
