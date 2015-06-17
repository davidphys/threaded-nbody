
#include <iostream>
#include <limits>
#include "PhysicsHandler.h"
#include "glm/glm.hpp"
#include "glm/gtc/random.hpp"
#include <string>
#include <sstream>
#include <time.h>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <chrono>

#include "PointSDL.h"

using namespace screen;

using namespace std;

/*for(int n=0;n<4000;n++){
glm::dvec2 position=glm::dvec2(gen(),gen());
double theta=atan2(position.y,position.x)+1.57079633;
double dist=sqrt(position.x*position.x+position.y*position.y);
double mag=5;
double x=dist;
mag=sqrt(exp(-5.5- 0.000290784593574659*x*x)*x*x);
glm::dvec2 vel=glm::dvec2(cos(theta)*mag+1,sin(theta)*mag);
phy.addMass(position,1,0.9,vel);
}*/

//5.5, 0.000290784593574659

std::string filename(std::string prefix, int num, std::string suffix){
	stringstream ss;
	ss << num;
	std::string x=ss.str();
	while(x.length()<5)
		x="0"+x;
	return prefix+x+suffix;
}

void placeBall(PhysicsHandler& phy,int n,double sdev,double size,double exprand1=100,double exprand2=0,double m=1){

	typedef boost::mt19937                     ENG;    // Mersenne Twister
	typedef boost::normal_distribution<double> DIST;   // Normal Distribution
	typedef boost::variate_generator<ENG, DIST> GEN;    // Variate generator
	ENG eng;
	DIST dist(0, sdev);
	GEN gen(eng, dist);


	for(int i = 0; i<n; i++){
		glm::dvec2 position = glm::dvec2(gen(), gen());
		double theta = atan2(position.y, position.x) + 1.57079633;
		double dist = sqrt(position.x*position.x + position.y*position.y);
		double x = dist;
		double mag = exp(-exprand1-exprand2*x*x)*x;
		/*
		if(dist<0.5){
		mag*=dist;
		} else {
		mag*=0.25/dist;
		}
		*/
		glm::dvec2 vel = glm::dvec2(cos(theta)*mag, sin(theta)*mag);
		phy.addMass(position, m, size, vel);
	}
}
int main(int argc, char** argv)
{

	int nmult=3;
    PhysicsHandler phy=PhysicsHandler(2*nmult,300,10);
	typedef boost::mt19937                     ENG;    // Mersenne Twister
    typedef boost::normal_distribution<double> DIST;   // Normal Distribution
    typedef boost::variate_generator<ENG,DIST> GEN;    // Variate generator
	ENG eng;
	DIST dist(0,80);
	GEN gen(eng,dist);





	TimeType start,end;



	start=std::chrono::system_clock::now();
	end=std::chrono::system_clock::now();
	/*placeBall(phy,2000/nmult,50,0.9,4,0.000007);

	placeBall(phy,12000/nmult,600,0.9,5.0,0.0000005);*/

	placeBall(phy,3000,600,4.9,4,0.000007,10);
	//placeBall(phy,24000,800,0.8,4.0,0.0000005,0.3);
    int frame=0;
    clock_t t=clock();



	int n = 0;
	double createTotal = 0;
	double phyTotal = 0;
	double deleteTotal = 0;
	int zoomParticle=0;

		for(int i=0;i<phy.masses.size();i++) {
			double abcdtmp=sqrt(phy.masses[i].position.x*phy.masses[i].position.x+phy.masses[i].position.y*phy.masses[i].position.y);

			if(750<abcdtmp && abcdtmp<850){
				zoomParticle=i;
				break;
			}
		}

	sInit();
	initDrawStylePoints();
initDrawStyleCCircles();
	setPointColor(0,0,255,200);
	phy.zeroMomentumAndCM();
	double a0=phy.getAngularMomentum();
	while(sLoop())
    {
	//phy.zeroMomentumAndCM();
    //
		for(int a=0;a<1;a++) {
			phy.calcGravForces();
			phy.calcSpringForces();
			phy.update(0.03);
		}
/*
		
		std::cout<<"Hash timing:"<<std::endl;
		std::cout<<"\tCreate:"<<phy.getTiming().hashCreate<<std::endl;
		std::cout<<"\tPhy:"<<phy.getTiming().hashPhy<<std::endl;

		std::cout<<"\tDelete:"<<phy.getTiming().hashDelete<<std::endl;
		std::cout<<"Quad timing:"<<std::endl;
		std::cout<<"\tCreate:"<<phy.getTiming().quadCreate<<std::endl;
		std::cout<<"\tPhy:"<<phy.getTiming().quadPhy<<std::endl;
		std::cout<<"\tDelete:"<<phy.getTiming().quadDelete<<std::endl;
		std::cout<<"Timestep:"<<phy.getTiming().timestepping<<std::endl;
		
        */
		
		double mass=phy.getMass();
		glm::dvec2 masspos=phy.getMassPosSum()/mass;
		glm::dvec2 momentum=phy.getMomentum();
		double a=phy.getAngularMomentum();
        /*
		std::cout<<"Momentum: "<<momentum.x<<", "<<momentum.y<<std::endl;
		std::cout<<"CM: "<<masspos.x<<", "<<masspos.y<<std::endl;
		std::cout<<"angular: "<<a<<std::endl;
		std::cout<<"angular0: "<<a0<<std::endl;
		std::cout<<std::endl;*/


		createTotal += phy.getTiming().hashCreate + phy.getTiming().quadCreate;
		phyTotal += phy.getTiming().hashPhy + phy.getTiming().quadPhy;
		deleteTotal += phy.getTiming().hashDelete + phy.getTiming().quadDelete;
		n++;
        /*

		std::cout << "Create Avg: " << createTotal / n << std::endl;
		std::cout << "Phy Avg: " << phyTotal / n << std::endl;
		std::cout << "Delete Avg: " << deleteTotal / n << std::endl;

		std::cout << "frame: " << n << std::endl;
        */


        //phy.drawToSurface(screen,1280,720,glm::dmat3(5,0,0,0,5,0,640,360,1));
        frame++;
initDrawStyleCCircles();
		double mult=cam.getScale();
		SDL_SetRenderDrawColor(gRenderer,0,0,0,255);
		SDL_RenderClear(gRenderer);
        double transx=cam.getX();
        double transy=cam.getY();
        cout<<"x: "<<transx<<"\t\t";
        cout<<"y: "<<transy<<"\t\t";
        cout<<"scale: "<<cam.getScale()<<endl;
        //SDL_RenderSetScale(gRenderer,double(scwidth)/cam.getScale(),double(scwidth)/cam.getScale());

        setPointSize((int)(cam.getScale()*10));
		for(int i=0;i<phy.masses.size();i++) {
			//drawPoint((int)floor((phy.masses[i].position.x-transx)*mult)+scwidth/2,(int)floor((phy.masses[i].position.y-transy)*mult)+scheight/2);
			drawPoint((int)floor((phy.masses[i].position.x-transx)*mult),(int)floor((phy.masses[i].position.y-transy)*mult));
		}
		
		SDL_SetRenderDrawColor(gRenderer,255,0,0,255);
		SDL_RenderDrawPoint(gRenderer,scwidth/2,scheight/2);

		sSync();

/*	
		saveScreenshotBMP(filename("Acore",frame,".bmp"),gWindow,gRenderer);

initDrawStyleTCircles();
		mult=1;
		SDL_SetRenderDrawColor(gRenderer,0,0,0,255);
		SDL_RenderClear(gRenderer);
		for(int i=0;i<phy.masses.size();i++) {
			drawPoint((int)floor(phy.masses[i].position.x*mult)+scwidth/2,(int)floor(phy.masses[i].position.y*mult)+scheight/2);
		}
		
		SDL_SetRenderDrawColor(gRenderer,255,0,0,255);
		SDL_RenderDrawPoint(gRenderer,scwidth/2,scheight/2);

		sSync();

		saveScreenshotBMP(filename("Bcore",frame,".bmp"),gWindow,gRenderer);

	initDrawStylePoints();
	setPointColor(0,0,255,200);
		mult=0.2;
		SDL_SetRenderDrawColor(gRenderer,0,0,0,255);
		SDL_RenderClear(gRenderer);
		for(int i=0;i<phy.masses.size();i++) {
			drawPoint((int)floor(phy.masses[i].position.x*mult)+scwidth/2,(int)floor(phy.masses[i].position.y*mult)+scheight/2);
		}
		
		SDL_SetRenderDrawColor(gRenderer,255,0,0,255);
		SDL_RenderDrawPoint(gRenderer,scwidth/2,scheight/2);

		sSync();

		saveScreenshotBMP(filename("galaxy",frame,".bmp"),gWindow,gRenderer);
initDrawStyleTCircles();
		mult=3;
		SDL_SetRenderDrawColor(gRenderer,0,0,0,255);
		SDL_RenderClear(gRenderer);
		for(int i=0;i<phy.masses.size();i++) {
			double x=phy.masses[i].position.x-phy.masses[zoomParticle].position.x;
			double y=phy.masses[i].position.y-phy.masses[zoomParticle].position.y;

			drawPoint((int)floor(x*mult)+scwidth/2,(int)floor(y*mult)+scheight/2);
		}*/
		
		SDL_SetRenderDrawColor(gRenderer,255,0,0,255);
		SDL_RenderDrawPoint(gRenderer,scwidth/2,scheight/2);

		sSync();

		//saveScreenshotBMP(filename("zoom",frame,".bmp"),gWindow,gRenderer);
//img.save("sphere"+std::to_string(fsize)+".bmp");
    }
	sQuit();
    return 0;
}
