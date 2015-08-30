
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

std::string filename(std::string prefix, int num, std::string suffix){
    stringstream ss;
    ss << num;
    std::string x=ss.str();
    while(x.length()<5)
        x="0"+x;
    return prefix+x+suffix;
}

void placeBall(PhysicsHandlerThreaded& phy,int n,double sdev,double size,double exprand1=100,double exprand2=0,double m=1){

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
    PhysicsHandlerThreaded phy(2*nmult,700,10);
    typedef boost::mt19937                     ENG;    // Mersenne Twister
    typedef boost::normal_distribution<double> DIST;   // Normal Distribution
    typedef boost::variate_generator<ENG,DIST> GEN;    // Variate generator
    ENG eng;
    DIST dist(0,80);
    GEN gen(eng,dist);





    TimeType start,end;



    start=std::chrono::system_clock::now();
    end=std::chrono::system_clock::now();

    EasyTimer timer;

    placeBall(phy,5000,300,4.9,3.5,0.000007,26);
    placeBall(phy,20000,1000,30,3.3,0.0000003,26);
    int frame=0;
    clock_t t=clock();



    int n = 0;
    double createTotal = 0;
    double phyTotal = 0;
    double deleteTotal = 0;
    int zoomParticle=0;


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
        
        phy.update(0.01);
        initDrawStyleCCircles();
        double mult=cam.getScale();
        SDL_SetRenderDrawColor(gRenderer,0,0,0,255);
        SDL_RenderClear(gRenderer);
        double transx=cam.getX();
        double transy=cam.getY();

        setPointSize((int)(mult*10));
        for(int i=0;i<phy.numParticles();i++) {
            //drawPoint((int)floor((phy.masses[i].position.x-transx)*mult)+scwidth/2,(int)floor((phy.masses[i].position.y-transy)*mult)+scheight/2);
            drawPoint((int)floor((phy.getPointMass(i).position.x-transx)*mult),(int)floor((phy.getPointMass(i).position.y-transy)*mult));
        }
        
        SDL_SetRenderDrawColor(gRenderer,255,0,0,255);
        SDL_RenderDrawPoint(gRenderer,scwidth/2,scheight/2);

        sSync();
        n++;
        /*if(n%300==0){
            saveScreenshotBMP(filename("spinny",frame,".bmp"),gWindow,gRenderer);
            frame++;
        }*/

/*  

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
