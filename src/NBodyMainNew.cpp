#include <iostream>
#include <limits>
#include "PhysicsHandlerThreaded.h"
#include "glm/glm.hpp"
#include "glm/gtc/random.hpp"
#include <string>
#include <sstream>
#include <time.h>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <chrono>
#include "ParticleMan.h"
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

int main(int argc, char** argv)
{
    int nmult=3;
    PhysicsHandlerThreaded phy(2*nmult,600,10);
    TimeType start,end;
    start=std::chrono::system_clock::now();
    end=std::chrono::system_clock::now();

    EasyTimer timer;

    ParticleManager man;
    
    man.setCursorParticleMass(20);
    man.setCursorPosition(-1000,0);
    man.setCursorVelocity(20,0);
    man.placeBall(100,1,0.01);
    man.setCursorVelocity(0,0);
    man.setCursorPosition(-1000,180);
    man.placeBall(100,1,0.0);
    man.setCursorPosition(4000,0);
    man.setCursorVelocity(-100,0);
    man.placeBall(500,1,-0.01);

    man.load("particles.txt");

    phy.replaceMassList(man.masses);

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
        
        sSync();
    }
    sQuit();
    return 0;
}
