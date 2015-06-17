#include "CameraController.h"
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

CameraController::CameraController(double xpos,double ypos,double scroll,double aspect)
{
    camera_center=glm::dvec2(xpos,ypos);
    this->scroll=scroll;
    this->scale=exp(scroll);
	veldrag=.9;
	scrolldrag=.87;
	mousezoompos=dragcurr=dragstart=vel=glm::vec2(0,0);
	isDragging=false;
	scrollvel=0;
}
    double CameraController::getX(){return camera_center.x;}
    double CameraController::getY(){
        return camera_center.y;
    }
    double CameraController::getScale(){return exp(scroll);}
void CameraController::update(double timestep)
{
    lastdt=timestep;
    //If the position of a pixel on the screen is Xp=(x-x1)sc1 for scale=exp(scroll), x1=camera_center, then 
    //for xp to stay in the same "real world spot" as x1 and sc1 change, we must have Xp/sc1+x1=Xp'/sc2+x2=x
    //satisfied at all times.
    glm::dvec2 x1=camera_center;
    camera_center+=vel*timestep;
	vel*=veldrag;

    mousezoompos+=exp(scroll)*(x1-camera_center); //satisfy the commented identity (sc1=sc2)

    double scale1=exp(scroll);
	scroll+=scrollvel*timestep;
	scrollvel*=scrolldrag;
    double scale2=exp(scroll);

    camera_center+=mousezoompos*(1/scale1-1/scale2); //satisfy the commented identity (Xp=Xp')
}

void CameraController::setMouseZoomPos(double x, double y)
{
	mousezoompos=glm::dvec2(x,y);
}
void CameraController::drag(double x, double y)
{
	if(isDragging)
	{
		dragstart=dragcurr;
		dragcurr=glm::dvec2(x,y);
		//std::cout<<cam.getAspect()/cam.getScaleAbsolute()*(dragstart-dragcurr).x;
		camera_center+=exp(-scroll)*(dragstart-dragcurr);
        vel=glm::dvec2(0,0);
	} else {
		isDragging=true;
		dragstart=dragcurr=glm::dvec2(x,y);
	}
}
void CameraController::enddrag()
{
	if(isDragging)
	{
		isDragging=false;
		vel-=(dragcurr-dragstart)/(getScale()*lastdt);
		dragcurr=dragstart=glm::dvec2(0,0);
	}
}
void CameraController::mouseZoom(double zamount)
{
	/*double factor=(1-exp(2.30258509*zamount))/cam.getScaleAbsolute();
	cam.addPos(glm::dvec2(cam.getAspect()*mousezoompos.x*factor,
						mousezoompos.y*factor));*/
	scrollvel+=zamount*scrolldrag;
}

