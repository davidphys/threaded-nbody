#pragma once
#include "glm/glm.hpp"
class CameraController
{

	double veldrag;
	glm::dvec2 vel;
    glm::dvec2 camera_center;


	bool isDragging;
	glm::dvec2 dragstart;//NDC on (-1,1)
	glm::dvec2 dragcurr;
	glm::dvec2 mousezoompos;

	double scrolldrag;
	double scrollvel;
    double scroll;
    double scale;
    double lastdt=1.0;
public:
	CameraController(double xpos=0,double ypos=0,double scroll=0,double aspect=1);
	void update(double timestep);
	void drag(double x, double y);
	void enddrag();
	void mouseZoom(double arg);
	void setMouseZoomPos(double x, double y);
    double getX();
    double getY();
    double getScale();
};

