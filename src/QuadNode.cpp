#include "QuadNode.h"

void QuadNode::calculateForces(PointMass *arg, const double& G) const
{

	if(subnodes[0][0]!=NULL)
	{
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
