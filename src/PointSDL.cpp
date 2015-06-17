#include "PointSDL.h"
#include <iostream>
#include <string>


/*

#include "PointSDL.h"
using namespace screen;


int main(int argc, char** argv)
{
	sInit();
		

	
	initDrawStyleCCircles();
	setPointColor(255,0,0);
	double x=0;
	double y=0;
	double a=2.01,b=-2.53,c=1.61,d=-0.33;
	while(sLoop())
	{
		for(int i=0;i<1;i++) {
			double nx=sin(a*y)-cos(b*x);
			y=sin(c*x)-cos(d*y);
			x=nx;
			drawPoint((int)(x*100+scwidth/2),(int)(y*100+scheight/2));
		}
		sSync();
	}
	sQuit();
	return 0;
}


*/



namespace screen {


void saveScreenshotBMP(std::string filepath, SDL_Window* SDLWindow, SDL_Renderer* SDLRenderer) {
	SDL_Surface *sshot = SDL_CreateRGBSurface(0, scwidth, scheight, 32, 0x00ff0000, 0x0000ff00, 0x000000ff, 0xff000000);
	SDL_RenderReadPixels(gRenderer, NULL, SDL_PIXELFORMAT_ARGB8888, sshot->pixels, sshot->pitch);
	SDL_SaveBMP(sshot, filepath.c_str());
	SDL_FreeSurface(sshot);
}
bool sRunning=true; //True means keep running. False means quit ASAP.
//Internal pointers needed to initialize/de-initialize SDL.
SDL_Window* gWindow=NULL;
SDL_Renderer *gRenderer=NULL;

bool sLoop() {
	sHandleEvents();
	return sRunning;
}
void sInit() {
    cam=CameraController(0,0,0,double(scheight)/scwidth);
	sRunning=true;
	if(SDL_Init(SDL_INIT_VIDEO)<0) {
		std::cerr<<"Video initialization failed: "<<SDL_GetError()<<std::endl;
		sRunning=false;
	} else {
		//Set texture filtering to linear
		if(!SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY,"1")) {
			std::cerr<<"Warning: Linear texture filtering not enabled!"<<std::endl;
		}

		//Create window
		gWindow=SDL_CreateWindow("SDL Tutorial",SDL_WINDOWPOS_UNDEFINED,SDL_WINDOWPOS_UNDEFINED,scwidth,scheight,SDL_WINDOW_SHOWN);
		if(gWindow==NULL) {
			std::cerr<<"Window could not be created! SDL Error: "<<SDL_GetError()<<std::endl;
			sRunning=false;
		} else {
			//Create renderer for window
			gRenderer=SDL_CreateRenderer(gWindow,-1,SDL_RENDERER_ACCELERATED);
			if(gRenderer==NULL) {
				std::cerr<<"Renderer could not be created! SDL Error: "<<SDL_GetError()<<std::endl;
				sRunning=false;
			} else {
				//Initialize renderer color
				SDL_SetRenderDrawColor(gRenderer,0xFF,0xFF,0xFF,0xFF);

				//Initialize PNG loading
				int imgFlags=IMG_INIT_PNG;
				if(!(IMG_Init(imgFlags) & imgFlags)) {
					std::cerr<<"SDL_image could not initialize! SDL_mage Error: "<<IMG_GetError()<<std::endl;
					sRunning=false;
				}
			}
		}

	}
	SDL_SetRenderDrawBlendMode(gRenderer,SDL_BLENDMODE_ADD);

}

void sHandleEvents() {
    	SDL_Event e;
	int mousezoom=0;
	while(SDL_PollEvent(&e))
	{
		switch(e.type)
		{
			case SDL_QUIT:
				sRunning=false;
				break;
			case SDL_KEYDOWN:
				switch(e.key.keysym.sym)
				{
					case SDLK_ESCAPE:
						sRunning=false;	
						break;
				}
			case SDL_MOUSEBUTTONDOWN:
				switch(e.button.button)
				{
					case SDL_BUTTON_LEFT:
						mousedown=true;
						ndcmousex=e.button.x;
						ndcmousey=e.button.y;
						break;
				}
				break;			
            case SDL_MOUSEWHEEL:
                mousezoom+=e.wheel.y;
                break;
			case SDL_MOUSEBUTTONUP:
				if(e.button.button == SDL_BUTTON_LEFT)
				{	
					mousedown=false;
					cam.enddrag();
				}	
				break;
			case SDL_MOUSEMOTION:
				ndcmousex=e.motion.x;
						ndcmousey=e.button.y;
				break;
			
			
		}
	}
    //std::cout<<"("<<ndcmousex<<", \t\t"<<ndcmousey<<")"<<std::endl;
	if(mousezoom!=0)
	{
		cam.mouseZoom(mousezoom);
	    cam.setMouseZoomPos(ndcmousex,ndcmousey);
	}
	if(mousedown)
	{
		cam.drag(ndcmousex,ndcmousey);
	}
    cam.update(1.0/30.0);
}
void sSync() {
	SDL_RenderPresent(gRenderer);
}
void sQuit() {
	SDL_DestroyWindow(gWindow);
	gWindow=NULL;
	SDL_Quit();
}




LTexture::LTexture() {
	//Initialize
	mTexture=NULL;
	mWidth=0;
	mHeight=0;
}

LTexture::~LTexture() {
	//Deallocate
	free();
}

bool LTexture::loadFromFile(std::string path) {
	//Get rid of preexisting texture
	free();

	//The final texture
	SDL_Texture* newTexture=NULL;

	//Load image at specified path
	SDL_Surface* loadedSurface=IMG_Load(path.c_str());
	if(loadedSurface==NULL) {
		std::cerr<<"Unable to load image"<<path<<". SDL_image Error: "<<IMG_GetError()<<std::endl;
	} else {
		//Color key image
		SDL_SetColorKey(loadedSurface,SDL_TRUE,SDL_MapRGB(loadedSurface->format,0,0xFF,0xFF));

		//Create texture from surface pixels
		newTexture=SDL_CreateTextureFromSurface(gRenderer,loadedSurface);
		if(newTexture==NULL) {
			std::cerr<<"Unable to create texture from"<<path<<". SDL Error: "<<SDL_GetError()<<std::endl;
		} else {
			//Get image dimensions
			mWidth=loadedSurface->w;
			mHeight=loadedSurface->h;
		}

		//Get rid of old loaded surface
		SDL_FreeSurface(loadedSurface);
	}

	//Return success
	mTexture=newTexture;
	return mTexture!=NULL;
}

void LTexture::free() {
	//Free texture if it exists
	if(mTexture!=NULL) {
		SDL_DestroyTexture(mTexture);
		mTexture=NULL;
		mWidth=0;
		mHeight=0;
	}
}

void LTexture::render(int x,int y,int w, int h) {
	if(w<=0 || h<=0) {
		w=mWidth;
		h=mHeight;
	}
	//Set rendering space and render to screen
	SDL_Rect renderQuad={x,y,w,h};


	//Render to screen
	SDL_RenderCopy(gRenderer,mTexture,NULL,&renderQuad);
}

int LTexture::getWidth() {
	return mWidth;
}

int LTexture::getHeight() {
	return mHeight;
}










     CameraController cam=CameraController();
     double ndcmousex=0;
     double ndcmousey=0;
     bool mousedown=false;
int ptype=0; //0=points, 1=white circles, 2=transparent circles, 3=colored
int psize=0;
int pcolr=0;
int pcolg=0;
int pcolb=0;
int pcola=255;
int pi=0;
std::vector<LTexture> particles;

void initDrawStylePoints() {
	ptype=0;
	pcolr=0;
	pcolg=0;
	pcolb=0;
	pcola=255;
	psize=0;
	particles.clear();
	pi=0;
}
void initDrawStyleCircles() {
	ptype=1;
	pcolr=0;
	pcolg=0;
	pcolb=0;
	pcola=255;
	psize=0;
	particles.clear();
	particles.push_back(LTexture());
	particles[0].loadFromFile("particle1.png");
	pi=0;
}
void drawPoint(int x,int y) {
	if(ptype==0) {
		SDL_SetRenderDrawColor(gRenderer,pcolr,pcolg,pcolb,pcola);
		SDL_RenderDrawPoint(gRenderer,x,y);
	} else if(ptype==1) {
		particles[0].render(x,y,psize,psize);
	} else if(ptype==2) {
		particles[0].render(x,y,psize,psize);
	} else if(ptype==3) {
		particles[pi%4].render(x,y,psize,psize);
		pi++;
	}
}
void initDrawStyleTCircles(){
	ptype=2;
	pcolr=0;
	pcolg=0;
	pcolb=0;
	pcola=255;
	psize=0;
	particles.clear();
	particles.push_back(LTexture());
	particles[0].loadFromFile("particle.png");
	pi=0;
}
void initDrawStyleCCircles() {
	ptype=3;
	pcolr=0;
	pcolg=0;
	pcolb=0;
	pcola=255;
	psize=0;
	particles.clear();
	particles.push_back(LTexture());
	particles.push_back(LTexture());
	particles.push_back(LTexture());
	particles.push_back(LTexture());
	particles[0].loadFromFile("particle1.png");
	particles[1].loadFromFile("particle2.png");
	particles[2].loadFromFile("particle3.png");
	particles[3].loadFromFile("particle4.png");
	pi=0;
}

void setPointColor(int r,int g,int b,int a) {
	pcolr=r;
	pcolg=g;
	pcolb=b;
	pcola=a;
}
void setPointSize(int s) {
	psize=s;
}


}
