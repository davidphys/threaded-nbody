#pragma once
#ifndef POINTSDL_H
#define POINTSDL_H

#include "SDL2/SDL.h"
#include "SDL2/SDL_image.h"
#include <iostream>
#include <vector>
#include "CameraController.h"
namespace screen {

    extern CameraController cam;
    extern double ndcmousex;
    extern double ndcmousey;
    extern bool mousedown;

    void saveScreenshotBMP(std::string filepath, SDL_Window* SDLWindow, SDL_Renderer* SDLRenderer); 
	extern bool sRunning; //True means keep running. False means quit ASAP.
	//Internal pointers needed to initialize/de-initialize SDL.
	extern SDL_Window* gWindow;
	extern SDL_Renderer *gRenderer;
	const int scwidth=1280;
	const int scheight=720;

	bool sLoop(); //sLoop handles events and returns "true" if the program should run another frame.
	void sInit(); //initializes the screen/SDL, and creates the window.
	void sHandleEvents(); //function that handles user input, which are passed into the program as "SDL_Event"s.
	void sSync(); //update the screen. Changes won't appear on the screen if this is not called.
	void sQuit(); //uninitializes screen resources and SDL, and then returns.



	//Texture wrapper class
	class LTexture
	{
		public:
			//Initializes variables
			LTexture();

			//Deallocates memory
			~LTexture();

			//Loads image at specified path
			bool loadFromFile( std::string path );

			//Deallocates texture
			void free();

			//Renders texture at given point
			void render( int x, int y, int w=0,int h=0);

			//Gets image dimensions
			int getWidth();
			int getHeight();

		private:
			//The actual hardware texture
			SDL_Texture* mTexture;

			//Image dimensions
			int mWidth;
			int mHeight;
	};





	extern int ptype; //0=points, 1=white circles, 2=transparent circles, 3=colored
	extern int psize;
	extern int pcolr;
	extern int pcolg;
	extern int pcolb;
	extern int pcola;
	extern int pi;
	extern std::vector<LTexture> particles;
	void initDrawStylePoints();
	void initDrawStyleCircles();
	void initDrawStyleTCircles();
	void initDrawStyleCCircles();

	void setPointColor(int r,int g,int b,int a=255);
	void setPointSize(int s);
	void drawPoint(int x,int y);
	




}


#endif
