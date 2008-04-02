//	---------------------------------------------------------------------------
//
//	@file		TwSimpleSDL.c
//	@brief		A simple example that uses AntTweakBar with OpenGL and SDL.
//
//				AntTweakBar: http://www.antisphere.com/Wiki/tools:anttweakbar
//				OpenGL:		 http://www.opengl.org
//				SDL:		 http://www.libsdl.org
//	
//	@author		Philippe Decaudin - http://www.antisphere.com
//	@date		2006/05/20
//
//	note:		TAB=4
//
//	Compilation:
//	http://www.antisphere.com/Wiki/tools:anttweakbar:examples#twsimplesdl
//
//	---------------------------------------------------------------------------


#include <AntTweakBar.h>
#include <SDL/SDL.h>
 
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#ifdef _WIN32
#	include <windows.h>	// required by gl.h
#endif
#include <GL/gl.h>
#include <GL/glu.h>

// SDL redefines main
#ifdef main
#	undef main
#endif


int main()
{
	const SDL_VideoInfo* video = NULL;
    int width  = 640, height = 480;
	int bpp, flags;
	int quit = 0;
	TwBar *bar;
	int n, numCubes = 30;
	float color0[] = { 1.0f, 0.5f, 0.0f };
	float color1[] = { 0.5f, 1.0f, 0.0f };
	double ka = 5.3, kb = 1.7, kc = 4.1;

	// Initialize SDL, then get the current video mode and use it to create a SDL window.
    if( SDL_Init(SDL_INIT_VIDEO)<0 )
	{
		fprintf(stderr, "Video initialization failed: %s\n", SDL_GetError());
		SDL_Quit();
        exit(1);
    }
    video = SDL_GetVideoInfo();
    if( !video ) 
	{
		fprintf(stderr, "Video query failed: %s\n", SDL_GetError());
		SDL_Quit();
        exit(1);
    }
    SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
	//SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 16);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
	bpp = video->vfmt->BitsPerPixel;
	flags = SDL_OPENGL | SDL_HWSURFACE | SDL_RESIZABLE;
	//flags |= SDL_FULLSCREEN;
	if( !SDL_SetVideoMode(width, height, bpp, flags) )
	{
        fprintf(stderr, "Video mode set failed: %s", SDL_GetError());
		SDL_Quit();
		exit(1);
	}
	SDL_WM_SetCaption("AntTweakBar simple example using SDL", "AntTweakBar+SDL");
	// Enable SDL unicode and key-repeat
	SDL_EnableUNICODE(1);
	SDL_EnableKeyRepeat(SDL_DEFAULT_REPEAT_DELAY, SDL_DEFAULT_REPEAT_INTERVAL);

	// Set OpenGL viewport and states
	glViewport(0, 0, width, height);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);	// use default light diffuse and position
	glEnable(GL_NORMALIZE);
	glEnable(GL_COLOR_MATERIAL);
	glDisable(GL_CULL_FACE);
	glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);

	// Initialize AntTweakBar
	TwInit(TW_OPENGL, NULL);
	// Tell the window size to AntTweakBar
	TwWindowSize(width, height);

	// Create a tweak bar
	bar = TwNewBar("TweakBar");

	// Add 'width' and 'height' to 'bar': they are read-only (RO) variables of type TW_TYPE_INT32.
	TwAddVarRO(bar, "Width", TW_TYPE_INT32, &width, " label='Wnd width (pixels)' ");
	TwAddVarRO(bar, "Height", TW_TYPE_INT32, &height, " label='Wnd height (pixels)' ");
	// Add 'quit' to 'bar': it is a modifiable (RW) variable of type TW_TYPE_BOOL32 (boolean stored in a 32 bits integer). Its shortcut is [ESC].
	TwAddVarRW(bar, "Quit", TW_TYPE_BOOL32, &quit, " label='Quit?' true='+' false='-' key='ESC' ");
	// Add 'numCurves' to 'bar': it is a modifiable variable of type TW_TYPE_INT32. Its shortcuts are [c] and [C].
	TwAddVarRW(bar, "NumCubes", TW_TYPE_INT32, &numCubes, " label='Number of cubes' min=1 max=100 keyIncr=c keyDecr=C ");
	// Add 'ka', 'kb and 'kc' to 'bar': they are modifiable variables of type TW_TYPE_DOUBLE
	TwAddVarRW(bar, "ka", TW_TYPE_DOUBLE, &ka, " label='X path coeff' keyIncr=1 keyDecr=CTRL+1 min=-10 max=10 step=0.01 ");
	TwAddVarRW(bar, "kb", TW_TYPE_DOUBLE, &kb, " label='Y path coeff' keyIncr=2 keyDecr=CTRL+2 min=-10 max=10 step=0.01 ");
	TwAddVarRW(bar, "kc", TW_TYPE_DOUBLE, &kc, " label='Z path coeff' keyIncr=3 keyDecr=CTRL+3 min=-10 max=10 step=0.01 ");
	// Add 'color0' and 'color1' to 'bar': they are modifable variables of type TW_TYPE_COLOR3F (3 floats color)
	TwAddVarRW(bar, "color0", TW_TYPE_COLOR3F, &color0, " label='Start color' ");
	TwAddVarRW(bar, "color1", TW_TYPE_COLOR3F, &color1, " label='End color' ");

	// Main loop:
	// - Draw some cubes
	// - Process events
    while( !quit )
	{
		SDL_Event event;
		int handled;

		// Clear screen
		glClearColor(0.6f, 0.95f, 1.0f, 1);
		glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

		// Set OpenGL camera
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		gluPerspective(40, (double)width/height, 1, 10);
		gluLookAt(0,0,3, 0,0,0, 0,1,0);

		// Draw cubes
		for( n=0; n<numCubes; ++n )
		{
			double t = 0.05*n - (double)SDL_GetTicks()/2000.0;
			double r = 5.0*n + (double)SDL_GetTicks()/10.0;
			float c = (float)n/numCubes;

			// Set cube position
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();
			glTranslated(0.4+0.6*cos(ka*t), 0.6*cos(kb*t), 0.6*sin(kc*t));
			glRotated(r, 0.2, 0.7, 0.2);
			glScaled(0.1, 0.1, 0.1);
			glTranslated(-0.5, -0.5, -0.5);

			// Set cube color
			glColor3f((1.0f-c)*color0[0]+c*color1[0], (1.0f-c)*color0[1]+c*color1[1], (1.0f-c)*color0[2]+c*color1[2]);

			// Draw cube
			glBegin(GL_QUADS);
				glNormal3f(0,0,-1); glVertex3f(0,0,0); glVertex3f(0,1,0); glVertex3f(1,1,0); glVertex3f(1,0,0);	// front face
				glNormal3f(0,0,+1); glVertex3f(0,0,1); glVertex3f(1,0,1); glVertex3f(1,1,1); glVertex3f(0,1,1);	// back face
				glNormal3f(-1,0,0); glVertex3f(0,0,0); glVertex3f(0,0,1); glVertex3f(0,1,1); glVertex3f(0,1,0);	// left face
				glNormal3f(+1,0,0); glVertex3f(1,0,0); glVertex3f(1,1,0); glVertex3f(1,1,1); glVertex3f(1,0,1); // right face
				glNormal3f(0,-1,0); glVertex3f(0,0,0); glVertex3f(1,0,0); glVertex3f(1,0,1); glVertex3f(0,0,1);	// bottom face	
				glNormal3f(0,+1,0); glVertex3f(0,1,0); glVertex3f(0,1,1); glVertex3f(1,1,1); glVertex3f(1,1,0);	// top face
			glEnd();
		}

		// Draw tweak bars
		TwDraw();

		// Present frame buffer
		SDL_GL_SwapBuffers();

        // Process incoming events
		while( SDL_PollEvent(&event) ) 
		{
			// Send event to AntTweakBar
			handled = TwEventSDL(&event);

			// If event has not been handled by AntTweakBar, process it
			if( !handled )
			{
				switch( event.type )
				{
				case SDL_QUIT:	// Window is closed
					quit = 1;
					break;
				case SDL_VIDEORESIZE:	// Window size has changed
					// Resize SDL video mode
 					width = event.resize.w;
					height = event.resize.h;
					if( !SDL_SetVideoMode(width, height, bpp, flags) )
						fprintf(stderr, "WARNING: Video mode set failed: %s", SDL_GetError());
					// Resize OpenGL viewport
					glViewport(0, 0, width, height);
					// Restore OpenGL states (SDL seems to lost them)
					glEnable(GL_DEPTH_TEST);
					glEnable(GL_LIGHTING);
					glEnable(GL_LIGHT0);
					glEnable(GL_NORMALIZE);
					glEnable(GL_COLOR_MATERIAL);
					glDisable(GL_CULL_FACE);
					glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
					// TwWindowSize has been called by TwEventSDL, so it is not necessary to call it again here.
					break;
				}
			}
		}
    } // End of main loop

	// Terminate AntTweakBar
	TwTerminate();

	// Terminate SDL
	SDL_Quit();

	return 0;
}  
