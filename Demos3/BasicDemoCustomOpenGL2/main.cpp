/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2014 Erwin Coumans  http://bulletphysics.org

//This file is Copyright (c) 2014 Google Inc.

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///This main.cpp replaces glut by Bullet 3 own platform management for OpenGL and input devices

#include <stdio.h>
#include "../../Demos/BasicDemo/BasicDemoPhysicsSetup.h"


#ifdef __APPLE__
#include "OpenGLWindow/MacOpenGLWindow.h"
#else

#include "GL/glew.h"
#ifdef _WIN32
#include "OpenGLWindow/Win32OpenGLWindow.h"
#else
//let's cross the fingers it is Linux/X11
#include "OpenGLWindow/X11OpenGLWindow.h"
#endif //_WIN32
#endif//__APPLE__

#include "../../Demos/OpenGL/DemoApplication.h"
#include "../../Demos/OpenGL/GLDebugDrawer.h"

int sWidth = 800;
int sHeight = 600;
DemoApplication* gApp = 0;
bool isShiftPressed = false;
bool isCtrlPressed = false;
bool isAltPressed = false;

/*
todo: add wheel callback
typedef void (*b3WheelCallback)(float deltax, float deltay);
*/

void MyKeyboardCallback(int orgKeycode, int state)
{
	int keycode = orgKeycode;

	if (gApp)
	{

		switch (orgKeycode)
		{
			case 	B3G_SHIFT:
			{
				isShiftPressed = (state==1);
				break;
			}
			case B3G_ALT:
			{
				isAltPressed = (state==1);
				break;
			}
			case B3G_CONTROL:
			{
				isCtrlPressed = (state==1);
				break;
			}
			default:
			{
			}
		};

		if (state)
		{
			gApp->keyboardCallback(keycode,0,0);
		} else
		{
			gApp->keyboardUpCallback(keycode,0,0);
		}
	}
}
void MyMouseMoveCallback( float x, float y)
{
	if (gApp)
	{
		//printf("mouseMotionFunc %f,%f\n",x,y);
		gApp->mouseMotionFunc(x,y);
	}
}

void MyMouseButtonCallback(int button, int state, float x, float y)
{
	if (gApp)
	{
		//printf("mouseFunc %d,%d, %f,%f\n",button,state,x,y);
		gApp->mouseFunc(button,1-state,x,y);
	}
}

void MyResizeCallback(float width, float height)
{
	sWidth = width;
	sHeight = height;
	if (gApp)
	{
		gApp->reshape(width,height);
	}
}

int main(int argc, char* argv[])
{

	b3gDefaultOpenGLWindow* window = new b3gDefaultOpenGLWindow();
	//window->setKeyboardCallback(keyCallback);
	b3gWindowConstructionInfo wci;
    wci.m_openglVersion = 2;
	wci.m_width = sWidth;
	wci.m_height = sHeight;
	//	wci.m_resizeCallback = MyResizeCallback;

	window->createWindow(wci);
	window->setResizeCallback(MyResizeCallback);
	window->setMouseButtonCallback(MyMouseButtonCallback);

	window->setMouseMoveCallback(MyMouseMoveCallback);
	window->setKeyboardCallback(MyKeyboardCallback);

//	window->setWindowTitle("render test");

    int majorGlVersion, minorGlVersion;

    if (!sscanf((const char*)glGetString(GL_VERSION), "%d.%d", &majorGlVersion, &minorGlVersion)==2)
    {
        printf("Exit: Error cannot extract OpenGL version from GL_VERSION string\n");
        exit(0);
    }
    if (majorGlVersion>=3 && wci.m_openglVersion>=3)
    {
       // float retinaScale = 1.f;

#ifndef __APPLE__
#ifndef _WIN32
    //we need glewExperimental on Linux
    glewExperimental = GL_TRUE;
#endif // _WIN32
        glewInit();
#endif

    //we need to call glGetError twice, because of some Ubuntu/Intel/OpenGL issue

   glGetError();
   glGetError();

    btAssert(glGetError()==GL_NO_ERROR);


        //retinaScale = window->getRetinaScale();

        //primRenderer = new GLPrimitiveRenderer(sWidth,sHeight);
        //sth_stash* font = initFont(primRenderer );
        //gwenRenderer = new GwenOpenGL3CoreRenderer(primRenderer,font,sWidth,sHeight,retinaScale);

    } else
    {
        //OpenGL 2.x
        /*gwenRenderer = new Gwen::Renderer::OpenGL_DebugFont();
        skin.SetRender( gwenRenderer );

        pCanvas = new Gwen::Controls::Canvas( &skin );
        pCanvas->SetSize( sWidth, sHeight);
        pCanvas->SetDrawBackground( true );
        pCanvas->SetBackgroundColor( Gwen::Color( 150, 170, 170, 255 ) );
		*/

    }


//    glClearColor(0.2,0.2,0.2,1);



	BasicDemoPhysicsSetup physicsSetup;
	GraphicsPhysicsBridge br;
	physicsSetup.initPhysics(br);


	struct MyAppie : public	DemoApplication
	{
		virtual void initPhysics()
		{
		}
		virtual void clientMoveAndDisplay()
		{
		}
		virtual		void swapBuffers()
		{
		}
		virtual		void	updateModifierKeys()
		{
			m_modifierKeys = 0;
			if (isAltPressed)
				m_modifierKeys |= BT_ACTIVE_ALT;

			if (isCtrlPressed)
				m_modifierKeys |= BT_ACTIVE_CTRL;

			if (isShiftPressed)
				m_modifierKeys |= BT_ACTIVE_SHIFT;

		}

	};

	{
		MyAppie appie;
		appie.setDynamicsWorld(physicsSetup.m_dynamicsWorld);
		appie.reshape(sWidth,sHeight);
		appie.setShadows(true);
		gApp = &appie;
		GLDebugDrawer draw;
		physicsSetup.m_dynamicsWorld->setDebugDrawer(&draw);
		btClock timer;
		unsigned  long prevTime = timer.getTimeMicroseconds();

		do
		{
			unsigned  long curTime = timer.getTimeMicroseconds();
			if (!appie.isIdle())
			{
				physicsSetup.stepSimulation((curTime-prevTime)*(1./1000000.));
			}
			prevTime = curTime;
			window->startRendering();
			br.syncPhysicsToGraphics(physicsSetup.m_dynamicsWorld);
			appie.renderme();
			physicsSetup.m_dynamicsWorld->debugDrawWorld();
			window->endRendering();

		} while (!window->requestedExit());
	}
	window->closeWindow();
	delete window;

	physicsSetup.exitPhysics();


	printf("hello\n");
}
