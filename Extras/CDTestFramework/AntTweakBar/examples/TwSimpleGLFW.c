//	---------------------------------------------------------------------------
//
//	@file		TwSimpleGLFW.c
//	@brief		A simple example that uses AntTweakBar with 
//				OpenGL and the GLFW windowing system.
//
//				AntTweakBar: http://www.antisphere.com/Wiki/tools:anttweakbar
//				OpenGL:		 http://www.opengl.org
//				GLFW:		 http://glfw.sourceforge.net
//	
//	@author		Philippe Decaudin - http://www.antisphere.com
//	@date		2006/05/20
//
//	note:		TAB=4
//
//	Compilation:
//	http://www.antisphere.com/Wiki/tools:anttweakbar:examples#twsimpleglfw
//
//	---------------------------------------------------------------------------


#include <AntTweakBar.h>

#define GLFW_DLL
#include "glfw.h"

#include <stdio.h>

// Callback function called by GLFW when window size changes
void GLFWCALL WindowSizeCB(int width, int height)
{
	// Set OpenGL viewport and camera
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(40, (double)width/height, 1, 10);
	gluLookAt(-1,0,3, 0,0,0, 0,1,0);	
	
	// Send the new window size to AntTweakBar
	TwWindowSize(width, height);
}


// This example program draws a possibly transparent cube 
void DrawModel(int wireframe)
{
	int pass, numPass;

	// Enable OpenGL transparency and light (could have been done once at init)
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHT0);	// use default light diffuse and position
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 1);
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
	glEnable(GL_LINE_SMOOTH);
	glLineWidth(3.0);
	
	if( wireframe )
	{
		glDisable(GL_CULL_FACE);	
		glDisable(GL_LIGHTING);
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		numPass = 1;
	}
	else
	{
		glEnable(GL_CULL_FACE);	
		glEnable(GL_LIGHTING);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		numPass = 2;
	}

	for( pass=0; pass<numPass; ++pass )
	{
		// Since the material could be transparent, we draw the convex model in 2 passes:
		// first its back faces, and second its front faces.
		glCullFace( (pass==0) ? GL_FRONT : GL_BACK );

		// Draw the model (a cube)
		glBegin(GL_QUADS);
			glNormal3f(0,0,-1); glVertex3f(0,0,0); glVertex3f(0,1,0); glVertex3f(1,1,0); glVertex3f(1,0,0);	// front face
			glNormal3f(0,0,+1); glVertex3f(0,0,1); glVertex3f(1,0,1); glVertex3f(1,1,1); glVertex3f(0,1,1);	// back face
			glNormal3f(-1,0,0); glVertex3f(0,0,0); glVertex3f(0,0,1); glVertex3f(0,1,1); glVertex3f(0,1,0);	// left face
			glNormal3f(+1,0,0); glVertex3f(1,0,0); glVertex3f(1,1,0); glVertex3f(1,1,1); glVertex3f(1,0,1); // right face
			glNormal3f(0,-1,0); glVertex3f(0,0,0); glVertex3f(1,0,0); glVertex3f(1,0,1); glVertex3f(0,0,1);	// bottom face	
			glNormal3f(0,+1,0); glVertex3f(0,1,0); glVertex3f(0,1,1); glVertex3f(1,1,1); glVertex3f(1,1,0);	// top face
		glEnd();
	}
}


// Main
int main() 
{
	GLFWvidmode mode; 	// GLFW video mode
	TwBar *bar;			// Pointer to a tweak bar
	
	double time = 0, dt;// Current time and enlapsed time
	double turn = 0; 	// Model turn counter
	double speed = 0.1;	// Model rotation speed
	int wire = 0;		// Draw model in wireframe?
	float bgColor[] = { 0.1f, 0.2f, 0.4f };			// Background color	
	unsigned char cubeColor[] = { 255, 0, 0, 255 };	// Model color (32bits RGBA)

	// Intialize GLFW	
	if( !glfwInit() )
	{
		// A fatal error occured
		fprintf(stderr, "GLFW initialization failed\n");
		return 1;
	}

	// Create a window
	glfwGetDesktopMode(&mode);
	if( !glfwOpenWindow(640, 480, mode.RedBits, mode.GreenBits, mode.BlueBits, 0, 16, 0, GLFW_WINDOW /* or GLFW_FULLSCREEN */) )
	{
		// A fatal error occured	
		fprintf(stderr, "Cannot open GLFW window\n");
		glfwTerminate();
		return 1;
	}
	glfwEnable(GLFW_MOUSE_CURSOR);
	glfwEnable(GLFW_KEY_REPEAT);
	glfwSetWindowTitle("AntTweakBar simple example using GLFW");

	// Initialize AntTweakBar
	if( !TwInit(TW_OPENGL, NULL) )
	{
		// A fatal error occured	
		fprintf(stderr, "AntTweakBar initialization failed: %s\n", TwGetLastError());
		glfwTerminate();
		return 1;
	}

	// Create a tweak bar
	bar = TwNewBar("TweakBar");

	// Add 'speed' to 'bar': it is a modifable (RW) variable of type TW_TYPE_DOUBLE. Its key shortcuts are [s] and [S].
	TwAddVarRW(bar, "speed", TW_TYPE_DOUBLE, &speed, " label='Rot speed (tr/sec)' min=0 max=2 step=0.01 keyIncr=s keyDecr=S ");
	// Add 'wire' to 'bar': it is a modifable variable of type TW_TYPE_BOOL32 (32 bits boolean). Its key shortcut is [w].
	TwAddVarRW(bar, "wire", TW_TYPE_BOOL32, &wire, " label='Wireframe mode' key=w ");
	// Add 'time' to 'bar': it is a read-only (RO) variable of type TW_TYPE_DOUBLE, with 1 precision digit
	TwAddVarRO(bar, "time", TW_TYPE_DOUBLE, &time, " label='Time (sec)' precision=1 ");			
	// Add 'bgColor' to 'bar': it is a modifable variable of type TW_TYPE_COLOR3F (3 floats color)
	TwAddVarRW(bar, "bgColor", TW_TYPE_COLOR3F, &bgColor, " label='Background color' ");
	// Add 'cubeColor' to 'bar': it is a modifable variable of type TW_TYPE_COLOR32 (32 bits color) with alpha
	TwAddVarRW(bar, "cubeColor", TW_TYPE_COLOR32, &cubeColor, " label='Cube color' alpha ");

	// Set GLFW event callbacks
	// - Redirect window size changes to the callback function WindowSizeCB
	glfwSetWindowSizeCallback(WindowSizeCB);
	// - Directly redirect GLFW mouse button events to AntTweakBar
	glfwSetMouseButtonCallback((GLFWmousebuttonfun)TwEventMouseButtonGLFW);
	// - Directly redirect GLFW mouse position events to AntTweakBar
	glfwSetMousePosCallback((GLFWmouseposfun)TwEventMousePosGLFW);
	// - Directly redirect GLFW mouse wheel events to AntTweakBar
	glfwSetMouseWheelCallback((GLFWmousewheelfun)TwEventMouseWheelGLFW);
	// - Directly redirect GLFW key events to AntTweakBar
	glfwSetKeyCallback((GLFWkeyfun)TwEventKeyGLFW);
	// - Directly redirect GLFW char events to AntTweakBar
	glfwSetCharCallback((GLFWcharfun)TwEventCharGLFW);
	// - Redirect window size changes to the callback function WindowSizeCB

	// Initialize time
	time = glfwGetTime();
	// Main loop (repeated while window is not closed and [ESC] is not pressed)
	while( glfwGetWindowParam(GLFW_OPENED) && !glfwGetKey(GLFW_KEY_ESC) )
	{
		// Clear frame buffer using bgColor
		glClearColor(bgColor[0], bgColor[1], bgColor[2], 1);
		glClear( GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT );

		// Rotate model
		dt = glfwGetTime() - time;
		time += dt;
		turn += speed*dt;
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glRotated(360.0*turn, 0.4, 1, 0.2);
		glTranslated(-0.5, -0.5, -0.5);		
	
		// Set color and draw model
		glColor4ubv(cubeColor);
		DrawModel(wire);
		
		// Draw tweak bars
		TwDraw();

		// Present frame buffer
		glfwSwapBuffers();
	}

	// Terminate AntTweakBar and GLFW
	TwTerminate();
	glfwTerminate();

	return 0;
}
