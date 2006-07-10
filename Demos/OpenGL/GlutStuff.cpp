/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifdef WIN32//for glut.h
#include <windows.h>
#endif


//think different
#if defined(__APPLE__) && !defined (VMDMESA)
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "IDebugDraw.h"
//see IDebugDraw.h for modes
static int	sDebugMode = 0;

int		getDebugMode()
{
	return sDebugMode ;
}

void	setDebugMode(int mode)
{
	sDebugMode = mode;
}




#include "GlutStuff.h"

void myinit(void) {

    GLfloat light_ambient[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat light_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
    /*	light_position is NOT default value	*/
    GLfloat light_position0[] = { 1.0, 1.0, 1.0, 0.0 };
    GLfloat light_position1[] = { -1.0, -1.0, -1.0, 0.0 };
  
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position0);
  
    glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular);
    glLightfv(GL_LIGHT1, GL_POSITION, light_position1);

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
 

    glShadeModel(GL_SMOOTH);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

	glClearColor(0.8,0.8,0.8,0);

    //  glEnable(GL_CULL_FACE);
    //  glCullFace(GL_BACK);
}


static float DISTANCE = 15; 
void	setCameraDistance(float dist)
{
	DISTANCE  = dist;
}

static float ele = 0, azi = 0;
float eye[3] = {0, 0, DISTANCE};
static float center[3] = {0, 0, 0};
static const double SCALE_BOTTOM = 0.5;
static const double SCALE_FACTOR = 2;

bool stepping= true;
bool singleStep = false;


static bool idle = false;
  
void toggleIdle() {

	
    if (idle) {
		glutIdleFunc(clientMoveAndDisplay);
        idle = false;
    }
    else {
        glutIdleFunc(0);
        idle = true;
    }
}

#include "SimdMatrix3x3.h"

SimdVector3 gCameraUp(0,1,0);
int	gForwardAxis = 2;

void setCamera() {


    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
	float rele = ele * 0.01745329251994329547;// rads per deg
    float razi = azi * 0.01745329251994329547;// rads per deg
    

	SimdQuaternion rot(gCameraUp,razi);


	SimdVector3 eyePos(0,0,0);
	eyePos[gForwardAxis] = -DISTANCE;

	SimdVector3 forward(eyePos[0],eyePos[1],eyePos[2]);
	SimdVector3 right = gCameraUp.cross(forward);
	SimdQuaternion roll(right,-rele);

	eyePos = SimdMatrix3x3(rot) * SimdMatrix3x3(roll) * eyePos;

	eye[0] = eyePos.getX();
	eye[1] = eyePos.getY();
	eye[2] = eyePos.getZ();
 
    glFrustum(-1.0, 1.0, -1.0, 1.0, 1.0, 100.0);
    gluLookAt(eye[0], eye[1], eye[2], 
              center[0], center[1], center[2], 
			  gCameraUp.getX(),gCameraUp.getY(),gCameraUp.getZ());
    glMatrixMode(GL_MODELVIEW);
}



const float STEPSIZE = 5;

void stepLeft() { azi -= STEPSIZE; if (azi < 0) azi += 360; setCamera(); }
void stepRight() { azi += STEPSIZE; if (azi >= 360) azi -= 360; setCamera(); }
void stepFront() { ele += STEPSIZE; if (azi >= 360) azi -= 360; setCamera(); }
void stepBack() { ele -= STEPSIZE; if (azi < 0) azi += 360; setCamera(); }
void zoomIn() { DISTANCE -= 1; setCamera(); }
void zoomOut() { DISTANCE += 1; setCamera(); }


int glutScreenWidth = 0;
int glutScreenHeight = 0;

void myReshape(int w, int h) {
	glutScreenWidth = w;
	glutScreenHeight = h;

    glViewport(0, 0, w, h);
    setCamera();
}

int lastKey  = 0;

void defaultKeyboard(unsigned char key, int x, int y)
{
	lastKey = 0;

    switch (key) 
    {
    case 'q' : exit(0); break;

    case 'l' : stepLeft(); break;
    case 'r' : stepRight(); break;
    case 'f' : stepFront(); break;
    case 'b' : stepBack(); break;
    case 'z' : zoomIn(); break;
    case 'x' : zoomOut(); break;
    case 'i' : toggleIdle(); break;
	case 'h':
			if (sDebugMode & IDebugDraw::DBG_NoHelpText)
				sDebugMode = sDebugMode & (~IDebugDraw::DBG_NoHelpText);
			else
				sDebugMode |= IDebugDraw::DBG_NoHelpText;
			break;

	case 'w':
			if (sDebugMode & IDebugDraw::DBG_DrawWireframe)
				sDebugMode = sDebugMode & (~IDebugDraw::DBG_DrawWireframe);
			else
				sDebugMode |= IDebugDraw::DBG_DrawWireframe;
		   break;

   case 'p':
	   if (sDebugMode & IDebugDraw::DBG_ProfileTimings)
		sDebugMode = sDebugMode & (~IDebugDraw::DBG_ProfileTimings);
	else
		sDebugMode |= IDebugDraw::DBG_ProfileTimings;
   break;

   case 'm':
	   if (sDebugMode & IDebugDraw::DBG_EnableSatComparison)
		sDebugMode = sDebugMode & (~IDebugDraw::DBG_EnableSatComparison);
	else
		sDebugMode |= IDebugDraw::DBG_EnableSatComparison;
   break;

   case 'n':
	   if (sDebugMode & IDebugDraw::DBG_DisableBulletLCP)
		sDebugMode = sDebugMode & (~IDebugDraw::DBG_DisableBulletLCP);
	else
		sDebugMode |= IDebugDraw::DBG_DisableBulletLCP;
   break;

	case 't' : 
			if (sDebugMode & IDebugDraw::DBG_DrawText)
				sDebugMode = sDebugMode & (~IDebugDraw::DBG_DrawText);
			else
				sDebugMode |= IDebugDraw::DBG_DrawText;
		   break;
	case 'y':		
			if (sDebugMode & IDebugDraw::DBG_DrawFeaturesText)
				sDebugMode = sDebugMode & (~IDebugDraw::DBG_DrawFeaturesText);
			else
				sDebugMode |= IDebugDraw::DBG_DrawFeaturesText;
		break;
	case 'a':	
		if (sDebugMode & IDebugDraw::DBG_DrawAabb)
				sDebugMode = sDebugMode & (~IDebugDraw::DBG_DrawAabb);
			else
				sDebugMode |= IDebugDraw::DBG_DrawAabb;
			break;
		case 'c' : 
			if (sDebugMode & IDebugDraw::DBG_DrawContactPoints)
				sDebugMode = sDebugMode & (~IDebugDraw::DBG_DrawContactPoints);
			else
				sDebugMode |= IDebugDraw::DBG_DrawContactPoints;
			break;

		case 'd' : 
			if (sDebugMode & IDebugDraw::DBG_NoDeactivation)
				sDebugMode = sDebugMode & (~IDebugDraw::DBG_NoDeactivation);
			else
				sDebugMode |= IDebugDraw::DBG_NoDeactivation;
			break;

		

	case 'o' :
		{
			stepping = !stepping;
			break;
		}
	case 's' : clientMoveAndDisplay(); break;
//    case ' ' : newRandom(); break;
	case ' ':
		clientResetScene();
			break;
	case '1':
		{
			if (sDebugMode & IDebugDraw::DBG_EnableCCD)
				sDebugMode = sDebugMode & (~IDebugDraw::DBG_EnableCCD);
			else
				sDebugMode |= IDebugDraw::DBG_EnableCCD;
			break;
		}
    default:
//        std::cout << "unused key : " << key << std::endl;
        break;
    }

	glutPostRedisplay();

}


void mySpecial(int key, int x, int y)
{
    switch (key) 
    {
    case GLUT_KEY_LEFT : stepLeft(); break;
    case GLUT_KEY_RIGHT : stepRight(); break;
    case GLUT_KEY_UP : stepFront(); break;
    case GLUT_KEY_DOWN : stepBack(); break;
    case GLUT_KEY_PAGE_UP : zoomIn(); break;
    case GLUT_KEY_PAGE_DOWN : zoomOut(); break;
    case GLUT_KEY_HOME : toggleIdle(); break;
    default:
//        std::cout << "unused (special) key : " << key << std::endl;
        break;
    }

	glutPostRedisplay();

}


void goodbye( void)
{
    printf("goodbye \n");
    exit(0);
}


void menu(int choice)
{
    static int fullScreen = 0;
    static int px, py, sx, sy;

    switch(choice) {
    case 1:
        if (fullScreen == 1) {
            glutPositionWindow(px,py);
            glutReshapeWindow(sx,sy);
            glutChangeToMenuEntry(1,"Full Screen",1);
            fullScreen = 0;
        } else {
            px=glutGet((GLenum)GLUT_WINDOW_X);
            py=glutGet((GLenum)GLUT_WINDOW_Y);
            sx=glutGet((GLenum)GLUT_WINDOW_WIDTH);
            sy=glutGet((GLenum)GLUT_WINDOW_HEIGHT);
            glutFullScreen();
            glutChangeToMenuEntry(1,"Close Full Screen",1);
            fullScreen = 1;
        }
        break;
    case 2:
        toggleIdle();
        break;
    case 3:
        goodbye();
        break;
    default:
        break;
    }
}

void createMenu()
{
    glutCreateMenu(menu);
    glutAddMenuEntry("Full Screen", 1);
    glutAddMenuEntry("Toggle Idle (Start/Stop)", 2);
    glutAddMenuEntry("Quit", 3);
    glutAttachMenu(GLUT_RIGHT_BUTTON);
}



int glutmain(int argc, char **argv,int width,int height,const char* title) {
    

	glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowPosition(0, 0);
    glutInitWindowSize(width, height);
    glutCreateWindow(title);
	
    myinit();
    glutKeyboardFunc(clientKeyboard);
    glutSpecialFunc(mySpecial);
    glutReshapeFunc(myReshape);
    //createMenu();
	glutIdleFunc(clientMoveAndDisplay);
	glutMouseFunc(clientMouseFunc);
	glutMotionFunc(clientMotionFunc);
	glutDisplayFunc( clientDisplay );

	clientMoveAndDisplay();
	
    glutMainLoop();
    return 0;
}
