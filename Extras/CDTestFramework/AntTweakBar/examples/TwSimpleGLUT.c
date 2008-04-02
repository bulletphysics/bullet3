//	---------------------------------------------------------------------------
//
//	@file		TwSimpleGLUT.c
//	@brief		A simple example that uses AntTweakBar with OpenGL and GLUT.
//
//				AntTweakBar: http://www.antisphere.com/Wiki/tools:anttweakbar
//				OpenGL:		 http://www.opengl.org
//				GLUT:		 http://opengl.org/resources/libraries/glut
//	
//	@author		Philippe Decaudin - http://www.antisphere.com
//	@date		2006/05/20
//
//	note:		TAB=4
//
//	Compilation:
//	http://www.antisphere.com/Wiki/tools:anttweakbar:examples#twsimpleglut
//
//	---------------------------------------------------------------------------


#include <AntTweakBar.h>

#include <stdlib.h>
#include <stdio.h>

#include <GL/glut.h>


// This example displays one of the following shapes
typedef enum { SHAPE_TEAPOT=1, SHAPE_TORUS, SHAPE_CONE } Shape;
#define NUM_SHAPES 3
Shape g_CurrentShape = SHAPE_TORUS;
// Shapes scale
float g_Zoom = 1.0f;
// Shapes material
float g_MatAmbient[] = { 0.5f, 0.0f, 0.0f, 1.0f };
float g_MatDiffuse[] = { 1.0f, 1.0f, 0.0f, 1.0f };
// Light parameter
float g_LightMultiplier = 1.0f;


// Callback function called by GLUT to render screen
void Display(void)
{
	float angle = (float)glutGet(GLUT_ELAPSED_TIME)/10.0f;
	float v[4];	// will be used to set light paramters

	// Clear frame buffer
	glClearColor(0, 0, 0, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glEnable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	glEnable(GL_NORMALIZE);

	// Set light
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	v[0] = v[1] = v[2] = g_LightMultiplier*0.4f; v[3] = 1.0f;
	glLightfv(GL_LIGHT0, GL_AMBIENT, v);
	v[0] = v[1] = v[2] = g_LightMultiplier*0.8f; v[3] = 1.0f;
	glLightfv(GL_LIGHT0, GL_DIFFUSE, v);
	v[0] = v[1] = v[2] = 1.0f; v[3] = 0.0f;
	glLightfv(GL_LIGHT0, GL_POSITION, v);

	// Set material
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, g_MatAmbient);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, g_MatDiffuse);

	// Rotate and draw shape
	glPushMatrix();
	glTranslatef(0.5f, -0.3f, 0.0f);
	glRotatef(angle, 1.0f, 5.0f, 0.0f);
	glScalef(g_Zoom, g_Zoom, g_Zoom);
	glCallList(g_CurrentShape);
	glPopMatrix();

	// Draw tweak bars
	TwDraw();

	// Present frame buffer
	glutSwapBuffers();

	// Recall Display at next frame
	glutPostRedisplay();
}


// Callback function called by GLUT when window size changes
void Reshape(int width, int height)
{
	// Set OpenGL viewport and camera
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(40, (double)width/height, 1, 10);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0,0,5, 0,0,0, 0,1,0);
	glTranslatef(0, 0.6f, -1);

	// Send the new window size to AntTweakBar
	TwWindowSize(width, height);
}


// Function called at exit
void Terminate(void)
{ 
	glDeleteLists(SHAPE_TEAPOT, NUM_SHAPES);

	TwTerminate();
}
 

// Main
int main(int argc, char *argv[])
{
	TwBar *bar;	// Pointer to a tweak bar

	// Initialize AntTweakBar
	// (note that AntTweakBar could also be intialize after GLUT, no matter)
	if( !TwInit(TW_OPENGL, NULL) )
	{
		// A fatal error occured	
		fprintf(stderr, "AntTweakBar initialization failed: %s\n", TwGetLastError());
		return 1;
	}

	// Initialize GLUT
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(640, 480);
	glutCreateWindow("AntTweakBar simple example using GLUT");
	glutCreateMenu(NULL);

	// Set GLUT callbacks
	glutDisplayFunc(Display);
	glutReshapeFunc(Reshape);
	atexit(Terminate);	// Called after glutMainLoop ends

	// Set GLUT event callbacks
	// - Directly redirect GLUT mouse button events to AntTweakBar
	glutMouseFunc((GLUTmousebuttonfun)TwEventMouseButtonGLUT);
	// - Directly redirect GLUT mouse motion events to AntTweakBar
	glutMotionFunc((GLUTmousemotionfun)TwEventMouseMotionGLUT);
	// - Directly redirect GLUT mouse "passive" motion events to AntTweakBar (same as MouseMotion)
	glutPassiveMotionFunc((GLUTmousemotionfun)TwEventMouseMotionGLUT);
	// - Directly redirect GLUT key events to AntTweakBar
	glutKeyboardFunc((GLUTkeyboardfun)TwEventKeyboardGLUT);
	// - Directly redirect GLUT special key events to AntTweakBar
	glutSpecialFunc((GLUTspecialfun)TwEventSpecialGLUT);
	// - Send 'glutGetModifers' function pointer to AntTweakBar;
	//   required because the GLUT key event functions do not report key modifiers states.
	TwGLUTModifiersFunc(glutGetModifiers);

	// Create some 3D objects (stored in display lists)
	glNewList(SHAPE_TEAPOT, GL_COMPILE);
	glutSolidTeapot(1.0);
	glEndList();
	glNewList(SHAPE_TORUS, GL_COMPILE);
	glutSolidTorus(0.3, 1.0, 16, 32);
	glEndList();
	glNewList(SHAPE_CONE, GL_COMPILE);
	glutSolidCone(1.0, 1.5, 64, 4);
	glEndList();

	// Create a tweak bar
	bar = TwNewBar("TweakBar");
	// Add 'g_Zoom' to 'bar': it is a modifable (RW) variable of type TW_TYPE_FLOAT. Its key shortcuts are [z] and [Z].
	TwAddVarRW(bar, "Zoom", TW_TYPE_FLOAT, &g_Zoom, " min=0.01 max=2.5 step=0.01 keyIncr=z keyDecr=Z ");
	// Add 'g_LightMultiplier' to 'bar': it is a variable of type TW_TYPE_FLOAT. Its key shortcuts are [+] and [-].
	TwAddVarRW(bar, "Multiplier", TW_TYPE_FLOAT, &g_LightMultiplier, " label='Light booster' min=0.1 max=4 step=0.02 keyIncr='+' keyDecr='-' ");
	// Add 'g_MatAmbient' to 'bar': it is a variable of type TW_TYPE_COLOR3F (3 floats color, alpha is ignored). It is inserted into a group named 'Material'.
	TwAddVarRW(bar, "Ambient", TW_TYPE_COLOR3F, &g_MatAmbient, " group='Material' ");
	// Add 'g_MatDiffuse' to 'bar': it is a variable of type TW_TYPE_COLOR3F (3 floats color, alpha is ignored). It is inserted into group 'Material'.
	TwAddVarRW(bar, "Diffuse", TW_TYPE_COLOR3F, &g_MatDiffuse, " group='Material' ");
	// Add the enum variable 'g_CurrentShape' to 'bar'
	// (before adding an enum variable, its enum type must be declared to AntTweakBar as follow)
	{
		// ShapeEV associates Shape enum values with labels that will be displayed instead of enum values
		TwEnumVal shapeEV[NUM_SHAPES] = { {SHAPE_TEAPOT, "Teapot"}, {SHAPE_TORUS, "Torus"}, {SHAPE_CONE, "Cone"} };
		// Create a type for the enum shapeEV
		TwType shapeType = TwDefineEnum("ShapeType", shapeEV, NUM_SHAPES);
		// add 'g_CurrentShape' to 'bar': it is a variable of type ShapeType. Its key shortcuts are [<] and [>].
		TwAddVarRW(bar, "Shape", shapeType, &g_CurrentShape, " keyIncr='<' keyDecr='>' ");
	}

	// Call the GLUT main loop
	glutMainLoop();

	return 0;
}

