/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#include <stdio.h>

#include "glui/GL/glui.h"
#include "LinearMath/btScalar.h"
#include "LinearMath/btMinMax.h"

#include "DemoApplication.h"
#include "DemoEntries.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"

#include "GLDebugDrawer.h"

static GLDebugDrawer gDebugDrawer;


namespace
{
	int testIndex = 0;
	int testSelection = 0;
	btDemoEntry* entry;
	DemoApplication* demo;
	int iterationCount = 10;
	int width = 640;
	int height = 480;
	int framePeriod = 16;
	int mainWindow;
	GLUI *glui;
	float hz;
	float viewZoom = 20.0f;
	float viewX = 0.0f;
	float viewY = 0.0f;
	int tx, ty, tw, th;
	int	gDrawAabb=0;
	int	gDebugDraw=0;
	int	gDebugContacts=0;
	int gDebugNoDeactivation = 0;
}


void Resize(int w, int h)
{
	width = w;
	height = h;

	GLUI_Master.get_viewport_area( &tx, &ty, &tw, &th );
	glViewport( tx, ty, tw, th );

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	double ratio = (double)tw / (double)th;


	if (demo)
		demo->reshape(w, h);
}

DemoApplication* CreatDemo(btDemoEntry* entry)
{
	DemoApplication* demo = entry->createFcn();
	btAssert(demo);
	if (demo->getDynamicsWorld())
	{
		demo->getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);
	}
	return demo;

}

/*b2Vec2 ConvertScreenToWorld(int x, int y)
{
	b2Vec2 p;

	float ratio = float(tw) / float(th);
	float u = x / float(tw);
	float v = (th - y) / float(th);
	p.x = viewZoom * (viewX - ratio) * (1.0f - u) + viewZoom * (ratio + viewX) * u;
	p.y = viewZoom * (viewY - 0.1f) * (1.0f - v) + viewZoom * (viewY + 1.9f) * v;
	return p;
}
*/

// This is used to control the frame rate (60Hz).
void Timer(int)
{
	glutSetWindow(mainWindow);
	glutPostRedisplay();
	glutTimerFunc(framePeriod, Timer, 0);
}

void SimulationLoop()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//test->SetTextLine(30);
	//test->Step(&settings);
	//sync debugging options
	if (gDrawAabb)
	{ 
		demo->setDebugMode(demo->getDebugMode() |btIDebugDraw::DBG_DrawAabb);
	} else
	{
		demo->setDebugMode(demo->getDebugMode() & (~btIDebugDraw::DBG_DrawAabb));
	}
	if (gDebugDraw)
	{
		demo->setDebugMode(demo->getDebugMode() |btIDebugDraw::DBG_DrawWireframe);
	} else
	{
		demo->setDebugMode(demo->getDebugMode() & (~btIDebugDraw::DBG_DrawWireframe));

	}
	if (gDebugContacts)
	{
		demo->setDebugMode(demo->getDebugMode() |btIDebugDraw::DBG_DrawContactPoints);
	} else
	{
		demo->setDebugMode(demo->getDebugMode() & (~btIDebugDraw::DBG_DrawContactPoints));
	}

	if (gDebugNoDeactivation)
	{
		demo->setDebugMode(demo->getDebugMode() |btIDebugDraw::DBG_NoDeactivation);
	} else
	{
		demo->setDebugMode(demo->getDebugMode() & (~btIDebugDraw::DBG_NoDeactivation));
	}

	if (demo->getDynamicsWorld()->getWorldType() == BT_DISCRETE_DYNAMICS_WORLD)
	{
		btDiscreteDynamicsWorld* discreteWorld = (btDiscreteDynamicsWorld*) demo->getDynamicsWorld();
		discreteWorld->getSolverInfo().m_numIterations = iterationCount;
	}


	demo->clientMoveAndDisplay();


///	DrawString(5, 15, entry->name);

	glutSwapBuffers();

	if (testSelection != testIndex)
	{
		testIndex = testSelection;
		delete demo;
		entry = g_demoEntries + testIndex;
		demo = CreatDemo(entry);
		viewZoom = 20.0f;
		viewX = 0.0f;
		viewY = 0.0f;
		Resize(width, height);
	}
}

void Keyboard(unsigned char key, int x, int y)
{

	switch (key)
	{
	case 27:
		exit(0);
		break;

		// Press 'r' to reset.
	case 'r':
		delete demo;
		demo = CreatDemo(entry);
		Resize(width,height);
		break;

	default:
		if (demo)
		{
			demo->keyboardCallback(key,x,y);
		}
	}
}

void KeyboardSpecial(int key, int x, int y)
{

	if (demo)
	{
		demo->specialKeyboard(key,x,y);
	}
	
}


void Mouse(int button, int state, int x, int y)
{
	if (demo)
		demo->mouseFunc(button,state,x,y);
}

void MouseMotion(int x, int y)
{
	demo->mouseMotionFunc(x,y);
}

int main(int argc, char** argv)
{

	int bulletVersion = btGetVersion();
	printf("Bullet version %d\n",bulletVersion);

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE |GLUT_DEPTH);
	glutInitWindowSize(width, height);
	mainWindow = glutCreateWindow("http://bulletphysics.com");
	//glutSetOption (GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);

	entry = g_demoEntries + testIndex;
	demo = CreatDemo(entry);
	
	glutDisplayFunc(SimulationLoop);
	GLUI_Master.set_glutReshapeFunc(Resize);  
	GLUI_Master.set_glutKeyboardFunc(Keyboard);
	GLUI_Master.set_glutSpecialFunc(KeyboardSpecial);
	GLUI_Master.set_glutMouseFunc(Mouse);
	glutMotionFunc(MouseMotion);


	glui = GLUI_Master.create_glui_subwindow( mainWindow, 
		GLUI_SUBWINDOW_RIGHT );

	glui->add_statictext("Tests");
	GLUI_Listbox* testList =
		glui->add_listbox("", &testSelection);

	glui->add_separator();

	GLUI_Spinner* iterationSpinner =
		glui->add_spinner("Iterations", GLUI_SPINNER_INT, &iterationCount);
	iterationSpinner->set_int_limits(1, 100);

/*	GLUI_Spinner* hertzSpinner =
		glui->add_spinner("Hertz", GLUI_SPINNER_FLOAT, &hz);
	hertzSpinner->set_float_limits(5.0f, 200.0f);
*/

//	glui->add_checkbox("Position Correction", &settings.enablePositionCorrection);
//	glui->add_checkbox("Warm Starting", &settings.enablePositionCorrection);
	glui->add_checkbox("DisableDeactivation", &gDebugNoDeactivation);

	glui->add_separator();

	GLUI_Panel* drawPanel =	glui->add_panel("Draw");

	glui->add_checkbox_to_panel(drawPanel, "AABBs", &gDrawAabb);
	glui->add_checkbox_to_panel(drawPanel, "DebugDrawer", &gDebugDraw);
	glui->add_checkbox_to_panel(drawPanel, "Contacts", &gDebugContacts);

//	glui->add_checkbox_to_panel(drawPanel, "Impulses", &settings.drawImpulses);
//	glui->add_checkbox_to_panel(drawPanel, "Statistics", &settings.drawStats);


	int testCount = 0;
	btDemoEntry* e = g_demoEntries;
	while (e->createFcn)
	{
		testList->add_item(testCount, e->name);
		++testCount;
		++e;
	}

	glui->add_button("Quit", 0,(GLUI_Update_CB)exit);
	glui->set_main_gfx_window( mainWindow );

	// Use a timer to control the frame rate.
	glutTimerFunc(framePeriod, Timer, 0);

	glutMainLoop();

	return 0;
}
