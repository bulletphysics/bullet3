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

#include "Render.h"
#include "Test.h"

namespace
{
	int testIndex = 0;
	int testSelection = 0;
	TestEntry* entry;
	Test* test;
	Settings settings;
	int width = 640;
	int height = 480;
	int framePeriod = 16;
	int mainWindow;
	GLUI *glui;
	float viewZoom = 20.0f;
	float viewX = 0.0f;
	float viewY = 0.0f;
	int tx, ty, tw, th;
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

	gluOrtho2D(viewZoom * (viewX - ratio), viewZoom * (ratio + viewX),
		viewZoom * (viewY - 0.1), viewZoom * (viewY + 1.9));
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

	test->SetTextLine(30);
	test->Step(&settings);

///	DrawString(5, 15, entry->name);

	glutSwapBuffers();

	if (testSelection != testIndex)
	{
		testIndex = testSelection;
		delete test;
		entry = g_testEntries + testIndex;
		test = entry->createFcn();
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

		// Press 'a' to zoom in.
	case 'a':
		viewZoom = btMax(viewZoom - 1.0f, 1.0f);
		Resize(width, height);
		break;

		// Press 'z' to zoom out.
	case 'z':
		viewZoom = btMin(viewZoom + 1.0f, 100.0f);
		Resize(width, height);
		break;

		// Press 'r' to reset.
	case 'r':
		delete test;
		test = entry->createFcn();
		break;

		// Press space to launch a bomb.
	case ' ':
		if (test)
		{
			test->LaunchBomb();
		}
		break;

	case 'h':
		settings.pause = !settings.pause;
		break;

	default:
		if (test)
		{
			test->Keyboard(key);
		}
	}
}

void KeyboardSpecial(int key, int x, int y)
{

	switch (key)
	{
		// Press left to pan left.
	case GLUT_KEY_LEFT:
		viewX += 0.1f;
		Resize(width, height);
		break;

		// Press right to pan right.
	case GLUT_KEY_RIGHT:
		viewX -= 0.1f;
		Resize(width, height);
		break;

		// Press down to pan down.
	case GLUT_KEY_DOWN:
		viewY += 0.1f;
		Resize(width, height);
		break;

		// Press up to pan up.
	case GLUT_KEY_UP:
		viewY -= 0.1f;
		Resize(width, height);
		break;

		// Press home to reset the view.
	case GLUT_KEY_HOME:
		viewZoom = 20.0f;
		viewX = 0.0f;
		viewY = 0.0f;
		Resize(width, height);
		break;
	}
}

void Mouse(int button, int state, int x, int y)
{
	// Use the mouse to move things around.
	if (button == GLUT_LEFT_BUTTON)
	{
		if (state == GLUT_DOWN)
		{
		//	b2Vec2 p = ConvertScreenToWorld(x, y);
		//	test->MouseDown(p);
		}
		
		if (state == GLUT_UP)
		{
			test->MouseUp();
		}
	}
}

void MouseMotion(int x, int y)
{
//	b2Vec2 p = ConvertScreenToWorld(x, y);
//	test->MouseMove(p);
}

int main(int argc, char** argv)
{
	entry = g_testEntries + testIndex;
	test = entry->createFcn();

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	glutInitWindowSize(width, height);
	mainWindow = glutCreateWindow("http://bulletphysics.com");
	//glutSetOption (GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);

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
		glui->add_spinner("Iterations", GLUI_SPINNER_INT, &settings.iterationCount);
	iterationSpinner->set_int_limits(1, 100);

	GLUI_Spinner* hertzSpinner =
		glui->add_spinner("Hertz", GLUI_SPINNER_FLOAT, &settings.hz);
	hertzSpinner->set_float_limits(5.0f, 200.0f);

	glui->add_checkbox("Position Correction", &settings.enablePositionCorrection);
	glui->add_checkbox("Warm Starting", &settings.enablePositionCorrection);

	glui->add_separator();

	GLUI_Panel* drawPanel =	glui->add_panel("Draw");
	glui->add_checkbox_to_panel(drawPanel, "AABBs", &settings.drawAABBs);
	glui->add_checkbox_to_panel(drawPanel, "Pairs", &settings.drawPairs);
	glui->add_checkbox_to_panel(drawPanel, "Contacts", &settings.drawContacts);
	glui->add_checkbox_to_panel(drawPanel, "Impulses", &settings.drawImpulses);
	glui->add_checkbox_to_panel(drawPanel, "Statistics", &settings.drawStats);

	int testCount = 0;
	TestEntry* e = g_testEntries;
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
