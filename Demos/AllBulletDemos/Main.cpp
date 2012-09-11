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
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "DemoApplication.h"
#include "DemoEntries.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"

#include "GLDebugDrawer.h"
static GLDebugDrawer dDebugDraw2;

#include "LinearMath/btQuickprof.h"



namespace
{
	int testIndex=1;
	int testSelection=0;
	btDemoEntry* entry;
	DemoApplication* demo;
	int iterationCount;
	int width;
	int height;
	int framePeriod;//todo: test if this value should be 0
	int mainWindow;
	GLUI *glui;
	//float hz;
	float viewZoom=20.f;
	float viewX;
	float viewY;
	int tx, ty, tw, th;
	int	gDrawAabb;
	int	gWireFrame;
    int gDrawNormals;
	int gHelpText;
	int gDebugConstraints;
	int	gDebugContacts;
	int	gDrawTextures=1;
	int gDrawShadows=0;
	int gDrawClusters=0;
	int gDebugNoDeactivation;
	int	gUseWarmstarting;
	int	gRandomizeConstraints;
	int	gUseSplitImpulse;
	float	gErp;
	float	gSlop;
	float	gErp2;
	float	gWarmStartingParameter;
}



void	setDefaultSettings()
{
	viewX = 0.0f;
	viewY = 0.0f;
	framePeriod = 6;//16;//16;//todo: test if this value should be 0
	
	width = 1280;
	height = 768;//480;
	iterationCount = 10;
	gDrawAabb=0;
    gDrawNormals=0;
	gWireFrame=0;
	gDebugContacts=0;
	//enable constraint debug visualization for first demo, only if user hasn't overridden the setting
	if (testSelection>1)
	{
		gDebugConstraints=0;
	} else
	{
		 gDebugConstraints=1;
	}
	gHelpText = 0;
	gDrawTextures=1;
	gDrawShadows=0;
	gDrawClusters=0;

	gDebugNoDeactivation = 0;
	gUseSplitImpulse = 1;
	gUseWarmstarting = 1;
	gRandomizeConstraints = 0;
	gErp = 0.2f;
	gSlop=0.0f;
	gErp2 = 0.81f;
	gWarmStartingParameter = 0.85f;
	
}

void	setDefaultSettingsAndSync()
{
	setDefaultSettings();
	glui->sync_live();
}


void	TogglePause()
{
	if (demo)
		demo->toggleIdle();
}

void	ResetScene()
{
	if (demo)
		demo->clientResetScene();
}

void	NextScene()
{
	testSelection++;
	if (testSelection>1)
        {
                gDebugConstraints=0;
        } else
        {
                 gDebugConstraints=1;
        }

	if(testSelection>28)
		testSelection=0;
	if (glui)
		glui->sync_live();
}


void	SingleSimulationStep()
{
	if (demo)
		demo->clientMoveAndDisplay();
}


void Resize(int w, int h)
{
	width = w;
	height = h;

	GLUI_Master.get_viewport_area( &tx, &ty, &tw, &th );
	glViewport( tx, ty, tw, th );

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	if (demo)
		demo->reshape(tw, th);
}

DemoApplication* CreatDemo(btDemoEntry* entry)
{
	DemoApplication* demo = entry->createFcn();
	btAssert(demo);
	if (demo->getDynamicsWorld())
	{
		demo->getDynamicsWorld()->setDebugDrawer(&dDebugDraw2);
		gDrawTextures = demo->getTexturing();
		gDrawShadows = demo->getShadows();
		if (glui)
			glui->sync_live();
	}
	
#ifndef BT_NO_PROFILE
	CProfileManager::Reset();
#endif //BT_NO_PROFILE

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
	Resize(width, height);

	

	if (gDrawAabb)
	{ 
		demo->setDebugMode(demo->getDebugMode() |btIDebugDraw::DBG_DrawAabb);
	} else
	{
		demo->setDebugMode(demo->getDebugMode() & (~btIDebugDraw::DBG_DrawAabb));
	}
	if (gWireFrame)
	{
		demo->setDebugMode(demo->getDebugMode() |btIDebugDraw::DBG_DrawWireframe);
	} else
	{
		demo->setDebugMode(demo->getDebugMode() & (~btIDebugDraw::DBG_DrawWireframe));
	}
    if (gDrawNormals)
	{
		demo->setDebugMode(demo->getDebugMode() |btIDebugDraw::DBG_DrawNormals);
	} else
	{
		demo->setDebugMode(demo->getDebugMode() & (~btIDebugDraw::DBG_DrawNormals));
	}
	if (gHelpText)
	{
		demo->setDebugMode(demo->getDebugMode() & (~btIDebugDraw::DBG_NoHelpText));
	} else
	{
		demo->setDebugMode(demo->getDebugMode() |btIDebugDraw::DBG_NoHelpText);
	}
	if (gDebugConstraints)
	{
		 demo->setDebugMode(demo->getDebugMode() |btIDebugDraw::DBG_DrawConstraints+btIDebugDraw::DBG_DrawConstraintLimits);
	} else
	{
		demo->setDebugMode(demo->getDebugMode() & (~btIDebugDraw::DBG_DrawConstraints+btIDebugDraw::DBG_DrawConstraintLimits));
	}
	if (gDebugContacts)
	{
		demo->setDebugMode(demo->getDebugMode() |btIDebugDraw::DBG_DrawContactPoints);
	} else
	{
		demo->setDebugMode(demo->getDebugMode() & (~btIDebugDraw::DBG_DrawContactPoints));
	}

	demo->setTexturing(0!=gDrawTextures);
	demo->setShadows(0!=gDrawShadows);
	demo->setDrawClusters(0!=gDrawClusters);
	
	if (gDebugNoDeactivation)
	{
		demo->setDebugMode(demo->getDebugMode() |btIDebugDraw::DBG_NoDeactivation);
	} else
	{
		demo->setDebugMode(demo->getDebugMode() & (~btIDebugDraw::DBG_NoDeactivation));
	}

	


	if (demo->getDynamicsWorld() && demo->getDynamicsWorld()->getWorldType() == BT_DISCRETE_DYNAMICS_WORLD)
	{
		btDiscreteDynamicsWorld* discreteWorld = (btDiscreteDynamicsWorld*) demo->getDynamicsWorld();
		discreteWorld->getSolverInfo().m_numIterations = iterationCount;
		discreteWorld->getSolverInfo().m_erp = gErp;
		discreteWorld->getSolverInfo().m_erp2 = gErp2;
		
		discreteWorld->getSolverInfo().m_linearSlop = gSlop;
				
		discreteWorld->getSolverInfo().m_warmstartingFactor = gWarmStartingParameter;
		discreteWorld->getSolverInfo().m_splitImpulse = gUseSplitImpulse;

	//	btSequentialImpulseConstraintSolver* solver = ((btSequentialImpulseConstraintSolver*) discreteWorld->getConstraintSolver());

		if (gUseWarmstarting)
		{
			discreteWorld->getSolverInfo().m_solverMode |= SOLVER_USE_WARMSTARTING;
		} else
		{
			discreteWorld->getSolverInfo().m_solverMode &= (~SOLVER_USE_WARMSTARTING);
		}
		if (gRandomizeConstraints)
		{
			discreteWorld->getSolverInfo().m_solverMode |= SOLVER_RANDMIZE_ORDER;
		} else
		{
			discreteWorld->getSolverInfo().m_solverMode &= (~SOLVER_RANDMIZE_ORDER);
		}
	}

	if (!demo->isIdle())
	{
		demo->clientMoveAndDisplay();
		

	}
	else
	{
		demo->displayCallback();
	}
	
	if (demo->getDynamicsWorld() && demo->getDynamicsWorld()->getWorldType()==BT_SOFT_RIGID_DYNAMICS_WORLD)
	{
		btSoftRigidDynamicsWorld* softWorld = (btSoftRigidDynamicsWorld*)demo->getDynamicsWorld();
		for (  int i=0;i<softWorld->getSoftBodyArray().size();i++)
		{
			btSoftBody*	psb=(btSoftBody*)softWorld->getSoftBodyArray()[i];
			if (softWorld->getDebugDrawer() && !(softWorld->getDebugDrawer()->getDebugMode() & (btIDebugDraw::DBG_DrawWireframe)))
			{
				btSoftBodyHelpers::DrawFrame(psb,softWorld->getDebugDrawer());
				btSoftBodyHelpers::Draw(psb,softWorld->getDebugDrawer(),softWorld->getDrawFlags());
			}
		}
	}
	
	if (testSelection != testIndex)
	{
		if (testSelection>1)
	        {
                gDebugConstraints=0;
        	} else
        	{
                gDebugConstraints=1;
        	}

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

void	RestartScene()
{
	delete demo;
	entry = g_demoEntries + testIndex;
	demo = CreatDemo(entry);
	viewZoom = 20.0f;
	viewX = 0.0f;
	viewY = 0.0f;
	Resize(width, height);
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

void KeyboardSpecialUp(int key, int x, int y)
{
	if (demo)
	{
		demo->specialKeyboardUp(key,x,y);
	}
	
}


void	GlutIdleFunc()
{
	int current_window, new_window=-1;
    current_window = glutGetWindow();
    if (GLUI_Master.gluis.first_child() != NULL )
	{
		new_window = ((GLUI_Main*)GLUI_Master.gluis.first_child())->getMainWindowId();
	}
    if ( (new_window > 0) && (new_window != current_window )) 
	{
		  //--- Window is changed only if its not already the current window ---
		glutSetWindow( new_window );
	}

	if (demo)
		demo->moveAndDisplay();

	glutSetWindow( current_window );
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

#ifdef BT_USE_FREEGLUT
#include "GL/freeglut_ext.h"
#endif

int main(int argc, char** argv)
{

//#define CHECK_FPU_EXCEPTIONS 1
#ifdef CHECK_FPU_EXCEPTIONS

	int cw = _control87(0, 0);

	// Set the exception masks off, turn exceptions on
	cw &= ~(EM_ZERODIVIDE | EM_INVALID);

	printf("control87 = %#x\n", cw);

	// Set the control word
	_control87(cw, MCW_EM);
#endif //CHECK_FPU_EXCEPTIONS


	setDefaultSettings();
	
	int bulletVersion = btGetVersion();
	printf("Bullet version %d\n",bulletVersion);

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE |GLUT_DEPTH | GLUT_STENCIL);
	glutInitWindowSize(width, height);
	mainWindow = glutCreateWindow("http://bulletphysics.com");
#ifdef BT_USE_FREEGLUT
	glutSetOption (GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
#endif
	entry = g_demoEntries + testIndex;
	demo = CreatDemo(entry);
	
	glutDisplayFunc(SimulationLoop);
	GLUI_Master.set_glutReshapeFunc(Resize);  
	GLUI_Master.set_glutKeyboardFunc(Keyboard);
	GLUI_Master.set_glutSpecialFunc(KeyboardSpecial);
	GLUI_Master.set_glutIdleFunc(GlutIdleFunc);
	GLUI_Master.set_glutSpecialUpFunc(KeyboardSpecialUp);
	GLUI_Master.set_glutMouseFunc(Mouse);
	glutMotionFunc(MouseMotion);


	glui = GLUI_Master.create_glui_subwindow( mainWindow, 
		GLUI_SUBWINDOW_RIGHT );

	

	glui->add_statictext("Tests");
	GLUI_Listbox* testList =
		glui->add_listbox("", &testSelection);
	glui->add_button("Next Scene", 0,(GLUI_Update_CB)NextScene);

	glui->add_separator();

	GLUI_Spinner* iterationSpinner =
		glui->add_spinner("Iterations", GLUI_SPINNER_INT, &iterationCount);
	iterationSpinner->set_int_limits(1, 250);

/*	GLUI_Spinner* hertzSpinner =
		glui->add_spinner("Hertz", GLUI_SPINNER_FLOAT, &hz);
	hertzSpinner->set_float_limits(5.0f, 200.0f);
*/

	
	glui->add_checkbox("DisableDeactivation", &gDebugNoDeactivation);
	glui->add_checkbox("Split Impulse", &gUseSplitImpulse);
	GLUI_Spinner* spinner = 0;

	spinner = glui->add_spinner("ERP", GLUI_SPINNER_FLOAT, &gErp);
//	spinner->set_float_limits(0.f,1.f);
//	spinner = glui->add_spinner("ERP2", GLUI_SPINNER_FLOAT, &gErp2);
	spinner->set_float_limits(0.f,1.f);
	spinner = glui->add_spinner("Slop", GLUI_SPINNER_FLOAT, &gSlop);
	spinner->set_float_limits(0.f,1.f);
//	spinner = glui->add_spinner("WSP", GLUI_SPINNER_FLOAT,&gWarmStartingParameter);
//	spinner->set_float_limits (0.f,1.0);
	glui->add_checkbox("Warmstarting", &gUseWarmstarting);
	glui->add_checkbox("Randomize Constraints", &gRandomizeConstraints);
	

	glui->add_button("Reset Defaults", 0,(GLUI_Update_CB)setDefaultSettingsAndSync);
	glui->add_separator();

	GLUI_Panel* drawPanel =	glui->add_panel("Debug Draw");

	
	glui->add_checkbox_to_panel(drawPanel, "Help", &gHelpText);
	glui->add_checkbox_to_panel(drawPanel, "AABBs", &gDrawAabb);
	glui->add_checkbox_to_panel(drawPanel, "Wireframe", &gWireFrame);
    glui->add_checkbox_to_panel(drawPanel, "Normals", &gDrawNormals);
	glui->add_checkbox_to_panel(drawPanel, "Contacts", &gDebugContacts);
	glui->add_checkbox_to_panel(drawPanel, "Constraints", &gDebugConstraints);

	glui->add_checkbox_to_panel(drawPanel, "Textures", &gDrawTextures);
	glui->add_checkbox_to_panel(drawPanel, "Shadows", &gDrawShadows);
	glui->add_checkbox_to_panel(drawPanel, "Clusters", &gDrawClusters);

	int testCount = 0;
	btDemoEntry* e = g_demoEntries;
	while (e->createFcn)
	{
		testList->add_item(testCount, e->name);
		++testCount;
		++e;
	}
	
	glui->add_separator();

	glui->add_button("Toggle Pause", 0,(GLUI_Update_CB)TogglePause);
	
	glui->add_button("Single Step", 0,(GLUI_Update_CB)SingleSimulationStep);
	glui->add_button("Reset Scene", 0,(GLUI_Update_CB)ResetScene);
	glui->add_button("Restart Scene", 0,(GLUI_Update_CB)RestartScene);

	glui->add_separator();
	
//	glui->add_button("Exit", 0,(GLUI_Update_CB)exit);

	glui->set_main_gfx_window( mainWindow );

	// Use a timer to control the frame rate.
	glutTimerFunc(framePeriod, Timer, 0);

	glutMainLoop();

	return 0;
}
