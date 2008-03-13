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

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btIDebugDraw.h"
#include "GLDebugDrawer.h"



//COLLADA_DOM and LibXML source code are included in Extras/ folder.
//COLLADA_DOM should compile under all platforms, and is enabled by default.

#include "ColladaConverter.h"

#include "BMF_Api.h"
#include <stdio.h> //printf debugging

float deltaTime = 1.f/60.f;

#include "ColladaDemo.h"
#include "GL_ShapeDrawer.h"
#include "GlutStuff.h"


///custom version of the converter, that creates physics objects/constraints
class MyColladaConverter : public ColladaConverter
{
	DemoApplication*	m_demoApp;
	
	public:
		MyColladaConverter(DemoApplication* demoApp)
		: ColladaConverter(demoApp->getDynamicsWorld()),
		m_demoApp(demoApp)
		{
		}
		
	
	virtual void	setCameraInfo(const btVector3& camUp,int forwardAxis) 
	{
		m_demoApp->setCameraUp(camUp);
		m_demoApp->setCameraForwardAxis(forwardAxis);
	}

};

MyColladaConverter* gColladaConverter = 0;





////////////////////////////////////



GLDebugDrawer debugDrawer;




int main(int argc,char** argv)
{

	/// Import Collada 1.4 Physics objects
	/// also can pass filename in as argument
	const char* filename = "jenga.dae";
	printf("argc=%i\n",argc);
	{
		for (int i=0;i<argc;i++)
		{
			printf("argv[%i]=%s\n",i,argv[i]);
		}
	}
	if (argc>1)
	{
		filename = argv[1];
	}

	ColladaDemo* colladaDemo = new ColladaDemo();

	colladaDemo->initPhysics(filename);

	
//	colladaDemo->clientResetScene();

	colladaDemo->setCameraDistance(26.f);

	return glutmain(argc, argv,640,480,"Bullet COLLADA Physics Viewer http://bullet.sourceforge.net",colladaDemo);
}

void	ColladaDemo::initPhysics(const char* filename)
{
	m_cameraUp = btVector3(0,0,1);
	m_ele = 60;
	m_forwardAxis = 1;

	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
	btVector3 worldMin(-1000,-1000,-1000);
	btVector3 worldMax(1000,1000,1000);
	btBroadphaseInterface* pairCache = new btAxisSweep3(worldMin,worldMax);
	btConstraintSolver* constraintSolver = new btSequentialImpulseConstraintSolver();
	m_dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,pairCache,constraintSolver,collisionConfiguration);

	//m_dynamicsWorld = new btSimpleDynamicsWorld();

	m_dynamicsWorld->setDebugDrawer(&debugDrawer);
	

	MyColladaConverter* converter = new MyColladaConverter(this);

	bool result = converter->load(filename);
	if (result)
	{
		gColladaConverter = converter;
	} else
	{
		gColladaConverter = 0;
		printf("gColladaConverter = 0\n");
	}
}


void ColladaDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	float dt = getDeltaTimeMicroseconds() * 0.000001f;

	m_dynamicsWorld->stepSimulation(dt);

	//optional but useful: debug drawing
	m_dynamicsWorld->debugDrawWorld();

	renderme();

	glFlush();
	glutSwapBuffers();

}



void ColladaDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 


	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	renderme();


	glFlush();
	glutSwapBuffers();
}






void ColladaDemo::keyboardCallback(unsigned char key, int x, int y)
{
	if (key =='e')
	{
		//save a COLLADA .dae physics snapshot
		if (gColladaConverter)
		{
			if (gColladaConverter->getLastFileName())
			{
				//deal with the name pre/post fixing etc.
				char	saveName[550];
				static int saveCount=1;
				sprintf(saveName,"%s%i",gColladaConverter->getLastFileName(),saveCount++);
				char* name = &saveName[0];
				if (name[0] == '/')
				{
					name = &saveName[1];
				} 
				gColladaConverter->save(name);

			} else
			{
				gColladaConverter->save("brandNew.dae");
			}
		}
	} else
	{
	DemoApplication::keyboardCallback(key,x,y);
	}
}

