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


#define QUAKE_BSP_IMPORTING 1

#ifdef QUAKE_BSP_IMPORTING
#include "BspLoader.h"
#include "BspConverter.h"
#endif //QUAKE_BSP_IMPORTING


#include <stdio.h> //printf debugging


#include "BspDemo.h"
#include "GL_ShapeDrawer.h"
#include "GlutStuff.h"




#define CUBE_HALF_EXTENTS 1
#define EXTRA_HEIGHT -20.f



///BspToBulletConverter  extends the BspConverter to convert to Bullet datastructures
class BspToBulletConverter : public BspConverter
{
	BspDemo* m_demoApp;

public:

	BspToBulletConverter(BspDemo*	demoApp)
		:m_demoApp(demoApp)
	{
	}

		virtual void	addConvexVerticesCollider(btAlignedObjectArray<btVector3>& vertices, bool isEntity, const btVector3& entityTargetLocation)
		{
			///perhaps we can do something special with entities (isEntity)
			///like adding a collision Triggering (as example)
			
			if (vertices.size() > 0)
			{
				float mass = 0.f;
				btTransform startTransform;
				//can use a shift
				startTransform.setIdentity();
				startTransform.setOrigin(btVector3(0,0,-10.f));
				//this create an internal copy of the vertices
				
				btCollisionShape* shape = new btConvexHullShape(&(vertices[0].getX()),vertices.size());
				m_demoApp->m_collisionShapes.push_back(shape);

				//btRigidBody* body = m_demoApp->localCreateRigidBody(mass, startTransform,shape);
				m_demoApp->localCreateRigidBody(mass, startTransform,shape);
			}
		}
};





////////////////////////////////////







BspDemo::~BspDemo()
{
		//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete solver
	delete m_solver;

	//delete broadphase
	delete m_broadphase;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;
}

void	BspDemo::initPhysics()
{
	const char* bspfilename = "BspDemo.bsp";
	
	initPhysics(bspfilename);
}



void	BspDemo::initPhysics(const char* bspfilename)
{
	setTexturing(true);
	setShadows(false);
	
	m_cameraUp = btVector3(0,0,1);
	m_forwardAxis = 1;

	setCameraDistance(22.f);

	///Setup a Physics Simulation Environment

	m_collisionConfiguration = new btDefaultCollisionConfiguration();
//	btCollisionShape* groundShape = new btBoxShape(btVector3(50,3,50));
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	btVector3 worldMin(-1000,-1000,-1000);
	btVector3 worldMax(1000,1000,1000);
	m_broadphase = new btDbvtBroadphase();
	//m_broadphase = new btAxisSweep3(worldMin,worldMax);
	//btOverlappingPairCache* broadphase = new btSimpleBroadphase();
	m_solver = new btSequentialImpulseConstraintSolver();
	//ConstraintSolver* solver = new OdeConstraintSolver;
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);

	m_dynamicsWorld->setGravity(-m_cameraUp * 10);


#ifdef QUAKE_BSP_IMPORTING

	void* memoryBuffer = 0;
	
	FILE* file = fopen(bspfilename,"r");
	if (!file)
	{
		//try again other path, 
		//sight... visual studio leaves the current working directory in the projectfiles folder
		//instead of executable folder. who wants this default behaviour?!?
		bspfilename = "../../BspDemo.bsp";
		file = fopen(bspfilename,"r");
	}
	if (!file)
	{

		//try again other path, cmake needs 4 levels deep back...
		bspfilename = "../../../../BspDemo.bsp";
		file = fopen(bspfilename,"r");
	}
	if (!file)
	{
		//try again other path, 
		//sight... visual studio leaves the current working directory in the projectfiles folder
		//instead of executable folder. who wants this default behaviour?!?
		bspfilename = "BspDemo.bsp";
		file = fopen(bspfilename,"r");
	}

	if (file)
	{
		BspLoader bspLoader;
		int size=0;
		if (fseek(file, 0, SEEK_END) || (size = ftell(file)) == EOF || fseek(file, 0, SEEK_SET)) {        /* File operations denied? ok, just close and return failure */
			printf("Error: cannot get filesize from %s\n", bspfilename);
		} else
		{
			//how to detect file size?
			memoryBuffer = malloc(size+1);
			fread(memoryBuffer,1,size,file);
			bspLoader.loadBSPFile( memoryBuffer);

			BspToBulletConverter bsp2bullet(this);
			float bspScaling = 0.1f;
			bsp2bullet.convertBsp(bspLoader,bspScaling);

		}
		fclose(file);
	}

#endif




	clientResetScene();

}


void BspDemo::clientMoveAndDisplay()
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



void BspDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 


	renderme();

	//optional but useful: debug drawing
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	glutSwapBuffers();
}






//some code that de-mangles the windows filename passed in as argument
char cleaned_filename[512];
char* getLastFileName()
{
	return cleaned_filename;
}
char* makeExeToBspFilename(const char* lpCmdLine)
{


	// We might get a windows-style path on the command line, this can mess up the DOM which expects
	// all paths to be URI's.  This block of code does some conversion to try and make the input
	// compliant without breaking the ability to accept a properly formatted URI.  Right now this only
	// displays the first filename
	const char *in = lpCmdLine;
	char* out = cleaned_filename;
	*out = '\0';
	// If the first character is a ", skip it (filenames with spaces in them are quoted)
	if(*in == '\"')
	{
		in++;
	}
	int i;
	for(i =0; i<512; i++)
	{
		//if we get '.' we stop as well, unless it's the first character. Then we add .bsp as extension
		// If we hit a null or a quote, stop copying.  This will get just the first filename.
		if(i && (in[0] == '.') && (in[1] == 'e') && (in[2] == 'x') && (in[3] == 'e'))
			break;
			
		// If we hit a null or a quote, stop copying.  This will get just the first filename.
		if(*in == '\0' || *in == '\"')
			break;
		// Copy while swapping backslashes for forward ones
		if(*in == '\\')
		{
			*out = '/';
		}
		else
		{
			*out = *in;
		}
		in++;
		out++;
	}
	*(out++) = '.';
	*(out++) = 'b';
	*(out++) = 's';
	*(out++) = 'p';
	*(out++) = 0;
	
	return cleaned_filename;
}
