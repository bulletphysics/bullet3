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

#include "CcdPhysicsEnvironment.h"
#include "CcdPhysicsController.h"

//#include "GL_LineSegmentShape.h"
#include "CollisionShapes/BoxShape.h"
#include "CollisionShapes/SphereShape.h"
#include "CollisionShapes/CylinderShape.h"
#include "CollisionShapes/ConeShape.h"
#include "CollisionShapes/StaticPlaneShape.h"
#include "CollisionShapes/ConvexHullShape.h"
#include "CollisionShapes/TriangleMesh.h"
#include "CollisionShapes/ConvexTriangleMeshShape.h"
#include "CollisionShapes/TriangleMeshShape.h"
#include "CollisionShapes/TriangleIndexVertexArray.h"
#include "CollisionShapes/CompoundShape.h"



#include "CollisionShapes/Simplex1to4Shape.h"
#include "CollisionShapes/EmptyShape.h"

#include "Dynamics/RigidBody.h"
#include "CollisionDispatch/CollisionDispatcher.h"
#include "BroadphaseCollision/SimpleBroadphase.h"
#include "BroadphaseCollision/AxisSweep3.h"
#include "ConstraintSolver/Point2PointConstraint.h"
#include "ConstraintSolver/HingeConstraint.h"

#include "quickprof.h"
#include "IDebugDraw.h"

#include "GLDebugDrawer.h"


#define QUAKE_BSP_IMPORTING 1

#ifdef QUAKE_BSP_IMPORTING
#include "BspLoader.h"
#include "BspConverter.h"
#endif //QUAKE_BSP_IMPORTING

#include "PHY_Pro.h"
#include "BMF_Api.h"
#include <stdio.h> //printf debugging

float deltaTime = 1.f/60.f;
#include "BspDemo.h"
#include "GL_ShapeDrawer.h"
#include "GlutStuff.h"




#define CUBE_HALF_EXTENTS 1
#define EXTRA_HEIGHT -20.f



///BspToBulletConverter  extends the BspConverter to convert to Bullet datastructures
class BspToBulletConverter : public BspConverter
{
	DemoApplication* m_demoApp;

public:

	BspToBulletConverter(DemoApplication*	demoApp)
		:m_demoApp(demoApp)
	{
	}

		virtual void	AddConvexVerticesCollider(std::vector<SimdVector3>& vertices, bool isEntity, const SimdVector3& entityTargetLocation)
		{
			///perhaps we can do something special with entities (isEntity)
			///like adding a collision Triggering (as example)
			
			if (vertices.size() > 0)
			{
				bool isDynamic = false;
				float mass = 0.f;
				SimdTransform startTransform;
				//can use a shift
				startTransform.setIdentity();
				startTransform.setOrigin(SimdVector3(0,0,-10.f));
				//this create an internal copy of the vertices
				CollisionShape* shape = new ConvexHullShape(&vertices[0],vertices.size());

				m_demoApp->LocalCreatePhysicsObject(isDynamic, mass, startTransform,shape);
			}
		}
};





////////////////////////////////////



GLDebugDrawer debugDrawer;

char* makeExeToBspFilename(const char* lpCmdLine);
char* getLastFileName();


int main(int argc,char** argv)
{

	BspDemo* bspDemo = new BspDemo();

	char* bspfilename = "BspDemo.bsp";

	printf("argc=%i\n",argc);
	{
		for (int i=0;i<argc;i++)
		{
			printf("argv[%i]=%s\n",i,argv[i]);
		}
		
		bspfilename = makeExeToBspFilename(argv[0]);
		printf("new name=%s\n",bspfilename);
	}
	if (argc>1)
	{
		bspfilename = argv[1];
	}

	bspDemo->initPhysics(bspfilename);
	
	bspDemo->setCameraDistance(22.f);

	return glutmain(argc, argv,640,480,"Bullet Quake BSP Physics Viewer http://bullet.sourceforge.net",bspDemo);
}

void	BspDemo::initPhysics(char* bspfilename)
{
	

	m_cameraUp = SimdVector3(0,0,1);
	m_forwardAxis = 1;

	///Setup a Physics Simulation Environment
	CollisionDispatcher* dispatcher = new	CollisionDispatcher();
	SimdVector3 worldAabbMin(-10000,-10000,-10000);
	SimdVector3 worldAabbMax(10000,10000,10000);
	OverlappingPairCache* broadphase = new AxisSweep3(worldAabbMin,worldAabbMax);
	//BroadphaseInterface* broadphase = new SimpleBroadphase();
	m_physicsEnvironmentPtr = new CcdPhysicsEnvironment(dispatcher,broadphase);
	m_physicsEnvironmentPtr->setDeactivationTime(2.f);
	m_physicsEnvironmentPtr->setGravity(0,0,-10);
	m_physicsEnvironmentPtr->setDebugDrawer(&debugDrawer);



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
			bspLoader.LoadBSPFile( memoryBuffer);

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

	m_physicsEnvironmentPtr->proceedDeltaTime(0.f,deltaTime);

	renderme();

	glFlush();
	glutSwapBuffers();

}



void BspDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 


	m_physicsEnvironmentPtr->UpdateAabbs(deltaTime);

	renderme();


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
	*out = NULL;
	// If the first character is a ", skip it (filenames with spaces in them are quoted)
	if(*in == '\"')
	{
		in++;
	}
	if(*(in+1) == ':')
	{
		// Second character is a :, assume we have a path with a drive letter and add a slash at the beginning
		*(out++) = '/';
	}
	int i;
	for(i =0; i<512; i++)
	{
		//if we get '.' we stop as well, unless it's the first character. Then we add .bsp as extension
		// If we hit a null or a quote, stop copying.  This will get just the first filename.
		if(i && (*in == '.'))
			break;
			
		// If we hit a null or a quote, stop copying.  This will get just the first filename.
		if(*in == NULL || *in == '\"')
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
