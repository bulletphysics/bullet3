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

/// September 2006: CharacterDemo is work in progress, this file is mostly just a placeholder
/// This CharacterDemo file is very early in development, please check it later
/// One todo is a basic engine model:
/// A function that maps user input (throttle) into torque/force applied on the wheels
/// with gears etc.
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"

#include "GLDebugDrawer.h"
#include <stdio.h> //printf debugging

#include "GL_ShapeDrawer.h"

#include "GlutStuff.h"
#include "CharacterDemo.h"
#ifdef DYNAMIC_CHARACTER_CONTROLLER
#include "DynamicCharacterController.h"
#else
#include "KinematicCharacterController.h"
#endif

const int maxProxies = 32766;
const int maxOverlap = 65535;

static int gForward = 0;
static int gBackward = 0;
static int gLeft = 0;
static int gRight = 0;
static int gJump = 0;

#define QUAKE_BSP_IMPORTING 1

#ifdef QUAKE_BSP_IMPORTING
#include "Demos/BspDemo/BspLoader.h"
#include "Demos/BspDemo/BspConverter.h"
#endif //QUAKE_BSP_IMPORTING


class BspToBulletConverter : public BspConverter
{
	CharacterDemo* m_demoApp;

public:

	BspToBulletConverter(CharacterDemo*	demoApp)
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
				startTransform.setOrigin(btVector3(0,-10.0f,0.0f));
				//this create an internal copy of the vertices
				for (int i = 0; i < vertices.size(); i++)
				{
					vertices[i] *= btScalar(0.5);
					float t = vertices[i].getZ() * btScalar(0.75);
					vertices[i].setZ(-vertices[i].getY());
					vertices[i].setY(t);
				}

				btCollisionShape* shape = new btConvexHullShape(&(vertices[0].getX()),vertices.size());
				m_demoApp->m_collisionShapes.push_back(shape);

				//btRigidBody* body = m_demoApp->localCreateRigidBody(mass, startTransform,shape);
				m_demoApp->localCreateRigidBody(mass, startTransform,shape);
			}
		}
};

CharacterDemo::CharacterDemo()
:
m_cameraHeight(4.f),
m_minCameraDistance(3.f),
m_maxCameraDistance(10.f),
m_indexVertexArrays(0),
m_vertices(0)
{
	m_character = 0;
	m_cameraPosition = btVector3(30,30,30);
}

CharacterDemo::~CharacterDemo()
{
	//cleanup in the reverse order of creation/initialization
	if (m_character)
		m_character->destroy (m_dynamicsWorld);


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

	delete m_indexVertexArrays;
	delete m_vertices;

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete solver
	delete m_constraintSolver;

	//delete broadphase
	delete m_overlappingPairCache;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;

}

class MyCustomOverlappingPairCallback : public btOverlappingPairCallback
{

	CharacterDemo*	m_characterDemo;
	btCollisionObject*	m_characterCollider;

	btHashedOverlappingPairCache*	m_hashPairCache;

	struct customOverlapFilterCallback : public btOverlapFilterCallback
	{
		bool needBroadphaseCollision(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1) const
		{
			// we already know that the character proxy is either proxy0 or proxy1
			return true;
		}
	} myCustomOverlapFilterCallback;

public:

	MyCustomOverlappingPairCallback(CharacterDemo* demo,btCollisionObject* characterCollider)
		:m_characterDemo(demo),
		m_characterCollider(characterCollider)
	{
		m_hashPairCache = new btHashedOverlappingPairCache();
		m_hashPairCache->setOverlapFilterCallback (&myCustomOverlapFilterCallback);
	}
	
	virtual ~MyCustomOverlappingPairCallback()
	{
		delete m_hashPairCache;
	}

	virtual btBroadphasePair*	addOverlappingPair(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1)
	{
		if (proxy0->m_clientObject==m_characterCollider || proxy1->m_clientObject==m_characterCollider)
		{
			//printf("addOverlappingPair (%p,%p)\n",proxy0,proxy1);
			return m_hashPairCache->addOverlappingPair(proxy0,proxy1);
		}
		return 0;
	}

	virtual void*	removeOverlappingPair(btBroadphaseProxy* proxy0,btBroadphaseProxy* proxy1,btDispatcher* dispatcher)
	{
		if (proxy0->m_clientObject==m_characterCollider || proxy1->m_clientObject==m_characterCollider)
		{
			//printf("removeOverlappingPair (%p,%p)\n",proxy0,proxy1);
			return m_hashPairCache->removeOverlappingPair(proxy0,proxy1,dispatcher);
		}
		return 0;
	}

	virtual void	removeOverlappingPairsContainingProxy(btBroadphaseProxy* proxy0,btDispatcher* dispatcher)
	{
		if (proxy0->m_clientObject==m_characterCollider)
		{
			//printf("removeOverlappingPairsContainingProxy (%p)\n",proxy0);
			m_hashPairCache->removeOverlappingPairsContainingProxy(proxy0,dispatcher);
		}
	}
	
	btBroadphasePairArray&	getOverlappingPairArray()
	{
		return m_hashPairCache->getOverlappingPairArray();
	}

	btOverlappingPairCache* getOverlappingPairCache()
	{
		return m_hashPairCache;
	}

};

void CharacterDemo::initPhysics()
{
	btCollisionShape* groundShape = new btBoxShape(btVector3(50,3,50));
	m_collisionShapes.push_back(groundShape);
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	btVector3 worldMin(-1000,-1000,-1000);
	btVector3 worldMax(1000,1000,1000);
	btAxisSweep3* sweepBP = new btAxisSweep3(worldMin,worldMax);
	m_overlappingPairCache = sweepBP;
		
	m_constraintSolver = new btSequentialImpulseConstraintSolver();
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_overlappingPairCache,m_constraintSolver,m_collisionConfiguration);
	//m_dynamicsWorld->setGravity(btVector3(0,0,0));
	btTransform tr;
	tr.setIdentity();

#define USE_BSP_STAGE
#ifdef USE_BSP_STAGE
#ifdef QUAKE_BSP_IMPORTING
	char* bspfilename = "BspDemo.bsp";
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
#else
#define  USE_TRIMESH_GROUND 1
#ifdef USE_TRIMESH_GROUND
	int i;

const float TRIANGLE_SIZE=20.f;

	//create a triangle-mesh ground
	int vertStride = sizeof(btVector3);
	int indexStride = 3*sizeof(int);

	const int NUM_VERTS_X = 20;
	const int NUM_VERTS_Y = 20;
	const int totalVerts = NUM_VERTS_X*NUM_VERTS_Y;
	
	const int totalTriangles = 2*(NUM_VERTS_X-1)*(NUM_VERTS_Y-1);

	m_vertices = new btVector3[totalVerts];
	int*	gIndices = new int[totalTriangles*3];

	

	for ( i=0;i<NUM_VERTS_X;i++)
	{
		for (int j=0;j<NUM_VERTS_Y;j++)
		{
			float wl = .2f;
			//height set to zero, but can also use curved landscape, just uncomment out the code
			float height = 20.f*sinf(float(i)*wl)*cosf(float(j)*wl);
#ifdef FORCE_ZAXIS_UP
			m_vertices[i+j*NUM_VERTS_X].setValue(
				(i-NUM_VERTS_X*0.5f)*TRIANGLE_SIZE,
				(j-NUM_VERTS_Y*0.5f)*TRIANGLE_SIZE,
				height
				);

#else
			m_vertices[i+j*NUM_VERTS_X].setValue(
				(i-NUM_VERTS_X*0.5f)*TRIANGLE_SIZE,
				height,
				(j-NUM_VERTS_Y*0.5f)*TRIANGLE_SIZE);
#endif

		}
	}

	int index=0;
	for ( i=0;i<NUM_VERTS_X-1;i++)
	{
		for (int j=0;j<NUM_VERTS_Y-1;j++)
		{
			gIndices[index++] = j*NUM_VERTS_X+i;
			gIndices[index++] = j*NUM_VERTS_X+i+1;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;

			gIndices[index++] = j*NUM_VERTS_X+i;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i+1;
			gIndices[index++] = (j+1)*NUM_VERTS_X+i;
		}
	}
	
	m_indexVertexArrays = new btTriangleIndexVertexArray(totalTriangles,
		gIndices,
		indexStride,
		totalVerts,(btScalar*) &m_vertices[0].x(),vertStride);

	bool useQuantizedAabbCompression = true;
	groundShape = new btBvhTriangleMeshShape(m_indexVertexArrays,useQuantizedAabbCompression);
	
	tr.setOrigin(btVector3(0,-4.5f,0));

#else
	//testing btHeightfieldTerrainShape
	int width=128;
	int length=128;
	unsigned char* heightfieldData = new unsigned char[width*length];
	{
		for (int i=0;i<width*length;i++)
		{
			heightfieldData[i]=0;
		}
	}

	char*	filename="heightfield128x128.raw";
	FILE* heightfieldFile = fopen(filename,"r");
	if (!heightfieldFile)
	{
		filename="../../heightfield128x128.raw";
		heightfieldFile = fopen(filename,"r");
	}
	if (heightfieldFile)
	{
		int numBytes =fread(heightfieldData,1,width*length,heightfieldFile);
		//btAssert(numBytes);
		if (!numBytes)
		{
			printf("couldn't read heightfield at %s\n",filename);
		}
		fclose (heightfieldFile);
	}
	

	btScalar maxHeight = 20000.f;
	
	bool useFloatDatam=false;
	bool flipQuadEdges=false;

	btHeightfieldTerrainShape* heightFieldShape = new btHeightfieldTerrainShape(width,length,heightfieldData,maxHeight,upIndex,useFloatDatam,flipQuadEdges);;
	groundShape = heightFieldShape;
	
	heightFieldShape->setUseDiamondSubdivision(true);

	btVector3 localScaling(20,20,20);
	localScaling[upIndex]=1.f;
	groundShape->setLocalScaling(localScaling);

	tr.setOrigin(btVector3(0,-64.5f,0));

#endif //

	m_collisionShapes.push_back(groundShape);
	//create ground object
	localCreateRigidBody(0,tr,groundShape);



#define CUBE_HALF_EXTENTS 0.5
#define EXTRA_HEIGHT 10.0
	btBoxShape* boxShape = new btBoxShape (btVector3(1.0, 1.0, 1.0));
	m_collisionShapes.push_back (boxShape);
#define DO_WALL
#ifdef DO_WALL
	for (i=0;i<50;i++)
	{
		btCollisionShape* shape = boxShape;
		//shape->setMargin(gCollisionMargin);

		bool isDyna = i>0;

		btTransform trans;
		trans.setIdentity();
		
		if (i>0)
		{
			//stack them
			int colsize = 10;
			int row = (i*CUBE_HALF_EXTENTS*2)/(colsize*2*CUBE_HALF_EXTENTS);
			int row2 = row;
			int col = (i)%(colsize)-colsize/2;


			if (col>3)
			{
				col=11;
				row2 |=1;
			}

			btVector3 pos(col*2*CUBE_HALF_EXTENTS + (row2%2)*CUBE_HALF_EXTENTS,
				row*2*CUBE_HALF_EXTENTS+CUBE_HALF_EXTENTS+EXTRA_HEIGHT,0);

			trans.setOrigin(pos);
		} else
		{
			trans.setOrigin(btVector3(0,EXTRA_HEIGHT-CUBE_HALF_EXTENTS,0));
		}

		float mass = 1.f;

		if (!isDyna)
			mass = 0.f;
	
		btRigidBody* body = localCreateRigidBody(mass,trans,shape);
#ifdef USE_KINEMATIC_GROUND
		if (mass == 0.f)
		{
			body->setCollisionFlags( body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
			body->setActivationState(DISABLE_DEACTIVATION);
		}
#endif //USE_KINEMATIC_GROUND
		
	}
#endif

#endif

#ifdef DYNAMIC_CHARACTER_CONTROLLER
	m_character = new DynamicCharacterController ();
#else
	m_character = new KinematicCharacterController ();
#endif
	m_character->setup (m_dynamicsWorld);

	//some custom callback sample
	m_customPairCallback = new MyCustomOverlappingPairCallback(this,m_character->getCollisionObject());
	sweepBP->setOverlappingPairUserCallback(m_customPairCallback);
	m_character->registerPairCache (m_customPairCallback->getOverlappingPairCache());
	clientResetScene();
	
	setCameraDistance(26.f);

}


//to be implemented by the demo
void CharacterDemo::renderme()
{
	updateCamera();

	DemoApplication::renderme();
}

void CharacterDemo::clientMoveAndDisplay()
{

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	float dt = getDeltaTimeMicroseconds() * 0.000001f;

	/* Character stuff &*/
	if (m_character)
	{			
		m_character->preStep (m_dynamicsWorld);
		m_character->playerStep (m_dynamicsWorld, dt, gForward, gBackward, gLeft, gRight, gJump);
	}

#if 0
	printf("numPairs = %d\n",m_customPairCallback->getOverlappingPairArray().size());
	{
		btManifoldArray	manifoldArray;
		for (int i=0;i<m_customPairCallback->getOverlappingPairArray().size();i++)
		{
			manifoldArray.clear();
			
			const btBroadphasePair& pair = m_customPairCallback->getOverlappingPairArray()[i];
			btBroadphasePair* collisionPair = m_overlappingPairCache->getOverlappingPairCache()->findPair(pair.m_pProxy0,pair.m_pProxy1);

			if (!collisionPair)
				continue;

			if (collisionPair->m_algorithm)
				collisionPair->m_algorithm->getAllContactManifolds(manifoldArray);

			for (int j=0;j<manifoldArray.size();j++)
			{
				btPersistentManifold* manifold = manifoldArray[j];
				for (int p=0;p<manifold->getNumContacts();p++)
				{
					const btManifoldPoint&pt = manifold->getContactPoint(p);

					m_dynamicsWorld->getDebugDrawer()->drawContactPoint(pt.getPositionWorldOnB(),pt.m_normalWorldOnB,pt.getDistance(),pt.getLifeTime(),btVector3(1.f,1.f,0.f));
				}
			}
		}
	}
#endif


	
	if (m_dynamicsWorld)
	{
		//during idle mode, just run 1 simulation step maximum
		int maxSimSubSteps = m_idle ? 1 : 2;
		if (m_idle)
			dt = 1.0/420.f;

		int numSimSteps = m_dynamicsWorld->stepSimulation(dt,maxSimSubSteps);
		
		//optional but useful: debug drawing
		if (m_dynamicsWorld)
			m_dynamicsWorld->debugDrawWorld();

//#define VERBOSE_FEEDBACK
#ifdef VERBOSE_FEEDBACK
		if (!numSimSteps)
			printf("Interpolated transforms\n");
		else
		{
			if (numSimSteps > maxSimSubSteps)
			{
				//detect dropping frames
				printf("Dropped (%i) simulation steps out of %i\n",numSimSteps - maxSimSubSteps,numSimSteps);
			} else
			{
				printf("Simulated (%i) steps\n",numSimSteps);
			}
		}
#endif //VERBOSE_FEEDBACK

	}

	
	




#ifdef USE_QUICKPROF 
        btProfiler::beginBlock("render"); 
#endif //USE_QUICKPROF 


	renderme(); 

#ifdef USE_QUICKPROF 
        btProfiler::endBlock("render"); 
#endif 
	

	glFlush();
	glutSwapBuffers();

}



void CharacterDemo::displayCallback(void) 
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	renderme();

	//optional but useful: debug drawing
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();


	glFlush();
	glutSwapBuffers();
}

void CharacterDemo::clientResetScene()
{	
	m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(m_character->getCollisionObject()->getBroadphaseHandle(),getDynamicsWorld()->getDispatcher());
	
	m_character->reset ();
	m_character->warp (btVector3(0.0, 1.75, 0.0));
}

void CharacterDemo::specialKeyboardUp(int key, int x, int y)
{
   switch (key) 
    {
    case GLUT_KEY_UP:
	{
		gForward = 0;
	}
	break;
	case GLUT_KEY_DOWN:
	{	
		gBackward = 0;
	}
	break;
	case GLUT_KEY_LEFT:
	{
		gLeft = 0;
	}
	break;
	case GLUT_KEY_RIGHT:
	{
		gRight = 0;
	}
	break;
	default:
		DemoApplication::specialKeyboardUp(key,x,y);
        break;
    }
}


void CharacterDemo::specialKeyboard(int key, int x, int y)
{

//	printf("key = %i x=%i y=%i\n",key,x,y);

    switch (key) 
    {
    case GLUT_KEY_UP:
	{
		gForward = 1;
	}
	break;
	case GLUT_KEY_DOWN:
	{	
		gBackward = 1;
	}
	break;
	case GLUT_KEY_LEFT:
	{
		gLeft = 1;
	}
	break;
	case GLUT_KEY_RIGHT:
	{
		gRight = 1;
	}
	break;
	case GLUT_KEY_F1:
	{
		if (m_character && m_character->canJump())
			gJump = 1;
	}
	break;
	default:
		DemoApplication::specialKeyboard(key,x,y);
        break;
    }

//	glutPostRedisplay();


}

void	CharacterDemo::updateCamera()
{
	
//#define DISABLE_CAMERA 1
#ifdef DISABLE_CAMERA
	DemoApplication::updateCamera();
	return;
#endif //DISABLE_CAMERA

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	btTransform characterWorldTrans;

	//look at the vehicle
	characterWorldTrans = m_character->getCollisionObject()->getWorldTransform();
	btVector3 up = characterWorldTrans.getBasis()[1];
	btVector3 backward = -characterWorldTrans.getBasis()[2];
	up.normalize ();
	backward.normalize ();

	m_cameraTargetPosition = characterWorldTrans.getOrigin();
	m_cameraPosition = m_cameraTargetPosition + up * 2.0 + backward * 2.0;
		
	//update OpenGL camera settings
    glFrustum(-1.0, 1.0, -1.0, 1.0, 1.0, 10000.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

    gluLookAt(m_cameraPosition[0],m_cameraPosition[1],m_cameraPosition[2],
		      m_cameraTargetPosition[0],m_cameraTargetPosition[1], m_cameraTargetPosition[2],
			  m_cameraUp.getX(),m_cameraUp.getY(),m_cameraUp.getZ());
  


}

