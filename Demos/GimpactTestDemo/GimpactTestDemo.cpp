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
#include "GimpactTestDemo.h"

#define SHOW_NUM_DEEP_PENETRATIONS

#include "LinearMath/btDefaultMotionState.h"
#include "LinearMath/btIDebugDraw.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btDefaultMotionState.h"
#include "GLDebugFont.h"
/// Including GIMPACT here



#include "GLDebugDrawer.h"

#include "GL_ShapeDrawer.h"
#include "GlutStuff.h"

/// Include Torus Mesh here
#include "TorusMesh.h"
#include "BunnyMesh.h"

#ifdef SHOW_NUM_DEEP_PENETRATIONS 
extern int gNumDeepPenetrationChecks;
extern int gNumSplitImpulseRecoveries;
extern int gNumGjkChecks;
#endif //



//Real			dts = 0.000001f;
Real			dts = 1.0 / 60.0;


///**************************************************************************************
///	GIMPACT Test Demo made by DevO
///
///**************************************************************************************



GimpactConcaveDemo::~GimpactConcaveDemo()
{

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

	delete m_dynamicsWorld;

	delete m_indexVertexArrays;
	delete m_trimeshShape;

	delete m_indexVertexArrays2;
	delete m_trimeshShape2;

	for (i=0;i<m_collisionShapes.size();i++)
	{
		btCollisionShape* shape = m_collisionShapes[i];
		delete shape;
	}


	delete m_gimpactCollisionCreateFunc;

	delete m_collisionConfiguration;
	delete m_dispatcher;
	delete m_broadphase;
	delete m_constraintSolver;

}




//------------------------------------------------------------------------------
void GimpactConcaveDemo::renderme()
{
	updateCamera();


	btScalar m[16];

	if (m_dynamicsWorld)
	{
		btVector3	worldBoundsMin,worldBoundsMax;
		getDynamicsWorld()->getBroadphase()->getBroadphaseAabb(worldBoundsMin,worldBoundsMax);


		int numObjects = m_dynamicsWorld->getNumCollisionObjects();
		btVector3 wireColor(1,0,0);
		for (int i=0;i<numObjects;i++)
		{
			btCollisionObject* colObj = m_dynamicsWorld->getCollisionObjectArray()[i];
			btRigidBody* body = btRigidBody::upcast(colObj);

			if (body && body->getMotionState())
			{
				btDefaultMotionState* myMotionState = (btDefaultMotionState*)body->getMotionState();
				myMotionState->m_graphicsWorldTrans.getOpenGLMatrix(m);
			} else
			{
				colObj->getWorldTransform().getOpenGLMatrix(m);
			}

			btVector3 wireColor(1.f,1.0f,0.5f); //wants deactivation
			if (i & 1)
			{
				wireColor = btVector3(0.f,0.0f,1.f);
			}
			///color differently for active, sleeping, wantsdeactivation states
			if (colObj->getActivationState() == 1) //active
			{
				if (i & 1)
				{
					wireColor += btVector3 (0.8f,0.1f,0.1f);
				} else
				{
					wireColor += btVector3 (0.5f,0.f,0.f);
				}
			}
			if (colObj->getActivationState() == 2) //ISLAND_SLEEPING
			{
				if (i & 1)
				{
					wireColor += btVector3 (0.5f,0.8f, 0.5f);
				} else
				{
					wireColor += btVector3 (0.f,0.5f,0.f);
				}
			}

			m_shapeDrawer->drawOpenGL(m,colObj->getCollisionShape(),wireColor,getDebugMode(),worldBoundsMin,worldBoundsMax);
		}


			float xOffset = 10.f;
			float yStart = 20.f;
			float yIncr = 20.f;
			char buf[124];

			glColor3f(0, 0, 0);

			setOrthographicProjection();

			glRasterPos3f(xOffset,yStart,0);
			sprintf(buf,"mouse to interact");
			GLDebugDrawString(xOffset,yStart,buf);
			yStart += yIncr;

		/*	glRasterPos3f(xOffset,yStart,0);
			sprintf(buf,"space to reset");
			GLDebugDrawString(xOffset,yStart,buf);
			yStart += yIncr;
		*/
			glRasterPos3f(xOffset,yStart,0);
			sprintf(buf,"cursor keys and z,x to navigate");
			GLDebugDrawString(xOffset,yStart,buf);
			yStart += yIncr;

			glRasterPos3f(xOffset,yStart,0);
			sprintf(buf,"i to toggle simulation, s single step");
			GLDebugDrawString(xOffset,yStart,buf);
			yStart += yIncr;

			glRasterPos3f(xOffset,yStart,0);
			sprintf(buf,"q to quit");
			GLDebugDrawString(xOffset,yStart,buf);
			yStart += yIncr;

			glRasterPos3f(xOffset,yStart,0);
			sprintf(buf,". to shoot TRIMESH (dot)");
			GLDebugDrawString(xOffset,yStart,buf);
			yStart += yIncr;

			// not yet hooked up again after refactoring...

/*			glRasterPos3f(xOffset,yStart,0);
			sprintf(buf,"d to toggle deactivation");
			GLDebugDrawString(xOffset,yStart,buf);
			yStart += yIncr;
*/

		/*
			glRasterPos3f(xOffset,yStart,0);
			sprintf(buf,"a to draw temporal AABBs");
			GLDebugDrawString(xOffset,yStart,buf);
			yStart += yIncr;
		*/

			glRasterPos3f(xOffset,yStart,0);
			sprintf(buf,"h to toggle help text");
			GLDebugDrawString(xOffset,yStart,buf);
			yStart += yIncr;

			//bool useBulletLCP = !(getDebugMode() & btIDebugDraw::DBG_DisableBulletLCP);

			bool useCCD = ((getDebugMode() & btIDebugDraw::DBG_EnableCCD) != 0);

			glRasterPos3f(xOffset,yStart,0);
			sprintf(buf,"1 CCD mode (adhoc) = %i",useCCD);
			GLDebugDrawString(xOffset,yStart,buf);
			yStart += yIncr;

			glRasterPos3f(xOffset,yStart,0);
			sprintf(buf,"+- shooting speed = %10.2f",m_ShootBoxInitialSpeed);
			GLDebugDrawString(xOffset,yStart,buf);
			yStart += yIncr;

			#ifdef SHOW_NUM_DEEP_PENETRATIONS
				
				glRasterPos3f(xOffset,yStart,0);
				sprintf(buf,"gNumDeepPenetrationChecks = %d",gNumDeepPenetrationChecks);
				GLDebugDrawString(xOffset,yStart,buf);
				yStart += yIncr;

				glRasterPos3f(xOffset,yStart,0);
				sprintf(buf,"gNumSplitImpulseRecoveries= %d",gNumSplitImpulseRecoveries);
				GLDebugDrawString(xOffset,yStart,buf);
				yStart += yIncr;

				


				glRasterPos3f(xOffset,yStart,0);
				sprintf(buf,"gNumGjkChecks= %d",gNumGjkChecks);
				GLDebugDrawString(xOffset,yStart,buf);
				yStart += yIncr;

			#endif //SHOW_NUM_DEEP_PENETRATIONS


			resetPerspectiveProjection();


	}

}

//------------------------------------------------------------------------------
void	GimpactConcaveDemo::initGImpactCollision()
{
	/// Create Torus Shape
	{
		m_indexVertexArrays = new btTriangleIndexVertexArray
			(NUM_TRIANGLES,
			&gIndices[0][0],
			3*sizeof(int),
			NUM_VERTICES,
			(Real*) &gVertices[0],sizeof(Real)*3);


#ifdef BULLET_GIMPACT
		#ifdef BULLET_GIMPACT_CONVEX_DECOMPOSITION
			btGImpactConvexDecompositionShape * trimesh  = new
			btGImpactConvexDecompositionShape(
			m_indexVertexArrays, btVector3(1.f,1.f,1.f),btScalar(0.01));
			trimesh->setMargin(0.07);
			trimesh->updateBound();


		#else
			btGImpactMeshShape * trimesh = new btGImpactMeshShape(m_indexVertexArrays);
			trimesh->setLocalScaling(btVector3(1.f,1.f,1.f));
			#ifdef BULLET_TRIANGLE_COLLISION 
			trimesh->setMargin(0.07f); ///?????
			#else
			trimesh->setMargin(0.0f);
			#endif
			trimesh->updateBound();
		#endif

		m_trimeshShape = trimesh;

#else
		m_trimeshShape = new btGIMPACTMeshData(m_indexVertexArrays);
#endif

	}

	/// Create Bunny Shape
	{
		m_indexVertexArrays2 = new btTriangleIndexVertexArray
			(BUNNY_NUM_TRIANGLES,
			&gIndicesBunny[0][0],
			3*sizeof(int),
			BUNNY_NUM_VERTICES,
			(Real*) &gVerticesBunny[0],sizeof(Real)*3);
#ifdef BULLET_GIMPACT

		#ifdef BULLET_GIMPACT_CONVEX_DECOMPOSITION
			btGImpactConvexDecompositionShape * trimesh2  = new
			btGImpactConvexDecompositionShape(
			m_indexVertexArrays2, btVector3(4.f,4.f,4.f),btScalar(0.01));
			trimesh2->setMargin(0.07);
			trimesh2->updateBound();
		#else
			btGImpactMeshShape * trimesh2 = new btGImpactMeshShape(m_indexVertexArrays2);
			trimesh2->setLocalScaling(btVector3(4.f,4.f,4.f));
			#ifdef BULLET_TRIANGLE_COLLISION 
			trimesh2->setMargin(0.07f); ///?????
			#else
			trimesh2->setMargin(0.0f);
			#endif
			trimesh2->updateBound();
		#endif



		m_trimeshShape2 = trimesh2;
#else
		m_trimeshShape2 = new btGIMPACTMeshData(m_indexVertexArrays2);

#endif

	}


	///register GIMPACT algorithm
	btCollisionDispatcher * dispatcher = static_cast<btCollisionDispatcher *>(m_dynamicsWorld ->getDispatcher());

#ifdef BULLET_GIMPACT
	btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);
#else
	btConcaveConcaveCollisionAlgorithm::registerAlgorithm(dispatcher);
#endif


}

#ifndef BULLET_GIMPACT
btCollisionShape * GimpactConcaveDemo::createTorusShape()
{
	btGIMPACTMeshShape * newtrimeshShape  = new btGIMPACTMeshShape(m_trimeshShape);
	newtrimeshShape->setLocalScaling(btVector3(1.f,1.f,1.f));
	return newtrimeshShape;
}
btCollisionShape * GimpactConcaveDemo::createBunnyShape()
{
	btGIMPACTMeshShape * newtrimeshShape  = new btGIMPACTMeshShape(m_trimeshShape2);
	newtrimeshShape->setLocalScaling(btVector3(4.f,4.f,4.f));
	return newtrimeshShape;
}
#endif
//------------------------------------------------------------------------------
void	GimpactConcaveDemo::initPhysics()
{
	setTexturing(true);
	setShadows(false);

	setCameraDistance(45.f);


	/// Init Bullet
	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	//btOverlappingPairCache* broadphase = new btSimpleBroadphase();
	//m_broadphase = new btSimpleBroadphase();

	int  maxProxies = 1024;
	btVector3 worldAabbMin(-10000,-10000,-10000);
	btVector3 worldAabbMax( 10000, 10000, 10000);
	m_broadphase = new bt32BitAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);

	m_constraintSolver = new btSequentialImpulseConstraintSolver();

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_constraintSolver,m_collisionConfiguration);

	//create trimesh model and shape
	initGImpactCollision();



	/// Create Scene
	float mass = 0.f;
	btTransform	startTransform;
	startTransform.setIdentity();


	btCollisionShape* staticboxShape1 = new btBoxShape(btVector3(200,1,200));//floor
	m_collisionShapes.push_back(staticboxShape1);
	startTransform.setOrigin(btVector3(0,-10,0));
	localCreateRigidBody(mass, startTransform,staticboxShape1);

	btCollisionShape* staticboxShape2 = new btBoxShape(btVector3(1,50,200));//left wall
	m_collisionShapes.push_back(staticboxShape2);
	startTransform.setOrigin(btVector3(-200,15,0));
	localCreateRigidBody(mass, startTransform,staticboxShape2);

	btCollisionShape* staticboxShape3 = new btBoxShape(btVector3(1,50,200));//right wall
	m_collisionShapes.push_back(staticboxShape3);
	startTransform.setOrigin(btVector3(200,15,0));
	localCreateRigidBody(mass, startTransform,staticboxShape3);

	btCollisionShape* staticboxShape4 = new btBoxShape(btVector3(200,50,1));//front wall
	m_collisionShapes.push_back(staticboxShape4);
	startTransform.setOrigin(btVector3(0,15,200));
	localCreateRigidBody(mass, startTransform,staticboxShape4);

	btCollisionShape* staticboxShape5 = new btBoxShape(btVector3(200,50,1));//back wall
	m_collisionShapes.push_back(staticboxShape5);
	startTransform.setOrigin(btVector3(0,15,-200));
	localCreateRigidBody(mass, startTransform,staticboxShape5);


	//static plane
	
	btVector3 normal(-0.5,0.5,0.0);
	normal.normalize();
	btCollisionShape* staticplaneShape6 = new btStaticPlaneShape(normal,0.0);// A plane
	m_collisionShapes.push_back(staticplaneShape6);
	startTransform.setOrigin(btVector3(0,-9,0));

	btRigidBody* staticBody2 = localCreateRigidBody(mass, startTransform,staticplaneShape6 );

	//another static plane
	
	normal.setValue(0.5,0.7,0.0);
	//normal.normalize();
	btCollisionShape* staticplaneShape7 = new btStaticPlaneShape(normal,0.0);// A plane
	m_collisionShapes.push_back(staticplaneShape7);
	startTransform.setOrigin(btVector3(0,-10,0));

	staticBody2 = localCreateRigidBody(mass, startTransform,staticplaneShape7 );

	/// Create Static Torus
	float  height = 28;
	float step = 2.5;
	float massT = 1.0;

	startTransform.setOrigin(btVector3(0,height,-5));
	startTransform.setRotation(btQuaternion(3.14159265*0.5,0,3.14159265*0.5));
#ifdef BULLET_GIMPACT
	kinematicTorus = localCreateRigidBody(0.0, startTransform,m_trimeshShape);

#else
	kinematicTorus = localCreateRigidBody(0.0, startTransform,createTorusShape());

#endif	//kinematicTorus->setCollisionFlags(kinematicTorus->getCollisionFlags()|btCollisionObject::CF_STATIC_OBJECT);
	//kinematicTorus->setActivationState(ISLAND_SLEEPING);

	kinematicTorus->setCollisionFlags( kinematicTorus->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
	kinematicTorus->setActivationState(DISABLE_DEACTIVATION);

	/// Kinematic
	kinTorusTran = btVector3(-0.1,0,0);
	kinTorusRot  = btQuaternion(0,3.14159265*0.01,0);

#ifdef TEST_GIMPACT_TORUS

#ifdef BULLET_GIMPACT
	/// Create dynamic Torus
	for (int i=0;i<6;i++)
	{
		height -= step;
		startTransform.setOrigin(btVector3(0,height,-5));
		startTransform.setRotation(btQuaternion(0,0,3.14159265*0.5));

		btRigidBody* bodyA;
		bodyA= localCreateRigidBody(massT, startTransform,m_trimeshShape);

		height -= step;
		startTransform.setOrigin(btVector3(0,height,-5));
		startTransform.setRotation(btQuaternion(3.14159265*0.5,0,3.14159265*0.5));
		btRigidBody* bodyB;
		bodyB= localCreateRigidBody(massT, startTransform,m_trimeshShape);

	}
#else

/// Create dynamic Torus
	for (int i=0;i<6;i++)
	{
		height -= step;
		startTransform.setOrigin(btVector3(0,height,-5));
		startTransform.setRotation(btQuaternion(0,0,3.14159265*0.5));

		btRigidBody* bodyA = localCreateRigidBody(massT, startTransform,createTorusShape());

		height -= step;
		startTransform.setOrigin(btVector3(0,height,-5));
		startTransform.setRotation(btQuaternion(3.14159265*0.5,0,3.14159265*0.5));
		btRigidBody* bodyB = localCreateRigidBody(massT, startTransform,createTorusShape());

	}
#endif //no BULLET_GIMPACT
#endif

	startTransform.setIdentity();


	/// Create Dynamic Boxes
	{
		for (int i=0;i<8;i++)
		{
			btCollisionShape* boxShape = new btBoxShape(btVector3(1,1,1));
			m_collisionShapes.push_back(boxShape);
			
			startTransform.setOrigin(btVector3(2*i-5,2,-3));
			localCreateRigidBody(1, startTransform,boxShape);
		}
	}


	//m_debugMode |= btIDebugDraw::DBG_DrawWireframe;

}

//------------------------------------------------------------------------------
void GimpactConcaveDemo::shootTrimesh(const btVector3& destination)
{

	if (m_dynamicsWorld)
	{
		float mass = 4.f;
		btTransform startTransform;
		startTransform.setIdentity();
		btVector3 camPos = getCameraPosition();
		startTransform.setOrigin(camPos);
#ifdef BULLET_GIMPACT
		btRigidBody* body = this->localCreateRigidBody(mass, startTransform,m_trimeshShape2);
#else
		btRigidBody* body = this->localCreateRigidBody(mass, startTransform,createBunnyShape());
#endif
		btVector3 linVel(destination[0]-camPos[0],destination[1]-camPos[1],destination[2]-camPos[2]);
		linVel.normalize();
		linVel*=m_ShootBoxInitialSpeed*0.25;

		body->getWorldTransform().setOrigin(camPos);
		body->getWorldTransform().setRotation(btQuaternion(0,0,0,1));
		body->setLinearVelocity(linVel);
		body->setAngularVelocity(btVector3(0,0,0));
	}
}

//------------------------------------------------------------------------------
void GimpactConcaveDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

#define USE_KINEMATIC_GROUND
#ifdef USE_KINEMATIC_GROUND
	//kinTorusTran = btVector3(-0.05,0,0);
	//kinTorusRot  = btQuaternion(0,3.14159265*0.1,0);

	//kinematic object
	btCollisionObject* colObj = kinematicTorus;
	//is this a rigidbody with a motionstate? then use the motionstate to update positions!
	if (colObj && btRigidBody::upcast(colObj) && btRigidBody::upcast(colObj)->getMotionState())
	{
		btTransform newTrans;
		btRigidBody::upcast(colObj)->getMotionState()->getWorldTransform(newTrans);

		newTrans.getOrigin() += kinTorusTran;
		newTrans.getBasis() = newTrans.getBasis() * btMatrix3x3(kinTorusRot);
		if(newTrans.getOrigin().getX() > 6.0){
			newTrans.getOrigin().setX(6.0);
			kinTorusTran = -kinTorusTran;
		}
		if(newTrans.getOrigin().getX() < -6.0){
			newTrans.getOrigin().setX(-6.0);
			kinTorusTran = -kinTorusTran;
		}

		btRigidBody::upcast(colObj)->getMotionState()->setWorldTransform(newTrans);
	} else
	{
		/*
		btTransform &newTrans =  m_dynamicsWorld->getCollisionObjectArray()[0]->getWorldTransform();
		newTrans.getOrigin() += kinTorusTran;
		if(newTrans.getOrigin().getX() > 0.1) kinTorusTran = -kinTorusTran;
		if(newTrans.getOrigin().getX() < 0.1) kinTorusTran = -kinTorusTran;
		*/
	}

#endif //USE_KINEMATIC_GROUND


	unsigned long int time = getDeltaTimeMicroseconds()/btScalar(1000);
	printf("%i time %i ms \n",m_steps_done,int(time));

//#ifdef BULLET_GIMPACT
//	printf("%i time %.1f ms \n",m_steps_done,btGImpactCollisionAlgorithm::getAverageTreeCollisionTime());
//#else
//	printf("%i time %.1f ms \n",m_steps_done,btConcaveConcaveCollisionAlgorithm::getAverageTreeCollisionTime());
//#endif

	//float dt = float(m_clock.getTimeMicroseconds()) * dts; //0.000001f;
	float dt = btScalar(1./60.);



	m_dynamicsWorld->stepSimulation(dt);

	//optional but useful: debug drawing
	m_dynamicsWorld->debugDrawWorld();

	m_steps_done++;

	//m_dynamicsWorld->stepSimulation(dts);

	renderme();

	glFlush();
	swapBuffers();

}

//------------------------------------------------------------------------------
void GimpactConcaveDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	renderme();

	//optional but useful: debug drawing
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	swapBuffers();
}

//------------------------------------------------------------------------------
void GimpactConcaveDemo::clientResetScene()
{
	m_steps_done = 0;
	DemoApplication::clientResetScene();
}

#define KEY_ESCAPE     0x1B


//------------------------------------------------------------------------------
void GimpactConcaveDemo::keyboardCallback(unsigned char key, int x, int y)
{
	switch (key)
	{
	case '.':
		{
			shootTrimesh(getCameraTargetPosition());
			break;
		}

	case '2':
		{
			dts += 0.000001f;
			break;
		}
	case '3':
		{
			dts -= 0.000001f; if(dts<0.000001f) dts = 0.000001f;
			break;
		}

	default:
		DemoApplication::keyboardCallback(key, x, y);
	}
}



