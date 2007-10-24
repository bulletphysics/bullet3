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

/// Including GIMPACT here



#include "BMF_Api.h"

#include "GLDebugDrawer.h"

#include "GL_ShapeDrawer.h"
#include "GlutStuff.h"

/// Include Torus Mesh here
#include "TorusMesh.h"
#include "BunnyMesh.h"

#ifdef SHOW_NUM_DEEP_PENETRATIONS 
extern int gNumDeepPenetrationChecks;
extern int gNumGjkChecks;
#endif //



//Real			dts = 0.000001f;
Real			dts = 1.0 / 60.0;


///**************************************************************************************
///	GIMPACT Test Demo made by DevO
///
///**************************************************************************************



//------------------------------------------------------------------------------
///User can override this material combiner by implementing gContactAddedCallback and setting body0->m_collisionFlags |= btCollisionObject::customMaterialCallback;
inline btScalar	calculateCombinedFriction(float friction0,float friction1)
{
	btScalar friction = friction0 * friction1;

	const btScalar MAX_FRICTION  = 10.f;
	if (friction < -MAX_FRICTION)
		friction = -MAX_FRICTION;
	if (friction > MAX_FRICTION)
		friction = MAX_FRICTION;
	return friction;

}

//------------------------------------------------------------------------------
inline btScalar	calculateCombinedRestitution(float restitution0,float restitution1)
{
	return restitution0 * restitution1;
}


//------------------------------------------------------------------------------
bool CustomMaterialCombinerCallback(btManifoldPoint& cp,	const btCollisionObject* colObj0,int partId0,int index0,const btCollisionObject* colObj1,int partId1,int index1)
{

	float friction0 = colObj0->getFriction();
	float friction1 = colObj1->getFriction();
	float restitution0 = colObj0->getRestitution();
	float restitution1 = colObj1->getRestitution();

	if (colObj0->getCollisionFlags() & btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK)
	{
		friction0 = 1.0;//partId0,index0
		restitution0 = 0.f;
	}
	if (colObj1->getCollisionFlags() & btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK)
	{
		if (index1&1)
		{
			friction1 = 1.0f;//partId1,index1
		} else
		{
			friction1 = 0.f;
		}
		restitution1 = 0.f;
	}

	cp.m_combinedFriction = calculateCombinedFriction(friction0,friction1);
	cp.m_combinedRestitution = calculateCombinedRestitution(restitution0,restitution1);

	//this return value is currently ignored, but to be on the safe side: return false if you don't calculate friction
	return true;
}

extern ContactAddedCallback		gContactAddedCallback;




//------------------------------------------------------------------------------
void GimpactConcaveDemo::renderme()
{
	updateCamera();


	btScalar m[16];

	if (m_dynamicsWorld)
	{
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

			GL_ShapeDrawer::drawOpenGL(m,colObj->getCollisionShape(),wireColor,getDebugMode());
		}


			float xOffset = 10.f;
			float yStart = 20.f;
			float yIncr = 20.f;
			char buf[124];

			glColor3f(0, 0, 0);

			setOrthographicProjection();

			glRasterPos3f(xOffset,yStart,0);
			sprintf(buf,"mouse to interact");
			BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
			yStart += yIncr;

		/*	glRasterPos3f(xOffset,yStart,0);
			sprintf(buf,"space to reset");
			BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
			yStart += yIncr;
		*/
			glRasterPos3f(xOffset,yStart,0);
			sprintf(buf,"cursor keys and z,x to navigate");
			BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
			yStart += yIncr;

			glRasterPos3f(xOffset,yStart,0);
			sprintf(buf,"i to toggle simulation, s single step");
			BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
			yStart += yIncr;

			glRasterPos3f(xOffset,yStart,0);
			sprintf(buf,"q to quit");
			BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
			yStart += yIncr;

			glRasterPos3f(xOffset,yStart,0);
			sprintf(buf,". to shoot TRIMESH (dot)");
			BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
			yStart += yIncr;

			// not yet hooked up again after refactoring...

/*			glRasterPos3f(xOffset,yStart,0);
			sprintf(buf,"d to toggle deactivation");
			BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
			yStart += yIncr;
*/

		/*
			glRasterPos3f(xOffset,yStart,0);
			sprintf(buf,"a to draw temporal AABBs");
			BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
			yStart += yIncr;
		*/

			glRasterPos3f(xOffset,yStart,0);
			sprintf(buf,"h to toggle help text");
			BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
			yStart += yIncr;

			//bool useBulletLCP = !(getDebugMode() & btIDebugDraw::DBG_DisableBulletLCP);

			bool useCCD = ((getDebugMode() & btIDebugDraw::DBG_EnableCCD) != 0);

			glRasterPos3f(xOffset,yStart,0);
			sprintf(buf,"1 CCD mode (adhoc) = %i",useCCD);
			BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
			yStart += yIncr;

			glRasterPos3f(xOffset,yStart,0);
			sprintf(buf,"+- shooting speed = %10.2f",m_ShootBoxInitialSpeed);
			BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
			yStart += yIncr;

			#ifdef SHOW_NUM_DEEP_PENETRATIONS
				
				glRasterPos3f(xOffset,yStart,0);
				sprintf(buf,"gNumDeepPenetrationChecks = %d",gNumDeepPenetrationChecks);
				BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
				yStart += yIncr;

				glRasterPos3f(xOffset,yStart,0);
				sprintf(buf,"gNumGjkChecks= %d",gNumGjkChecks);
				BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
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

//	{
//		//btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher); /// Register GIMPACT !!!
//		if(!m_gimpactCollisionCreateFunc) m_gimpactCollisionCreateFunc = new btGImpactCollisionAlgorithm::CreateFunc;  /// NEW
//
//		for (int i = 0;i < MAX_BROADPHASE_COLLISION_TYPES ;i++ )
//		{
//			m_dispatcher->registerCollisionCreateFunc(GIMPACT_SHAPE_PROXYTYPE,i ,m_gimpactCollisionCreateFunc);
//		}
//		for (int i = 0;i < MAX_BROADPHASE_COLLISION_TYPES ;i++ )
//		{
//			m_dispatcher->registerCollisionCreateFunc(i,GIMPACT_SHAPE_PROXYTYPE ,m_gimpactCollisionCreateFunc);
//		}
//	}
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
	btCollisionShape* staticboxShape2 = new btBoxShape(btVector3(1,50,200));//left wall
	btCollisionShape* staticboxShape3 = new btBoxShape(btVector3(1,50,200));//right wall
	btCollisionShape* staticboxShape4 = new btBoxShape(btVector3(200,50,1));//front wall
	btCollisionShape* staticboxShape5 = new btBoxShape(btVector3(200,50,1));//back wall

	btCompoundShape* staticScenario = new btCompoundShape();//static scenario

	startTransform.setOrigin(btVector3(0,-10,0));
	staticScenario->addChildShape(startTransform,staticboxShape1);
	startTransform.setOrigin(btVector3(-200,15,0));
	staticScenario->addChildShape(startTransform,staticboxShape2);
	startTransform.setOrigin(btVector3(200,15,0));
	staticScenario->addChildShape(startTransform,staticboxShape3);
	startTransform.setOrigin(btVector3(0,15,200));
	staticScenario->addChildShape(startTransform,staticboxShape4);
	startTransform.setOrigin(btVector3(0,15,-200));
	staticScenario->addChildShape(startTransform,staticboxShape5);

	startTransform.setOrigin(btVector3(0,0,0));

	btRigidBody* staticBody = localCreateRigidBody(mass, startTransform,staticScenario);

	staticBody->setCollisionFlags(staticBody->getCollisionFlags()|btCollisionObject::CF_STATIC_OBJECT);
	staticBody->setActivationState(ISLAND_SLEEPING);

	//enable custom material callback
	staticBody->setCollisionFlags(staticBody->getCollisionFlags()|btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);


	//static plane
	
	btVector3 normal(-0.5,0.5,0.0);
	normal.normalize();
	btCollisionShape* staticplaneShape6 = new btStaticPlaneShape(normal,0.0);// A plane

	startTransform.setOrigin(btVector3(0,-9,0));

	btRigidBody* staticBody2 = localCreateRigidBody(mass, startTransform,staticplaneShape6 );

	staticBody2->setCollisionFlags(staticBody2->getCollisionFlags()|btCollisionObject::CF_STATIC_OBJECT);
	staticBody2->setActivationState(ISLAND_SLEEPING);
	

	//another static plane
	
	normal.setValue(0.5,0.7,0.0);
	//normal.normalize();
	btCollisionShape* staticplaneShape7 = new btStaticPlaneShape(normal,0.0);// A plane

	startTransform.setOrigin(btVector3(0,-10,0));

	staticBody2 = localCreateRigidBody(mass, startTransform,staticplaneShape7 );

	staticBody2->setCollisionFlags(staticBody2->getCollisionFlags()|btCollisionObject::CF_STATIC_OBJECT);
	staticBody2->setActivationState(ISLAND_SLEEPING);


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

		btRigidBody* bodyA = localCreateRigidBody(massT, startTransform,m_trimeshShape);

		height -= step;
		startTransform.setOrigin(btVector3(0,height,-5));
		startTransform.setRotation(btQuaternion(3.14159265*0.5,0,3.14159265*0.5));
		btRigidBody* bodyB = localCreateRigidBody(massT, startTransform,m_trimeshShape);

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
	if (btRigidBody::upcast(colObj) && btRigidBody::upcast(colObj)->getMotionState())
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


	unsigned long int time = m_clock.getTimeMilliseconds();
	printf("%i time %i ms \n",m_steps_done,time);

//#ifdef BULLET_GIMPACT
//	printf("%i time %.1f ms \n",m_steps_done,btGImpactCollisionAlgorithm::getAverageTreeCollisionTime());
//#else
//	printf("%i time %.1f ms \n",m_steps_done,btConcaveConcaveCollisionAlgorithm::getAverageTreeCollisionTime());
//#endif

	//float dt = float(m_clock.getTimeMicroseconds()) * dts; //0.000001f;
	float dt = btScalar(1./60.);



	m_clock.reset();
	m_dynamicsWorld->stepSimulation(dt);
	m_steps_done++;

	//m_dynamicsWorld->stepSimulation(dts);

	renderme();

	glFlush();
	glutSwapBuffers();

}

//------------------------------------------------------------------------------
void GimpactConcaveDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	renderme();

	glFlush();
	glutSwapBuffers();
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

/*
//------------------------------------------------------------------------------
void GimpactConcaveDemo::keyboardCallback(unsigned char key, int x, int y)
{
	m_lastKey = 0;

	switch (key)
	{
	case 'q' :
	case  KEY_ESCAPE   :
		exit(0); break;


		//GLUT_KEY_LEFT
	case 'l' : stepLeft(); break;
	case 'r' : stepRight(); break;
	case 'f' : stepFront(); break;
	case 'b' : stepBack(); break;
	case 'z' : zoomIn(); break;
	case 'x' : zoomOut(); break;
	case 'i' : toggleIdle(); break;
	case 'h':
		if (m_debugMode & btIDebugDraw::DBG_NoHelpText)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_NoHelpText);
		else
			m_debugMode |= btIDebugDraw::DBG_NoHelpText;
		break;

	case 'w':
		if (m_debugMode & btIDebugDraw::DBG_DrawWireframe)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DrawWireframe);
		else
			m_debugMode |= btIDebugDraw::DBG_DrawWireframe;
		break;

	case 'p':
		if (m_debugMode & btIDebugDraw::DBG_ProfileTimings)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_ProfileTimings);
		else
			m_debugMode |= btIDebugDraw::DBG_ProfileTimings;
		break;

	case 'm':
		if (m_debugMode & btIDebugDraw::DBG_EnableSatComparison)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_EnableSatComparison);
		else
			m_debugMode |= btIDebugDraw::DBG_EnableSatComparison;
		break;

	case 'n':
		if (m_debugMode & btIDebugDraw::DBG_DisableBulletLCP)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DisableBulletLCP);
		else
			m_debugMode |= btIDebugDraw::DBG_DisableBulletLCP;
		break;

	case 't' :
		if (m_debugMode & btIDebugDraw::DBG_DrawText)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DrawText);
		else
			m_debugMode |= btIDebugDraw::DBG_DrawText;
		break;
	case 'y':
		if (m_debugMode & btIDebugDraw::DBG_DrawFeaturesText)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DrawFeaturesText);
		else
			m_debugMode |= btIDebugDraw::DBG_DrawFeaturesText;
		break;
	case 'a':
		if (m_debugMode & btIDebugDraw::DBG_DrawAabb){
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DrawAabb);
			//printf("Not Draw AABB \n");
		}else{
			m_debugMode |= btIDebugDraw::DBG_DrawAabb;
			//printf("Draw AABB \n");
		}
		break;
	case 'c' :
		if (m_debugMode & btIDebugDraw::DBG_DrawContactPoints)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DrawContactPoints);
		else
			m_debugMode |= btIDebugDraw::DBG_DrawContactPoints;
		break;

	case 'd' :
		if (m_debugMode & btIDebugDraw::DBG_NoDeactivation)
			m_debugMode = m_debugMode & (~btIDebugDraw::DBG_NoDeactivation);
		else
			m_debugMode |= btIDebugDraw::DBG_NoDeactivation;
		if (m_debugMode | btIDebugDraw::DBG_NoDeactivation)
		{
			gDisableDeactivation = true;
		} else
		{
			gDisableDeactivation = false;
		}
		break;




	case 'o' :
		{
			m_stepping = !m_stepping;
			break;
		}
	case 's' : clientMoveAndDisplay(); break;
		//    case ' ' : newRandom(); break;
	case ' ':
		clientResetScene();
		break;
	case '1':
		{
			if (m_debugMode & btIDebugDraw::DBG_EnableCCD)
				m_debugMode = m_debugMode & (~btIDebugDraw::DBG_EnableCCD);
			else
				m_debugMode |= btIDebugDraw::DBG_EnableCCD;
			break;
		}

	case '.':
		{
			shootTrimesh(getCameraTargetPosition());
			break;
		}

	case '+':
		{
			m_ShootBoxInitialSpeed += 10.f;
			break;
		}
	case '-':
		{
			m_ShootBoxInitialSpeed -= 10.f;
			break;
		}

	default:
		//        std::cout << "unused key : " << key << std::endl;
		break;
	}

	if (getDynamicsWorld() && getDynamicsWorld()->getDebugDrawer())
		getDynamicsWorld()->getDebugDrawer()->setDebugMode(m_debugMode);

	glutPostRedisplay();

}
*/

