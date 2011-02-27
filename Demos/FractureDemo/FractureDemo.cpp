/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2011 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


///FractureDemo shows how to break objects.
///It assumes a btCompoundShaps (where the childshapes are the pre-fractured pieces)
///The btFractureBody is a class derived from btRigidBody, dealing with the collision impacts.
///Press the F key to toggle between fracture and glue mode
///This is preliminary work



//#define TEST_SERIALIZATION 1


///create 125 (5x5x5) dynamic object
#define ARRAY_SIZE_X 10
#define ARRAY_SIZE_Y 1
#define ARRAY_SIZE_Z 10

//maximum number of objects (and allow user to shoot additional boxes)
#define MAX_PROXIES (ARRAY_SIZE_X*ARRAY_SIZE_Y*ARRAY_SIZE_Z + 1024)
#define CUBE_HALF_EXTENTS 1.f
#define EXTRA_HEIGHT 1.f
///scaling of the objects (0.1 = 20 centimeter boxes )
#define SCALING 1.
#define START_POS_X -5
#define START_POS_Y -5
#define START_POS_Z -3

#include "FractureDemo.h"
#include "GlutStuff.h"
#include "GLDebugFont.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionDispatch/btUnionFind.h"

#ifdef TEST_SERIALIZATION
#include "LinearMath/btSerializer.h"
#endif //TEST_SERIALIZATION

#include <stdio.h> //printf debugging

bool sFracturingMode = false;
#define CUSTOM_FRACTURE_TYPE (btRigidBody::CO_USER_TYPE+1)
int sFrameNumber = 0;

void FractureDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();
	
	///step the simulation
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(ms / 1000000.f);
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
	}
	
	

	renderme(); 

	showMessage();
	
	glFlush();

	swapBuffers();

}

void FractureDemo::showMessage()
{
	if((getDebugMode() & btIDebugDraw::DBG_DrawText))
	{
		setOrthographicProjection();
		glDisable(GL_LIGHTING);
		glColor3f(0, 0, 0);
		char buf[124];
		
		int lineWidth=350;
		int xStart = m_glutScreenWidth - lineWidth;
		int yStart = 20;

		glRasterPos3f(xStart, yStart, 0);
		if (sFracturingMode)
		{
			sprintf(buf,"Fracture mode");
		} else
		{
			sprintf(buf,"Glue mode");
		}
		GLDebugDrawString(xStart,20,buf);
		yStart+=20;
		glRasterPos3f(xStart, yStart, 0);
		sprintf(buf,"f to toggle fracture/glue mode");		
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);
		resetPerspectiveProjection();
		glEnable(GL_LIGHTING);
	}

}


void FractureDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	
	renderme();

	showMessage();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	swapBuffers();
}

void FractureDemo::keyboardCallback(unsigned char key, int x, int y)
{	
	if (key=='f')
	{
	} else
	{
		PlatformDemoApplication::keyboardCallback(key,x,y);
	}
}

void FractureDemo::keyboardUpCallback(unsigned char key, int x, int y)
{
	if (key=='f')
	{
		sFracturingMode = !sFracturingMode;
	}

	PlatformDemoApplication::keyboardUpCallback(key,x,y);
	
}



struct btConnection
{
	
	btCollisionShape*	m_childShape0;
	btCollisionShape*	m_childShape1;
	int	m_childIndex0;
	int	m_childIndex1;
	btScalar	m_strength;
};

class btFractureBody : public btRigidBody
{
	//connections
public:

	btDynamicsWorld*	m_world;
	btAlignedObjectArray<btScalar>	m_masses;
	btAlignedObjectArray<btConnection>	m_connections;



	btFractureBody(	const btRigidBodyConstructionInfo& constructionInfo, btDynamicsWorld*	world)
		:btRigidBody(constructionInfo),
		m_world(world)
	{
		m_masses.push_back(constructionInfo.m_mass);
		m_internalType=CUSTOM_FRACTURE_TYPE+CO_RIGID_BODY;
	}



	///btRigidBody constructor for backwards compatibility. 
	///To specify friction (etc) during rigid body construction, please use the other constructor (using btRigidBodyConstructionInfo)
	btFractureBody(	btScalar mass, btMotionState* motionState, btCollisionShape* collisionShape, const btVector3& localInertia, btScalar* masses, int numMasses, btDynamicsWorld*	world)
		:btRigidBody(mass,motionState,collisionShape,localInertia),
		m_world(world)
	{

		for (int i=0;i<numMasses;i++)
			m_masses.push_back(masses[i]);

		m_internalType=CUSTOM_FRACTURE_TYPE+CO_RIGID_BODY;
	}

	
	void	recomputeConnectivity(btCollisionWorld* world)
	{
		m_connections.clear();
		//@todo use the AABB tree to avoid N^2 checks

		if (getCollisionShape()->isCompound())
		{
			btCompoundShape* compound = (btCompoundShape*)getCollisionShape();
			for (int i=0;i<compound->getNumChildShapes();i++)
			{
				for (int j=i+1;j<compound->getNumChildShapes();j++)
				{

					struct   MyContactResultCallback : public btCollisionWorld::ContactResultCallback
					{
						bool m_connected;
						MyContactResultCallback() :m_connected(false)
						{
						}
						virtual   btScalar   addSingleResult(btManifoldPoint& cp,   const btCollisionObject* colObj0,int partId0,int index0,const btCollisionObject* colObj1,int partId1,int index1)
						{
							//@todo additional check on cp?
							m_connected = true;
							return 1.f;
						}
				   };

					MyContactResultCallback result;

					btCollisionObject obA;
					obA.setWorldTransform(compound->getChildTransform(i));
					obA.setCollisionShape(compound->getChildShape(i));
					btCollisionObject obB;
					obB.setWorldTransform(compound->getChildTransform(j));
					obB.setCollisionShape(compound->getChildShape(j));
					world->contactPairTest(&obA,&obB,result);
					if (result.m_connected)
					{
						btConnection tmp;
						tmp.m_childIndex0 = i;
						tmp.m_childIndex1 = j;
						tmp.m_childShape0 = compound->getChildShape(i);
						tmp.m_childShape1 = compound->getChildShape(j);
						tmp.m_strength = 1.f;//??
						m_connections.push_back(tmp);
					}
				}
			}
		}
		

	}

	static btCompoundShape* shiftTransform(btCompoundShape* boxCompound,btScalar* masses,btTransform& shift, btVector3& principalInertia)
	{
		btTransform principal;

		boxCompound->calculatePrincipalAxisTransform(masses,principal,principalInertia);


		///create a new compound with world transform/center of mass properly aligned with the principal axis

		///non-recursive compound shapes perform better
		
	#ifdef USE_RECURSIVE_COMPOUND

		btCompoundShape* newCompound = new btCompoundShape();
		newCompound->addChildShape(principal.inverse(),boxCompound);
		newBoxCompound = newCompound;
		//m_collisionShapes.push_back(newCompound);

		//btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
		//btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,newCompound,principalInertia);

	#else
	#ifdef CHANGE_COMPOUND_INPLACE
		newBoxCompound = boxCompound;
		for (int i=0;i<boxCompound->getNumChildShapes();i++)
		{
			btTransform newChildTransform = principal.inverse()*boxCompound->getChildTransform(i);
			///updateChildTransform is really slow, because it re-calculates the AABB each time. todo: add option to disable this update
			boxCompound->updateChildTransform(i,newChildTransform);
		}
		bool isDynamic = (mass != 0.f);
		btVector3 localInertia(0,0,0);
		if (isDynamic)
			boxCompound->calculateLocalInertia(mass,localInertia);
		
	#else
		///creation is faster using a new compound to store the shifted children
		btCompoundShape* newBoxCompound = new btCompoundShape();
		for (int i=0;i<boxCompound->getNumChildShapes();i++)
		{
			btTransform newChildTransform = principal.inverse()*boxCompound->getChildTransform(i);
			///updateChildTransform is really slow, because it re-calculates the AABB each time. todo: add option to disable this update
			newBoxCompound->addChildShape(newChildTransform,boxCompound->getChildShape(i));
		}



	#endif

	#endif//USE_RECURSIVE_COMPOUND

		shift = principal;
		return newBoxCompound;
	}

	static btCompoundShape* shiftTransformDistributeMass(btCompoundShape* boxCompound,btScalar mass,btTransform& shift)
	{
	
		btVector3 principalInertia;

		btScalar* masses = new btScalar[boxCompound->getNumChildShapes()];
		for (int j=0;j<boxCompound->getNumChildShapes();j++)
		{
			//evenly distribute mass
			masses[j]=mass/boxCompound->getNumChildShapes();
		}

		return shiftTransform(boxCompound,masses,shift,principalInertia);

	}

	static bool collisionCallback(btManifoldPoint& cp,	const btCollisionObject* colObj0,int partId0,int index0,const btCollisionObject* colObj1,int partId1,int index1);

};

struct	btFracturePair
{
	btFractureBody* m_fracObj;
	btAlignedObjectArray<btPersistentManifold*>	m_contactManifolds;
};

btAlignedObjectArray<btFractureBody*> sFractureBodies;
btAlignedObjectArray<btFracturePair> sFracturePairs;





void addNewBody(const btTransform& oldTransform,btScalar* masses, btCompoundShape* oldCompound, btDynamicsWorld* world)
{
	int i;
	
	btTransform shift;
	shift.setIdentity();
	btVector3 localInertia;
	btCompoundShape* newCompound = btFractureBody::shiftTransform(oldCompound,masses,shift,localInertia);
	btScalar totalMass = 0;
	for (i=0;i<newCompound->getNumChildShapes();i++)
		totalMass += masses[i];
	//newCompound->calculateLocalInertia(totalMass,localInertia);

	btFractureBody* newBody = new btFractureBody(totalMass,0,newCompound,localInertia, masses,newCompound->getNumChildShapes(), world);
	newBody->recomputeConnectivity(world);
	sFractureBodies.push_back(newBody);
	
	newBody->setCollisionFlags(newBody->getCollisionFlags()|btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	newBody->setWorldTransform(oldTransform*shift);
	world->addRigidBody(newBody);
}


void	breakDisconnectedParts( btFractureBody* fracObj, btDynamicsWorld* world)
{
	printf("breakingCheck\n");

	if (!fracObj->getCollisionShape()->isCompound())
		return;

	btCompoundShape* compound = (btCompoundShape*)fracObj->getCollisionShape();
	int numChildren = compound->getNumChildShapes();

	if (numChildren<=1)
		return;

	//compute connectivity
	btUnionFind unionFind;

	btAlignedObjectArray<int> tags;
	tags.resize(numChildren);
	int i, index = 0;
	for ( i=0;i<numChildren;i++)
	{
#ifdef STATIC_SIMULATION_ISLAND_OPTIMIZATION
		tags[i] = index++;
#else
		tags[i] = i;
		index=i+1;
#endif
	}

	unionFind.reset(index);
	int numElem = unionFind.getNumElements();
	for (i=0;i<fracObj->m_connections.size();i++)
	{
		btConnection& connection = fracObj->m_connections[i];
		if (connection.m_strength > 0.)
		{
			int tag0 = tags[connection.m_childIndex0];
			int tag1 = tags[connection.m_childIndex1];
			unionFind.unite(tag0, tag1);
		}
	}
	numElem = unionFind.getNumElements();
	
	index=0;
	for (int ai=0;ai<numChildren;ai++)
	{
		int tag = unionFind.find(index);
		tags[ai] = tag;
		//Set the correct object offset in Collision Object Array
#if STATIC_SIMULATION_ISLAND_OPTIMIZATION
		unionFind.getElement(index).m_sz = ai;
#endif //STATIC_SIMULATION_ISLAND_OPTIMIZATION
		index++;
	}
	unionFind.sortIslands();

	int endIslandIndex=1;
	int startIslandIndex;

	btAlignedObjectArray<btCollisionObject*> removedObjects;

	int numIslands = 0;

	for ( startIslandIndex=0;startIslandIndex<numElem;startIslandIndex = endIslandIndex)
	{
		int islandId = unionFind.getElement(startIslandIndex).m_id;
		for (endIslandIndex = startIslandIndex+1;(endIslandIndex<numElem) && (unionFind.getElement(endIslandIndex).m_id == islandId);endIslandIndex++)
		{
		}

		int fractureObjectIndex = -1;

		int numShapes=0;


		btCompoundShape* newCompound = new btCompoundShape();
		btAlignedObjectArray<btScalar> masses;

		int idx;
		for (idx=startIslandIndex;idx<endIslandIndex;idx++)
		{
			int i = unionFind.getElement(idx).m_sz;
			btCollisionShape* shape = compound->getChildShape(i);
			newCompound->addChildShape(compound->getChildTransform(i),compound->getChildShape(i));
			masses.push_back(fracObj->m_masses[i]);
			numShapes++;
		}
		if (numShapes)
		{
			addNewBody(fracObj->getWorldTransform(),&masses[0],newCompound,world);
			numIslands++;
		}
	}



	/*

	//fracture (split) disconnected parts
	
	int numElems = oldCompound->getNumChildShapes()/2;
	btScalar* newMasses0 = (btScalar*) malloc(sizeof(btScalar)*numElems);
	int mc = 0;
	for (int c=0;c<oldCompound->getNumChildShapes()/2;c++,mc++)
	{
		newCompound0->addChildShape(oldCompound->getChildTransform(c),oldCompound->getChildShape(c));
		newMasses0[mc] = sFracturePairs[i].m_fracObj->m_masses[c];
	}
	

	btCompoundShape* newCompound1 = new btCompoundShape();
	numElems = oldCompound->getNumChildShapes() - oldCompound->getNumChildShapes()/2;
	mc = 0;
	btScalar* newMasses1 = (btScalar*) malloc(sizeof(btScalar)*numElems);
	for (int c=oldCompound->getNumChildShapes()/2;c<oldCompound->getNumChildShapes();c++,mc++)
	{
		newCompound1->addChildShape(oldCompound->getChildTransform(c),oldCompound->getChildShape(c));
		newMasses1[mc] = sFracturePairs[i].m_fracObj->m_masses[c];
	}

	addNewBody(sFracturePairs[i].m_fracObj->getWorldTransform(),newMasses1,newCompound1,world);

	*/

	world->removeRigidBody(fracObj);
	sFractureBodies.remove(fracObj);

}




void	FractureDemo::shootBox(const btVector3& destination)
{

	if (m_dynamicsWorld)
	{
		btScalar mass = 1.f;
		btTransform startTransform;
		startTransform.setIdentity();
		btVector3 camPos = getCameraPosition();
		startTransform.setOrigin(camPos);

		setShootBoxShape ();

		btAssert((!m_shootBoxShape || m_shootBoxShape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			m_shootBoxShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

		btFractureBody* body = new btFractureBody(mass,0,m_shootBoxShape,localInertia,&mass,1,m_dynamicsWorld);
		sFractureBodies.push_back(body);
		body->setWorldTransform(startTransform);

		m_dynamicsWorld->addRigidBody(body);

	
		body->setLinearFactor(btVector3(1,1,1));
		//body->setRestitution(1);

		btVector3 linVel(destination[0]-camPos[0],destination[1]-camPos[1],destination[2]-camPos[2]);
		linVel.normalize();
		linVel*=m_ShootBoxInitialSpeed;

		body->getWorldTransform().setOrigin(camPos);
		body->getWorldTransform().setRotation(btQuaternion(0,0,0,1));
		body->setLinearVelocity(linVel);
		body->setAngularVelocity(btVector3(0,0,0));
		body->setCcdMotionThreshold(1.);
		body->setCcdSweptSphereRadius(0.2f);
		
	}
}

void glueCallback(btDynamicsWorld* world, btScalar timeStep)
{

	int numManifolds = world->getDispatcher()->getNumManifolds();

	btUnionFind unionFind;

	int index = 0;
	{

		int i;
		for (i=0;i<world->getCollisionObjectArray().size(); i++)
		{
			btCollisionObject*   collisionObject= world->getCollisionObjectArray()[i];
			btRigidBody* body = btRigidBody::upcast(collisionObject);
			//Adding filtering here
#ifdef STATIC_SIMULATION_ISLAND_OPTIMIZATION
			if (!collisionObject->isStaticOrKinematicObject())
			{
				collisionObject->setIslandTag(index++);
			} else
			{
				collisionObject->setIslandTag(-1);
			}
#else
			collisionObject->setIslandTag(i);
			index=i+1;
#endif
		}
	}

	unionFind.reset(index);

	int numElem = unionFind.getNumElements();

	for (int i=0;i<numManifolds;i++)
	{
		btPersistentManifold* manifold = world->getDispatcher()->getManifoldByIndexInternal(i);
		if (!manifold->getNumContacts())
			continue;

		btCollisionObject* colObj0 = (btCollisionObject*)manifold->getBody0();
		btCollisionObject* colObj1 = (btCollisionObject*)manifold->getBody1();
		int tag0 = (colObj0)->getIslandTag();
		int tag1 = (colObj1)->getIslandTag();
		btRigidBody* body0 = btRigidBody::upcast(colObj0);
		btRigidBody* body1 = btRigidBody::upcast(colObj1);

		
		if (!colObj0->isStaticOrKinematicObject() && !colObj1->isStaticOrKinematicObject())
		{
			unionFind.unite(tag0, tag1);
		}
	}



	
	numElem = unionFind.getNumElements();



	index=0;
	for (int ai=0;ai<world->getCollisionObjectArray().size();ai++)
	{
		btCollisionObject* collisionObject= world->getCollisionObjectArray()[ai];
		if (!collisionObject->isStaticOrKinematicObject())
		{
			int tag = unionFind.find(index);
			
			collisionObject->setIslandTag( tag);
			
			//Set the correct object offset in Collision Object Array
#if STATIC_SIMULATION_ISLAND_OPTIMIZATION
			unionFind.getElement(index).m_sz = ai;
#endif //STATIC_SIMULATION_ISLAND_OPTIMIZATION

			index++;
		}
	}
	unionFind.sortIslands();



	int endIslandIndex=1;
	int startIslandIndex;

	btAlignedObjectArray<btCollisionObject*> removedObjects;

	//update the sleeping state for bodies, if all are sleeping
	for ( startIslandIndex=0;startIslandIndex<numElem;startIslandIndex = endIslandIndex)
	{
		int islandId = unionFind.getElement(startIslandIndex).m_id;
		for (endIslandIndex = startIslandIndex+1;(endIslandIndex<numElem) && (unionFind.getElement(endIslandIndex).m_id == islandId);endIslandIndex++)
		{
		}

		int fractureObjectIndex = -1;

		int numObjects=0;

		int idx;
		for (idx=startIslandIndex;idx<endIslandIndex;idx++)
		{
			int i = unionFind.getElement(idx).m_sz;
			btCollisionObject* colObj0 = world->getCollisionObjectArray()[i];
			if (colObj0->getInternalType()& CUSTOM_FRACTURE_TYPE)
			{
				fractureObjectIndex = i;
			}
			btRigidBody* otherObject = btRigidBody::upcast(colObj0);
			if (!otherObject || !otherObject->getInvMass())
				continue;
			numObjects++;
		}

		
		if (fractureObjectIndex>=0 && numObjects>1)
		{

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			btFractureBody* fracObj = (btFractureBody*)world->getCollisionObjectArray()[fractureObjectIndex];
			
			removedObjects.push_back(fracObj);
			sFractureBodies.remove(fracObj);

			btAlignedObjectArray<btScalar> massArray;
			btScalar totalMass = 0.f;


			btCompoundShape* compound = new btCompoundShape();
			if (fracObj->getCollisionShape()->isCompound())
			{
				btTransform tr;
				tr.setIdentity();
				btCompoundShape* oldCompound = (btCompoundShape*)fracObj->getCollisionShape();
				for (int c=0;c<oldCompound->getNumChildShapes();c++)
				{
					compound->addChildShape(oldCompound->getChildTransform(c),oldCompound->getChildShape(c));
					massArray.push_back(fracObj->m_masses[c]);
					totalMass+=fracObj->m_masses[c];
				}

			} else
			{
				btTransform tr;
				tr.setIdentity();
				compound->addChildShape(tr,fracObj->getCollisionShape());
				massArray.push_back(fracObj->m_masses[0]);
				totalMass+=fracObj->m_masses[0];
			}

			for (idx=startIslandIndex;idx<endIslandIndex;idx++)
			{
				
				int i = unionFind.getElement(idx).m_sz;

				if (i==fractureObjectIndex)
					continue;

				btCollisionObject* otherCollider = world->getCollisionObjectArray()[i];

				btRigidBody* otherObject = btRigidBody::upcast(otherCollider);
				if (!otherObject || !otherObject->getInvMass())
					continue;
				
				removedObjects.push_back(otherObject);
				sFractureBodies.remove((btFractureBody*)otherObject);

				btScalar curMass = 1./otherObject->getInvMass();


				if (otherObject->getCollisionShape()->isCompound())
				{
					btTransform tr;
					btCompoundShape* oldCompound = (btCompoundShape*)otherObject->getCollisionShape();
					for (int c=0;c<oldCompound->getNumChildShapes();c++)
					{
						tr = fracObj->getWorldTransform().inverseTimes(otherObject->getWorldTransform()*oldCompound->getChildTransform(c));
						compound->addChildShape(tr,oldCompound->getChildShape(c));
						massArray.push_back(curMass/(btScalar)oldCompound->getNumChildShapes());

					}
				} else
				{
					btTransform tr;
					tr = fracObj->getWorldTransform().inverseTimes(otherObject->getWorldTransform());
					compound->addChildShape(tr,otherObject->getCollisionShape());
					massArray.push_back(curMass);
				}
				totalMass+=curMass;
			}

			

			btTransform shift;
			shift.setIdentity();
			btCompoundShape* newCompound = btFractureBody::shiftTransformDistributeMass(compound,totalMass,shift);
			int numChildren = newCompound->getNumChildShapes();
			btAssert(numChildren == massArray.size());

			btVector3 localInertia;
			newCompound->calculateLocalInertia(totalMass,localInertia);
			btFractureBody* newBody = new btFractureBody(totalMass,0,newCompound,localInertia, &massArray[0], numChildren,world);
			newBody->recomputeConnectivity(world);
			sFractureBodies.push_back(newBody);
			newBody->setWorldTransform(fracObj->getWorldTransform()*shift);
			world->addRigidBody(newBody);


		}

		
	}

	//remove the objects from the world at the very end, 
	//otherwise the island tags would not match the world collision object array indices anymore
	while (removedObjects.size())
	{
		btCollisionObject* otherCollider = removedObjects[removedObjects.size()-1];
		removedObjects.pop_back();

		btRigidBody* otherObject = btRigidBody::upcast(otherCollider);
		if (!otherObject || !otherObject->getInvMass())
			continue;
		world->removeRigidBody(otherObject);
	}

}




void fractureCallback(btDynamicsWorld* world, btScalar timeStep)
{
	if (!sFracturingMode)
	{
		glueCallback(world,timeStep);
		return;
	}

	int numManifolds = world->getDispatcher()->getNumManifolds();

	sFracturePairs.clear();

	
	for (int i=0;i<numManifolds;i++)
	{
		btPersistentManifold* manifold = world->getDispatcher()->getManifoldByIndexInternal(i);
		if (!manifold->getNumContacts())
			continue;
		
		btScalar totalImpact = 0.f;
		for (int p=0;p<manifold->getNumContacts();p++)
		{
			totalImpact += manifold->getContactPoint(p).m_appliedImpulse;
		}

		static float maxImpact = 0;
		if (totalImpact>maxImpact)
			maxImpact = totalImpact;

		//some threshold otherwise resting contact would break objects after a while
		if (totalImpact < 10)
			continue;

		printf("strong impact\n");


		//@todo: add better logic to decide what parts to fracture
		//For example use the idea from the SIGGRAPH talk about the fracture in the movie 2012:
		//
		//Breaking thresholds can be stored as connectivity information between child shapes in the fracture object
		//
		//You can calculate some "impact value" by simulating all the individual child shapes 
		//as rigid bodies, without constraints, running it in a separate simulation world 
		//(or by running the constraint solver without actually modifying the dynamics world)
		//Then measure some "impact value" using the offset and applied impulse for each child shape
		//weaken the connections based on this "impact value" and only break 
		//if this impact value exceeds the breaking threshold.
		//you can propagate the weakening and breaking of connections using the connectivity information

		int f0 = sFractureBodies.findLinearSearch((btFractureBody*)manifold->getBody0());
		int f1 = sFractureBodies.findLinearSearch((btFractureBody*)manifold->getBody1());

		if (f0 == f1 == sFractureBodies.size())
			continue;


		if (f0<sFractureBodies.size())
		{
			int j=f0;

			btCollisionObject* colOb = (btCollisionObject*)manifold->getBody1();
			btRigidBody* otherOb = btRigidBody::upcast(colOb);
		//	if (!otherOb->getInvMass())
		//		continue;

			int pi=-1;

			for (int p=0;p<sFracturePairs.size();p++)
			{
				if (sFracturePairs[p].m_fracObj == sFractureBodies[j])
				{
					pi = p; break;
				}
			}

			if (pi<0)
			{
				btFracturePair p;
				p.m_fracObj = sFractureBodies[j];
				p.m_contactManifolds.push_back(manifold);
				sFracturePairs.push_back(p);
			} else
			{
				btAssert(sFracturePairs[pi].m_contactManifolds.findLinearSearch(manifold)==sFracturePairs[pi].m_contactManifolds.size());
				sFracturePairs[pi].m_contactManifolds.push_back(manifold);
			}
		}
		
		
		if (f1 < sFractureBodies.size())
		{
			int j=f1;
			{
				btCollisionObject* colOb = (btCollisionObject*)manifold->getBody0();
				btRigidBody* otherOb = btRigidBody::upcast(colOb);
			//	if (!otherOb->getInvMass())
			//		continue;


				int pi=-1;

				for (int p=0;p<sFracturePairs.size();p++)
				{
					if (sFracturePairs[p].m_fracObj == sFractureBodies[j])
					{
						pi = p; break;
					}
				}
				if (pi<0)
				{
					btFracturePair p;
					p.m_fracObj = sFractureBodies[j];
					p.m_contactManifolds.push_back( manifold);
					sFracturePairs.push_back(p);
				} else
				{
					btAssert(sFracturePairs[pi].m_contactManifolds.findLinearSearch(manifold)==sFracturePairs[pi].m_contactManifolds.size());
					sFracturePairs[pi].m_contactManifolds.push_back(manifold);
				}
			}
		}

		//
	}

	//printf("sFractureBodies size=%d\n",sFractureBodies.size());
	//printf("sFracturePairs size=%d\n",sFracturePairs.size());
	if (!sFracturePairs.size())
		return;


	{
//		printf("fracturing\n");

		for (int i=0;i<sFracturePairs.size();i++)
		{
			//check impulse/displacement at impact

			//weaken/break connections (and propagate breaking)

			//compute connectivity of connected child shapes

		
			if (sFracturePairs[i].m_fracObj->getCollisionShape()->isCompound())
			{
				btTransform tr;
				tr.setIdentity();
				btCompoundShape* oldCompound = (btCompoundShape*)sFracturePairs[i].m_fracObj->getCollisionShape();
				if (oldCompound->getNumChildShapes()>1)
				{
					bool needsBreakingCheck = false;


					//weaken/break the connections

					//@todo: propagate along the connection graph
					for (int j=0;j<sFracturePairs[i].m_contactManifolds.size();j++)
					{
						btPersistentManifold* manifold = sFracturePairs[i].m_contactManifolds[j];
						for (int k=0;k<manifold->getNumContacts();k++)
						{
							btManifoldPoint& pt = manifold->getContactPoint(k);
							if (manifold->getBody0()==sFracturePairs[i].m_fracObj)
							{
								for (int f=0;f<sFracturePairs[i].m_fracObj->m_connections.size();f++)
								{
									btConnection& connection = sFracturePairs[i].m_fracObj->m_connections[f];
									if (	(connection.m_childIndex0 == pt.m_index0) ||
											(connection.m_childIndex1 == pt.m_index0)
										)
									{
										connection.m_strength -= pt.m_appliedImpulse;
										if (connection.m_strength<0)
										{
											//remove or set to zero
											connection.m_strength=0.f;
											needsBreakingCheck = true;
										}
									}
								}
							} else
							{
								for (int f=0;f<sFracturePairs[i].m_fracObj->m_connections.size();f++)
								{
									btConnection& connection = sFracturePairs[i].m_fracObj->m_connections[f];
									if (	(connection.m_childIndex0 == pt.m_index1) ||
											(connection.m_childIndex1 == pt.m_index1)
										)
									{
										connection.m_strength -= pt.m_appliedImpulse;
										if (connection.m_strength<0)
										{
											//remove or set to zero
											connection.m_strength=0.f;
											needsBreakingCheck = true;
										}
									}
								}
							}
						}
					}

					if (needsBreakingCheck)
					{
						breakDisconnectedParts(sFracturePairs[i].m_fracObj, world);
					}
				}

			}

		}
	}

	sFracturePairs.clear();

}






void	FractureDemo::initPhysics()
{
	
	setTexturing(true);
	setShadows(true);

	setDebugMode(btIDebugDraw::DBG_DrawText|btIDebugDraw::DBG_NoHelpText);

	setCameraDistance(btScalar(SCALING*20.));

	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	//m_collisionConfiguration->setConvexConvexMultipointIterations();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	m_solver = sol;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	

	//m_splitImpulse removes the penetration resolution from the applied impulse, otherwise objects might fracture due to deep penetrations.
	m_dynamicsWorld->getSolverInfo().m_splitImpulse = true;
	
	m_dynamicsWorld->setInternalTickCallback(fractureCallback,m_dynamicsWorld);
	m_dynamicsWorld->setGravity(btVector3(0,-10,0));

	///create a few basic rigid bodies
	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));
//	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
	
	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-50,0));

	//We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
	{
		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		//add the body to the dynamics world
		m_dynamicsWorld->addRigidBody(body);
	}


	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		btCollisionShape* colShape = new btBoxShape(btVector3(SCALING*1,SCALING*1,SCALING*1));
		//btCollisionShape* colShape = new btCapsuleShape(SCALING*0.4,SCALING*1);
		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		m_collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass,localInertia);
#if 0
		float start_x = START_POS_X - ARRAY_SIZE_X/2;
		float start_y = START_POS_Y;
		float start_z = START_POS_Z - ARRAY_SIZE_Z/2;

		for (int k=0;k<ARRAY_SIZE_Y;k++)
		{
			for (int i=0;i<ARRAY_SIZE_X;i++)
			{
				for(int j = 0;j<ARRAY_SIZE_Z;j++)
				{
					startTransform.setOrigin(SCALING*btVector3(
										btScalar(3.0*i + start_x),
										btScalar(20+2.0*k + start_y),
										btScalar(2.0*j + start_z)));

			
					//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
					btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
					btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
					btFractureBody* body = new btFractureBody(rbInfo, m_dynamicsWorld);
					sFractureBodies.push_back(body);
					btTransform tr;
					tr.setIdentity();
					btQuaternion orn(btVector3(1,0,0),45);
					tr.setRotation(orn);
					body->setWorldTransform(tr);
					
					body->setActivationState(ISLAND_SLEEPING);

					m_dynamicsWorld->addRigidBody(body);
					body->setActivationState(ISLAND_SLEEPING);

					btVector3 pivot(0,0,0);
					btVector3 axis(0,1,0);
					//btHingeConstraint* hinge = new btHingeConstraint(*body,pivot,axis,true);
					//hinge->setAngularOnly(true);
					//m_dynamicsWorld->addConstraint(hinge);

				}
			}
		}
#else

	int gNumObjects = ARRAY_SIZE_X * ARRAY_SIZE_Y * ARRAY_SIZE_Z;

	for (int i=0;i<gNumObjects;i++)
	{
		btTransform trans;
		trans.setIdentity();

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

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(trans);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
		btFractureBody* body = new btFractureBody(rbInfo, m_dynamicsWorld);
		sFractureBodies.push_back(body);
		btTransform tr;
		tr.setIdentity();
		btQuaternion orn(btVector3(1,0,0),45);
		tr.setRotation(orn);
		body->setWorldTransform(tr);
		
		body->setActivationState(ISLAND_SLEEPING);

		m_dynamicsWorld->addRigidBody(body);
		body->setActivationState(ISLAND_SLEEPING);

		
	}
#endif

	}


	clientResetScene();


#ifdef TEST_SERIALIZATION
	//test serializing this 

	btDefaultSerializer*	serializer = new btDefaultSerializer();
	m_dynamicsWorld->serialize(serializer);
	
	FILE* f2 = fopen("testFile.bullet","wb");
	fwrite(serializer->m_buffer,serializer->m_currentSize,1,f2);
	fclose(f2);
#endif


}
	

void	FractureDemo::exitPhysics()
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

	delete m_dynamicsWorld;
	
	delete m_solver;
	
	delete m_broadphase;
	
	delete m_dispatcher;

	delete m_collisionConfiguration;

	
}




