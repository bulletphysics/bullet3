/*
Physics Effects Copyright(C) 2010 Sony Computer Entertainment Inc.
All rights reserved.

Physics Effects is open software; you can redistribute it and/or
modify it under the terms of the BSD License.

Physics Effects is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the BSD License for more details.

A copy of the BSD License is distributed with
Physics Effects under the filename: physics_effects_license.txt
*/

#include "physics_func.h"
#include "../common/perf_func.h"

#include "BulletPhysicsEffects/btBulletPhysicsEffectsWorld.h"
#include "BulletPhysicsEffects/btLowLevelBroadphase.h"
#include "BulletPhysicsEffects/btLowLevelCollisionDispatcher.h"
#include "BulletPhysicsEffects/btLowLevelData.h"
#include "BulletPhysicsEffects/btLowLevelConstraintSolver.h"
#include "BulletPhysicsEffects/btLowLevelConstraintSolver2.h"


#include "LinearMath/btIDebugDraw.h"
#include "BulletMultiThreaded/vectormath2bullet.h"
#include "../common/render_func.h"
#if 0
#ifdef _WIN32
#include "BulletMultiThreaded/Win32ThreadSupport.h"
#endif

btThreadSupportInterface* createSolverThreadSupport(int maxNumThreads)
{
#define SEQUENTIAL
#ifdef SEQUENTIAL
	SequentialThreadSupport::SequentialThreadConstructionInfo tci("solverThreads",SolverThreadFunc,SolverlsMemoryFunc);
	SequentialThreadSupport* threadSupport = new SequentialThreadSupport(tci);
	threadSupport->startSPU();
#else

#ifdef _WIN32
	Win32ThreadSupport::Win32ThreadConstructionInfo threadConstructionInfo("solverThreads",SolverThreadFunc,SolverlsMemoryFunc,maxNumThreads);
	Win32ThreadSupport* threadSupport = new Win32ThreadSupport(threadConstructionInfo);
	threadSupport->startSPU();
#elif defined (USE_PTHREADS)
	PosixThreadSupport::ThreadConstructionInfo solverConstructionInfo("solver", SolverThreadFunc,
																	  SolverlsMemoryFunc, maxNumThreads);
	
	PosixThreadSupport* threadSupport = new PosixThreadSupport(solverConstructionInfo);
	
#else
	SequentialThreadSupport::SequentialThreadConstructionInfo tci("solverThreads",SolverThreadFunc,SolverlsMemoryFunc);
	SequentialThreadSupport* threadSupport = new SequentialThreadSupport(tci);
	threadSupport->startSPU();
#endif
	
#endif

	return threadSupport;
}
#endif


class PEDebugDrawer : public btIDebugDraw
{
	int m_debugMode;

public:

	PEDebugDrawer()
		:m_debugMode(0)
	{
	}
	virtual void	drawLine(const btVector3& from,const btVector3& to,const btVector3& clr)
	{
		PfxVector3 position1 = getVmVector3(from);
		PfxVector3 position2 = getVmVector3(to);
		PfxVector3 color = getVmVector3(clr);
		render_debug_line(position1,position2,color);
	}
	virtual void	drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color)
	{

	}

	virtual void	reportErrorWarning(const char* warningString)
	{
		SCE_PFX_PRINTF(warningString);
	}

	virtual void	draw3dText(const btVector3& location,const char* textString)
	{
	}
	
	virtual void	setDebugMode(int debugMode)
	{
		m_debugMode = debugMode;
	}
	
	virtual int		getDebugMode() const 
	{
		return m_debugMode;
	}
};

///////////////////////////////////////////////////////////////////////////////
// Simulation Data

#define NUM_RIGIDBODIES 5000
#define NUM_JOINTS    500
#define NUM_CONTACTS  20000

const float timeStep = 0.016f;
const float separateBias = 0.1f;
int iteration = 5;


//J ワールドサイズ
//E World size
PfxVector3 worldCenter(0.0f);
PfxVector3 worldExtent(500.0f);

//J 剛体
//E Rigid body
PfxRigidState states[NUM_RIGIDBODIES];
PfxRigidBody  bodies[NUM_RIGIDBODIES];
PfxCollidable collidables[NUM_RIGIDBODIES];
PfxSolverBody solverBodies[NUM_RIGIDBODIES];
int numRigidBodies = 0;
//PfxConstraintPair jointPairs[NUM_JOINTS];
PfxJoint joints[NUM_JOINTS];
int numJoints = 0;

//J 地形を表現するためのラージメッシュ
//E Large mesh for representing a landscape
#include "landscape.h"
PfxLargeTriMesh gLargeMesh;

//J 凸メッシュ
//E Convex Mesh
#include "barrel.h"
PfxConvexMesh gConvex;



int physics_get_num_rigidbodies()
{
	return numRigidBodies;
}

const PfxRigidState& physics_get_state(int id)
{
	return states[id];
}

const PfxRigidBody& physics_get_body(int id)
{
	return bodies[id];
}

const PfxCollidable& physics_get_collidable(int id)
{
	return collidables[id];
}





///////////////////////////////////////////////////////////////////////////////
// Create Scene

void createBrick(int id,const PfxVector3 &pos,const PfxQuat &rot,const PfxVector3 &boxSize,PfxFloat mass)
{
	PfxBox box(boxSize);
	PfxShape shape;
	shape.reset();
	shape.setBox(box);
	collidables[id].reset();
	collidables[id].addShape(shape);
	collidables[id].finish();
	bodies[id].reset();
	bodies[id].setRestitution(0.0f);
	bodies[id].setMass(mass);
	bodies[id].setInertia(pfxCalcInertiaBox(boxSize,mass));
	states[id].reset();
	states[id].setPosition(pos);
	states[id].setOrientation(rot);
	states[id].setMotionType(kPfxMotionTypeActive);
	states[id].setRigidBodyId(id);
}

void createWall(const PfxVector3 &offsetPosition,int stackSize,const PfxVector3 &boxSize)
{
	PfxFloat bodyMass = 0.5f;

	PfxFloat diffX = boxSize[0] * 1.02f;
	PfxFloat diffY = boxSize[1] * 1.02f;
	PfxFloat diffZ = boxSize[2] * 1.02f;

	PfxFloat offset = -stackSize * (diffZ * 2.0f) * 0.5f;
	PfxVector3 pos(0.0f, diffY, 0.0f);

	while(stackSize) {
		for(int i=0;i<stackSize;i++) {
			pos[2] = offset + (PfxFloat)i * (diffZ * 2.0f);
		
			createBrick(numRigidBodies++,offsetPosition+pos,PfxQuat::identity(),boxSize,bodyMass);
		}
		offset += diffZ;
		pos[1] += (diffY * 2.0f);
		stackSize--;
	}
}

void createPyramid(const PfxVector3 &offsetPosition,int stackSize,const PfxVector3 &boxSize)
{
	PfxFloat space = 0.0001f;
	PfxVector3 pos(0.0f, boxSize[1], 0.0f);

	PfxFloat diffX = boxSize[0] * 1.02f;
	PfxFloat diffY = boxSize[1] * 1.02f;
	PfxFloat diffZ = boxSize[2] * 1.02f;

	PfxFloat offsetX = -stackSize * (diffX * 2.0f + space) * 0.5f;
	PfxFloat offsetZ = -stackSize * (diffZ * 2.0f + space) * 0.5f;
	while(stackSize) {
		for(int j=0;j<stackSize;j++) {
			pos[2] = offsetZ + (PfxFloat)j * (diffZ * 2.0f + space);
			for(int i=0;i<stackSize;i++) {
				pos[0] = offsetX + (PfxFloat)i * (diffX * 2.0f + space);
				createBrick(numRigidBodies++,offsetPosition+pos,PfxQuat::identity(),boxSize,1.0f);
			}
		}
		offsetX += diffX;
		offsetZ += diffZ;
		pos[1] += (diffY * 2.0f + space);
		stackSize--;
	}
}

void createTowerCircle(const PfxVector3 &offsetPosition,int stackSize,int rotSize,const PfxVector3 &boxSize)
{
	PfxFloat radius = 1.3f * rotSize * boxSize[0] / SCE_PFX_PI;

	// create active boxes
	PfxQuat rotY = PfxQuat::identity();
	PfxFloat posY = boxSize[1];

	for(int i=0;i<stackSize;i++) {
		for(int j=0;j<rotSize;j++) {
			createBrick(numRigidBodies++,offsetPosition+rotate(rotY,PfxVector3(0.0f , posY, radius)),rotY,boxSize,0.5f);

			rotY *= PfxQuat::rotationY(SCE_PFX_PI/(rotSize*0.5f));
		}

		posY += boxSize[1] * 2.0f;
		rotY *= PfxQuat::rotationY(SCE_PFX_PI/(PfxFloat)rotSize);
	}
}

void createScenePrimitives()
{

	// sphere
	{
		int id = numRigidBodies++;
		PfxSphere sphere(1.0f);
		PfxShape shape;
		shape.reset();
		shape.setSphere(sphere);
		collidables[id].reset();
		collidables[id].addShape(shape);
		collidables[id].finish();
		bodies[id].reset();
		bodies[id].setMass(1.0f);
		bodies[id].setInertia(pfxCalcInertiaSphere(1.0f,1.0f));
		states[id].reset();
		states[id].setPosition(PfxVector3(-5.0f,5.0f,0.0f));
		states[id].setMotionType(kPfxMotionTypeActive);
		states[id].setRigidBodyId(id);
	}

	// box
	{
		int id = numRigidBodies++;
		PfxBox box(1.0f,1.0f,1.0f);
		PfxShape shape;
		shape.reset();
		shape.setBox(box);
		collidables[id].reset();
		collidables[id].addShape(shape);
		collidables[id].finish();
		bodies[id].reset();
		bodies[id].setMass(1.0f);
		bodies[id].setInertia(pfxCalcInertiaBox(PfxVector3(1.0f),1.0f));
		states[id].reset();
		states[id].setPosition(PfxVector3(0.0f,5.0f,5.0f));
		states[id].setMotionType(kPfxMotionTypeActive);
		states[id].setRigidBodyId(id);
	}

	// capsule
	{
		int id = numRigidBodies++;
		PfxCapsule capsule(1.5f,0.5f);
		PfxShape shape;
		shape.reset();
		shape.setCapsule(capsule);
		collidables[id].reset();
		collidables[id].addShape(shape);
		collidables[id].finish();
		bodies[id].reset();
		bodies[id].setMass(2.0f);
		bodies[id].setInertia(pfxCalcInertiaCylinderX(2.0f,0.5f,2.0f));
		states[id].reset();
		states[id].setPosition(PfxVector3(5.0f,5.0f,0.0f));
		states[id].setMotionType(kPfxMotionTypeActive);
		states[id].setRigidBodyId(id);
	}

	// cylinder
	{
		int id = numRigidBodies++;
		PfxCylinder cylinder(0.5f,1.5f);
		PfxShape shape;
		shape.reset();
		shape.setCylinder(cylinder);
		collidables[id].reset();
		collidables[id].addShape(shape);
		collidables[id].finish();
		bodies[id].reset();
		bodies[id].setMass(3.0f);
		bodies[id].setInertia(pfxCalcInertiaCylinderX(0.5f,1.5f,3.0f));
		states[id].reset();
		states[id].setPosition(PfxVector3(0.0f,10.0f,0.0f));
		states[id].setMotionType(kPfxMotionTypeActive);
		states[id].setRigidBodyId(id);
	}

	// convex mesh
	{
		PfxCreateConvexMeshParam param;

		param.verts = BarrelVtx;
		param.numVerts = BarrelVtxCount;
		param.vertexStrideBytes = sizeof(float)*6;

		param.triangles = BarrelIdx;
		param.numTriangles = BarrelIdxCount/3;
		param.triangleStrideBytes = sizeof(unsigned short)*3;

		PfxInt32 ret = pfxCreateConvexMesh(gConvex,param);
		if(ret != SCE_PFX_OK) {
			SCE_PFX_PRINTF("Can't create gConvex mesh.\n");
		}

		int id = numRigidBodies++;
		PfxShape shape;
		shape.reset();
		shape.setConvexMesh(&gConvex);
		collidables[id].reset();
		collidables[id].addShape(shape);
		collidables[id].finish();
		bodies[id].reset();
		bodies[id].setMass(3.0f);
		bodies[id].setInertia(pfxCalcInertiaSphere(1.0f,1.0f));
		states[id].reset();
		states[id].setPosition(PfxVector3(0.0f,15.0f,0.0f));
		states[id].setMotionType(kPfxMotionTypeActive);
		states[id].setRigidBodyId(id);
	}

	// combined primitives
	{
		int id = numRigidBodies++;

		//E Both shapes and incides buffer have to be kept when creating a combined shape.
		static PfxShape shapes[3];
		PfxUInt16 shapeIds[3]={0,1,2};
		collidables[id].reset(shapes,shapeIds,3);
		{
			PfxBox box(0.5f,0.5f,1.5f);
			PfxShape shape;
			shape.reset();
			shape.setBox(box);
			shape.setOffsetPosition(PfxVector3(-2.0f,0.0f,0.0f));
			collidables[id].addShape(shape);
		}
		{
			PfxBox box(0.5f,1.5f,0.5f);
			PfxShape shape;
			shape.reset();
			shape.setBox(box);
			shape.setOffsetPosition(PfxVector3(2.0f,0.0f,0.0f));
			collidables[id].addShape(shape);
		}
		{
			PfxCapsule cap(1.5f,0.5f);
			PfxShape shape;
			shape.reset();
			shape.setCapsule(cap);
			collidables[id].addShape(shape);
		}
		collidables[id].finish();
		bodies[id].reset();
		bodies[id].setMass(3.0f);
		bodies[id].setInertia(pfxCalcInertiaBox(PfxVector3(2.5f,1.0f,1.0f),3.0f));
		states[id].reset();
		states[id].setPosition(PfxVector3(0.0f,5.0f,0.0f));
		states[id].setMotionType(kPfxMotionTypeActive);
		states[id].setRigidBodyId(id);
	}


}

void createSceneJoints()
{
	
	const int n = 10;

	int startId = numRigidBodies;

	PfxVector3 boxSize(1.0f);
	PfxFloat boxMass = 1.0f;

	for(int i=0;i<n;i++) {
		createBrick(numRigidBodies++,PfxVector3(0,3.0f+i*2.5f*boxSize[1],0),PfxQuat::identity(),boxSize,boxMass);
	}

	for(int i=startId;i<startId+n;i++) {
		PfxRigidState &stateA = states[i];
		PfxRigidState &stateB = states[(i+1)%numRigidBodies];
		PfxVector3 anchor;
		if(i == numRigidBodies-1) {
			anchor = stateA.getPosition() + PfxVector3(0,boxSize[1],0);
		}
		else {
			anchor = ( stateA.getPosition() + stateB.getPosition() ) * 0.5f;
		}

		PfxSwingTwistJointInitParam jparam;
		jparam.anchorPoint = anchor;
		jparam.twistAxis = PfxVector3(0,1,0);

		pfxInitializeSwingTwistJoint(joints[numJoints],stateA,stateB,jparam);
		joints[numJoints].m_constraints[4].m_damping = 0.1f;
		joints[numJoints].m_constraints[5].m_damping = 0.1f;


		SCE_PFX_ASSERT(numJoints<NUM_JOINTS);
		numJoints++;
	}

	states[startId].setLinearVelocity(PfxVector3(0,0,5));
	states[startId].setLinearDamping(0.95f);
	states[startId].setAngularDamping(0.95f);
	
}

void createSceneStacking()
{
	createTowerCircle(PfxVector3(0.0f,0.0f,0.0f),8,24,PfxVector3(1));
}

void createSceneBoxGround()
{
	int id = numRigidBodies++;
	PfxBox box(150.0f,2.5f,150.0f);
	PfxShape shape;
	shape.reset();
	shape.setBox(box);
	collidables[id].reset();
	collidables[id].addShape(shape);
	collidables[id].finish();
	bodies[id].reset();
	states[id].reset();
	states[id].setPosition(PfxVector3(0.0f,-2.5f,0.0f));
	states[id].setMotionType(kPfxMotionTypeFixed);
	states[id].setRigidBodyId(id);
}

void createSceneLandscape()
{
	PfxCreateLargeTriMeshParam param;

	param.verts = LargeMeshVtx;
	param.numVerts = LargeMeshVtxCount;
	param.vertexStrideBytes = sizeof(float)*6;

	param.triangles = LargeMeshIdx;
	param.numTriangles = LargeMeshIdxCount/3;
	param.triangleStrideBytes = sizeof(unsigned short)*3;

	if(gLargeMesh.m_numIslands > 0) {
		pfxReleaseLargeTriMesh(gLargeMesh);
	}

	PfxInt32 ret = pfxCreateLargeTriMesh(gLargeMesh,param);
	if(ret != SCE_PFX_OK) {
		SCE_PFX_PRINTF("Can't create large mesh.\n");
	}

	int id = numRigidBodies++;
	PfxShape shape;
	shape.reset();
	shape.setLargeTriMesh(&gLargeMesh);
	collidables[id].reset();
	collidables[id].addShape(shape);
	collidables[id].finish();
	bodies[id].reset();
	states[id].reset();
	states[id].setPosition(PfxVector3(0.0f,-5.0f,0.0f));
	states[id].setOrientation(PfxQuat::rotationX(0.5f)*PfxQuat::rotationY(0.7f));
	states[id].setMotionType(kPfxMotionTypeFixed);
	states[id].setRigidBodyId(id);
}

#include <btBulletDynamicsCommon.h>
#include "BulletMultiThreaded/vectormath2bullet.h"

btDefaultCollisionConfiguration*	m_config=0;
btCollisionDispatcher*				m_dispatcher=0;
btBroadphaseInterface*				m_broadphase=0;
btConstraintSolver*					m_solver=0;
btDiscreteDynamicsWorld*			m_dynamicsWorld=0;
btThreadSupportInterface*			sThreadSupport=0;

void	debugRenderPhysics()
{
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();
}

static void cleanupBullet(void)
{
	if (m_dynamicsWorld)
	{
		delete m_dynamicsWorld;
		delete m_solver;
//		delete sThreadSupport;
		delete m_broadphase;
		delete m_dispatcher;
		delete m_config;
	}
}

void physics_simulate()
{
	//run the simulation

	static btClock clock;
	static bool first = true;
	if (first)
	{
		first=false;
		clock.reset();
	}
	btScalar dt = (btScalar)clock.getTimeMicroseconds();
	clock.reset();


	
	m_dynamicsWorld->stepSimulation(dt/1000000.f);

	int i;
	for (i=0;i<m_dynamicsWorld->getNumCollisionObjects();i++)
	{
		btRigidBody* body = btRigidBody::upcast(m_dynamicsWorld->getCollisionObjectArray()[i]);
		if (body)
		{
			
			PfxRigidState* state = (PfxRigidState*) body->getUserPointer();

			PfxVector3 pe_pos = getVmVector3(body->getWorldTransform().getOrigin());
			PfxQuat pe_orn = getVmQuat(body->getWorldTransform().getRotation());
			PfxVector3 pe_lvel = getVmVector3(body->getLinearVelocity());
			PfxVector3 pe_avel = getVmVector3(body->getAngularVelocity());

			state->setPosition(pe_pos);
			state->setOrientation(pe_orn);
			state->setLinearVelocity(pe_lvel);
			state->setAngularVelocity(pe_avel);
		}
	}
	
	
}

btRigidBody*	addRigidBody(btCollisionShape* shape, btScalar mass, const btTransform& startTransform)
{
	btVector3 localInertia;
	localInertia.setZero();

	if (mass)
		shape->calculateLocalInertia(mass,localInertia);
		
	btRigidBody* body = new btRigidBody(mass,0,shape,localInertia);	
	body->setWorldTransform(startTransform);
	body->setActivationState(DISABLE_DEACTIVATION);
	//body->setFriction(2);
	m_dynamicsWorld->addRigidBody(body);
	return body;
}

static void convertToBullet(void)
{

	cleanupBullet();

	m_config = new btDefaultCollisionConfiguration();
	
	btHashedOverlappingPairCache* pairCache = new btHashedOverlappingPairCache();
	//m_broadphase = new btDbvtBroadphase();
	btLowLevelData* lowLevelData = new btLowLevelData;
	
	lowLevelData->m_maxContacts = NUM_CONTACTS;//8024;
	lowLevelData->m_contactIdPool = (int*) btAlignedAlloc(sizeof(int)*lowLevelData->m_maxContacts ,16);
	lowLevelData->m_contacts = (PfxContactManifold*) btAlignedAlloc(sizeof(PfxContactManifold)*lowLevelData->m_maxContacts,16);
	lowLevelData->m_maxPairs = lowLevelData->m_maxContacts;//??
	lowLevelData->m_pairsBuff[0] = (PfxBroadphasePair*) btAlignedAlloc(sizeof(PfxBroadphasePair)*lowLevelData->m_maxPairs,16);
	lowLevelData->m_pairsBuff[1] = (PfxBroadphasePair*) btAlignedAlloc(sizeof(PfxBroadphasePair)*lowLevelData->m_maxPairs,16);

#define USE_LL_BP
#ifdef USE_LL_BP
	btLowLevelBroadphase* llbp = new btLowLevelBroadphase(lowLevelData,pairCache);
	m_broadphase = llbp;
#else //USE_LL_BP
	m_broadphase = new btDbvtBroadphase();
#endif //USE_LL_BP

	//m_dispatcher = new btCollisionDispatcher(m_config);
	m_dispatcher = new btLowLevelCollisionDispatcher(lowLevelData,m_config,NUM_CONTACTS);
	
//#ifdef USE_PARALLEL_SOLVER
//	m_solver = new btSequentialImpulseConstraintSolver();
//#else
	//sThreadSupport = createSolverThreadSupport(4);
	//m_solver = new btLowLevelConstraintSolver(sThreadSupport);
	m_solver = new btLowLevelConstraintSolver2(lowLevelData);
//#endif //USE_PARALLEL_SOLVER

	//m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_config);
	m_dynamicsWorld = new btBulletPhysicsEffectsWorld(lowLevelData, m_dispatcher,llbp,m_solver,m_config,0);
	PEDebugDrawer* drawer = new PEDebugDrawer();
	drawer->setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawAabb);

	m_dynamicsWorld->setDebugDrawer(drawer);
	
	m_dynamicsWorld ->getSolverInfo().m_splitImpulse = true;
	
	for(int i=0;i<physics_get_num_rigidbodies();i++) {

		btRigidBody* body = 0;
		btAlignedObjectArray<btCollisionShape*> shapes;
		btAlignedObjectArray<btTransform> childTtransforms;

		PfxRigidState &state = states[i];//physics_get_state(i);
		state.setUserData(0);

		
		const PfxCollidable &coll = physics_get_collidable(i);

		PfxTransform3 rbT(state.getOrientation(), state.getPosition());

		PfxShapeIterator itrShape(coll);
		for(int j=0;j<coll.getNumShapes();j++,++itrShape) {
			const PfxShape &shape = *itrShape;
			PfxTransform3 offsetT = shape.getOffsetTransform();
			PfxTransform3 worldT = rbT * offsetT;
			btTransform childTransform;
			childTransform.setIdentity();
			childTransform.setOrigin(getBtVector3(offsetT.getTranslation()));
			PfxQuat quat(offsetT.getUpper3x3());
			childTransform.setBasis(btMatrix3x3(getBtQuat(quat)));
			childTtransforms.push_back(childTransform);

			switch(shape.getType()) {
				case kPfxShapeSphere:
					{
					btSphereShape* sphere = new btSphereShape(shape.getSphere().m_radius);
					shapes.push_back(sphere);
					//render_sphere(	worldT,	Vectormath::Aos::Vector3(1,1,1),Vectormath::floatInVec(shape.getSphere().m_radius));
					}
				break;

				case kPfxShapeBox:
					{
					btBoxShape* box = new btBoxShape(getBtVector3(shape.getBox().m_half));
					shapes.push_back(box);
					}
					//render_box(		worldT,		Vectormath::Aos::Vector3(1,1,1),	shape.getBox().m_half);
				break;

				case kPfxShapeCapsule:
					shapes.push_back(new btCapsuleShapeX(shape.getCapsule().m_radius,2.f*shape.getCapsule().m_halfLen));
					//render_capsule(		worldT,		Vectormath::Aos::Vector3(1,1,1),	Vectormath::floatInVec(shape.getCapsule().m_radius),	Vectormath::floatInVec(shape.getCapsule().m_halfLen));
				break;

				case kPfxShapeCylinder:
					shapes.push_back(new btCylinderShapeX(btVector3(shape.getCylinder().m_halfLen,shape.getCylinder().m_radius,shape.getCylinder().m_radius)));
					//render_cylinder(	worldT,	Vectormath::Aos::Vector3(1,1,1),	Vectormath::floatInVec(shape.getCylinder().m_radius),Vectormath::floatInVec(shape.getCylinder().m_halfLen));
				break;

				case kPfxShapeConvexMesh:
					{
					const PfxConvexMesh* convex = shape.getConvexMesh();
					const btScalar* vertices = (const btScalar*)&convex->m_verts[0];
					btConvexHullShape* convexHull = new btConvexHullShape(vertices,convex->m_numVerts, sizeof(PfxVector3));
					shapes.push_back(convexHull);
					}
				break;

				case kPfxShapeLargeTriMesh:
					{
						const PfxLargeTriMesh* mesh = shape.getLargeTriMesh();
						btTriangleIndexVertexArray* meshInterface = new btTriangleIndexVertexArray();
						int i;
						for (i= 0; i < mesh->m_numIslands;i++)
						{
							PfxTriMesh* island = &mesh->m_islands[i];
							
						//mesh->m_islands
							//mesh->m_numIslands
							btIndexedMesh indexedMesh;
							indexedMesh.m_indexType = PHY_UCHAR;
							indexedMesh.m_numTriangles = island->m_numFacets;
							indexedMesh.m_triangleIndexBase = &island->m_facets[0].m_vertIds[0];
							indexedMesh.m_triangleIndexStride = sizeof(PfxFacet);
							indexedMesh.m_vertexBase = (const unsigned char*) &island->m_verts[0];
							indexedMesh.m_numVertices = island->m_numVerts;//unused
							indexedMesh.m_vertexStride = sizeof(PfxVector3);
							meshInterface->addIndexedMesh(indexedMesh,PHY_UCHAR);
						}
						
						btBvhTriangleMeshShape* trimesh = new btBvhTriangleMeshShape(meshInterface,true);
						shapes.push_back(trimesh);
					}
					
				break;

				default:
					{
						printf("unknown\n");
					}
				break;
			}
		}
	
		if(shapes.size()>0)
		{
			printf("shapes!\n");
			btCollisionShape* colShape = 0;
			if (shapes.size()==1 && childTtransforms[0].getOrigin().fuzzyZero())
//todo: also check orientation
			{
				colShape = shapes[0];
			}
			else
			{
				btCompoundShape* compound = new btCompoundShape();
				for (int i=0;i<shapes.size();i++)
				{
					compound->addChildShape(childTtransforms[i],shapes[i]);
				}
				colShape = compound;

			}
			
			btTransform worldTransform;
			worldTransform.setIdentity();
			worldTransform.setOrigin(getBtVector3(rbT.getTranslation()));
			PfxQuat quat(rbT.getUpper3x3());
			worldTransform.setBasis(btMatrix3x3(getBtQuat(quat)));
			
			btScalar mass = physics_get_body(i).getMass();
			btRigidBody* body = addRigidBody(colShape,mass,worldTransform);
			void* ptr = (void*)&state;
			body->setUserPointer(ptr);
			state.setUserData(body);
		}
		
	}

	//very basic joint conversion (only limits of PFX_JOINT_SWINGTWIST joint)
	for (int i=0;i<numJoints;i++)
	{
		PfxJoint& joint = joints[i];
		switch (joint.m_type)
		{
		case kPfxJointSwingtwist:
			{
				//btConeTwistConstraint* coneTwist = new btConeTwistConstraint(rbA,rbB,frameA,frameB);
				bool referenceA = true;
				btRigidBody* bodyA = (btRigidBody*)states[joint.m_rigidBodyIdA].getUserData();
				btRigidBody* bodyB = (btRigidBody*)states[joint.m_rigidBodyIdB].getUserData();
				if (bodyA&&bodyB)
				{
					btTransform frameA,frameB;
					frameA.setIdentity();
					frameB.setIdentity();
					frameA.setOrigin(getBtVector3(joint.m_anchorA));
					frameB.setOrigin(getBtVector3(joint.m_anchorB));
					bool referenceA = false;
					btGeneric6DofConstraint* dof6 = new btGeneric6DofConstraint(*bodyA,*bodyB,frameA,frameB,referenceA);
					
					for (int i=0;i<joint.m_numConstraints;i++)
					{
						switch (joint.m_constraints[i].m_lock)
						{
						case SCE_PFX_JOINT_LOCK_FIX:
							{
								dof6->setLimit(i,0,0);
								break;
							}
						case SCE_PFX_JOINT_LOCK_FREE:
							{
								dof6->setLimit(i,1,0);
								break;
							}
						case SCE_PFX_JOINT_LOCK_LIMIT:
							{
								//deal with the case where angular limits of Y-axis are free and/or beyond -PI/2 and PI/2
								if ((i==4) && ((joint.m_constraints[i].m_movableLowerLimit<-SIMD_PI/2)||(joint.m_constraints[i].m_movableUpperLimit>SIMD_PI/2)))
								{
									printf("error with joint limits, forcing DOF to fixed\n");
									dof6->setLimit(i,0,0);
								} else
								{
									dof6->setLimit(i,joint.m_constraints[i].m_movableLowerLimit,joint.m_constraints[i].m_movableUpperLimit);
								}
								
								break;
							}
						default:
							{
								printf("unknown joint lock state\n");
							}
						}

					}
					
					m_dynamicsWorld->addConstraint(dof6,true);
					
				} else
				{
					printf("error: missing bodies during joint conversion\n");
				}
				break;
			};
		case kPfxJointBall:
		case kPfxJointHinge:
		case kPfxJointSlider:
		case kPfxJointFix:
		case kPfxJointUniversal:
		case kPfxJointAnimation:
		default:
			{
				printf("unknown joint\n");
			}
		}
	}

	//create a large enough buffer. There is no method to pre-calculate the buffer size yet.
	int maxSerializeBufferSize = 1024*1024*5;
 
	btDefaultSerializer*	serializer = new btDefaultSerializer(maxSerializeBufferSize);
	m_dynamicsWorld->serialize(serializer);
 
	FILE* file = fopen("testFile.bullet","wb");
	fwrite(serializer->getBufferPointer(),serializer->getCurrentBufferSize(),1, file);
	fclose(file);

}

void physics_create_scene(int sceneId)
{
	const int numScenes = 4;
	int sid = sceneId % numScenes;
	
	numRigidBodies= 0;
//	numPairs = 0;
//	numContacts = 0;
	numJoints = 0;
//	//frame = 0;
	
	switch(sid) {
		case 0: // simple primitives
		createSceneBoxGround();
		createScenePrimitives();
		break;
		
		case 1: // joints
		createSceneBoxGround();
		createSceneJoints();
		break;

		case 2: // stacking
		createSceneBoxGround();
		createSceneStacking();
		break;

		case 3: // landscape
		createSceneLandscape();
		createScenePrimitives();
		break;
	}

	SCE_PFX_PRINTF("----- Size of rigid body buffer ------\n");
	SCE_PFX_PRINTF("                    size *   num = total\n");
	SCE_PFX_PRINTF("PfxRigidState      %5d * %5d = %5d bytes\n",sizeof(PfxRigidState),numRigidBodies,sizeof(PfxRigidState)*numRigidBodies);
	SCE_PFX_PRINTF("PfxRigidBody       %5d * %5d = %5d bytes\n",sizeof(PfxRigidBody),numRigidBodies,sizeof(PfxRigidBody)*numRigidBodies);
	SCE_PFX_PRINTF("PfxCollidable      %5d * %5d = %5d bytes\n",sizeof(PfxCollidable),numRigidBodies,sizeof(PfxCollidable)*numRigidBodies);
	//SCE_PFX_PRINTF("PfxJoint           %5d * %5d = %5d bytes\n",sizeof(PfxJoint),numJoints,sizeof(PfxJoint)*numJoints);
	SCE_PFX_PRINTF("PfxSolverBody      %5d * %5d = %5d bytes\n",sizeof(PfxSolverBody),numRigidBodies,sizeof(PfxSolverBody)*numRigidBodies);
	SCE_PFX_PRINTF("PfxBroadphaseProxy %5d * %5d = %5d bytes\n",sizeof(PfxBroadphaseProxy),numRigidBodies,sizeof(PfxBroadphaseProxy)*numRigidBodies);
	SCE_PFX_PRINTF("PfxContactManifold %5d * %5d = %5d bytes\n",sizeof(PfxContactManifold),NUM_CONTACTS,sizeof(PfxContactManifold)*NUM_CONTACTS);
	SCE_PFX_PRINTF("PfxBroadphasePair  %5d * %5d = %5d bytes\n",sizeof(PfxBroadphasePair),NUM_CONTACTS,sizeof(PfxBroadphasePair)*NUM_CONTACTS);

	int totalBytes = 
		(sizeof(PfxRigidState) + sizeof(PfxRigidBody) + sizeof(PfxCollidable) + sizeof(PfxSolverBody) + sizeof(PfxBroadphaseProxy)) * numRigidBodies +
		(sizeof(PfxContactManifold) + sizeof(PfxBroadphasePair)) * NUM_CONTACTS;

	//convert to Bullet
	convertToBullet();


	SCE_PFX_PRINTF("----------------------------------------------------------\n");
	SCE_PFX_PRINTF("Total %5d bytes\n",totalBytes);
}

bool physics_init()
{
	return true;
}

void physics_release()
{
}


#if 0

//J プロキシ
//E Proxies
PfxBroadphaseProxy proxies[NUM_RIGIDBODIES];

//J ジョイント
//E Joint
PfxConstraintPair jointPairs[NUM_JOINTS];
PfxJoint joints[NUM_JOINTS];
int numJoints = 0;

//J ペア
//E Pairs
unsigned int numPairs;
PfxBroadphasePair pairs[NUM_CONTACTS];

//J コンタクト
//E Contacts
PfxContactManifold contacts[NUM_CONTACTS];
int numContacts;

//J 一時バッファ
//E Temporary buffers
#define POOL_BYTES (5*1024*1024)
unsigned char PFX_ALIGNED(128) poolBuff[POOL_BYTES];

//J 一時バッファ用スタックアロケータ
//E Stack allocator for temporary buffers
PfxHeapManager pool(poolBuff,POOL_BYTES);

///////////////////////////////////////////////////////////////////////////////
// Simulation Function

int frame = 0;

void broadphase()
{
	//J 剛体が最も分散している軸を見つける
	//E Find the axis along which all rigid bodies are most widely positioned
	int axis = 0;
	{
		PfxVector3 s(0.0f),s2(0.0f);
		for(int i=0;i<numRigidBodies;i++) {
			PfxVector3 c = states[i].getPosition();
			s += c;
			s2 += mulPerElem(c,c);
		}
		PfxVector3 v = s2 - mulPerElem(s,s) / (float)numRigidBodies;
		if(v[1] > v[0]) axis = 1;
		if(v[2] > v[axis]) axis = 2;
	}

	//J ブロードフェーズプロキシの更新
	//E Create broadpahse proxies
	{
		for(int i=0;i<numRigidBodies;i++) {
			pfxUpdateBroadphaseProxy(proxies[i],states[i],collidables[i],worldCenter,worldExtent,axis);
		}
		
		int workBytes = sizeof(PfxBroadphaseProxy) * numRigidBodies;
		void *workBuff = pool.allocate(workBytes);
		
		pfxParallelSort(proxies,numRigidBodies,workBuff,workBytes);
		
		pool.deallocate(workBuff);
	}

	//J 交差ペア探索
	//E Find overlapped pairs
	{
		PfxFindPairsParam param;
		param.workBytes = pfxGetWorkBytesOfFindPairs(NUM_CONTACTS);
		param.workBuff = pool.allocate(param.workBytes);
		param.pairBytes = pfxGetPairBytesOfFindPairs(NUM_CONTACTS);
		param.pairBuff = pool.allocate(param.pairBytes);
		param.proxies = proxies;
		param.numProxies = numRigidBodies;
		param.maxPairs = NUM_CONTACTS;
		param.axis = axis;

		PfxFindPairsResult result;

		int ret = pfxFindPairs(param,result);
		if(ret != SCE_PFX_OK) SCE_PFX_PRINTF("pfxFindPairs failed %d\n",ret);
		
		memcpy(pairs,result.pairs,sizeof(PfxBroadphasePair)*result.numPairs);
		numPairs = result.numPairs;
		
		pool.deallocate(param.pairBuff);
		pool.deallocate(param.workBuff);
	}

	//J 新規ペアのコンタクトを初期化
	//E Add new contacts and initialize
	for(PfxUInt32 i=0;i<numPairs;i++) {
		pfxSetContactId(pairs[i],i);
		PfxContactManifold &contact = contacts[i];
		contact.reset(pfxGetObjectIdA(pairs[i]),pfxGetObjectIdB(pairs[i]));
	}

	numContacts = numPairs;
}

void collision()
{
	//J 衝突検出
	//E Detect collisions
	{
		PfxDetectCollisionParam param;
		param.contactPairs = pairs;
		param.numContactPairs = numPairs;
		param.offsetContactManifolds = contacts;
		param.offsetRigidStates = states;
		param.offsetCollidables = collidables;
		param.numRigidBodies = numRigidBodies;

		int ret = pfxDetectCollision(param);
		if(ret != SCE_PFX_OK) SCE_PFX_PRINTF("pfxDetectCollision failed %d\n",ret);
	}
}

void constraintSolver()
{
	PfxPerfCounter pc;

	pc.countBegin("setup solver bodies");
	{
		PfxSetupSolverBodiesParam param;
		param.states = states;
		param.bodies = bodies;
		param.solverBodies = solverBodies;
		param.numRigidBodies = numRigidBodies;
		
		int ret = pfxSetupSolverBodies(param);
		if(ret != SCE_PFX_OK) SCE_PFX_PRINTF("pfxSetupSolverBodies failed %d\n",ret);
	}
	pc.countEnd();

	pc.countBegin("setup contact constraints");
	{
		PfxSetupContactConstraintsParam param;
		param.contactPairs = pairs;
		param.numContactPairs = numPairs;
		param.offsetContactManifolds = contacts;
		param.offsetRigidStates = states;
		param.offsetRigidBodies = bodies;
		param.offsetSolverBodies = solverBodies;
		param.numRigidBodies = numRigidBodies;
		param.timeStep = timeStep;
		param.separateBias = separateBias;
		
		int ret = pfxSetupContactConstraints(param);
		if(ret != SCE_PFX_OK) SCE_PFX_PRINTF("pfxSetupJointConstraints failed %d\n",ret);
	}
	pc.countEnd();

	pc.countBegin("setup joint constraints");
	{
		PfxSetupJointConstraintsParam param;
		param.jointPairs = jointPairs;
		param.numJointPairs = numJoints;
		param.offsetJoints = joints;
		param.offsetRigidStates = states;
		param.offsetRigidBodies = bodies;
		param.offsetSolverBodies = solverBodies;
		param.numRigidBodies = numRigidBodies;
		param.timeStep = timeStep;

		for(int i=0;i<numJoints;i++) {
			pfxUpdateJointPairs(jointPairs[i],i,joints[i],states[joints[i].m_rigidBodyIdA],states[joints[i].m_rigidBodyIdB]);
		}

		int ret = pfxSetupJointConstraints(param);
		if(ret != SCE_PFX_OK) SCE_PFX_PRINTF("pfxSetupJointConstraints failed %d\n",ret);
	}
	pc.countEnd();

	pc.countBegin("solve constraints");
	{
		PfxSolveConstraintsParam param;
		param.workBytes = pfxGetWorkBytesOfSolveConstraints(numRigidBodies,numPairs,numJoints);
		param.workBuff = pool.allocate(param.workBytes);
		param.contactPairs = pairs;
		param.numContactPairs = numPairs;
		param.offsetContactManifolds = contacts;
		param.jointPairs = jointPairs;
		param.numJointPairs = numJoints;
		param.offsetJoints = joints;
		param.offsetRigidStates = states;
		param.offsetSolverBodies = solverBodies;
		param.numRigidBodies = numRigidBodies;
		param.iteration = iteration;

		int ret = pfxSolveConstraints(param);
		if(ret != SCE_PFX_OK) SCE_PFX_PRINTF("pfxSolveConstraints failed %d\n",ret);
		
		pool.deallocate(param.workBuff);
	}
	pc.countEnd();

	//pc.printCount();
}

void integrate()
{
	PfxUpdateRigidStatesParam param;
	param.states = states;
	param.bodies = bodies;
	param.numRigidBodies = numRigidBodies;
	param.timeStep = timeStep;
	
	pfxUpdateRigidStates(param);
}

void physics_simulate()
{
	PfxPerfCounter pc;

	for(int i=1;i<numRigidBodies;i++) {
		pfxApplyExternalForce(states[i],bodies[i],bodies[i].getMass()*PfxVector3(0.0f,-9.8f,0.0f),PfxVector3(0.0f),timeStep);
	}
	
	perf_push_marker("broadphase");
	pc.countBegin("broadphase");
	broadphase();
	pc.countEnd();
	perf_pop_marker();
	
	perf_push_marker("collision");
	pc.countBegin("collision");
	collision();
	pc.countEnd();
	perf_pop_marker();
	
	perf_push_marker("solver");
	pc.countBegin("solver");
	constraintSolver();
	pc.countEnd();
	perf_pop_marker();
	
	perf_push_marker("integrate");
	pc.countBegin("integrate");
	integrate();
	pc.countEnd();
	perf_pop_marker();
	
	frame++;
	
	if(frame%100 == 0) {
		float broadphaseTime = pc.getCountTime(0);
		float collisionTime  = pc.getCountTime(2);
		float solverTime     = pc.getCountTime(4);
		float integrateTime  = pc.getCountTime(6);
		SCE_PFX_PRINTF("frame %3d broadphase %.2f collision %.2f solver %.2f integrate %.2f | total %.2f\n",frame,
			broadphaseTime,collisionTime,solverTime,integrateTime,
			broadphaseTime+collisionTime+solverTime+integrateTime);
	}
}


///////////////////////////////////////////////////////////////////////////////
// Initialize / Finalize Engine


///////////////////////////////////////////////////////////////////////////////
// Pick

PfxVector3 physics_pick_start(const PfxVector3 &p1,const PfxVector3 &p2)
{
	return PfxVector3(0.0f);
}

void physics_pick_update(const PfxVector3 &p)
{
}

void physics_pick_end()
{
}

///////////////////////////////////////////////////////////////////////////////
// Get Information

int physics_get_num_contacts()
{
	return numPairs;
}

const PfxContactManifold &physics_get_contact(int id)
{
	return contacts[pfxGetConstraintId(pairs[id])];
}

#endif
