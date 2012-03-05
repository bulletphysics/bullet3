/*
   Copyright (C) 2010 Sony Computer Entertainment Inc.
   All rights reserved.

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

*/


#include "btLowLevelConstraintSolver2.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"

#include "LinearMath/btQuickprof.h"
#include "BulletMultiThreaded/btThreadSupportInterface.h"
#include "BulletMultiThreaded/HeapManager.h"
#include "BulletMultiThreaded/PlatformDefinitions.h"
#include "BulletPhysicsEffects/btLowLevelData.h"
#include "BulletMultiThreaded/vectormath2bullet.h"
//#include "PfxSimdUtils.h"
#include "LinearMath/btScalar.h"
#include "BulletMultiThreaded/HeapManager.h"



/////////////////


#define TMP_BUFF_BYTES (15*1024*1024)
static unsigned char ATTRIBUTE_ALIGNED128(tmp_buff[TMP_BUFF_BYTES]);




btLowLevelConstraintSolver2::btLowLevelConstraintSolver2(btLowLevelData* lowLevelData)
:m_lowLevelData(lowLevelData)
{
	
}
	
btLowLevelConstraintSolver2::~btLowLevelConstraintSolver2()
{
	
}


static void solveConstraints(btLowLevelData* lowLevelData, btScalar timeStep, btScalar separateBias, int iteration)
{
	PfxPerfCounter pc;
	HeapManager pool((unsigned char*)tmp_buff,TMP_BUFF_BYTES);

	unsigned int numCurrentPairs = lowLevelData->m_numPairs[lowLevelData->m_pairSwap];
	PfxBroadphasePair *currentPairs = lowLevelData->m_pairsBuff[lowLevelData->m_pairSwap];

	pc.countBegin("setup solver bodies");
	{
		PfxSetupSolverBodiesParam param;
		param.states = lowLevelData->m_states;
		param.bodies = lowLevelData->m_bodies;
		param.solverBodies = lowLevelData->m_solverBodies;
		param.numRigidBodies = lowLevelData->m_numRigidBodies;
		
		int ret = pfxSetupSolverBodies(param);
		if(ret != SCE_PFX_OK) 
			SCE_PFX_PRINTF("pfxSetupSolverBodies failed %d\n",ret);
	}
	pc.countEnd();

	pc.countBegin("setup contact constraints");
	{
		PfxSetupContactConstraintsParam param;
		param.contactPairs = currentPairs;
		param.numContactPairs = numCurrentPairs;
		param.offsetContactManifolds = lowLevelData->m_contacts;
		param.offsetRigidStates = lowLevelData->m_states;
		param.offsetRigidBodies = lowLevelData->m_bodies;
		param.offsetSolverBodies = lowLevelData->m_solverBodies;
		param.numRigidBodies = lowLevelData->m_numRigidBodies;
		param.timeStep = timeStep;
		param.separateBias = separateBias;
		
		int ret = pfxSetupContactConstraints(param);
		if(ret != SCE_PFX_OK) SCE_PFX_PRINTF("pfxSetupJointConstraints failed %d\n",ret);
	}
	pc.countEnd();
#if 0
	pc.countBegin("setup joint constraints");
	{
		PfxSetupJointConstraintsParam param;
		param.jointPairs = 0;//jointPairs;
		param.numJointPairs = 0;//numJoints;
		param.offsetJoints = 0;//joints;
		param.offsetRigidStates = lowLevelData->m_states;
		param.offsetRigidBodies = lowLevelData->m_bodies;
		param.offsetSolverBodies = lowLevelData->m_solverBodies;
		param.numRigidBodies = lowLevelData->m_numRigidBodies;
		param.timeStep = timeStep;

		for(int i=0;i<numJoints;i++) {
			pfxUpdateJointPairs(jointPairs[i],i,joints[i],states[joints[i].m_rigidBodyIdA],states[joints[i].m_rigidBodyIdB]);
		}

		int ret = pfxSetupJointConstraints(param);
		if(ret != SCE_PFX_OK) SCE_PFX_PRINTF("pfxSetupJointConstraints failed %d\n",ret);
	}
	pc.countEnd();
#endif

	pc.countBegin("solve constraints");
	{
		PfxSolveConstraintsParam param;
		param.workBytes = pfxGetWorkBytesOfSolveConstraints(lowLevelData->m_numRigidBodies,numCurrentPairs,0);//numJoints);
		param.workBuff = pool.allocate(param.workBytes);
		param.contactPairs = currentPairs;
		param.numContactPairs = numCurrentPairs;
		param.offsetContactManifolds = lowLevelData->m_contacts;
		param.jointPairs = 0;//jointPairs;
		param.numJointPairs = 0;//numJoints;
		param.offsetJoints = 0;//joints;
		param.offsetRigidStates = lowLevelData->m_states;
		param.offsetSolverBodies = lowLevelData->m_solverBodies;
		param.numRigidBodies = lowLevelData->m_numRigidBodies;
		param.iteration = iteration;

		int ret = pfxSolveConstraints(param);
		if(ret != SCE_PFX_OK) SCE_PFX_PRINTF("pfxSolveConstraints failed %d\n",ret);
		
		pool.deallocate(param.workBuff);
	}
	pc.countEnd();


	//pc.printCount();
}



btScalar btLowLevelConstraintSolver2::solveGroup(btCollisionObject** bodies1,int numRigidBodies,btPersistentManifold** manifoldPtr,int numManifolds,btTypedConstraint** constraints,int numConstraints,const btContactSolverInfo& infoGlobal, btIDebugDraw* debugDrawer, btStackAlloc* stackAlloc,btDispatcher* dispatcher)
{

	//copy velocity from BT to PE
	{
		BT_PROFILE("copy back");
		for (int i=0;i<numRigidBodies;i++)
		{
			btCollisionObject* obj = bodies1[i];
			btRigidBody* rb = btRigidBody::upcast(obj);
			if (rb && (rb->getInvMass()>0.f))
			{
				int objectIndex = rb->getBroadphaseProxy()->m_uniqueId;
				sce::PhysicsEffects::PfxRigidState& state = m_lowLevelData->m_states[objectIndex];

				state.setLinearVelocity(getVmVector3(rb->getLinearVelocity()));
				state.setAngularVelocity(getVmVector3(rb->getAngularVelocity()));

			}
		}
	}

	btScalar separateBias = 0.1f;
	solveConstraints(m_lowLevelData, infoGlobal.m_timeStep, separateBias, infoGlobal.m_numIterations);


	//copy resulting velocity back from PE to BT
	{
		BT_PROFILE("copy back");
		for (int i=0;i<numRigidBodies;i++)
		{
			btCollisionObject* obj = bodies1[i];
			btRigidBody* rb = btRigidBody::upcast(obj);
			if (rb && (rb->getInvMass()>0.f))
			{
				int objectIndex = rb->getBroadphaseProxy()->m_uniqueId;
				sce::PhysicsEffects::PfxRigidState& state = m_lowLevelData->m_states[objectIndex];

				rb->setLinearVelocity(btVector3(state.getLinearVelocity().getX(),state.getLinearVelocity().getY(),state.getLinearVelocity().getZ()));
				rb->setAngularVelocity(btVector3(state.getAngularVelocity().getX(),state.getAngularVelocity().getY(),state.getAngularVelocity().getZ()));
			}
		}
	}

	return 0.f;
}
