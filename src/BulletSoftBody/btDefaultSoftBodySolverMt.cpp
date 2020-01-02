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

#include "BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"

#include "btDefaultSoftBodySolverMt.h"
#include "BulletCollision/CollisionShapes/btCapsuleShape.h"
#include "BulletSoftBody/btSoftBody.h"
#include "LinearMath/btThreads.h"

struct SoftBodyUpdater : public btIParallelForBody
{
	btAlignedObjectArray<btSoftBody*>* mSoftBodySet;

	SoftBodyUpdater(btAlignedObjectArray<btSoftBody*>* softBodySet)
	{
		mSoftBodySet = softBodySet;
	}
	void forLoop(int iBegin, int iEnd) const
	{
		for (int i = iBegin; i < iEnd; ++i)
		{
			btSoftBody* psb = (btSoftBody*)(*mSoftBodySet)[i];
			if (psb->isActive())
			{
				psb->integrateMotion();
			}
		}
	}
};

void btDefaultSoftBodySolverMt::updateSoftBodies()
{
	if (m_softBodySet.size() > 0)
	{
		SoftBodyUpdater loop(&m_softBodySet);
		int grainSize = 10;
		btParallelFor(0, m_softBodySet.size(), grainSize, loop);
	}
}  // updateSoftBodies

struct SolveConstraints : public btIParallelForBody
{
	btAlignedObjectArray<btSoftBody*>* mSoftBodySet;

	SolveConstraints(btAlignedObjectArray<btSoftBody*>* softBodySet)
	{
		mSoftBodySet = softBodySet;
	}
	void forLoop(int iBegin, int iEnd) const
	{
		for (int i = iBegin; i < iEnd; ++i)
		{
			btSoftBody* psb = (btSoftBody*)(*mSoftBodySet)[i];
			if (psb->isActive())
			{
				psb->solveConstraints();
			}
		}
	}
};

void btDefaultSoftBodySolverMt::solveConstraints(btScalar solverdt)
{
	// Solve constraints for non-solver softbodies
	if (m_softBodySet.size() > 0)
	{
		SolveConstraints loop(&m_softBodySet);
		int grainSize = 10;
		btParallelFor(0, m_softBodySet.size(), grainSize, loop);
	}
}  // btDefaultSoftBodySolverMt::solveConstraints

struct PredictMotion : public btIParallelForBody
{
	btAlignedObjectArray<btSoftBody*>* mSoftBodySet;
	btScalar mTimeStep;

	PredictMotion(btAlignedObjectArray<btSoftBody*>* softBodySet, btScalar timeStep)
	{
		mSoftBodySet = softBodySet;
		mTimeStep = timeStep;
	}
	void forLoop(int iBegin, int iEnd) const
	{
		for (int i = iBegin; i < iEnd; ++i)
		{
			btSoftBody* psb = (btSoftBody*)(*mSoftBodySet)[i];
			if (psb->isActive())
			{
				psb->predictMotion(mTimeStep, true);
			}
		}
	}
};

void btDefaultSoftBodySolverMt::predictMotion(btScalar timeStep)
{
	if (m_softBodySet.size() > 0)
	{
		PredictMotion loop(&m_softBodySet, timeStep);
		int grainSize = 10;
		btParallelFor(0, m_softBodySet.size(), grainSize, loop);
	}
}
