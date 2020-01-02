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

btDefaultSoftBodySolverMt::btDefaultSoftBodySolverMt()
{
	// Initial we will clearly need to update solver constants
	// For now this is global for the cloths linked with this solver - we should probably make this body specific
	// for performance in future once we understand more clearly when constants need to be updated
	m_updateSolverConstants = true;
}

btDefaultSoftBodySolverMt::~btDefaultSoftBodySolverMt()
{
}

// In this case the data is already in the soft bodies so there is no need for us to do anything
void btDefaultSoftBodySolverMt::copyBackToSoftBodies(bool bMove)
{
}

void btDefaultSoftBodySolverMt::optimize(btAlignedObjectArray<btSoftBody *> &softBodies, bool forceUpdate)
{
	m_softBodySet.copyFromArray(softBodies);
}

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

bool btDefaultSoftBodySolverMt::checkInitialized()
{
	return true;
}

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

void btDefaultSoftBodySolverMt::copySoftBodyToVertexBuffer(const btSoftBody *const softBody, btVertexBufferDescriptor *vertexBuffer)
{
	// Currently only support CPU output buffers
	// TODO: check for DX11 buffers. Take all offsets into the same DX11 buffer
	// and use them together on a single kernel call if possible by setting up a
	// per-cloth target buffer array for the copy kernel.

	if (vertexBuffer->getBufferType() == btVertexBufferDescriptor::CPU_BUFFER)
	{
		const btAlignedObjectArray<btSoftBody::Node> &clothVertices(softBody->m_nodes);
		int numVertices = clothVertices.size();

		const btCPUVertexBufferDescriptor *cpuVertexBuffer = static_cast<btCPUVertexBufferDescriptor *>(vertexBuffer);
		float *basePointer = cpuVertexBuffer->getBasePointer();

		if (vertexBuffer->hasVertexPositions())
		{
			const int vertexOffset = cpuVertexBuffer->getVertexOffset();
			const int vertexStride = cpuVertexBuffer->getVertexStride();
			float *vertexPointer = basePointer + vertexOffset;

			for (int vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex)
			{
				btVector3 position = clothVertices[vertexIndex].m_x;
				*(vertexPointer + 0) = (float)position.getX();
				*(vertexPointer + 1) = (float)position.getY();
				*(vertexPointer + 2) = (float)position.getZ();
				vertexPointer += vertexStride;
			}
		}
		if (vertexBuffer->hasNormals())
		{
			const int normalOffset = cpuVertexBuffer->getNormalOffset();
			const int normalStride = cpuVertexBuffer->getNormalStride();
			float *normalPointer = basePointer + normalOffset;

			for (int vertexIndex = 0; vertexIndex < numVertices; ++vertexIndex)
			{
				btVector3 normal = clothVertices[vertexIndex].m_n;
				*(normalPointer + 0) = (float)normal.getX();
				*(normalPointer + 1) = (float)normal.getY();
				*(normalPointer + 2) = (float)normal.getZ();
				normalPointer += normalStride;
			}
		}
	}
}  // btDefaultSoftBodySolverMt::copySoftBodyToVertexBuffer

void btDefaultSoftBodySolverMt::processCollision(btSoftBody *softBody, btSoftBody *otherSoftBody)
{
	softBody->defaultCollisionHandler(otherSoftBody);
}

// For the default solver just leave the soft body to do its collision processing
void btDefaultSoftBodySolverMt::processCollision(btSoftBody *softBody, const btCollisionObjectWrapper *collisionObjectWrap)
{
	softBody->defaultCollisionHandler(collisionObjectWrap);
}  // btDefaultSoftBodySolverMt::processCollision

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
				psb->predictMotion(mTimeStep);
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
