/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://bulletphysics.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include <new>
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "LinearMath/btAlignedAllocator.h"
#include "SpuBatchRaycaster.h"

SpuBatchRaycaster::SpuBatchRaycaster (class	btThreadSupportInterface* threadInterface, int maxNumOutstandingTasks)
{
	m_threadInterface = threadInterface;

	castUponObjectWrappers = NULL;
	numCastUponObjectWrappers = 0;

	m_spuRaycastTaskProcess = new SpuRaycastTaskProcess(m_threadInterface,maxNumOutstandingTasks); // FIXME non constant
}

SpuBatchRaycaster::~SpuBatchRaycaster ()
{
	if (castUponObjectWrappers)
	{
		btAlignedFree (castUponObjectWrappers);
		castUponObjectWrappers = NULL;
	}
}

void
SpuBatchRaycaster::setCollisionObjects (btCollisionObjectArray& castUponObjects, int numCastUponObjects)
{
	if (castUponObjectWrappers)
	{
		btAlignedFree (castUponObjectWrappers);
		castUponObjectWrappers = NULL;
	}

	castUponObjectWrappers = (SpuCollisionObjectWrapper*)btAlignedAlloc (sizeof(SpuCollisionObjectWrapper) * numCastUponObjects,16);
	numCastUponObjectWrappers = numCastUponObjects;

	for (int i = 0; i < numCastUponObjectWrappers; i++)
	{
		castUponObjectWrappers[i] = SpuCollisionObjectWrapper(castUponObjects[i]);
	}
}

void
SpuBatchRaycaster::setCollisionObjectsSkipPE (btCollisionObjectArray& castUponObjects, int numCastUponObjects)
{
	if (castUponObjectWrappers)
	{
		btAlignedFree (castUponObjectWrappers);
		castUponObjectWrappers = NULL;
	}

	int numNonPEShapes = 0;
	for (int i = 0; i < numCastUponObjects; i++)
	{
		const btCollisionShape* shape = castUponObjects[i]->getCollisionShape();

		if (shape->getShapeType () == BOX_SHAPE_PROXYTYPE ||
			shape->getShapeType () == SPHERE_SHAPE_PROXYTYPE ||
			shape->getShapeType () == CAPSULE_SHAPE_PROXYTYPE)
		{
			continue;
		}

		numNonPEShapes++;
	}

	castUponObjectWrappers = (SpuCollisionObjectWrapper*)btAlignedAlloc (sizeof(SpuCollisionObjectWrapper) * numNonPEShapes,16);
	numCastUponObjectWrappers = numNonPEShapes;

	int index = 0;
	for (int i = 0; i < numCastUponObjects; i++)
	{
		const btCollisionShape* shape = castUponObjects[i]->getCollisionShape();

		if (shape->getShapeType () == BOX_SHAPE_PROXYTYPE ||
			shape->getShapeType () == SPHERE_SHAPE_PROXYTYPE ||
			shape->getShapeType () == CAPSULE_SHAPE_PROXYTYPE)
		{
			continue;
		}

		castUponObjectWrappers[index] = SpuCollisionObjectWrapper(castUponObjects[i]);
		index++;
	}

//	printf("Number of shapes bullet is casting against: %d\n", numNonPEShapes);
	btAssert (index == numNonPEShapes);
}

void
SpuBatchRaycaster::addRay (const btVector3& rayFrom, const btVector3& rayTo, const btScalar hitFraction)
{
	SpuRaycastTaskWorkUnitOut workUnitOut;
	workUnitOut.hitFraction = hitFraction;
	workUnitOut.hitNormal = btVector3(0.0, 1.0, 0.0);

	rayBatchOutput.push_back (workUnitOut);

	SpuRaycastTaskWorkUnit workUnit;
	workUnit.rayFrom = rayFrom;
	workUnit.rayTo = rayTo;
	rayBatch.push_back (workUnit);
}

void
SpuBatchRaycaster::clearRays ()
{
	rayBatch.clear ();
	rayBatchOutput.clear ();
}

void
SpuBatchRaycaster::performBatchRaycast ()
{
	m_spuRaycastTaskProcess->initialize2 (castUponObjectWrappers, numCastUponObjectWrappers);

	for (int i = 0; i < rayBatch.size(); i++)
	{
		rayBatch[i].output = &rayBatchOutput[i]; // assign output memory location
		m_spuRaycastTaskProcess->addWorkToTask(rayBatch[i]);
	}

	m_spuRaycastTaskProcess->flush2 ();
}

const SpuRaycastTaskWorkUnitOut&
SpuBatchRaycaster::operator [] (int i) const
{
	return rayBatchOutput[i];
}

int
SpuBatchRaycaster::getNumRays () const
{
	return rayBatchOutput.size();
}
