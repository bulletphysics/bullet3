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

#ifndef SPU_BATCH_RAYCASTER_H
#define SPU_BATCH_RAYCASTER_H

#include "LinearMath/btAlignedObjectArray.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "SpuRaycastTaskProcess.h"
#include "SpuRaycastTask/SpuRaycastTask.h"
#include "SpuCollisionObjectWrapper.h"

/* FIXME:
 * Need to decide how callbacks are performed...
 */
class SpuBatchRaycaster 
{
protected:
	SpuCollisionObjectWrapper* castUponObjectWrappers;
	int numCastUponObjectWrappers;
	btAlignedObjectArray<SpuRaycastTaskWorkUnit> rayBatch;
	btAlignedObjectArray<SpuRaycastTaskWorkUnitOut> rayBatchOutput;
	SpuRaycastTaskProcess* m_spuRaycastTaskProcess;
	class	btThreadSupportInterface*	m_threadInterface;
public:
	SpuBatchRaycaster (class btThreadSupportInterface* threadInterface, int maxNumOutstandingTasks);
	~SpuBatchRaycaster ();
	void setCollisionObjects (btCollisionObjectArray& castUponObjects, int numCastUponObjects);
	void setCollisionObjectsSkipPE (btCollisionObjectArray& castUponObjects, int numCastUponObjects);
	void addRay (const btVector3& rayFrom, const btVector3& rayTo, const btScalar hitFraction = 1.0);
	void clearRays ();
	void performBatchRaycast ();
	const SpuRaycastTaskWorkUnitOut& operator [] (int i) const;
	int getNumRays () const;
};

#endif
