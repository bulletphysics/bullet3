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

#include "../../../include/physics_effects/base_level/base/pfx_perf_counter.h"
#include "../../../include/physics_effects/base_level/collision/pfx_shape_iterator.h"
#include "../../../include/physics_effects/low_level/collision/pfx_collision_detection.h"
#include "../../base_level/broadphase/pfx_check_collidable.h"
#include "../../base_level/collision/pfx_contact_cache.h"
#include "pfx_detect_collision_func.h"

namespace sce {
namespace PhysicsEffects {

///////////////////////////////////////////////////////////////////////////////

int pfxCheckParamOfDetectCollision(PfxDetectCollisionParam &param)
{
	if(!param.contactPairs || !param.offsetContactManifolds || !param.offsetRigidStates|| !param.offsetCollidables ) return SCE_PFX_ERR_INVALID_VALUE;
	if(!SCE_PFX_PTR_IS_ALIGNED16(param.contactPairs) || !SCE_PFX_PTR_IS_ALIGNED16(param.offsetContactManifolds) || 
		!SCE_PFX_PTR_IS_ALIGNED16(param.offsetRigidStates) || !SCE_PFX_PTR_IS_ALIGNED16(param.offsetCollidables)) return SCE_PFX_ERR_INVALID_ALIGN;
	return SCE_PFX_OK;
}

///////////////////////////////////////////////////////////////////////////////
// SINGLE THREAD

#define SCE_PFX_CONTACT_THRESHOLD 0.0f

PfxInt32 pfxDetectCollision(PfxDetectCollisionParam &param)
{
	PfxInt32 ret = pfxCheckParamOfDetectCollision(param);
	if(ret != SCE_PFX_OK) 
		return ret;

	SCE_PFX_PUSH_MARKER("pfxDetectCollision");

	PfxConstraintPair *contactPairs = param.contactPairs;
	PfxUInt32 numContactPairs = param.numContactPairs;
	PfxContactManifold *offsetContactManifolds = param.offsetContactManifolds;
	PfxRigidState *offsetRigidStates = param.offsetRigidStates;
	PfxCollidable *offsetCollidables = param.offsetCollidables;
	PfxUInt32 numRigidBodies = param.numRigidBodies;

	for(PfxUInt32 i=0;i<numContactPairs;i++) {
		const PfxBroadphasePair &pair = contactPairs[i];
		if(!pfxCheckCollidableInCollision(pair)) {
			continue;
		}

		PfxUInt32 iContact = pfxGetContactId(pair);
		PfxUInt32 iA = pfxGetObjectIdA(pair);
		PfxUInt32 iB = pfxGetObjectIdB(pair);

		PfxContactManifold &contact = offsetContactManifolds[iContact];

		SCE_PFX_ALWAYS_ASSERT(iA==contact.getRigidBodyIdA());
		SCE_PFX_ALWAYS_ASSERT(iB==contact.getRigidBodyIdB());

		PfxRigidState &stateA = offsetRigidStates[iA];
		PfxRigidState &stateB = offsetRigidStates[iB];
		PfxCollidable &collA = offsetCollidables[iA];
		PfxCollidable &collB = offsetCollidables[iB];
		PfxTransform3 tA0(stateA.getOrientation(), stateA.getPosition());
		PfxTransform3 tB0(stateB.getOrientation(), stateB.getPosition());
		
		PfxContactCache contactCache;
		
		PfxShapeIterator itrShapeA(collA);
		for(PfxUInt32 j=0;j<collA.getNumShapes();j++,++itrShapeA) {
			const PfxShape &shapeA = *itrShapeA;
			PfxTransform3 offsetTrA = shapeA.getOffsetTransform();
			PfxTransform3 worldTrA = tA0 * offsetTrA;

			PfxShapeIterator itrShapeB(collB);
			for(PfxUInt32 k=0;k<collB.getNumShapes();k++,++itrShapeB) {
				const PfxShape &shapeB = *itrShapeB;
				PfxTransform3 offsetTrB = shapeB.getOffsetTransform();
				PfxTransform3 worldTrB = tB0 * offsetTrB;

				if( (shapeA.getContactFilterSelf()&shapeB.getContactFilterTarget()) && 
				    (shapeA.getContactFilterTarget()&shapeB.getContactFilterSelf()) ) {
					pfxGetDetectCollisionFunc(shapeA.getType(),shapeB.getType())(
						contactCache,
						shapeA,offsetTrA,worldTrA,j,
						shapeB,offsetTrB,worldTrB,k,
						SCE_PFX_CONTACT_THRESHOLD);
				}
			}
		}
		
		for(int j=0;j<contactCache.getNumContacts();j++) {
			const PfxCachedContactPoint &cp = contactCache.getContactPoint(j);

			contact.addContactPoint(
				cp.m_distance,
				cp.m_normal,
				cp.m_localPointA,
				cp.m_localPointB,
				cp.m_subData
				);
		}
	}

	SCE_PFX_POP_MARKER();
	
	(void) numRigidBodies;

	return SCE_PFX_OK;
}

} //namespace PhysicsEffects
} //namespace sce
