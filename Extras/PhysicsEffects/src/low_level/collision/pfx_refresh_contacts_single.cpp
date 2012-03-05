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
#include "../../../include/physics_effects/base_level/sort/pfx_sort.h"
#include "../../../include/physics_effects/low_level/collision/pfx_refresh_contacts.h"

namespace sce {
namespace PhysicsEffects {

int pfxCheckParamOfRefreshContacts(PfxRefreshContactsParam &param)
{
	if(!param.contactPairs || !param.offsetContactManifolds || !param.offsetRigidStates ) return SCE_PFX_ERR_INVALID_VALUE;
	if(!SCE_PFX_PTR_IS_ALIGNED16(param.contactPairs) || !SCE_PFX_PTR_IS_ALIGNED16(param.offsetContactManifolds) || 
		!SCE_PFX_PTR_IS_ALIGNED16(param.offsetRigidStates)) return SCE_PFX_ERR_INVALID_ALIGN;
	return SCE_PFX_OK;
}

///////////////////////////////////////////////////////////////////////////////
// SINGLE THREAD

PfxInt32 pfxRefreshContacts(PfxRefreshContactsParam &param)
{
	PfxInt32 ret = pfxCheckParamOfRefreshContacts(param);
	if(ret != SCE_PFX_OK) return ret;
	
	SCE_PFX_PUSH_MARKER("pfxRefreshContacts");

	PfxConstraintPair *contactPairs = param.contactPairs;
	PfxUInt32 numContactPairs = param.numContactPairs;
	PfxContactManifold *offsetContactManifolds = param.offsetContactManifolds;
	PfxRigidState *offsetRigidStates = param.offsetRigidStates;
	PfxUInt32 numRigidBodies = param.numRigidBodies;
	
	for(PfxUInt32 i=0;i<numContactPairs;i++) {
		PfxBroadphasePair &pair = contactPairs[i];
		
		PfxUInt32 iContact = pfxGetContactId(pair);
		PfxUInt32 iA = pfxGetObjectIdA(pair);
		PfxUInt32 iB = pfxGetObjectIdB(pair);

		PfxContactManifold &contact = offsetContactManifolds[iContact];

		SCE_PFX_ALWAYS_ASSERT(iA==contact.getRigidBodyIdA());
		SCE_PFX_ALWAYS_ASSERT(iB==contact.getRigidBodyIdB());

		PfxRigidState &instA = offsetRigidStates[iA];
		PfxRigidState &instB = offsetRigidStates[iB];
		
		contact.refresh(
			instA.getPosition(),instA.getOrientation(),
			instB.getPosition(),instB.getOrientation() );
	}

	SCE_PFX_POP_MARKER();

	(void) numRigidBodies;

	return SCE_PFX_OK;
}

} //namespace PhysicsEffects
} //namespace sce
