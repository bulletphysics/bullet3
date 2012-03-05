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

#ifndef _SCE_PFX_COLLISION_DETECTION_H_
#define _SCE_PFX_COLLISION_DETECTION_H_

#include "../../base_level/rigidbody/pfx_rigid_state.h"
#include "../../base_level/collision/pfx_collidable.h"
#include "../../base_level/collision/pfx_contact_manifold.h"
#include "../../base_level/solver/pfx_constraint_pair.h"
#include "../task/pfx_task_manager.h"

namespace sce {
namespace PhysicsEffects {

///////////////////////////////////////////////////////////////////////////////
// Detect Collision

struct PfxDetectCollisionParam {
	PfxConstraintPair *contactPairs;
	PfxUInt32 numContactPairs;
	PfxContactManifold *offsetContactManifolds;
	PfxRigidState *offsetRigidStates;
	PfxCollidable *offsetCollidables;
	PfxUInt32 numRigidBodies;
};

PfxInt32 pfxDetectCollision(PfxDetectCollisionParam &param);

PfxInt32 pfxDetectCollision(PfxDetectCollisionParam &param,PfxTaskManager *taskManager);

} //namespace PhysicsEffects
} //namespace sce

#endif /* _SCE_PFX_COLLISION_DETECTION_H_ */
