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

#ifndef _SCE_PFX_UPDATE_RIGID_STATES_H_
#define _SCE_PFX_UPDATE_RIGID_STATES_H_

#include "../../base_level/rigidbody/pfx_rigid_body.h"
#include "../../base_level/rigidbody/pfx_rigid_state.h"

#include "../task/pfx_task_manager.h"

namespace sce {
namespace PhysicsEffects {

///////////////////////////////////////////////////////////////////////////////
// Update Rigid Body States

struct PfxUpdateRigidStatesParam {
	PfxRigidState *states;
	PfxRigidBody *bodies;
	PfxUInt32 numRigidBodies;
	PfxFloat timeStep;
};

PfxInt32 pfxUpdateRigidStates(PfxUpdateRigidStatesParam &param);

PfxInt32 pfxUpdateRigidStates(PfxUpdateRigidStatesParam &param,PfxTaskManager *taskManager);

} //namespace PhysicsEffects
} //namespace sce

#endif /* _SCE_PFX_UPDATE_RIGID_STATES_H_ */
