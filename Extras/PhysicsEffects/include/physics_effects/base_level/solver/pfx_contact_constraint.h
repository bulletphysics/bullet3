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

#ifndef _SCE_PFX_CONTACT_CONSTRAINT_H
#define _SCE_PFX_CONTACT_CONSTRAINT_H

#include "../rigidbody/pfx_rigid_state.h"
#include "pfx_constraint_row.h"
#include "pfx_solver_body.h"

namespace sce {
namespace PhysicsEffects {

void pfxSetupContactConstraint(
	PfxConstraintRow &constraintResponse,
	PfxConstraintRow &constraintFriction1,
	PfxConstraintRow &constraintFriction2,
	PfxFloat penetrationDepth,
	PfxFloat restitution,
	PfxFloat friction,
	const PfxVector3 &contactNormal,
	const PfxVector3 &contactPointA,
	const PfxVector3 &contactPointB,
	const PfxRigidState &stateA,
	const PfxRigidState &stateB,
	PfxSolverBody &solverBodyA,
	PfxSolverBody &solverBodyB,
	PfxFloat separateBias,
	PfxFloat timeStep
	);

void pfxSolveContactConstraint(
	PfxConstraintRow &constraintResponse,
	PfxConstraintRow &constraintFriction1,
	PfxConstraintRow &constraintFriction2,
	const PfxVector3 &contactPointA,
	const PfxVector3 &contactPointB,
	PfxSolverBody &solverBodyA,
	PfxSolverBody &solverBodyB,
	PfxFloat friction
	);


} //namespace PhysicsEffects
} //namespace sce
#endif // _SCE_PFX_CONTACT_CONSTRAINT_H
