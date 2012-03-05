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

#ifndef _SCE_PFX_INTEGRATE_H_
#define _SCE_PFX_INTEGRATE_H_

#include "../base/pfx_common.h"
#include "../rigidbody/pfx_rigid_body.h"
#include "../rigidbody/pfx_rigid_state.h"

namespace sce {
namespace PhysicsEffects {

template <typename T>
static SCE_PFX_FORCE_INLINE T pfxRungeKutta(const T &deriv,PfxFloat dt)
{
	T	k0, k1, k2, k3;
	k0 = deriv * dt;
	k1 = (deriv + k0 * 0.5f) * dt;
	k2 = (deriv + k1 * 0.5f) * dt;
	k3 = (deriv + k2) * dt;
	return (k0 + k1*2.0f + k2*2.0f + k3) / 6.0f;
}

#define SCE_PFX_MOTION_MASK_INTEGRATE ((1<<kPfxMotionTypeFixed)|(1<<kPfxMotionTypeTrigger))

static SCE_PFX_FORCE_INLINE
void pfxIntegrate(
	PfxRigidState &state,
	const PfxRigidBody &body,
	PfxFloat timeStep
	)
{
	(void) body;
	if(((1<<state.getMotionMask())&SCE_PFX_MOTION_MASK_INTEGRATE) || state.isAsleep()) return;

	PfxVector3 position = state.getPosition();
	PfxQuat    orientation = state.getOrientation();
	PfxVector3 linearVelocity = state.getLinearVelocity();
	PfxVector3 angularVelocity = state.getAngularVelocity();

	PfxVector3 dx = linearVelocity;
	PfxQuat    dq = PfxQuat(angularVelocity,0) * orientation * 0.5f;

#ifdef SCE_PFX_ODE_EULER
	position += dx * timeStep;
	orientation += dq * timeStep;
	orientation = normalize(orientation);
#else
	position += pfxRungeKutta(dx,timeStep);
	orientation += pfxRungeKutta(dq,timeStep);
	orientation = normalize(orientation);
#endif

	state.setPosition(position);
	state.setOrientation(orientation);
}

#define SCE_PFX_MOTION_MASK_APPLYFORCE ((1<<kPfxMotionTypeFixed)|(1<<kPfxMotionTypeTrigger)|(1<<kPfxMotionTypeKeyframe))

static SCE_PFX_FORCE_INLINE
void pfxApplyExternalForce(
	PfxRigidState &state,
	const PfxRigidBody &body,
	const PfxVector3 &extForce,
	const PfxVector3 &extTorque,
	PfxFloat timeStep
	)
{
	if(((1<<state.getMotionType())&SCE_PFX_MOTION_MASK_APPLYFORCE) || state.isAsleep()) return;
	
	PfxVector3 angularVelocity = state.getAngularVelocity();
	PfxMatrix3 m(state.getOrientation());
	PfxMatrix3 worldInertia = m * body.getInertia() * transpose(m);
	PfxMatrix3 worldInertiaInv = m * body.getInertiaInv() * transpose(m);
	PfxVector3 dv = extForce * body.getMassInv();
	PfxVector3 dw = worldInertiaInv * (cross(worldInertia * angularVelocity, angularVelocity) + extTorque);
	
	PfxVector3 nv = state.getLinearVelocity();
	PfxVector3 nw = state.getAngularVelocity();

#ifdef SCE_PFX_ODE_EULER
	nv += dv * timeStep;
	nw += dw * timeStep;
#else
	nv += pfxRungeKutta(dv,timeStep);
	nw += pfxRungeKutta(dw,timeStep);
#endif

	nv *= state.getLinearDamping();
	nw *= state.getAngularDamping();

	if(length(nv) > state.getMaxLinearVelocity())
	{
		nv = normalize( nv ) * state.getMaxLinearVelocity();
	}
	if(length(nw) > state.getMaxAngularVelocity())
	{
		nw = normalize( nw ) * state.getMaxAngularVelocity();
	}

	state.setLinearVelocity(nv);
	state.setAngularVelocity(nw);
}
} //namespace PhysicsEffects
} //namespace sce

#endif /* _SCE_PFX_INTEGRATE_H_ */
