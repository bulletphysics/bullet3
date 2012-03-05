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

#ifndef _SCE_PFX_RIGID_BODY_H
#define _SCE_PFX_RIGID_BODY_H

#include "../base/pfx_common.h"

namespace sce {
namespace PhysicsEffects {

class SCE_PFX_ALIGNED(128) PfxRigidBody
{
private:
	// Rigid Body constants
	PfxMatrix3     m_inertia;     // Inertia matrix in body's coords
	PfxMatrix3     m_inertiaInv;  // Inertia matrix inverse in body's coords
	PfxFloat       m_mass;        // Rigid Body mass
	PfxFloat       m_massInv;     // Inverse of mass
	PfxFloat       m_restitution; // Coefficient of restitution
	PfxFloat       m_friction;    // Coefficient of friction
	SCE_PFX_PADDING(1,16)
	
public:
	PfxFloat       getMassInv() const {return m_massInv;}
	void           setMassInv(PfxFloat massInv) {m_massInv=massInv;}

	const PfxMatrix3& getInertiaInv() const {return m_inertiaInv;}
	void              setInertiaInv(const PfxMatrix3 SCE_VECTORMATH_AOS_MATRIX_ARG inertiaInv) {m_inertiaInv = inertiaInv;}

public:
	inline void reset();

	PfxFloat       getMass() const {return m_mass;};
	void           setMass(PfxFloat mass) {m_mass=mass;m_massInv=mass>0.0f?1.0f/mass:0.0f;}

	const PfxMatrix3& getInertia() const {return m_inertia;}
	void              setInertia(const PfxMatrix3 SCE_VECTORMATH_AOS_MATRIX_ARG inertia) {m_inertia = inertia;m_inertiaInv = inverse(inertia);}

	PfxFloat       getRestitution() const {return m_restitution;}
	void           setRestitution(PfxFloat restitution) {m_restitution = restitution;}

	PfxFloat       getFriction() const {return m_friction;}
	void           setFriction(PfxFloat friction) {m_friction = friction;}
};

inline
void PfxRigidBody::reset()
{
	m_mass = 0.0f;
	m_restitution = 0.2f;
	m_friction = 0.6f;
}

} // namespace PhysicsEffects
} // namespace sce

#endif // _SCE_PFX_RIGID_BODY_H
