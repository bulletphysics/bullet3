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

#ifndef _SCE_PFX_SPHERE_H
#define _SCE_PFX_SPHERE_H

#include "../base/pfx_common.h"

namespace sce {
namespace PhysicsEffects {

struct PfxSphere {
	PfxFloat m_radius;

	PfxSphere() {}
	PfxSphere( PfxFloat radius );

	void  set( PfxFloat radius );
};

inline
PfxSphere::PfxSphere( PfxFloat radius )
{
	m_radius = radius;
}

inline
void PfxSphere::set( PfxFloat radius )
{
	m_radius = radius;
}

} // namespace PhysicsEffects
} // namespace sce

#endif // _SCE_PFX_SPHERE_H
