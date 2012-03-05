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

#ifndef _SCE_PFX_CAPSULE_H
#define _SCE_PFX_CAPSULE_H

#include "../base/pfx_common.h"

namespace sce {
namespace PhysicsEffects {

struct PfxCapsule
{
	PfxFloat m_halfLen;
	PfxFloat m_radius;

	PfxCapsule() {}
	PfxCapsule(PfxFloat halfLength, PfxFloat radius);

	void set(PfxFloat halfLength, PfxFloat radius);
};

inline
PfxCapsule::PfxCapsule(PfxFloat halfLength, PfxFloat radius)
{
	m_halfLen = halfLength;
	m_radius = radius;
}

inline
void PfxCapsule::set(PfxFloat halfLength, PfxFloat radius)
{
	m_halfLen = halfLength;
	m_radius = radius;
}

} // namespace PhysicsEffects
} // namespace sce

#endif // _SCE_PFX_CAPSULE_H
