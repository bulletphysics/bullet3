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

#ifndef _SCE_PFX_BOX_H
#define _SCE_PFX_BOX_H

#include "../base/pfx_common.h"

namespace sce{
namespace PhysicsEffects{
struct PfxBox {
	PfxVector3 m_half;

	inline PfxBox() {}
	inline PfxBox(const PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG half);
	inline PfxBox(PfxFloat hx, PfxFloat hy, PfxFloat hz);

	inline void set(const PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG half);
	inline void set(PfxFloat hx, PfxFloat hy, PfxFloat hz);
};

inline
PfxBox::PfxBox(const PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG half)
{
	set(half);
}

inline
PfxBox::PfxBox(PfxFloat hx, PfxFloat hy, PfxFloat hz)
{
	set(hx, hy, hz);
}

inline
void PfxBox::set(const PfxVector3 SCE_VECTORMATH_AOS_VECTOR_ARG half)
{
	m_half = half;
}

inline
void PfxBox::set(PfxFloat hx, PfxFloat hy, PfxFloat hz)
{
	m_half = PfxVector3(hx, hy, hz);
}
} // namespace PhysicsEffects
} // namespace sce

#endif // _SCE_PFX_BOX_H
