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

#ifndef _SCE_PFX_RAY_H
#define _SCE_PFX_RAY_H

#include "../base/pfx_common.h"
#include "pfx_sub_data.h"

namespace sce {
namespace PhysicsEffects {

#define SCE_PFX_RAY_FACET_MODE_FRONT_ONLY     0
#define SCE_PFX_RAY_FACET_MODE_BACK_ONLY      1
#define SCE_PFX_RAY_FACET_MODE_FRONT_AND_BACK 2

struct SCE_PFX_ALIGNED(16) PfxRayInput
{
	PfxVector3 m_startPosition;
	PfxVector3 m_direction;
	PfxUInt32 m_contactFilterSelf;
	PfxUInt32 m_contactFilterTarget;
	PfxUInt8  m_facetMode;
	SCE_PFX_PADDING(1,7)

	void reset()
	{
		m_contactFilterSelf = m_contactFilterTarget = 0xffffffff;
		m_facetMode = SCE_PFX_RAY_FACET_MODE_FRONT_ONLY;
	}
};

struct SCE_PFX_ALIGNED(16) PfxRayOutput
{
	PfxVector3 m_contactPoint;
	PfxVector3 m_contactNormal;
	PfxFloat   m_variable;
	PfxUInt16  m_objectId;
	PfxUInt8   m_shapeId;
	PfxBool    m_contactFlag : 1;
	PfxSubData m_subData;
};
} //namespace PhysicsEffects
} //namespace sce

#endif // _SCE_PFX_RAY_H
