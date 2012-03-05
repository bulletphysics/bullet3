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

#include "../../../include/physics_effects/base_level/collision/pfx_collidable.h"

namespace sce {
namespace PhysicsEffects {

void PfxCollidable::addShape(const PfxShape &shape)
{
	if(m_numShapes<m_maxShapes) {
		PfxShape &newShape = getNewShape();
		newShape = shape;
	}
}

void PfxCollidable::finish()
{
	if(m_numShapes == 0) return;

	// compute AABB
	PfxVector3 halfMax(-SCE_PFX_FLT_MAX),halfMin(SCE_PFX_FLT_MAX);

	for(PfxUInt32 i=0;i<getNumShapes();i++) {
		const PfxShape &shape = getShape(i);
		PfxVector3 aabbMin,aabbMax;
		shape.getAabb(aabbMin,aabbMax);
		halfMax = maxPerElem(halfMax,aabbMax);
		halfMin = minPerElem(halfMin,aabbMin);
	}
	
	PfxVector3 allCenter = ( halfMin + halfMax ) * 0.5f;
	PfxVector3 allHalf = ( halfMax - halfMin ) * 0.5f;
	m_center[0] = allCenter[0];
	m_center[1] = allCenter[1];
	m_center[2] = allCenter[2];
	m_half[0] = allHalf[0];
	m_half[1] = allHalf[1];
	m_half[2] = allHalf[2];
}

} //namespace PhysicsEffects
} //namespace sce
