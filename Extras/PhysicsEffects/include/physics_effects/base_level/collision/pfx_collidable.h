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

#ifndef _SCE_PFX_COLLIDABLE_H
#define _SCE_PFX_COLLIDABLE_H

#include "pfx_shape.h"

namespace sce {
namespace PhysicsEffects {

#define SCE_PFX_NUMPRIMS 64

///////////////////////////////////////////////////////////////////////////////
// Collidable Object

class SCE_PFX_ALIGNED(128) PfxCollidable
{
friend class PfxShapeIterator;

private:
	PfxShape *m_shapeBase;
	PfxUInt16 m_shapeIds[SCE_PFX_NUMPRIMS];
	PfxUInt8 m_numShapes;
	PfxUInt8 m_maxShapes;
	SCE_PFX_PADDING(1,2)
	PfxFloat m_center[3];	// AABB center (Local)
	PfxFloat m_half[3];		// AABB half (Local)
	PfxShape m_defShape;
	SCE_PFX_PADDING(2,32)

	inline PfxShape &getNewShape();

public:
	inline void reset();
	inline void reset(PfxShape *base,PfxUInt16 *ids,int n=1);

	void finish();

	void addShape(const PfxShape &shape);

	inline PfxUInt32 getNumShapes() const;
	const PfxShape& getDefShape() const {return m_defShape;}
	PfxShape& getDefShape() {return m_defShape;}

	inline PfxUInt16 getShapeId(int i) const;

	inline const PfxShape& getShape(int i) const;
	inline PfxShape& getShape(int i);

	inline PfxVector3 getHalf() const;
	inline PfxVector3 getCenter() const;
};

#include "pfx_collidable_implementation.h"

} // namespace PhysicsEffects
} // namespace sce

#endif // _SCE_PFX_COLLIDABLE_H

