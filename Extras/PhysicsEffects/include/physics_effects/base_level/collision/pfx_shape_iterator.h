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

#ifndef _SCE_PFX_SHAPE_ITERATOR_H
#define _SCE_PFX_SHAPE_ITERATOR_H

#include "pfx_collidable.h"


namespace sce {
namespace PhysicsEffects {

class PfxShapeIterator
{
private:
PfxUInt32 m_numShapes;
PfxShape *m_shapeBase;
const PfxUInt16 *m_shapeIds;
const PfxShape *m_curShape;
PfxUInt32 m_index;

public:
PfxShapeIterator(const PfxCollidable &coll) : m_shapeIds(coll.m_shapeIds)
{
	m_numShapes = coll.m_numShapes;
	m_shapeBase = coll.m_shapeBase;
	m_index = 0;
	m_curShape = &coll.m_defShape;
}

~PfxShapeIterator() {}

inline PfxShapeIterator& operator++()
{
	m_curShape = &m_shapeBase[m_shapeIds[m_index++]];
	return *this;
}

const PfxShape& operator*() const {return *m_curShape;}
};

} //namespace PhysicsEffects
} //namespace sce

#endif // _SCE_PFX_SHAPE_ITERATOR_H
