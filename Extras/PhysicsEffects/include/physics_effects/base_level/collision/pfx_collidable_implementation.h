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

#ifndef _SCE_PFX_COLLIDABLE_IMPLEMENTATION_H
#define _SCE_PFX_COLLIDABLE_IMPLEMENTATION_H

inline
void PfxCollidable::reset()
{
	m_shapeBase = NULL;
	m_numShapes = 0;
	m_maxShapes = 1;
	m_center[0] = 0.0f;
	m_center[1] = 0.0f;
	m_center[2] = 0.0f;
	m_half[0] = 0.0f;
	m_half[1] = 0.0f;
	m_half[2] = 0.0f;
}

inline
void PfxCollidable::reset(PfxShape *base,PfxUInt16 *ids,int n)
{
	m_shapeBase = base;
	m_numShapes = 0;
	m_maxShapes = n;
	for(int i=0;i<n;i++) {
		m_shapeIds[i] = ids[i];
	}
	m_center[0] = 0.0f;
	m_center[1] = 0.0f;
	m_center[2] = 0.0f;
	m_half[0] = 0.0f;
	m_half[1] = 0.0f;
	m_half[2] = 0.0f;
}

inline
PfxUInt32 PfxCollidable::getNumShapes() const
{
	return m_numShapes;
}

inline
PfxUInt16 PfxCollidable::getShapeId(int i) const
{
	SCE_PFX_ASSERT(i>0);
	return m_shapeIds[i-1];
}

inline
const PfxShape& PfxCollidable::getShape(int i) const
{
	SCE_PFX_ASSERT(i<m_numShapes);
	if(i>0) {
		return m_shapeBase[m_shapeIds[i-1]];
	}
	else {
		return m_defShape;
	}
}

inline
PfxShape& PfxCollidable::getShape(int i)
{
	SCE_PFX_ASSERT(i<m_numShapes);
	if(i>0) {
		return m_shapeBase[m_shapeIds[i-1]];
	}
	else {
		return m_defShape;
	}
}

inline
PfxShape &PfxCollidable::getNewShape()
{
	SCE_PFX_ASSERT(m_numShapes<=m_maxShapes);
	if(m_numShapes == 0) {
		m_numShapes++;
		return m_defShape;
	}
	else {
		m_numShapes++;
		return m_shapeBase[m_shapeIds[m_numShapes-2]];
	}
}

inline
PfxVector3 PfxCollidable::getHalf() const
{
	return pfxReadVector3(m_half);
}

inline
PfxVector3 PfxCollidable::getCenter() const
{
	return pfxReadVector3(m_center);
}

#endif // _SCE_PFX_COLLIDABLE_IMPLEMENTATION_H
