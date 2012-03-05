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

#ifndef _SCE_PFX_SHAPE_IMPLEMENTATION_H
#define _SCE_PFX_SHAPE_IMPLEMENTATION_H

inline
void PfxShape::reset()
{
	m_type = kPfxShapeSphere;
	m_offsetPosition[0] = m_offsetPosition[1] = m_offsetPosition[2]= 0.0f;
	m_offsetOrientation[0] = m_offsetOrientation[1] = m_offsetOrientation[2]= 0.0f;
	m_offsetOrientation[3] = 1.0f;
	m_contactFilterSelf = m_contactFilterTarget = 0xffffffff;
}

inline
void PfxShape::setBox(PfxBox SCE_VECTORMATH_AOS_VECTOR_ARG box)
{
	m_vecDataF[0] = box.m_half[0];
	m_vecDataF[1] = box.m_half[1];
	m_vecDataF[2] = box.m_half[2];
	m_type = kPfxShapeBox;
}

inline
void PfxShape::setCapsule(PfxCapsule capsule)
{
	m_vecDataF[0] = capsule.m_halfLen;
	m_vecDataF[1] = capsule.m_radius;
	m_vecDataF[2] = 0.0f;
	m_type = kPfxShapeCapsule;
}

inline
void PfxShape::setCylinder(PfxCylinder cylinder)
{
	m_vecDataF[0] = cylinder.m_halfLen;
	m_vecDataF[1] = cylinder.m_radius;
	m_vecDataF[2] = 0.0f;
	m_type = kPfxShapeCylinder;
}

inline
void PfxShape::setSphere(PfxSphere sphere)
{
	m_vecDataF[0] = sphere.m_radius;
	m_vecDataF[1] = 0.0f;
	m_vecDataF[2] = 0.0f;
	m_type = kPfxShapeSphere;
}

inline
void PfxShape::setConvexMesh(const PfxConvexMesh *convexMesh)
{
	m_vecDataPtr[0] = (void*)convexMesh;
	m_vecDataPtr[1] = NULL;
	m_type = kPfxShapeConvexMesh;
}

inline
void PfxShape::setLargeTriMesh(const PfxLargeTriMesh *largeMesh)
{
	m_vecDataPtr[0] = (void*)largeMesh;
	m_vecDataPtr[1] = NULL;
	m_type = kPfxShapeLargeTriMesh;
}

inline
void PfxShape::setOffsetTransform(const PfxTransform3 & xfrm)
{
	setOffsetOrientation(PfxQuat(xfrm.getUpper3x3()));
	setOffsetPosition(xfrm.getTranslation());
}

inline
PfxUInt8 PfxShape::getType() const
{
	return m_type;
}

inline
PfxBox PfxShape::getBox() const
{
	SCE_PFX_ALWAYS_ASSERT(m_type==kPfxShapeBox);
	return PfxBox(m_vecDataF[0],m_vecDataF[1],m_vecDataF[2]);
}

inline
PfxCapsule PfxShape::getCapsule() const
{
	SCE_PFX_ALWAYS_ASSERT(m_type==kPfxShapeCapsule);
	return PfxCapsule(m_vecDataF[0], m_vecDataF[1]);
}

inline
PfxCylinder PfxShape::getCylinder() const
{
	SCE_PFX_ALWAYS_ASSERT(m_type==kPfxShapeCylinder);
	return PfxCylinder(m_vecDataF[0], m_vecDataF[1]);
}

inline
PfxSphere PfxShape::getSphere() const
{
	SCE_PFX_ALWAYS_ASSERT(m_type==kPfxShapeSphere);
	return PfxSphere(m_vecDataF[0]);
}

inline
const PfxConvexMesh *PfxShape::getConvexMesh() const
{
	SCE_PFX_ALWAYS_ASSERT(m_type==kPfxShapeConvexMesh);
	SCE_PFX_ALWAYS_ASSERT(m_vecDataPtr[0]!=NULL);
	return (PfxConvexMesh*)m_vecDataPtr[0];
}

inline
const PfxLargeTriMesh *PfxShape::getLargeTriMesh() const
{
	SCE_PFX_ALWAYS_ASSERT(m_type==kPfxShapeLargeTriMesh);
	SCE_PFX_ALWAYS_ASSERT(m_vecDataPtr[0]!=NULL);
	return (PfxLargeTriMesh*)m_vecDataPtr[0];
}

inline
PfxTransform3 PfxShape::getOffsetTransform() const
{
	return PfxTransform3(getOffsetOrientation(),getOffsetPosition());
}

#endif // _SCE_PFX_SHAPE_IMPLEMENTATION_H
