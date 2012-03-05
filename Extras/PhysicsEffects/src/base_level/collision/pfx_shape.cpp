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

#include "../../../include/physics_effects/base_level/collision/pfx_shape.h"

namespace sce {
namespace PhysicsEffects {
void pfxGetShapeAabbDummy(const PfxShape &shape,PfxVector3 &aabbMin,PfxVector3 &aabbMax)
{
	(void)shape,(void)aabbMin,(void)aabbMax;
}

void pfxGetShapeAabbSphere(const PfxShape &shape,PfxVector3 &aabbMin,PfxVector3 &aabbMax)
{
	aabbMin = shape.getOffsetPosition() - PfxVector3(shape.getSphere().m_radius);
	aabbMax = shape.getOffsetPosition() + PfxVector3(shape.getSphere().m_radius);
}

void pfxGetShapeAabbBox(const PfxShape &shape,PfxVector3 &aabbMin,PfxVector3 &aabbMax)
{
	PfxVector3 boxSize = absPerElem(PfxMatrix3(shape.getOffsetOrientation())) * shape.getBox().m_half;
	aabbMin = shape.getOffsetPosition() - boxSize;
	aabbMax = shape.getOffsetPosition() + boxSize;
}

void pfxGetShapeAabbCapsule(const PfxShape &shape,PfxVector3 &aabbMin,PfxVector3 &aabbMax)
{
	PfxVector3 dir = rotate(shape.getOffsetOrientation(),PfxVector3(1.0f,0.0f,0.0f));
	PfxVector3 capSize = absPerElem(dir) * shape.getCapsule().m_halfLen + 
		PfxVector3(shape.getCapsule().m_radius);
	aabbMin = shape.getOffsetPosition() - capSize;
	aabbMax = shape.getOffsetPosition() + capSize;
}

void pfxGetShapeAabbCylinder(const PfxShape &shape,PfxVector3 &aabbMin,PfxVector3 &aabbMax)
{
	PfxVector3 capSize = absPerElem(PfxMatrix3(shape.getOffsetOrientation())) * 
		PfxVector3(shape.getCylinder().m_halfLen,shape.getCylinder().m_radius,shape.getCylinder().m_radius);
	aabbMin = shape.getOffsetPosition() - capSize;
	aabbMax = shape.getOffsetPosition() + capSize;
}

void pfxGetShapeAabbConvexMesh(const PfxShape &shape,PfxVector3 &aabbMin,PfxVector3 &aabbMax)
{
	const PfxConvexMesh *convex = shape.getConvexMesh();
	PfxVector3 half = absPerElem(PfxMatrix3(shape.getOffsetOrientation())) * convex->m_half;
	aabbMin = shape.getOffsetPosition() - half;
	aabbMax = shape.getOffsetPosition() + half;
}

void pfxGetShapeAabbLargeTriMesh(const PfxShape &shape,PfxVector3 &aabbMin,PfxVector3 &aabbMax)
{
	const PfxLargeTriMesh *largemesh = shape.getLargeTriMesh();
	PfxVector3 half = absPerElem(PfxMatrix3(shape.getOffsetOrientation())) *  largemesh->m_half;
	aabbMin = shape.getOffsetPosition() - half;
	aabbMax = shape.getOffsetPosition() + half;
}

typedef void (*PfxFuncGetShapeAabb)(const PfxShape &shape,PfxVector3 &aabbMin,PfxVector3 &aabbMax);

PfxFuncGetShapeAabb pfxFuncGetShapeAabb[kPfxShapeCount] = {
	pfxGetShapeAabbSphere,
	pfxGetShapeAabbBox,
	pfxGetShapeAabbCapsule,
	pfxGetShapeAabbCylinder,
	pfxGetShapeAabbConvexMesh,
	pfxGetShapeAabbLargeTriMesh,
	pfxGetShapeAabbDummy,
	pfxGetShapeAabbDummy,
	pfxGetShapeAabbDummy,
	pfxGetShapeAabbDummy,
	pfxGetShapeAabbDummy,
	pfxGetShapeAabbDummy,
};

void PfxShape::getAabb(PfxVector3 &aabbMin,PfxVector3 &aabbMax) const
{
	return pfxFuncGetShapeAabb[m_type](*this,aabbMin,aabbMax);
}

} //namespace PhysicsEffects
} //namespace sce
