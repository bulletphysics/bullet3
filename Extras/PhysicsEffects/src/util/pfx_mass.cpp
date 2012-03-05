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

#include "../../include/physics_effects/util/pfx_mass.h"

namespace sce {
namespace PhysicsEffects {

///////////////////////////////////////////////////////////////////////////////
// Box

PfxFloat pfxCalcMassBox(PfxFloat density,const PfxVector3 &halfExtent)
{
	return density * halfExtent[0] * halfExtent[1] * halfExtent[2] * 8;
}

PfxMatrix3 pfxCalcInertiaBox(const PfxVector3 &halfExtent,PfxFloat mass)
{
	PfxVector3 sqrSz = halfExtent * 2.0f;
	sqrSz = mulPerElem(sqrSz,sqrSz);
	PfxMatrix3 inertia = PfxMatrix3::identity();
	inertia[0][0] = (mass*(sqrSz[1]+sqrSz[2]))/12.0f;
	inertia[1][1] = (mass*(sqrSz[0]+sqrSz[2]))/12.0f;
	inertia[2][2] = (mass*(sqrSz[0]+sqrSz[1]))/12.0f;
	return inertia;
}

///////////////////////////////////////////////////////////////////////////////
// Sphere

PfxFloat pfxCalcMassSphere(PfxFloat density,PfxFloat radius)
{
	return (4.0f/3.0f) * SCE_PFX_PI * radius * radius * radius * density;
}

PfxMatrix3 pfxCalcInertiaSphere(PfxFloat radius,PfxFloat mass)
{
	PfxMatrix3 inertia = PfxMatrix3::identity();
	inertia[0][0] = inertia[1][1] = inertia[2][2] = 0.4f * mass * radius * radius;
	return inertia;
}

///////////////////////////////////////////////////////////////////////////////
// Cylinder

PfxFloat pfxCalcMassCylinder(PfxFloat density,PfxFloat halfLength,PfxFloat radius)
{
	return SCE_PFX_PI * radius * radius * 2.0f * halfLength * density;
}

static inline
PfxMatrix3 pfxCalcInertiaCylinder(PfxFloat halfLength,PfxFloat radius,PfxFloat mass,int axis)
{
	PfxMatrix3 inertia = PfxMatrix3::identity();
	inertia[0][0] = inertia[1][1] = inertia[2][2] = 0.25f * mass * radius * radius + 0.33f * mass * halfLength * halfLength; 
	inertia[axis][axis] = 0.5f * mass * radius * radius;
	return inertia;
}

PfxMatrix3 pfxCalcInertiaCylinderX(PfxFloat halfLength,PfxFloat radius,PfxFloat mass)
{
	return pfxCalcInertiaCylinder(radius,halfLength,mass,0);
}

PfxMatrix3 pfxCalcInertiaCylinderY(PfxFloat halfLength,PfxFloat radius,PfxFloat mass)
{
	return pfxCalcInertiaCylinder(radius,halfLength,mass,1);
}

PfxMatrix3 pfxCalcInertiaCylinderZ(PfxFloat halfLength,PfxFloat radius,PfxFloat mass)
{
	return pfxCalcInertiaCylinder(radius,halfLength,mass,2);
}

///////////////////////////////////////////////////////////////////////////////

PfxMatrix3 pfxMassTranslate(PfxFloat mass,const PfxMatrix3 &inertia,const PfxVector3 &translation)
{
	PfxMatrix3 m = crossMatrix(translation);
	return inertia + mass * (-m*m);
}

PfxMatrix3 pfxMassRotate(const PfxMatrix3 &inertia,const PfxMatrix3 &rotate)
{
	return rotate * inertia * transpose(rotate);
}

} //namespace PhysicsEffects
} //namespace sce
