//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2019 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  


#ifndef PX_PHYSICS_EXTENSIONS_INERTIATENSOR_H
#define PX_PHYSICS_EXTENSIONS_INERTIATENSOR_H

#include "foundation/PxMat33.h"
#include "foundation/PxMathUtils.h"
#include "CmPhysXCommon.h"
#include "PsMathUtils.h"

namespace physx
{
namespace Ext
{
	class InertiaTensorComputer
	{
		public:
									InertiaTensorComputer(bool initTozero = true);
									InertiaTensorComputer(const PxMat33& inertia, const PxVec3& com, PxReal mass);
									~InertiaTensorComputer();

		PX_INLINE	void			zero();																//sets to zero mass
		PX_INLINE	void			setDiagonal(PxReal mass, const PxVec3& diagonal);					//sets as a diagonal tensor
		PX_INLINE	void			rotate(const PxMat33& rot);											//rotates the mass
					void			translate(const PxVec3& t);											//translates the mass
		PX_INLINE	void			transform(const PxTransform& transform);							//transforms the mass
		PX_INLINE	void			scaleDensity(PxReal densityScale);									//scales by a density factor
		PX_INLINE	void			add(const InertiaTensorComputer& it);								//adds a mass
		PX_INLINE	void			center();															//recenters inertia around center of mass

					void			setBox(const PxVec3& halfWidths);									//sets as an axis aligned box
		PX_INLINE	void			setBox(const PxVec3& halfWidths, const PxTransform* pose);			//sets as an oriented box

					void			setSphere(PxReal radius);
		PX_INLINE	void			setSphere(PxReal radius, const PxTransform* pose);

					void			setCylinder(int dir, PxReal r, PxReal l);
		PX_INLINE	void			setCylinder(int dir, PxReal r, PxReal l, const PxTransform* pose);

					void			setCapsule(int dir, PxReal r, PxReal l);
		PX_INLINE	void			setCapsule(int dir, PxReal r, PxReal l, const PxTransform* pose);
					void			addCapsule(PxReal density, int dir, PxReal r, PxReal l, const PxTransform* pose = 0);

					void			setEllipsoid(PxReal rx, PxReal ry, PxReal rz);
		PX_INLINE	void			setEllipsoid(PxReal rx, PxReal ry, PxReal rz, const PxTransform* pose);

		PX_INLINE	PxVec3			getCenterOfMass()				const	{ return mG;	}
		PX_INLINE	PxReal			getMass()						const	{ return mMass;	}
		PX_INLINE	PxMat33			getInertia()					const	{ return mI;	}

		private:
					PxMat33				mI;
					PxVec3				mG;
					PxReal				mMass;
	};


	//--------------------------------------------------------------
	//
	// Helper routines
	//
	//--------------------------------------------------------------

	// Special version allowing 2D quads
	PX_INLINE PxReal volume(const PxVec3& extents)
	{
		PxReal v = 1.0f;
		if(extents.x != 0.0f)	v*=extents.x;
		if(extents.y != 0.0f)	v*=extents.y;
		if(extents.z != 0.0f)	v*=extents.z;
		return v;
	}

	// Sphere
	PX_INLINE PxReal computeSphereRatio(PxReal radius)						{ return (4.0f/3.0f) * PxPi * radius * radius * radius;	}
	PxReal computeSphereMass(PxReal radius, PxReal density)					{ return density * computeSphereRatio(radius);				}
	PxReal computeSphereDensity(PxReal radius, PxReal mass)					{ return mass / computeSphereRatio(radius);					}

	// Box
	PX_INLINE PxReal computeBoxRatio(const PxVec3& extents)					{ return volume(extents);									}
	PxReal computeBoxMass(const PxVec3& extents, PxReal density)			{ return density * computeBoxRatio(extents);				}
	PxReal computeBoxDensity(const PxVec3& extents, PxReal mass)			{ return mass / computeBoxRatio(extents);					}

	// Ellipsoid
	PX_INLINE PxReal computeEllipsoidRatio(const PxVec3& extents)			{ return (4.0f/3.0f) * PxPi * volume(extents);		}
	PxReal computeEllipsoidMass(const PxVec3& extents, PxReal density)		{ return density * computeEllipsoidRatio(extents);			}
	PxReal computeEllipsoidDensity(const PxVec3& extents, PxReal mass)		{ return mass / computeEllipsoidRatio(extents);				}

	// Cylinder
	PX_INLINE PxReal computeCylinderRatio(PxReal r, PxReal l)				{ return PxPi * r  * r * (2.0f*l);					}
	PxReal computeCylinderMass(PxReal r, PxReal l, PxReal density)			{ return density * computeCylinderRatio(r, l);				}
	PxReal computeCylinderDensity(PxReal r, PxReal l, PxReal mass)			{ return mass / computeCylinderRatio(r, l);					}

	// Capsule
	PX_INLINE PxReal computeCapsuleRatio(PxReal r, PxReal l)				{ return computeSphereRatio(r) + computeCylinderRatio(r, l);}
	PxReal computeCapsuleMass(PxReal r, PxReal l, PxReal density)			{ return density * computeCapsuleRatio(r, l);				}
	PxReal computeCapsuleDensity(PxReal r, PxReal l, PxReal mass)			{ return mass / computeCapsuleRatio(r, l);					}

	// Cone
	PX_INLINE PxReal computeConeRatio(PxReal r, PxReal l)					{ return PxPi * r * r * PxAbs(l)/3.0f;			}
	PxReal computeConeMass(PxReal r, PxReal l, PxReal density)				{ return density * computeConeRatio(r, l);					}
	PxReal computeConeDensity(PxReal r, PxReal l, PxReal mass)				{ return mass / computeConeRatio(r, l);						}

	void computeBoxInertiaTensor(PxVec3& inertia, PxReal mass, PxReal xlength, PxReal ylength, PxReal zlength);
	void computeSphereInertiaTensor(PxVec3& inertia, PxReal mass, PxReal radius, bool hollow);
	bool jacobiTransform(PxI32 n, PxF64 a[], PxF64 w[]);
	bool diagonalizeInertiaTensor(const PxMat33& denseInertia, PxVec3& diagonalInertia, PxMat33& rotation);

} // namespace Ext

void Ext::computeBoxInertiaTensor(PxVec3& inertia, PxReal mass, PxReal xlength, PxReal ylength, PxReal zlength)
{
	//to model a hollow block, one would have to multiply coeff by up to two.
	const PxReal coeff = mass/12;
	inertia.x = coeff * (ylength*ylength + zlength*zlength);
	inertia.y = coeff * (xlength*xlength + zlength*zlength);
	inertia.z = coeff * (xlength*xlength + ylength*ylength);

	PX_ASSERT(inertia.x != 0.0f);
	PX_ASSERT(inertia.y != 0.0f);
	PX_ASSERT(inertia.z != 0.0f);
	PX_ASSERT(inertia.isFinite());
}


void Ext::computeSphereInertiaTensor(PxVec3& inertia, PxReal mass, PxReal radius, bool hollow)
{
	inertia.x = mass * radius * radius;
	if (hollow) 
		inertia.x *= PxReal(2 / 3.0);
	else
		inertia.x *= PxReal(2 / 5.0);

	inertia.z = inertia.y = inertia.x;
	PX_ASSERT(inertia.isFinite());
}

//--------------------------------------------------------------
//
// InertiaTensorComputer implementation
//
//--------------------------------------------------------------

Ext::InertiaTensorComputer::InertiaTensorComputer(bool initTozero)
{
	if (initTozero)
		zero();
}


Ext::InertiaTensorComputer::InertiaTensorComputer(const PxMat33& inertia, const PxVec3& com, PxReal mass) :
	mI(inertia),
	mG(com),
	mMass(mass)
{
}


Ext::InertiaTensorComputer::~InertiaTensorComputer()
{
	//nothing
}


PX_INLINE void Ext::InertiaTensorComputer::zero()
{
	mMass = 0.0f;
	mI = PxMat33(PxZero);
	mG = PxVec3(0);
}


PX_INLINE void Ext::InertiaTensorComputer::setDiagonal(PxReal mass, const PxVec3& diag)
{
	mMass = mass;
	mI = PxMat33::createDiagonal(diag);
	mG = PxVec3(0);
	PX_ASSERT(mI.column0.isFinite() && mI.column1.isFinite() && mI.column2.isFinite());
	PX_ASSERT(PxIsFinite(mMass));
}


void Ext::InertiaTensorComputer::setBox(const PxVec3& halfWidths)
{
	// Setup inertia tensor for a cube with unit density
	const PxReal mass = 8.0f * computeBoxRatio(halfWidths);
	const PxReal s =(1.0f/3.0f) * mass;

	const PxReal x = halfWidths.x*halfWidths.x;
	const PxReal y = halfWidths.y*halfWidths.y;
	const PxReal z = halfWidths.z*halfWidths.z;

	setDiagonal(mass, PxVec3(y+z, z+x, x+y) * s);
}


PX_INLINE void Ext::InertiaTensorComputer::rotate(const PxMat33& rot)
{
	//well known inertia tensor rotation expression is: RIR' -- this could be optimized due to symmetry, see code to do that in Body::updateGlobalInverseInertia
	mI = rot * mI * rot.getTranspose();
	PX_ASSERT(mI.column0.isFinite() && mI.column1.isFinite() && mI.column2.isFinite());
	//com also needs to be rotated
	mG = rot * mG;
	PX_ASSERT(mG.isFinite());
}


void Ext::InertiaTensorComputer::translate(const PxVec3& t)
{
	if (!t.isZero())	//its common for this to be zero
	{
		PxMat33 t1, t2;

		t1.column0 = PxVec3(0, mG.z, -mG.y);
		t1.column1 = PxVec3(-mG.z, 0, mG.x);
		t1.column2 = PxVec3(mG.y, -mG.x, 0);

		PxVec3 sum = mG + t;
		if (sum.isZero())
		{
			mI += (t1 * t1)*mMass;
		}
		else
		{			
			t2.column0 = PxVec3(0, sum.z, -sum.y);
			t2.column1 = PxVec3(-sum.z, 0, sum.x);
			t2.column2 = PxVec3(sum.y, -sum.x, 0);
			mI += (t1 * t1 - t2 * t2)*mMass;
		}

		//move center of mass
		mG += t;

		PX_ASSERT(mI.column0.isFinite() && mI.column1.isFinite() && mI.column2.isFinite());
		PX_ASSERT(mG.isFinite());
	}
}


PX_INLINE void Ext::InertiaTensorComputer::transform(const PxTransform& transform)
{
	rotate(PxMat33(transform.q));
	translate(transform.p);
}


PX_INLINE void Ext::InertiaTensorComputer::setBox(const PxVec3& halfWidths, const PxTransform* pose)
{
	setBox(halfWidths);
	if (pose)
		transform(*pose);

}


PX_INLINE void Ext::InertiaTensorComputer::scaleDensity(PxReal densityScale)
{
	mI *= densityScale;
	mMass *= densityScale;
	PX_ASSERT(mI.column0.isFinite() && mI.column1.isFinite() && mI.column2.isFinite());
	PX_ASSERT(PxIsFinite(mMass));
}


PX_INLINE void Ext::InertiaTensorComputer::add(const InertiaTensorComputer& it)
{
	const PxReal TotalMass = mMass + it.mMass;
	mG = (mG * mMass + it.mG * it.mMass) / TotalMass;

	mMass = TotalMass;
	mI += it.mI;
	PX_ASSERT(mI.column0.isFinite() && mI.column1.isFinite() && mI.column2.isFinite());
	PX_ASSERT(mG.isFinite());
	PX_ASSERT(PxIsFinite(mMass));
}


PX_INLINE void Ext::InertiaTensorComputer::center()
{
	PxVec3 center = -mG;
	translate(center);
}


void Ext::InertiaTensorComputer::setSphere(PxReal radius)
{
	// Compute mass of the sphere
	const PxReal m = computeSphereRatio(radius);
	// Compute moment of inertia
	const PxReal s = m * radius * radius * (2.0f/5.0f);
	setDiagonal(m,PxVec3(s,s,s));
}


PX_INLINE void Ext::InertiaTensorComputer::setSphere(PxReal radius, const PxTransform* pose)
{
	setSphere(radius);
	if (pose)
		transform(*pose);
}


void Ext::InertiaTensorComputer::setCylinder(int dir, PxReal r, PxReal l)
{
	// Compute mass of cylinder
	const PxReal m = computeCylinderRatio(r, l);

	const PxReal i1 = r*r*m/2.0f;
	const PxReal i2 = (3.0f*r*r+4.0f*l*l)*m/12.0f;

	switch(dir)
	{
	case 0:		setDiagonal(m,PxVec3(i1,i2,i2));	break;
	case 1:		setDiagonal(m,PxVec3(i2,i1,i2));	break;
	default:	setDiagonal(m,PxVec3(i2,i2,i1));	break;
	}
}


PX_INLINE void Ext::InertiaTensorComputer::setCylinder(int dir, PxReal r, PxReal l, const PxTransform* pose)
{
	setCylinder(dir, r, l);
	if (pose)
		transform(*pose);
}


void Ext::InertiaTensorComputer::setCapsule(int dir, PxReal r, PxReal l)
{
	// Compute mass of capsule
	const PxReal m = computeCapsuleRatio(r, l);

	const PxReal t  = PxPi * r * r;
	const PxReal i1 = t * ((r*r*r * 8.0f/15.0f) + (l*r*r));
	const PxReal i2 = t * ((r*r*r * 8.0f/15.0f) + (l*r*r * 3.0f/2.0f) + (l*l*r * 4.0f/3.0f) + (l*l*l * 2.0f/3.0f));

	switch(dir)
	{
	case 0:		setDiagonal(m,PxVec3(i1,i2,i2));	break;
	case 1:		setDiagonal(m,PxVec3(i2,i1,i2));	break;
	default:	setDiagonal(m,PxVec3(i2,i2,i1));	break;
	}
}


PX_INLINE void Ext::InertiaTensorComputer::setCapsule(int dir, PxReal r, PxReal l, const PxTransform* pose)
{
	setCapsule(dir, r, l);
	if (pose)
		transform(*pose);
}


void Ext::InertiaTensorComputer::setEllipsoid(PxReal rx, PxReal ry, PxReal rz)
{
	// Compute mass of ellipsoid
	const PxReal m = computeEllipsoidRatio(PxVec3(rx, ry, rz));

	// Compute moment of inertia
	const PxReal s = m * (2.0f/5.0f);

	// Setup inertia tensor for an ellipsoid centered at the origin
	setDiagonal(m,PxVec3(ry*rz,rz*rx,rx*ry)*s);
}


PX_INLINE void Ext::InertiaTensorComputer::setEllipsoid(PxReal rx, PxReal ry, PxReal rz, const PxTransform* pose)
{
	setEllipsoid(rx,ry,rz);
	if (pose)
		transform(*pose);
}

}

#endif
