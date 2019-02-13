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
// Copyright (c) 2008-2018 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "GuSweepSphereSphere.h"
#include "PsUtilities.h"

using namespace physx;
using namespace Gu;

// Adapted from Gamasutra (Gomez article)
// Return true if r1 and r2 are real
static PX_FORCE_INLINE bool quadraticFormula(const PxReal a, const PxReal b, const PxReal c, PxReal& r1, PxReal& r2)
{
	const PxReal q = b*b - 4*a*c; 
	if(q>=0.0f)
	{
		PX_ASSERT(a!=0.0f);
		const PxReal sq = PxSqrt(q);
		const PxReal d = 1.0f / (2.0f*a);
		r1 = (-b + sq) * d;
		r2 = (-b - sq) * d;
		return true;//real roots
	}
	else
	{
		return false;//complex roots
	}
}

static bool sphereSphereSweep(	const PxReal ra, //radius of sphere A
								const PxVec3& A0, //previous position of sphere A
								const PxVec3& A1, //current position of sphere A
								const PxReal rb, //radius of sphere B
								const PxVec3& B0, //previous position of sphere B
								const PxVec3& B1, //current position of sphere B
								PxReal& u0, //normalized time of first collision
								PxReal& u1 //normalized time of second collision
								)
{
	const PxVec3 va = A1 - A0;
	const PxVec3 vb = B1 - B0;
	const PxVec3 AB = B0 - A0;
	const PxVec3 vab = vb - va;	// relative velocity (in normalized time)
	const PxReal rab = ra + rb;

	const PxReal a = vab.dot(vab);		//u*u coefficient
	const PxReal b = 2.0f*(vab.dot(AB));	//u coefficient

	const PxReal c = (AB.dot(AB)) - rab*rab;	//constant term

	//check if they're currently overlapping
	if(c<=0.0f || a==0.0f)
	{
		u0 = 0.0f;
		u1 = 0.0f;
		return true;
	}

	//check if they hit each other during the frame
	if(quadraticFormula(a, b, c, u0, u1))
	{
		if(u0>u1)
			Ps::swap(u0, u1);

		// u0<u1
//		if(u0<0.0f || u1>1.0f)	return false;
		if(u1<0.0f || u0>1.0f)	return false;

		return true;
	}
	return false;
}

bool Gu::sweepSphereSphere(const PxVec3& center0, PxReal radius0, const PxVec3& center1, PxReal radius1, const PxVec3& motion, PxReal& d, PxVec3& nrm)
{
	const PxVec3 movedCenter = center1 + motion;

	PxReal tmp;
	if(!sphereSphereSweep(radius0, center0, center0, radius1, center1, movedCenter, d, tmp))
		return false;

	// Compute normal
	// PT: if spheres initially overlap, the convention is that returned normal = -sweep direction
	if(d==0.0f)
		nrm = -motion;
	else
		nrm = (center1 + d * motion) - center0;
	nrm.normalize();
	return true;
}
