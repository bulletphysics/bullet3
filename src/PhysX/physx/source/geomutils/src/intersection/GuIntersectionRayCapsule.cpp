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

#include "GuIntersectionRayCapsule.h"
#include "GuIntersectionRaySphere.h"

using namespace physx;

// PT: ray-capsule intersection code, originally from the old Magic Software library.
PxU32 Gu::intersectRayCapsuleInternal(const PxVec3& origin, const PxVec3& dir, const PxVec3& p0, const PxVec3& p1, float radius, PxReal s[2])
{
	// set up quadratic Q(t) = a*t^2 + 2*b*t + c
	PxVec3 kW = p1 - p0;
	const float fWLength = kW.magnitude();
	if(fWLength!=0.0f)
		kW /= fWLength;

	// PT: if the capsule is in fact a sphere, switch back to dedicated sphere code.
	// This is not just an optimization, the rest of the code fails otherwise.
	if(fWLength<=1e-6f)
	{
		const float d0 = (origin - p0).magnitudeSquared();
		const float d1 = (origin - p1).magnitudeSquared();
		const float approxLength = (PxMax(d0, d1) + radius)*2.0f;
		return PxU32(Gu::intersectRaySphere(origin, dir, approxLength, p0, radius, s[0]));
	}

	// generate orthonormal basis
	PxVec3 kU(0.0f);

	if (fWLength > 0.0f)
	{
		PxReal fInvLength;
		if ( PxAbs(kW.x) >= PxAbs(kW.y) )
		{
			// W.x or W.z is the largest magnitude component, swap them
			fInvLength = PxRecipSqrt(kW.x*kW.x + kW.z*kW.z);
			kU.x = -kW.z*fInvLength;
			kU.y = 0.0f;
			kU.z = kW.x*fInvLength;
		}
		else
		{
			// W.y or W.z is the largest magnitude component, swap them
			fInvLength = PxRecipSqrt(kW.y*kW.y + kW.z*kW.z);
			kU.x = 0.0f;
			kU.y = kW.z*fInvLength;
			kU.z = -kW.y*fInvLength;
		}
	}

	PxVec3 kV = kW.cross(kU);
	kV.normalize();	// PT: fixed november, 24, 2004. This is a bug in Magic.

	// compute intersection

	PxVec3 kD(kU.dot(dir), kV.dot(dir), kW.dot(dir));
	const float fDLength = kD.magnitude();
	const float fInvDLength = fDLength!=0.0f ? 1.0f/fDLength : 0.0f;
	kD *= fInvDLength;

	const PxVec3 kDiff = origin - p0;
	const PxVec3 kP(kU.dot(kDiff), kV.dot(kDiff), kW.dot(kDiff));
	const PxReal fRadiusSqr = radius*radius;

	// Is the velocity parallel to the capsule direction? (or zero)
	if ( PxAbs(kD.z) >= 1.0f - PX_EPS_REAL || fDLength < PX_EPS_REAL )
	{
		const float fAxisDir = dir.dot(kW);

		const PxReal fDiscr = fRadiusSqr - kP.x*kP.x - kP.y*kP.y;
		if ( fAxisDir < 0 && fDiscr >= 0.0f )
		{
			// Velocity anti-parallel to the capsule direction
			const PxReal fRoot = PxSqrt(fDiscr);
			s[0] = (kP.z + fRoot)*fInvDLength;
			s[1] = -(fWLength - kP.z + fRoot)*fInvDLength;
			return 2;
		}
		else if ( fAxisDir > 0  && fDiscr >= 0.0f )
		{
			// Velocity parallel to the capsule direction
			const PxReal fRoot = PxSqrt(fDiscr);
			s[0] = -(kP.z + fRoot)*fInvDLength;
			s[1] = (fWLength - kP.z + fRoot)*fInvDLength;
			return 2;
		}
		else
		{
			// sphere heading wrong direction, or no velocity at all
			return 0;
		}   
	}

	// test intersection with infinite cylinder
	PxReal fA = kD.x*kD.x + kD.y*kD.y;
	PxReal fB = kP.x*kD.x + kP.y*kD.y;
	PxReal fC = kP.x*kP.x + kP.y*kP.y - fRadiusSqr;
	PxReal fDiscr = fB*fB - fA*fC;
	if ( fDiscr < 0.0f )
	{
		// line does not intersect infinite cylinder
		return 0;
	}

	PxU32 iQuantity = 0;

	if ( fDiscr > 0.0f )
	{
		// line intersects infinite cylinder in two places
		const PxReal fRoot = PxSqrt(fDiscr);
		const PxReal fInv = 1.0f/fA;
		PxReal fT = (-fB - fRoot)*fInv;
		PxReal fTmp = kP.z + fT*kD.z;
		const float epsilon = 1e-3f;	// PT: see TA35174
		if ( fTmp >= -epsilon && fTmp <= fWLength+epsilon )
			s[iQuantity++] = fT*fInvDLength;

		fT = (-fB + fRoot)*fInv;
		fTmp = kP.z + fT*kD.z;
		if ( fTmp >= -epsilon && fTmp <= fWLength+epsilon )
			s[iQuantity++] = fT*fInvDLength;

		if ( iQuantity == 2 )
		{
			// line intersects capsule wall in two places
			return 2;
		}
	}
	else
	{
		// line is tangent to infinite cylinder
		const PxReal fT = -fB/fA;
		const PxReal fTmp = kP.z + fT*kD.z;
		if ( 0.0f <= fTmp && fTmp <= fWLength )
		{
			s[0] = fT*fInvDLength;
			return 1;
		}
	}

	// test intersection with bottom hemisphere
	// fA = 1
	fB += kP.z*kD.z;
	fC += kP.z*kP.z;
	fDiscr = fB*fB - fC;
	if ( fDiscr > 0.0f )
	{
		const PxReal fRoot = PxSqrt(fDiscr);
		PxReal fT = -fB - fRoot;
		PxReal fTmp = kP.z + fT*kD.z;
		if ( fTmp <= 0.0f )
		{
			s[iQuantity++] = fT*fInvDLength;
			if ( iQuantity == 2 )
				return 2;
		}

		fT = -fB + fRoot;
		fTmp = kP.z + fT*kD.z;
		if ( fTmp <= 0.0f )
		{
			s[iQuantity++] = fT*fInvDLength;
			if ( iQuantity == 2 )
				return 2;
		}
	}
	else if ( fDiscr == 0.0f )
	{
		const PxReal fT = -fB;
		const PxReal fTmp = kP.z + fT*kD.z;
		if ( fTmp <= 0.0f )
		{
			s[iQuantity++] = fT*fInvDLength;
			if ( iQuantity == 2 )
				return 2;
		}
	}

	// test intersection with top hemisphere
	// fA = 1
	fB -= kD.z*fWLength;
	fC += fWLength*(fWLength - 2.0f*kP.z);

	fDiscr = fB*fB - fC;
	if ( fDiscr > 0.0f )
	{
		const PxReal fRoot = PxSqrt(fDiscr);
		PxReal fT = -fB - fRoot;
		PxReal fTmp = kP.z + fT*kD.z;
		if ( fTmp >= fWLength )
		{
			s[iQuantity++] = fT*fInvDLength;
			if ( iQuantity == 2 )
				return 2;
		}

		fT = -fB + fRoot;
		fTmp = kP.z + fT*kD.z;
		if ( fTmp >= fWLength )
		{
			s[iQuantity++] = fT*fInvDLength;
			if ( iQuantity == 2 )
				return 2;
		}
	}
	else if ( fDiscr == 0.0f )
	{
		const PxReal fT = -fB;
		const PxReal fTmp = kP.z + fT*kD.z;
		if ( fTmp >= fWLength )
		{
			s[iQuantity++] = fT*fInvDLength;
			if ( iQuantity == 2 )
				return 2;
		}
	}
	return iQuantity;
}
