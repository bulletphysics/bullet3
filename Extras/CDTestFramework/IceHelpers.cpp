/*
CDTestFramework http://codercorner.com
Copyright (c) 2007-2008 Pierre Terdiman,  pierre@codercorner.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "stdafx.h"
#include "IceHelpers.h"

// Misc functions borrowed & adapted from ICE

void RotX(Matrix3x3& m, float angle)
{
	float Cos = cosf(angle);
	float Sin = sinf(angle);
	m.Identity();
	m.m[1][1] = m.m[2][2] = Cos;
	m.m[2][1] = -Sin;
	m.m[1][2] = Sin;
}

void RotY(Matrix3x3& m, float angle)
{
	float Cos = cosf(angle);
	float Sin = sinf(angle);
	m.Identity();
	m.m[0][0] = m.m[2][2] = Cos;
	m.m[2][0] = Sin;
	m.m[0][2] = -Sin;
}

void RotZ(Matrix3x3& m, float angle)
{
	float Cos = cosf(angle);
	float Sin = sinf(angle);
	m.Identity();
	m.m[0][0] = m.m[1][1] = Cos;
	m.m[1][0] = -Sin;
	m.m[0][1] = Sin;
}

bool SegmentSphere(const Point& origin, const Point& dir, float length, const Point& center, float radius, float& dist, Point& hit_pos)
{
	// get the offset vector
	Point offset = center - origin;

	// get the distance along the ray to the center point of the sphere
	float ray_dist = dir | offset;

	// get the squared distances
	float off2 = offset | offset;
	float rd2 = radius * radius;
	if(off2 <= rd2)
	{
		// we're in the sphere
		hit_pos	= origin;
		dist	= 0.0f;
		return true;
	}

	if(ray_dist <= 0 || (ray_dist - length) > radius)
	{
		// moving away from object or too far away
		return false;
	}

	// find hit distance squared
	float d = rd2 - (off2 - ray_dist * ray_dist);
	if(d<0.0f)
	{
		// ray passes by sphere without hitting
		return false;
	}

	// get the distance along the ray
	dist = ray_dist - sqrtf(d);
	if(dist > length)
	{
		// hit point beyond length
		return false;
	}

	// sort out the details
	hit_pos = origin + dir * dist;
	return true;
}

bool /*Ctc::*/RayAABB2(const Point& min, const Point& max, const Point& origin, const Point& dir, Point& coord)
{
	BOOL Inside = TRUE;
	Point MaxT;
	MaxT.x=MaxT.y=MaxT.z=-1.0f;

	// Find candidate planes.
	for(udword i=0;i<3;i++)
	{
		if(origin[i] < min[i])
		{
			coord[i]	= min[i];
			Inside		= FALSE;

			// Calculate T distances to candidate planes
			if(IR(dir[i]))	MaxT[i] = (min[i] - origin[i]) / dir[i];
		}
		else if(origin[i] > max[i])
		{
			coord[i]	= max[i];
			Inside		= FALSE;

			// Calculate T distances to candidate planes
			if(IR(dir[i]))	MaxT[i] = (max[i] - origin[i]) / dir[i];
		}
	}

	// Ray origin inside bounding box
	if(Inside)
	{
		coord = origin;
		return true;
	}

	// Get largest of the maxT's for final choice of intersection
	udword WhichPlane = 0;
	if(MaxT[1] > MaxT[WhichPlane])	WhichPlane = 1;
	if(MaxT[2] > MaxT[WhichPlane])	WhichPlane = 2;

	// Check final candidate actually inside box
	if(IR(MaxT[WhichPlane])&0x80000000) return false;

	for(udword i=0;i<3;i++)
	{
		if(i!=WhichPlane)
		{
			coord[i] = origin[i] + MaxT[WhichPlane] * dir[i];
#ifdef RAYAABB_EPSILON
			if(coord[i] < min[i] - RAYAABB_EPSILON || coord[i] > max[i] + RAYAABB_EPSILON)	return false;
#else
			if(coord[i] < min[i] || coord[i] > max[i])	return false;
#endif
		}
	}
	return true;	// ray hits box
}

static const bool gNormalize = true;

udword /*Ctc::*/RayCapsuleOverlap(const Point& origin, const Point& dir, const LSS& capsule, float s[2])
{
	// set up quadratic Q(t) = a*t^2 + 2*b*t + c
	Point kU, kV, kW, capsDir;
	capsule.ComputeDirection(capsDir);
	kW = capsDir;
	
	float fWLength = kW.Magnitude();
	kW.Normalize();
	
	// generate orthonormal basis
	
    float fInvLength;
	if ( fabsf(kW.x) >= fabsf(kW.y) )
    {
        // W.x or W.z is the largest magnitude component, swap them
		fInvLength = 1.0f/sqrtf(kW.x*kW.x + kW.z*kW.z);
        kU.x = -kW.z*fInvLength;
        kU.y = 0.0f;
        kU.z = +kW.x*fInvLength;
    }
    else
    {
        // W.y or W.z is the largest magnitude component, swap them
        fInvLength = 1.0f/sqrtf(kW.y*kW.y + kW.z*kW.z);
        kU.x = 0.0f;
        kU.y = +kW.z*fInvLength;
        kU.z = -kW.y*fInvLength;
    }
    kV = kW^kU;
kU.Normalize();
	if(gNormalize)
	kV.Normalize();

	// compute intersection

	Point kD(kU|dir, kV|dir, kW|dir);
	float fDLength = kD.Magnitude();
	kD.Normalize();

	float fInvDLength = 1.0f/fDLength;
	Point kDiff = origin - capsule.mP0;
	Point kP(kU|kDiff, kV|kDiff, kW|kDiff);
	float fRadiusSqr = capsule.mRadius*capsule.mRadius;

	float fInv, fA, fB, fC, fDiscr, fRoot, fT, fTmp;

	// Is the velocity parallel to the capsule direction? (or zero)
	if ( fabsf(kD.z) >= 1.0f - FLT_EPSILON || fDLength < FLT_EPSILON )
		{

		float fAxisDir = dir|capsDir;

		fDiscr = fRadiusSqr - kP.x*kP.x - kP.y*kP.y;
		if ( fAxisDir < 0 && fDiscr >= 0.0f )
			{
			// Velocity anti-parallel to the capsule direction
			fRoot = sqrtf(fDiscr);
			s[0] = (kP.z + fRoot)*fInvDLength;
			s[1] = -(fWLength - kP.z + fRoot)*fInvDLength;
			return 2;
			}
		else if ( fAxisDir > 0  && fDiscr >= 0.0f )
			{
			// Velocity parallel to the capsule direction
			fRoot = sqrtf(fDiscr);
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
	fA = kD.x*kD.x + kD.y*kD.y;
	fB = kP.x*kD.x + kP.y*kD.y;
	fC = kP.x*kP.x + kP.y*kP.y - fRadiusSqr;
	fDiscr = fB*fB - fA*fC;
	if ( fDiscr < 0.0f )
		{
		// line does not intersect infinite cylinder
		return 0;
		}

	int iQuantity = 0;

	if ( fDiscr > 0.0f )
		{
		// line intersects infinite cylinder in two places
		fRoot = sqrtf(fDiscr);
		fInv = 1.0f/fA;
		fT = (-fB - fRoot)*fInv;
		fTmp = kP.z + fT*kD.z;
		if ( 0.0f <= fTmp && fTmp <= fWLength )
			s[iQuantity++] = fT*fInvDLength;

		fT = (-fB + fRoot)*fInv;
		fTmp = kP.z + fT*kD.z;
		if ( 0.0f <= fTmp && fTmp <= fWLength )
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
		fT = -fB/fA;
		fTmp = kP.z + fT*kD.z;
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
		fRoot = sqrtf(fDiscr);
		fT = -fB - fRoot;
		fTmp = kP.z + fT*kD.z;
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
		fT = -fB;
		fTmp = kP.z + fT*kD.z;
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
		fRoot = sqrtf(fDiscr);
		fT = -fB - fRoot;
		fTmp = kP.z + fT*kD.z;
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
		fT = -fB;
		fTmp = kP.z + fT*kD.z;
		if ( fTmp >= fWLength )
			{
			s[iQuantity++] = fT*fInvDLength;
			if ( iQuantity == 2 )
				return 2;
			}
		}

	return iQuantity;
}

