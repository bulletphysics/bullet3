/*
 *	OPCODE - Optimized Collision Detection
 * http://www.codercorner.com/Opcode.htm
 * 
 * Copyright (c) 2001-2008 Pierre Terdiman,  pierre@codercorner.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


// Opcode 1.1: ray-AABB overlap tests based on Woo's code
// Opcode 1.2: ray-AABB overlap tests based on the separating axis theorem
//
// The point of intersection is not computed anymore. The distance to impact is not needed anymore
// since we now have two different queries for segments or rays.

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes a segment-AABB overlap test using the separating axis theorem. Segment is cached within the class.
 *	\param		center	[in] AABB center
 *	\param		extents	[in] AABB extents
 *	\return		true on overlap
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline_ BOOL RayCollider::SegmentAABBOverlap(const Point& center, const Point& extents)
{
	// Stats
	mNbRayBVTests++;

	float Dx = mData2.x - center.x;		if(fabsf(Dx) > extents.x + mFDir.x)	return FALSE;
	float Dy = mData2.y - center.y;		if(fabsf(Dy) > extents.y + mFDir.y)	return FALSE;
	float Dz = mData2.z - center.z;		if(fabsf(Dz) > extents.z + mFDir.z)	return FALSE;

	float f;
	f = mData.y * Dz - mData.z * Dy;	if(fabsf(f) > extents.y*mFDir.z + extents.z*mFDir.y)	return FALSE;
	f = mData.z * Dx - mData.x * Dz;	if(fabsf(f) > extents.x*mFDir.z + extents.z*mFDir.x)	return FALSE;
	f = mData.x * Dy - mData.y * Dx;	if(fabsf(f) > extents.x*mFDir.y + extents.y*mFDir.x)	return FALSE;

	return TRUE;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes a ray-AABB overlap test using the separating axis theorem. Ray is cached within the class.
 *	\param		center	[in] AABB center
 *	\param		extents	[in] AABB extents
 *	\return		true on overlap
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline_ BOOL RayCollider::RayAABBOverlap(const Point& center, const Point& extents)
{
	// Stats
	mNbRayBVTests++;

//	float Dx = mOrigin.x - center.x;	if(fabsf(Dx) > extents.x && Dx*mDir.x>=0.0f)	return FALSE;
//	float Dy = mOrigin.y - center.y;	if(fabsf(Dy) > extents.y && Dy*mDir.y>=0.0f)	return FALSE;
//	float Dz = mOrigin.z - center.z;	if(fabsf(Dz) > extents.z && Dz*mDir.z>=0.0f)	return FALSE;

	float Dx = mOrigin.x - center.x;	if(GREATER(Dx, extents.x) && Dx*mDir.x>=0.0f)	return FALSE;
	float Dy = mOrigin.y - center.y;	if(GREATER(Dy, extents.y) && Dy*mDir.y>=0.0f)	return FALSE;
	float Dz = mOrigin.z - center.z;	if(GREATER(Dz, extents.z) && Dz*mDir.z>=0.0f)	return FALSE;

//	float Dx = mOrigin.x - center.x;	if(GREATER(Dx, extents.x) && ((SIR(Dx)-1)^SIR(mDir.x))>=0.0f)	return FALSE;
//	float Dy = mOrigin.y - center.y;	if(GREATER(Dy, extents.y) && ((SIR(Dy)-1)^SIR(mDir.y))>=0.0f)	return FALSE;
//	float Dz = mOrigin.z - center.z;	if(GREATER(Dz, extents.z) && ((SIR(Dz)-1)^SIR(mDir.z))>=0.0f)	return FALSE;

	float f;
	f = mDir.y * Dz - mDir.z * Dy;		if(fabsf(f) > extents.y*mFDir.z + extents.z*mFDir.y)	return FALSE;
	f = mDir.z * Dx - mDir.x * Dz;		if(fabsf(f) > extents.x*mFDir.z + extents.z*mFDir.x)	return FALSE;
	f = mDir.x * Dy - mDir.y * Dx;		if(fabsf(f) > extents.x*mFDir.y + extents.y*mFDir.x)	return FALSE;

	return TRUE;
}
