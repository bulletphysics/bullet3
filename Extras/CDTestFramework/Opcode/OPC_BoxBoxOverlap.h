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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	OBB-OBB overlap test using the separating axis theorem.
 *	- original code by Gomez / Gamasutra (similar to Gottschalk's one in RAPID)
 *	- optimized for AABB trees by computing the rotation matrix once (SOLID-fashion)
 *	- the fabs matrix is precomputed as well and epsilon-tweaked (RAPID-style, we found this almost mandatory)
 *	- Class III axes can be disabled... (SOLID & Intel fashion)
 *	- ...or enabled to perform some profiling
 *	- CPU comparisons used when appropriate
 *	- lazy evaluation sometimes saves some work in case of early exits (unlike SOLID)
 *
 *	\param		ea	[in] extents from box A
 *	\param		ca	[in] center from box A
 *	\param		eb	[in] extents from box B
 *	\param		cb	[in] center from box B
 *	\return		true if boxes overlap
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline_ BOOL AABBTreeCollider::BoxBoxOverlap(const Point& ea, const Point& ca, const Point& eb, const Point& cb)
{
	// Stats
	mNbBVBVTests++;

	float t,t2;

	// Class I : A's basis vectors
	float Tx = (mR1to0.m[0][0]*cb.x + mR1to0.m[1][0]*cb.y + mR1to0.m[2][0]*cb.z) + mT1to0.x - ca.x;
	t = ea.x + eb.x*mAR.m[0][0] + eb.y*mAR.m[1][0] + eb.z*mAR.m[2][0];
	if(GREATER(Tx, t))	return FALSE;

	float Ty = (mR1to0.m[0][1]*cb.x + mR1to0.m[1][1]*cb.y + mR1to0.m[2][1]*cb.z) + mT1to0.y - ca.y;
	t = ea.y + eb.x*mAR.m[0][1] + eb.y*mAR.m[1][1] + eb.z*mAR.m[2][1];
	if(GREATER(Ty, t))	return FALSE;

	float Tz = (mR1to0.m[0][2]*cb.x + mR1to0.m[1][2]*cb.y + mR1to0.m[2][2]*cb.z) + mT1to0.z - ca.z;
	t = ea.z + eb.x*mAR.m[0][2] + eb.y*mAR.m[1][2] + eb.z*mAR.m[2][2];
	if(GREATER(Tz, t))	return FALSE;

	// Class II : B's basis vectors
	t = Tx*mR1to0.m[0][0] + Ty*mR1to0.m[0][1] + Tz*mR1to0.m[0][2];	t2 = ea.x*mAR.m[0][0] + ea.y*mAR.m[0][1] + ea.z*mAR.m[0][2] + eb.x;
	if(GREATER(t, t2))	return FALSE;

	t = Tx*mR1to0.m[1][0] + Ty*mR1to0.m[1][1] + Tz*mR1to0.m[1][2];	t2 = ea.x*mAR.m[1][0] + ea.y*mAR.m[1][1] + ea.z*mAR.m[1][2] + eb.y;
	if(GREATER(t, t2))	return FALSE;

	t = Tx*mR1to0.m[2][0] + Ty*mR1to0.m[2][1] + Tz*mR1to0.m[2][2];	t2 = ea.x*mAR.m[2][0] + ea.y*mAR.m[2][1] + ea.z*mAR.m[2][2] + eb.z;
	if(GREATER(t, t2))	return FALSE;

	// Class III : 9 cross products
	// Cool trick: always perform the full test for first level, regardless of settings.
	// That way pathological cases (such as the pencils scene) are quickly rejected anyway !
	if(mFullBoxBoxTest || mNbBVBVTests==1)
	{
		t = Tz*mR1to0.m[0][1] - Ty*mR1to0.m[0][2];	t2 = ea.y*mAR.m[0][2] + ea.z*mAR.m[0][1] + eb.y*mAR.m[2][0] + eb.z*mAR.m[1][0];	if(GREATER(t, t2))	return FALSE;	// L = A0 x B0
		t = Tz*mR1to0.m[1][1] - Ty*mR1to0.m[1][2];	t2 = ea.y*mAR.m[1][2] + ea.z*mAR.m[1][1] + eb.x*mAR.m[2][0] + eb.z*mAR.m[0][0];	if(GREATER(t, t2))	return FALSE;	// L = A0 x B1
		t = Tz*mR1to0.m[2][1] - Ty*mR1to0.m[2][2];	t2 = ea.y*mAR.m[2][2] + ea.z*mAR.m[2][1] + eb.x*mAR.m[1][0] + eb.y*mAR.m[0][0];	if(GREATER(t, t2))	return FALSE;	// L = A0 x B2
		t = Tx*mR1to0.m[0][2] - Tz*mR1to0.m[0][0];	t2 = ea.x*mAR.m[0][2] + ea.z*mAR.m[0][0] + eb.y*mAR.m[2][1] + eb.z*mAR.m[1][1];	if(GREATER(t, t2))	return FALSE;	// L = A1 x B0
		t = Tx*mR1to0.m[1][2] - Tz*mR1to0.m[1][0];	t2 = ea.x*mAR.m[1][2] + ea.z*mAR.m[1][0] + eb.x*mAR.m[2][1] + eb.z*mAR.m[0][1];	if(GREATER(t, t2))	return FALSE;	// L = A1 x B1
		t = Tx*mR1to0.m[2][2] - Tz*mR1to0.m[2][0];	t2 = ea.x*mAR.m[2][2] + ea.z*mAR.m[2][0] + eb.x*mAR.m[1][1] + eb.y*mAR.m[0][1];	if(GREATER(t, t2))	return FALSE;	// L = A1 x B2
		t = Ty*mR1to0.m[0][0] - Tx*mR1to0.m[0][1];	t2 = ea.x*mAR.m[0][1] + ea.y*mAR.m[0][0] + eb.y*mAR.m[2][2] + eb.z*mAR.m[1][2];	if(GREATER(t, t2))	return FALSE;	// L = A2 x B0
		t = Ty*mR1to0.m[1][0] - Tx*mR1to0.m[1][1];	t2 = ea.x*mAR.m[1][1] + ea.y*mAR.m[1][0] + eb.x*mAR.m[2][2] + eb.z*mAR.m[0][2];	if(GREATER(t, t2))	return FALSE;	// L = A2 x B1
		t = Ty*mR1to0.m[2][0] - Tx*mR1to0.m[2][1];	t2 = ea.x*mAR.m[2][1] + ea.y*mAR.m[2][0] + eb.x*mAR.m[1][2] + eb.y*mAR.m[0][2];	if(GREATER(t, t2))	return FALSE;	// L = A2 x B2
	}
	return TRUE;
}

//! A dedicated version when one box is constant
inline_ BOOL OBBCollider::BoxBoxOverlap(const Point& extents, const Point& center)
{
	// Stats
	mNbVolumeBVTests++;

	float t,t2;

	// Class I : A's basis vectors
	float Tx = mTBoxToModel.x - center.x;	t = extents.x + mBBx1;	if(GREATER(Tx, t))	return FALSE;
	float Ty = mTBoxToModel.y - center.y;	t = extents.y + mBBy1;	if(GREATER(Ty, t))	return FALSE;
	float Tz = mTBoxToModel.z - center.z;	t = extents.z + mBBz1;	if(GREATER(Tz, t))	return FALSE;

	// Class II : B's basis vectors
	t = Tx*mRBoxToModel.m[0][0] + Ty*mRBoxToModel.m[0][1] + Tz*mRBoxToModel.m[0][2];
	t2 = extents.x*mAR.m[0][0] + extents.y*mAR.m[0][1] + extents.z*mAR.m[0][2] + mBoxExtents.x;
	if(GREATER(t, t2))	return FALSE;

	t = Tx*mRBoxToModel.m[1][0] + Ty*mRBoxToModel.m[1][1] + Tz*mRBoxToModel.m[1][2];
	t2 = extents.x*mAR.m[1][0] + extents.y*mAR.m[1][1] + extents.z*mAR.m[1][2] + mBoxExtents.y;
	if(GREATER(t, t2))	return FALSE;

	t = Tx*mRBoxToModel.m[2][0] + Ty*mRBoxToModel.m[2][1] + Tz*mRBoxToModel.m[2][2];
	t2 = extents.x*mAR.m[2][0] + extents.y*mAR.m[2][1] + extents.z*mAR.m[2][2] + mBoxExtents.z;
	if(GREATER(t, t2))	return FALSE;

	// Class III : 9 cross products
	// Cool trick: always perform the full test for first level, regardless of settings.
	// That way pathological cases (such as the pencils scene) are quickly rejected anyway !
	if(mFullBoxBoxTest || mNbVolumeBVTests==1)
	{
		t = Tz*mRBoxToModel.m[0][1] - Ty*mRBoxToModel.m[0][2];	t2 = extents.y*mAR.m[0][2] + extents.z*mAR.m[0][1] + mBB_1;	if(GREATER(t, t2))	return FALSE;	// L = A0 x B0
		t = Tz*mRBoxToModel.m[1][1] - Ty*mRBoxToModel.m[1][2];	t2 = extents.y*mAR.m[1][2] + extents.z*mAR.m[1][1] + mBB_2;	if(GREATER(t, t2))	return FALSE;	// L = A0 x B1
		t = Tz*mRBoxToModel.m[2][1] - Ty*mRBoxToModel.m[2][2];	t2 = extents.y*mAR.m[2][2] + extents.z*mAR.m[2][1] + mBB_3;	if(GREATER(t, t2))	return FALSE;	// L = A0 x B2
		t = Tx*mRBoxToModel.m[0][2] - Tz*mRBoxToModel.m[0][0];	t2 = extents.x*mAR.m[0][2] + extents.z*mAR.m[0][0] + mBB_4;	if(GREATER(t, t2))	return FALSE;	// L = A1 x B0
		t = Tx*mRBoxToModel.m[1][2] - Tz*mRBoxToModel.m[1][0];	t2 = extents.x*mAR.m[1][2] + extents.z*mAR.m[1][0] + mBB_5;	if(GREATER(t, t2))	return FALSE;	// L = A1 x B1
		t = Tx*mRBoxToModel.m[2][2] - Tz*mRBoxToModel.m[2][0];	t2 = extents.x*mAR.m[2][2] + extents.z*mAR.m[2][0] + mBB_6;	if(GREATER(t, t2))	return FALSE;	// L = A1 x B2
		t = Ty*mRBoxToModel.m[0][0] - Tx*mRBoxToModel.m[0][1];	t2 = extents.x*mAR.m[0][1] + extents.y*mAR.m[0][0] + mBB_7;	if(GREATER(t, t2))	return FALSE;	// L = A2 x B0
		t = Ty*mRBoxToModel.m[1][0] - Tx*mRBoxToModel.m[1][1];	t2 = extents.x*mAR.m[1][1] + extents.y*mAR.m[1][0] + mBB_8;	if(GREATER(t, t2))	return FALSE;	// L = A2 x B1
		t = Ty*mRBoxToModel.m[2][0] - Tx*mRBoxToModel.m[2][1];	t2 = extents.x*mAR.m[2][1] + extents.y*mAR.m[2][0] + mBB_9;	if(GREATER(t, t2))	return FALSE;	// L = A2 x B2
	}
	return TRUE;
}

//! A special version for 2 axis-aligned boxes
inline_ BOOL AABBCollider::AABBAABBOverlap(const Point& extents, const Point& center)
{
	// Stats
	mNbVolumeBVTests++;

	float tx = mBox.mCenter.x - center.x;	float ex = extents.x + mBox.mExtents.x;	if(GREATER(tx, ex))	return FALSE;
	float ty = mBox.mCenter.y - center.y;	float ey = extents.y + mBox.mExtents.y;	if(GREATER(ty, ey))	return FALSE;
	float tz = mBox.mCenter.z - center.z;	float ez = extents.z + mBox.mExtents.z;	if(GREATER(tz, ez))	return FALSE;

	return TRUE;
}
