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

// This is used by the box-sweep & capsule-sweep code

#if PX_VC 
	#pragma warning(disable: 4505)	// unreferenced local function has been removed
#endif

#include "PsBasicTemplates.h"

namespace
{
#ifdef SWEEP_AABB_IMPL
struct BoxSweepParams : RayParams
#else
struct BoxSweepParams : OBBTestParams
#endif
{
	const IndTri32*	PX_RESTRICT	mTris32;
	const IndTri16*	PX_RESTRICT	mTris16;
	const PxVec3*	PX_RESTRICT	mVerts;

#ifndef SWEEP_AABB_IMPL
	Box					mLocalBox;
#endif
	PxVec3				mLocalDir_Padded;
	RaycastHitInternal	mStabbedFace;

	PxU32				mBackfaceCulling;
	PxU32				mEarlyExit;

	PxVec3				mP0, mP1, mP2;
	PxVec3				mBestTriNormal;

	float				mOffset;
	PxVec3				mProj;
	PxVec3				mDP;

#ifndef SWEEP_AABB_IMPL
	PxMat33				mAR;				//!< Absolute rotation matrix
#endif

	PxMat33				mRModelToBox_Padded;	//!< Rotation from model space to obb space
	PxVec3				mTModelToBox_Padded;	//!< Translation from model space to obb space
	PxVec3				mOriginalExtents_Padded;
	PxVec3				mOriginalDir_Padded;
	PxVec3				mOneOverDir_Padded;
	PxVec3				mOneOverOriginalDir;

#ifndef SWEEP_AABB_IMPL
	PX_FORCE_INLINE void ShrinkOBB(float d)
	{
		const PxVec3 BoxExtents = mDP + d * mProj;
		mTBoxToModel_PaddedAligned = mLocalBox.center + mLocalDir_Padded*d*0.5f;

		setupBoxData(this, BoxExtents, &mAR);
	}
#endif
};
}

// PT: TODO: check asm again in PhysX version, compare to original (TA34704)
static void prepareSweepData(const Box& box, const PxVec3& dir, float maxDist, BoxSweepParams* PX_RESTRICT params)
{
	invertBoxMatrix(params->mRModelToBox_Padded, params->mTModelToBox_Padded, box);

	params->mOriginalExtents_Padded = box.extents;

	const PxVec3 OriginalDir = params->mRModelToBox_Padded.transform(dir);
	params->mOriginalDir_Padded = OriginalDir;

	const PxVec3 OneOverOriginalDir(OriginalDir.x!=0.0f ? 1.0f/OriginalDir.x : 0.0f,
									OriginalDir.y!=0.0f ? 1.0f/OriginalDir.y : 0.0f,
									OriginalDir.z!=0.0f ? 1.0f/OriginalDir.z : 0.0f);

	params->mOneOverOriginalDir = OneOverOriginalDir;
	params->mOneOverDir_Padded = OneOverOriginalDir / maxDist;

	{
		const Box& LocalBox = box;
		const PxVec3& LocalDir = dir;

		params->mLocalDir_Padded		= LocalDir;
		params->mStabbedFace.mDistance	= maxDist;
#ifndef SWEEP_AABB_IMPL
		params->mLocalBox				= LocalBox;	// PT: TODO: check asm for operator=
#endif

		PxMat33 boxToModelR;

		// Original code:
		// OBB::CreateOBB(LocalBox, LocalDir, 0.5f)
		{
			PxVec3 R1, R2;
			{
				float dd[3];
				dd[0] = fabsf(LocalBox.rot.column0.dot(LocalDir));
				dd[1] = fabsf(LocalBox.rot.column1.dot(LocalDir));
				dd[2] = fabsf(LocalBox.rot.column2.dot(LocalDir));
				float dmax = dd[0];
				PxU32 ax0=1;
				PxU32 ax1=2;
				if(dd[1]>dmax)
				{
					dmax=dd[1];
					ax0=0;
					ax1=2;
				}
				if(dd[2]>dmax)
				{
					dmax=dd[2];
					ax0=0;
					ax1=1;
				}
				if(dd[ax1]<dd[ax0])
					Ps::swap(ax0, ax1);

				R1 = LocalBox.rot[ax0];
				R1 -= R1.dot(LocalDir)*LocalDir;	// Project to plane whose normal is dir
				R1.normalize();
				R2 = LocalDir.cross(R1);
			}
			// Original code:
			// mRot = params->mRBoxToModel
			boxToModelR.column0 = LocalDir;
			boxToModelR.column1 = R1;
			boxToModelR.column2 = R2;

			// Original code:
			// float Offset[3];
			// 0.5f comes from the Offset[r]*0.5f, doesn't mean 'd' is 0.5f
			params->mProj.x = 0.5f;
			params->mProj.y = LocalDir.dot(R1)*0.5f;
			params->mProj.z = LocalDir.dot(R2)*0.5f;

			// Original code:
			//mExtents[r] = Offset[r]*0.5f + fabsf(box.mRot[0]|R)*box.mExtents.x + fabsf(box.mRot[1]|R)*box.mExtents.y + fabsf(box.mRot[2]|R)*box.mExtents.z;
			// => we store the first part of the computation, minus 'Offset[r]*0.5f'
			for(PxU32 r=0;r<3;r++)
			{
				const PxVec3& R = boxToModelR[r];
				params->mDP[r] =	fabsf(LocalBox.rot.column0.dot(R)*LocalBox.extents.x)
								+	fabsf(LocalBox.rot.column1.dot(R)*LocalBox.extents.y)
								+	fabsf(LocalBox.rot.column2.dot(R)*LocalBox.extents.z);
			}
			// In the original code, both mCenter & mExtents depend on 'd', and thus we will need to recompute these two members.
			//
			// For mExtents we have:
			// 
			//	float Offset[3];
			//	Offset[0] = d;
			//	Offset[1] = d*(dir|R1);
			//	Offset[2] = d*(dir|R2);
			//
			//		mExtents[r] = Offset[r]*0.5f + fabsf(box.mRot[0]|R)*box.mExtents.x + fabsf(box.mRot[1]|R)*box.mExtents.y + fabsf(box.mRot[2]|R)*box.mExtents.z;
			// <=>	mExtents[r] = Offset[r]*0.5f + Params.mDP[r];		We precompute the second part that doesn't depend on d, stored in mDP
			// <=>	mExtents[r] = Params.mProj[r]*d + Params.mDP[r];	We extract d from the first part, store what is left in mProj
			//
			// Thus in ShrinkOBB the code needed to update the extents is just:
			//	mBoxExtents = mDP + d * mProj;
			//
			// For mCenter we have:
			//
			//	mCenter = box.mCenter + dir*d*0.5f;
			//
			// So we simply use this formula directly, with the new d. Result is stored in 'mTBoxToModel'
/*
			PX_FORCE_INLINE void ShrinkOBB(float d)
			{
				mBoxExtents = mDP + d * mProj;
				mTBoxToModel = mLocalBox.mCenter + mLocalDir*d*0.5f;
*/
		}

		// This one is for culling tris, unrelated to CreateOBB
		params->mOffset = params->mDP.x + LocalBox.center.dot(LocalDir);

#ifndef SWEEP_AABB_IMPL
		precomputeData(params, &params->mAR, &boxToModelR);

		params->ShrinkOBB(maxDist);
#endif
	}
}
