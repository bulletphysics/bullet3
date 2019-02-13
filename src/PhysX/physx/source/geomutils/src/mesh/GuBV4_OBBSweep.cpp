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

#include "GuBV4.h"
using namespace physx;
using namespace Gu;

#if PX_INTEL_FAMILY  && !defined(PX_SIMD_DISABLED)
#include "PsVecMath.h"
using namespace physx::shdfnd::aos;
#include "GuBV4_BoxSweep_Internal.h"

Ps::IntBool Sweep_AABB_BV4(const Box& localBox, const PxVec3& localDir, float maxDist, const BV4Tree& tree, SweepHit* PX_RESTRICT hit, PxU32 flags);
void GenericSweep_AABB_CB(const Box& localBox, const PxVec3& localDir, float maxDist, const BV4Tree& tree, MeshSweepCallback callback, void* userData, PxU32 flags);
void Sweep_AABB_BV4_CB(const Box& localBox, const PxVec3& localDir, float maxDist, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned, SweepUnlimitedCallback callback, void* userData, PxU32 flags, bool nodeSorting);

// PT: TODO: optimize this (TA34704)
static PX_FORCE_INLINE void computeLocalData(Box& localBox, PxVec3& localDir, const Box& box, const PxVec3& dir, const PxMat44* PX_RESTRICT worldm_Aligned)
{
	if(worldm_Aligned)
	{
		PxMat44 IWM;
		invertPRMatrix(&IWM, worldm_Aligned);

		localDir = IWM.rotate(dir);

		rotateBox(localBox, IWM, box);
	}
	else
	{
		localDir = dir;
		localBox = box;	// PT: TODO: check asm for operator= (TA34704)
	}
}

static PX_FORCE_INLINE bool isAxisAligned(const PxVec3& axis)
{
	const PxReal minLimit = 1e-3f;
	const PxReal maxLimit = 1.0f - 1e-3f;

	const PxReal absX = PxAbs(axis.x);
	if(absX>minLimit && absX<maxLimit)
		return false;

	const PxReal absY = PxAbs(axis.y);
	if(absY>minLimit && absY<maxLimit)
		return false;

	const PxReal absZ = PxAbs(axis.z);
	if(absZ>minLimit && absZ<maxLimit)
		return false;

	return true;
}

static PX_FORCE_INLINE bool isAABB(const Box& box)
{
	if(!isAxisAligned(box.rot.column0))
		return false;
	if(!isAxisAligned(box.rot.column1))
		return false;
	if(!isAxisAligned(box.rot.column2))
		return false;
	return true;
}

Ps::IntBool BV4_BoxSweepSingle(const Box& box, const PxVec3& dir, float maxDist, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned, SweepHit* PX_RESTRICT hit, PxU32 flags)
{
	Box localBox;
	PxVec3 localDir;
	computeLocalData(localBox, localDir, box, dir, worldm_Aligned);

	Ps::IntBool Status;
	if(isAABB(localBox))
		Status = Sweep_AABB_BV4(localBox, localDir, maxDist, tree, hit, flags);
	else
		Status = Sweep_OBB_BV4(localBox, localDir, maxDist, tree, hit, flags);
	if(Status && worldm_Aligned)
	{
		// Move to world space
		// PT: TODO: optimize (TA34704)
		hit->mPos = worldm_Aligned->transform(hit->mPos);
		hit->mNormal = worldm_Aligned->rotate(hit->mNormal);
	}
	return Status;
}

// PT: for design decisions in this function, refer to the comments of BV4_GenericSweepCB().
void BV4_BoxSweepCB(const Box& box, const PxVec3& dir, float maxDist, const BV4Tree& tree, const PxMat44* PX_RESTRICT worldm_Aligned, SweepUnlimitedCallback callback, void* userData, PxU32 flags, bool nodeSorting)
{
	Box localBox;
	PxVec3 localDir;
	computeLocalData(localBox, localDir, box, dir, worldm_Aligned);

	if(isAABB(localBox))
		Sweep_AABB_BV4_CB(localBox, localDir, maxDist, tree, worldm_Aligned, callback, userData, flags, nodeSorting);
	else
		Sweep_OBB_BV4_CB(localBox, localDir, maxDist, tree, worldm_Aligned, callback, userData, flags, nodeSorting);
}


// PT: this generic sweep uses an OBB because this is the most versatile volume, but it does not mean this function is
// a "box sweep function" per-se. In fact it could be used all alone to implement all sweeps in the SDK (but that would
// have an impact on performance).
//
// So the idea here is simply to provide and use a generic function for everything that the BV4 code does not support directly.
// In particular this should be used:
// - for convex sweeps (where the OBB is the box around the swept convex)
// - for non-trivial sphere/capsule/box sweeps where mesh scaling or inflation 
//
// By design we don't do leaf tests inside the BV4 traversal code here (because we don't support them, e.g. convex
// sweeps. If we could do them inside the BV4 traversal code, like we do for regular sweeps, then this would not be a generic
// sweep function, but instead a built-in, natively supported query). So the leaf tests are performed outside of BV4, in the
// client code, through MeshSweepCallback. This has a direct impact on the design & parameters of MeshSweepCallback.
//
// On the other hand this is used for "regular sweeps with shapes we don't natively support", i.e. SweepSingle kind of queries.
// This means that we need to support an early-exit codepath (without node-sorting) and a regular sweep single codepath (with
// node sorting) for this generic function. The leaf tests are external, but everything traversal-related should be exactly the
// same as the regular box-sweep function otherwise.
//
// As a consequence, this function is not well-suited to implement "unlimited results" kind of queries, a.k.a. "sweep all":
//
// - for regular sphere/capsule/box "sweep all" queries, the leaf tests should be internal (same as sweep single queries). This
//   means the existing MeshSweepCallback can't be reused.
//
// - there is no need to support "sweep any" (it is already supported by the other sweep functions).
//
// - there may be no need for ordered traversal/node sorting/ray shrinking, since we want to return all results anyway. But this
//   may not be true if the "sweep all" function is used to emulate the Epic Tweak. In that case we still want to shrink the ray
//   and use node sorting. Since both versions are useful, we should probably have a bool param to enable/disable node sorting.
//
// - we are interested in all hits so we can't delay the computation of impact data (computing it only once in the end, for the
//   closest hit). We actually need to compute the data for all hits, possibly within the traversal code.
void BV4_GenericSweepCB(const Box& localBox, const PxVec3& localDir, float maxDist, const BV4Tree& tree, MeshSweepCallback callback, void* userData, bool anyHit)
{
	const PxU32 flags = anyHit ? PxU32(QUERY_MODIFIER_ANY_HIT) : 0;

	if(isAABB(localBox))
		GenericSweep_AABB_CB(localBox, localDir, maxDist, tree, callback, userData, flags);
	else
		GenericSweep_OBB_CB(localBox, localDir, maxDist, tree, callback, userData, flags);
}

#endif
