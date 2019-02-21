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

#include "PxSceneQueryExt.h"

using namespace physx;

bool PxSceneQueryExt::raycastAny(	const PxScene& scene,
									const PxVec3& origin, const PxVec3& unitDir, const PxReal distance,
									PxSceneQueryHit& hit, const PxSceneQueryFilterData& filterData,
									PxSceneQueryFilterCallback* filterCall, const PxSceneQueryCache* cache)
{
	PxSceneQueryFilterData fdAny = filterData;
	fdAny.flags |= PxQueryFlag::eANY_HIT;
	PxRaycastBuffer buf;
	scene.raycast(origin, unitDir, distance, buf, PxHitFlags(), fdAny, filterCall, cache);
	hit = buf.block;
	return buf.hasBlock;
}

bool PxSceneQueryExt::raycastSingle(const PxScene& scene,
									const PxVec3& origin, const PxVec3& unitDir, const PxReal distance,
									PxSceneQueryFlags outputFlags, PxRaycastHit& hit,
									const PxSceneQueryFilterData& filterData,
									PxSceneQueryFilterCallback* filterCall, const PxSceneQueryCache* cache)
{
	PxRaycastBuffer buf;
	PxQueryFilterData fd1 = filterData;
	scene.raycast(origin, unitDir, distance, buf, outputFlags, fd1, filterCall, cache);
	hit = buf.block;
	return buf.hasBlock;
}

PxI32 PxSceneQueryExt::raycastMultiple(	const PxScene& scene,
										const PxVec3& origin, const PxVec3& unitDir, const PxReal distance,
										PxSceneQueryFlags outputFlags,
										PxRaycastHit* hitBuffer, PxU32 hitBufferSize, bool& blockingHit,
										const PxSceneQueryFilterData& filterData,
										PxSceneQueryFilterCallback* filterCall, const PxSceneQueryCache* cache)
{
	PxRaycastBuffer buf(hitBuffer, hitBufferSize);
	PxQueryFilterData fd1 = filterData;
	scene.raycast(origin, unitDir, distance, buf, outputFlags, fd1, filterCall, cache);
	blockingHit = buf.hasBlock;
	if(blockingHit)
	{
		if(buf.nbTouches < hitBufferSize)
		{
			hitBuffer[buf.nbTouches] = buf.block;
			return PxI32(buf.nbTouches+1);
		}
		else // overflow, drop the last touch
		{
			hitBuffer[hitBufferSize-1] = buf.block;
			return -1;
		}
	} else
		// no block
		return PxI32(buf.nbTouches);
}

bool PxSceneQueryExt::sweepAny(	const PxScene& scene,
								const PxGeometry& geometry, const PxTransform& pose, const PxVec3& unitDir, const PxReal distance,
								PxSceneQueryFlags queryFlags,
								PxSceneQueryHit& hit,
								const PxSceneQueryFilterData& filterData,
								PxSceneQueryFilterCallback* filterCall,
								const PxSceneQueryCache* cache,
								PxReal inflation)
{
	PxSceneQueryFilterData fdAny = filterData;
	fdAny.flags |= PxQueryFlag::eANY_HIT;
	PxSweepBuffer buf;
	scene.sweep(geometry, pose, unitDir, distance, buf, queryFlags, fdAny, filterCall, cache, inflation);
	hit = buf.block;
	return buf.hasBlock;
}

bool PxSceneQueryExt::sweepSingle(	const PxScene& scene,
									const PxGeometry& geometry, const PxTransform& pose, const PxVec3& unitDir, const PxReal distance,
									PxSceneQueryFlags outputFlags,
									PxSweepHit& hit,
									const PxSceneQueryFilterData& filterData,
									PxSceneQueryFilterCallback* filterCall,
									const PxSceneQueryCache* cache,
									PxReal inflation)
{
	PxSweepBuffer buf;
	PxQueryFilterData fd1 = filterData;
	scene.sweep(geometry, pose, unitDir, distance, buf, outputFlags, fd1, filterCall, cache, inflation);
	hit = buf.block;
	return buf.hasBlock;
}

PxI32 PxSceneQueryExt::sweepMultiple(	const PxScene& scene,
										const PxGeometry& geometry, const PxTransform& pose, const PxVec3& unitDir, const PxReal distance,
										PxSceneQueryFlags outputFlags, PxSweepHit* hitBuffer, PxU32 hitBufferSize, bool& blockingHit,
										const PxSceneQueryFilterData& filterData,
										PxSceneQueryFilterCallback* filterCall, const PxSceneQueryCache* cache,
										PxReal inflation)
{
	PxQueryFilterData fd1 = filterData;
	PxSweepBuffer buf(hitBuffer, hitBufferSize);
	scene.sweep(geometry, pose, unitDir, distance, buf, outputFlags, fd1, filterCall, cache, inflation);
	blockingHit = buf.hasBlock;
	if(blockingHit)
	{
		if(buf.nbTouches < hitBufferSize)
		{
			hitBuffer[buf.nbTouches] = buf.block;
			return PxI32(buf.nbTouches+1);
		}
		else // overflow, drop the last touch
		{
			hitBuffer[hitBufferSize-1] = buf.block;
			return -1;
		}
	} else
		// no block
		return PxI32(buf.nbTouches);
}

PxI32 PxSceneQueryExt::overlapMultiple(	const PxScene& scene,
										const PxGeometry& geometry, const PxTransform& pose,
										PxOverlapHit* hitBuffer, PxU32 hitBufferSize,
										const PxSceneQueryFilterData& filterData,
										PxSceneQueryFilterCallback* filterCall)
{
	PxQueryFilterData fd1 = filterData;
	fd1.flags |= PxQueryFlag::eNO_BLOCK;
	PxOverlapBuffer buf(hitBuffer, hitBufferSize);
	scene.overlap(geometry, pose, buf, fd1, filterCall);
	if(buf.hasBlock)
	{
		if(buf.nbTouches < hitBufferSize)
		{
			hitBuffer[buf.nbTouches] = buf.block;
			return PxI32(buf.nbTouches+1);
		}
		else // overflow, drop the last touch
		{
			hitBuffer[hitBufferSize-1] = buf.block;
			return -1;
		}
	} else
		// no block
		return PxI32(buf.nbTouches);
}

bool PxSceneQueryExt::overlapAny(	const PxScene& scene,
									const PxGeometry& geometry, const PxTransform& pose,
									PxOverlapHit& hit,
									const PxSceneQueryFilterData& filterData,
									PxSceneQueryFilterCallback* filterCall)
{
	PxSceneQueryFilterData fdAny = filterData;
	fdAny.flags |= (PxQueryFlag::eANY_HIT | PxQueryFlag::eNO_BLOCK);
	PxOverlapBuffer buf;
	scene.overlap(geometry, pose, buf, fdAny, filterCall);
	hit = buf.block;
	return buf.hasBlock;
}
