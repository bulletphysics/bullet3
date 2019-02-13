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

#include "ScSqBoundsManager.h"
#include "ScBodySim.h"
#include "ScShapeSim.h"
#include "common/PxProfileZone.h"

using namespace physx;
using namespace Sc;

SqBoundsManager::SqBoundsManager() :
	mShapes			(PX_DEBUG_EXP("SqBoundsManager::mShapes")),
	mRefs			(PX_DEBUG_EXP("SqBoundsManager::mRefs")),				
	mBoundsIndices	(PX_DEBUG_EXP("SqBoundsManager::mBoundsIndices")),
	mRefless		(PX_DEBUG_EXP("SqBoundsManager::mRefless"))
{
}

void SqBoundsManager::addShape(ShapeSim& shape)
{
	PX_ASSERT(shape.getFlags() & PxShapeFlag::eSCENE_QUERY_SHAPE);
	PX_ASSERT(!shape.getBodySim()->usingSqKinematicTarget());
	PX_ASSERT(!shape.getBodySim()->isFrozen());

	const PxU32 id = mShapes.size();
	PX_ASSERT(id == mRefs.size());
	PX_ASSERT(id == mBoundsIndices.size());

	shape.setSqBoundsId(id);

	mShapes.pushBack(&shape);
	mRefs.pushBack(PX_INVALID_U32);	// PT: TODO: should be INVALID_PRUNERHANDLE but cannot include SqPruner.h
	mBoundsIndices.pushBack(shape.getElementID());
	mRefless.pushBack(&shape);
}

void SqBoundsManager::removeShape(ShapeSim& shape)
{
	const PxU32 id = shape.getSqBoundsId();
	PX_ASSERT(id!=PX_INVALID_U32);

	shape.setSqBoundsId(PX_INVALID_U32);
	mShapes[id] = mShapes.back();
	mBoundsIndices[id] = mBoundsIndices.back();
	mRefs[id] = mRefs.back();

	if(id+1 != mShapes.size())
		mShapes[id]->setSqBoundsId(id);

	mShapes.popBack();
	mRefs.popBack();
	mBoundsIndices.popBack();
}

void SqBoundsManager::syncBounds(SqBoundsSync& sync, SqRefFinder& finder, const PxBounds3* bounds, PxU64 contextID, const Cm::BitMap& dirtyShapeSimMap)
{
	PX_PROFILE_ZONE("Sim.sceneQuerySyncBounds", contextID);
	PX_UNUSED(contextID);

#if PX_DEBUG
	for(PxU32 i=0;i<mShapes.size();i++)
	{
		ShapeSim& shape = *mShapes[i];
		PX_UNUSED(shape);
		PX_ASSERT(shape.getFlags() & PxShapeFlag::eSCENE_QUERY_SHAPE);
		PX_ASSERT(!shape.getBodySim()->usingSqKinematicTarget());
		PX_ASSERT(!shape.getBodySim()->isFrozen());
	}
#endif

	ShapeSim*const * shapes = mRefless.begin();
	for(PxU32 i=0, size = mRefless.size();i<size;i++)
	{
		const PxU32 id = shapes[i]->getSqBoundsId();
		// PT:
		//
		// If id == PX_INVALID_U32, the shape has been removed and not re-added. Nothing to do in this case, we just ignore it.
		// This case didn't previously exist since mRefless only contained valid (added) shapes. But now we left removed shapes in the
		// structure, and these have an id == PX_INVALID_U32.
		//
		// Now if the id is valid but mRefs[id] == PX_INVALID_U32, this is a regular shape that has been added and not processed yet.
		// So we process it.
		//
		// Finally, if both id and mRefs[id] are not PX_INVALID_U32, this is a shape that has been added, removed, and re-added. The
		// array contains the same shape twice and we only need to process it once.
		if(id!=PX_INVALID_U32)
		{
			if(mRefs[id] == PX_INVALID_U32)	// PT: TODO: should be INVALID_PRUNERHANDLE but cannot include SqPruner.h
				mRefs[id] = finder.find(static_cast<PxRigidBody*>(shapes[i]->getBodySim()->getPxActor()), shapes[i]->getPxShape());
		}
	}
	mRefless.clear();

	sync.sync(mRefs.begin(), mBoundsIndices.begin(), bounds, mShapes.size(), dirtyShapeSimMap);
}
