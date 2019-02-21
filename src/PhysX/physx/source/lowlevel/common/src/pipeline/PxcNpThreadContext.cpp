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

#include "PxcConstraintBlockStream.h"
#include "PxcNpThreadContext.h"

using namespace physx;

PxcNpThreadContext::PxcNpThreadContext(PxcNpContext* params) : 
	mRenderOutput						(params->mRenderBuffer),
	mContactBlockStream					(params->mNpMemBlockPool),
	mNpCacheStreamPair					(params->mNpMemBlockPool),
	mNarrowPhaseParams					(0.0f, params->mMeshContactMargin, params->mToleranceLength),
	mPCM								(false),
	mContactCache						(false),
	mCreateContactStream				(params->mCreateContactStream),
	mCreateAveragePoint					(false),
#if PX_ENABLE_SIM_STATS
	mCompressedCacheSize				(0),
	mNbDiscreteContactPairsWithCacheHits(0),
	mNbDiscreteContactPairsWithContacts	(0),
#endif
	mMaxPatches							(0),
	mTotalCompressedCacheSize			(0),
	mContactStreamPool					(params->mContactStreamPool),
	mPatchStreamPool					(params->mPatchStreamPool),
	mForceAndIndiceStreamPool			(params->mForceAndIndiceStreamPool),
	mMaterialManager					(params->mMaterialManager),
	mLocalNewTouchCount					(0), 
	mLocalLostTouchCount				(0),
	mLocalFoundPatchCount				(0),	
	mLocalLostPatchCount				(0)
{
#if PX_ENABLE_SIM_STATS
	clearStats();
#endif
}

PxcNpThreadContext::~PxcNpThreadContext()
{
}

#if PX_ENABLE_SIM_STATS
void PxcNpThreadContext::clearStats()
{
	PxMemSet(mDiscreteContactPairs, 0, sizeof(mDiscreteContactPairs));
	PxMemSet(mModifiedContactPairs, 0, sizeof(mModifiedContactPairs));
	mCompressedCacheSize					= 0;
	mNbDiscreteContactPairsWithCacheHits	= 0;
	mNbDiscreteContactPairsWithContacts		= 0;
}
#endif

void PxcNpThreadContext::reset(PxU32 cmCount)
{
	mContactBlockStream.reset();
	mNpCacheStreamPair.reset();

	mLocalChangeTouch.clear();
	mLocalChangeTouch.resize(cmCount);
	mLocalPatchCountChange.clear();
	mLocalPatchCountChange.resize(cmCount);
	mLocalNewTouchCount = 0;
	mLocalLostTouchCount = 0;
	mLocalFoundPatchCount = 0;
	mLocalLostPatchCount = 0;
}
