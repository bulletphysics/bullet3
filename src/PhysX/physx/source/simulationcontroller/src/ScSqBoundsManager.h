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

#ifndef PX_PHYSICS_SCP_SQ_BOUNDS_MANAGER
#define PX_PHYSICS_SCP_SQ_BOUNDS_MANAGER

#include "CmPhysXCommon.h"
#include "foundation/PxBounds3.h"
#include "PsArray.h"
#include "PsUserAllocated.h"
#include "PsHashSet.h"
#include "CmBitMap.h"

namespace physx
{
namespace Sq
{
typedef PxU32 PrunerHandle;	// PT: we should get this from SqPruner.h but it cannot be included from here
}

namespace Sc
{
struct SqBoundsSync;
struct SqRefFinder;
class ShapeSim;

class SqBoundsManager : public Ps::UserAllocated
{
	PX_NOCOPY(SqBoundsManager)
public:
									SqBoundsManager();

	void							addShape(ShapeSim& shape);
	void							removeShape(ShapeSim& shape);
	void							syncBounds(SqBoundsSync& sync, SqRefFinder& finder, const PxBounds3* bounds, PxU64 contextID, const Cm::BitMap& dirtyShapeSimMap);

private:

	Ps::Array<ShapeSim*>			mShapes;		// 
	Ps::Array<Sq::PrunerHandle>		mRefs;			// SQ pruner references
	Ps::Array<PxU32>				mBoundsIndices;	// indices into the Sc bounds array
	Ps::Array<ShapeSim*>			mRefless;		// shapesims without references
};
}
}

#endif
