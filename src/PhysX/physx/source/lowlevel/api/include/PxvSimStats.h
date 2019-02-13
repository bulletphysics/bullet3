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


#ifndef PXV_SIM_STATS_H
#define PXV_SIM_STATS_H

#include "foundation/PxAssert.h"
#include "foundation/PxMemory.h"
#include "CmPhysXCommon.h"
#include "PxGeometry.h"

namespace physx
{

/*!
\file
Context handling
*/

/************************************************************************/
/* Context handling, types                                              */
/************************************************************************/

/*!
Description: contains statistics for the simulation.
*/
struct PxvSimStats
{
	PxvSimStats() { clearAll(); }
	void clearAll() { PxMemZero(this, sizeof(PxvSimStats)); }		// set counters to zero

	PX_FORCE_INLINE void incCCDPairs(PxGeometryType::Enum g0, PxGeometryType::Enum g1)
	{
		PX_ASSERT(g0 <= g1);  // That's how they should be sorted
		mNbCCDPairs[g0][g1]++;
	}

	PX_FORCE_INLINE void decCCDPairs(PxGeometryType::Enum g0, PxGeometryType::Enum g1)
	{
		PX_ASSERT(g0 <= g1);  // That's how they should be sorted
		PX_ASSERT(mNbCCDPairs[g0][g1]);
		mNbCCDPairs[g0][g1]--;
	}

	PX_FORCE_INLINE void incModifiedContactPairs(PxGeometryType::Enum g0, PxGeometryType::Enum g1)
	{
		PX_ASSERT(g0 <= g1);  // That's how they should be sorted
		mNbModifiedContactPairs[g0][g1]++;
	}

	PX_FORCE_INLINE void decModifiedContactPairs(PxGeometryType::Enum g0, PxGeometryType::Enum g1)
	{
		PX_ASSERT(g0 <= g1);  // That's how they should be sorted
		PX_ASSERT(mNbModifiedContactPairs[g0][g1]);
		mNbModifiedContactPairs[g0][g1]--;
	}

	// PT: those guys are now persistent and shouldn't be cleared each frame
	PxU32	mNbDiscreteContactPairs	[PxGeometryType::eGEOMETRY_COUNT][PxGeometryType::eGEOMETRY_COUNT];
	PxU32	mNbCCDPairs				[PxGeometryType::eGEOMETRY_COUNT][PxGeometryType::eGEOMETRY_COUNT];

	PxU32	mNbModifiedContactPairs	[PxGeometryType::eGEOMETRY_COUNT][PxGeometryType::eGEOMETRY_COUNT];

	PxU32	mNbDiscreteContactPairsTotal;		// PT: sum of mNbDiscreteContactPairs, i.e. number of pairs reaching narrow phase
	PxU32	mNbDiscreteContactPairsWithCacheHits;
	PxU32	mNbDiscreteContactPairsWithContacts;
	PxU32	mNbActiveConstraints;
	PxU32	mNbActiveDynamicBodies;
	PxU32	mNbActiveKinematicBodies;

	PxU32	mNbAxisSolverConstraints;
	PxU32	mTotalCompressedContactSize;
	PxU32	mTotalConstraintSize;
	PxU32	mPeakConstraintBlockAllocations;

	PxU32	mNbNewPairs;
	PxU32	mNbLostPairs;

	PxU32	mNbNewTouches;
	PxU32	mNbLostTouches;

	PxU32	mNbPartitions;
};

}

#endif
