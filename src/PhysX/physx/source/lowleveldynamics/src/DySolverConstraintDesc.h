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


#ifndef DY_SOLVERCONSTRAINTDESC_H
#define DY_SOLVERCONSTRAINTDESC_H

#include "PxvConfig.h"
#include "DySolverConstraintTypes.h"
#include "PsUtilities.h"
#include "PxConstraintDesc.h"
#include "solver/PxSolverDefs.h"

namespace physx
{

struct PxcNpWorkUnit;

struct PxsContactManagerOutput;

namespace Cm
{
	class SpatialVector;
}

struct PxSolverBody;
struct PxSolverBodyData;

namespace Dy
{

struct FsData;




// dsequeira: moved this articulation stuff here to sever a build dep on Articulation.h through DyThreadContext.h and onward

struct SelfConstraintBlock
{
	PxU32	startId;				
	PxU32	numSelfConstraints;	
	PxU16	fsDataLength;		
	PxU16	requiredSolverProgress;
	uintptr_t eaFsData;
};

//This class rolls together multiple contact managers into a single contact manager.
struct CompoundContactManager
{
	PxU32 mStartIndex;
	PxU16 mStride;
	PxU16 mReducedContactCount;

	PxcNpWorkUnit* unit;			//This is a work unit but the contact buffer has been adjusted to contain all the contacts for all the subsequent pairs
	PxsContactManagerOutput* cmOutput;
	PxU8* originalContactPatches;	//This is the original contact buffer that we replaced with a combined buffer	
	PxU8* originalContactPoints;
	PxU8 originalContactCount;
	PxU8 originalPatchCount;
	PxU8 originalStatusFlags;
	PxReal* originalForceBuffer;	//This is the original force buffer that we replaced with a combined force buffer
	PxU16* forceBufferList;			//This is a list of indices from the reduced force buffer to the original force buffers - we need this to fix up the write-backs from the solver	
};

struct SolverConstraintPrepState
{
enum Enum 
{
	eOUT_OF_MEMORY,
	eUNBATCHABLE,
	eSUCCESS
};
};

PX_FORCE_INLINE bool isArticulationConstraint(const PxSolverConstraintDesc& desc)
{
	return desc.linkIndexA != PxSolverConstraintDesc::NO_LINK || 
		desc.linkIndexB != PxSolverConstraintDesc::NO_LINK;
}


PX_FORCE_INLINE void setConstraintLength(PxSolverConstraintDesc& desc, const PxU32 constraintLength)
{
	PX_ASSERT(0==(constraintLength & 0x0f));
	PX_ASSERT(constraintLength <= PX_MAX_U16 * 16);
	desc.constraintLengthOver16 = Ps::to16(constraintLength >> 4);
}

PX_FORCE_INLINE void setWritebackLength(PxSolverConstraintDesc& desc, const PxU32 writeBackLength)
{
	PX_ASSERT(0==(writeBackLength & 0x03));
	PX_ASSERT(writeBackLength <= PX_MAX_U16 * 4);
	desc.writeBackLengthOver4 = Ps::to16(writeBackLength >> 2);
}

PX_FORCE_INLINE PxU32 getConstraintLength(const PxSolverConstraintDesc& desc)
{
	return PxU32(desc.constraintLengthOver16 << 4);
}


PX_FORCE_INLINE PxU32 getWritebackLength(const PxSolverConstraintDesc& desc)
{
	return PxU32(desc.writeBackLengthOver4 << 2);
}

PX_COMPILE_TIME_ASSERT(0 == (0x0f & sizeof(PxSolverConstraintDesc)));

#define MAX_PERMITTED_SOLVER_PROGRESS 0xFFFF

}

}

#endif //DY_SOLVERCONSTRAINTDESC_H
