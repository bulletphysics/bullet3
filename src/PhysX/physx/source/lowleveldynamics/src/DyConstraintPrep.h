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


#ifndef DY_CONSTRAINTSHADER_H
#define DY_CONSTRAINTSHADER_H

#include "DyConstraint.h"

#include "DySolverConstraintDesc.h"
#include "PsArray.h"

#define DY_ARTICULATION_MIN_RESPONSE 1e-5f

#define DY_ARTICULATION_BAD_RESPONSE 0.02f

namespace physx
{

class PxcConstraintBlockStream;
class PxsConstraintBlockManager;
struct PxSolverBody;
struct PxSolverBodyData;
struct PxSolverConstraintDesc;

namespace Cm
{
	struct SpatialVectorF;
}

namespace Dy
{

	static const PxU32 MAX_CONSTRAINT_ROWS = 12;

struct SolverConstraintShaderPrepDesc
{
	const Constraint* constraint;
	PxConstraintSolverPrep solverPrep;
	const void* constantBlock;
	PxU32 constantBlockByteSize;
};

SolverConstraintPrepState::Enum setupSolverConstraint4
	(SolverConstraintShaderPrepDesc* PX_RESTRICT constraintShaderDescs,
	PxSolverConstraintPrepDesc* PX_RESTRICT constraintDescs,
		const PxReal dt, const PxReal recipdt, PxU32& totalRows,
		 PxConstraintAllocator& allocator);

SolverConstraintPrepState::Enum setupSolverConstraint4
	(PxSolverConstraintPrepDesc* PX_RESTRICT constraintDescs,
	const PxReal dt, const PxReal recipdt, PxU32& totalRows,
	PxConstraintAllocator& allocator, PxU32 maxRows);

PxU32 SetupSolverConstraint(SolverConstraintShaderPrepDesc& shaderDesc,
							PxSolverConstraintPrepDesc& prepDesc,
							   PxConstraintAllocator& allocator,
							   PxReal dt, PxReal invdt, Cm::SpatialVectorF* Z);


class ConstraintHelper
{
public:

	static PxU32 setupSolverConstraint(
		PxSolverConstraintPrepDesc& prepDesc,
		PxConstraintAllocator& allocator,
		PxReal dt, PxReal invdt, Cm::SpatialVectorF* Z);
};

}

}

#endif //DY_CONSTRAINTSHADER_H
