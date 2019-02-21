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


#ifndef DY_CONTACTPREP_H
#define DY_CONTACTPREP_H

#include "DySolverConstraintDesc.h"
#include "PxSceneDesc.h"
#include "DySolverContact4.h"

namespace physx
{

struct PxcNpWorkUnit;
class PxsConstraintBlockManager;
struct PxsContactManagerOutput;
struct PxSolverBody;
struct PxSolverBodyData;
struct PxSolverConstraintDesc;
class PxsContactManager;

namespace Dy
{
	class ThreadContext;
	struct CorrelationBuffer;

#define CREATE_FINALIZE_SOLVER_CONTACT_METHOD_ARGS			\
	PxSolverContactDesc& contactDesc,						\
	PxsContactManagerOutput& output,						\
	ThreadContext& threadContext,							\
	const PxReal invDtF32,									\
	PxReal bounceThresholdF32,								\
	PxReal frictionOffsetThreshold,							\
	PxReal correlationDistance,								\
	PxReal solverOffsetSlop,								\
	PxConstraintAllocator& constraintAllocator,				\
	Cm::SpatialVectorF* Z

#define CREATE_FINALIZE_SOVLER_CONTACT_METHOD_ARGS_4									\
								 PxsContactManagerOutput** outputs,						\
								 ThreadContext& threadContext,							\
								 PxSolverContactDesc* blockDescs,						\
								 const PxReal invDtF32,									\
								 PxReal bounceThresholdF32,								\
								 PxReal	frictionThresholdF32,							\
								 PxReal	correlationDistanceF32,							\
								 PxReal solverOffsetSlopF32,							\
								 PxConstraintAllocator& constraintAllocator				

	
/*!
Method prototype for create finalize solver contact
*/

typedef	bool (*PxcCreateFinalizeSolverContactMethod)(CREATE_FINALIZE_SOLVER_CONTACT_METHOD_ARGS);

extern PxcCreateFinalizeSolverContactMethod createFinalizeMethods[3];

typedef	SolverConstraintPrepState::Enum (*PxcCreateFinalizeSolverContactMethod4)(CREATE_FINALIZE_SOVLER_CONTACT_METHOD_ARGS_4);

extern PxcCreateFinalizeSolverContactMethod4 createFinalizeMethods4[3];


bool createFinalizeSolverContacts(	PxSolverContactDesc& contactDesc,
									PxsContactManagerOutput& output,
									ThreadContext& threadContext,
									const PxReal invDtF32,
									PxReal bounceThresholdF32,
									PxReal frictionOffsetThreshold,
									PxReal correlationDistance,
									PxReal solverOffsetSlop,
									PxConstraintAllocator& constraintAllocator,
									Cm::SpatialVectorF* Z);

bool createFinalizeSolverContacts(	PxSolverContactDesc& contactDesc,
									CorrelationBuffer& c,
									const PxReal invDtF32,
									PxReal bounceThresholdF32,
									PxReal frictionOffsetThreshold,
									PxReal correlationDistance,
									PxReal solverOffsetSlop,
									PxConstraintAllocator& constraintAllocator,
									Cm::SpatialVectorF* Z);

SolverConstraintPrepState::Enum createFinalizeSolverContacts4(	PxsContactManagerOutput** outputs,
																 ThreadContext& threadContext,
																 PxSolverContactDesc* blockDescs,
																 const PxReal invDtF32,
																 PxReal bounceThresholdF32,
																 PxReal frictionOffsetThreshold,
																 PxReal correlationDistance,
																 PxReal solverOffsetSlop,
																 PxConstraintAllocator& constraintAllocator);

SolverConstraintPrepState::Enum createFinalizeSolverContacts4(	Dy::CorrelationBuffer& c,
																PxSolverContactDesc* blockDescs,
																const PxReal invDtF32,
																PxReal bounceThresholdF32,
																PxReal	frictionOffsetThreshold,
																PxReal correlationDistance,
																PxReal solverOffsetSlop,
																PxConstraintAllocator& constraintAllocator);



bool createFinalizeSolverContactsCoulomb1D(PxSolverContactDesc& contactDesc,
											 PxsContactManagerOutput& output,
											 ThreadContext& threadContext,
											 const PxReal invDtF32,
											 PxReal bounceThresholdF32,
											 PxReal frictionOffsetThreshold,
											 PxReal correlationDistance,
											 PxReal solverOffsetSlop,
											 PxConstraintAllocator& constraintAllocator,
											 Cm::SpatialVectorF* Z);

bool createFinalizeSolverContactsCoulomb2D(PxSolverContactDesc& contactDesc,
											PxsContactManagerOutput& output,
											ThreadContext& threadContext,
											const PxReal invDtF32,
											PxReal bounceThresholdF32,
											PxReal frictionOffsetThreshold,
											PxReal correlationDistance,
											PxReal solverOffsetSlop,
											PxConstraintAllocator& constraintAllocator,
											Cm::SpatialVectorF* Z);


SolverConstraintPrepState::Enum createFinalizeSolverContacts4Coulomb1D(	PxsContactManagerOutput** outputs,
																		ThreadContext& threadContext,
																		 PxSolverContactDesc* blockDescs,
																		 const PxReal invDtF32,
																		 PxReal bounceThresholdF32,
																		 PxReal frictionOffsetThreshold,
																		 PxReal correlationDistance,
																		 PxReal solverOffsetSlop,
																		 PxConstraintAllocator& constraintAllocator);

SolverConstraintPrepState::Enum createFinalizeSolverContacts4Coulomb2D(PxsContactManagerOutput** outputs,
																		ThreadContext& threadContext,
																		PxSolverContactDesc* blockDescs,
																		const PxReal invDtF32,
																		PxReal bounceThresholdF32,
																		PxReal frictionOffsetThreshold,
																		PxReal correlationDistance,
																		PxReal solverOffsetSlop,
																		PxConstraintAllocator& constraintAllocator);


PxU32 getContactManagerConstraintDesc(const PxsContactManagerOutput& cmOutput, const PxsContactManager& cm, PxSolverConstraintDesc& desc);

}

}

#endif //DY_CONTACTPREP_H
