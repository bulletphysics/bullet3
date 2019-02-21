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

#ifndef DY_SOLVERCONTACT4_H
#define DY_SOLVERCONTACT4_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"
#include "PxvConfig.h"
#include "PsVecMath.h"
#include "DySolverContact.h"

namespace physx
{

struct PxcNpWorkUnit;
struct PxSolverBody;
struct PxSolverBodyData;
struct PxSolverConstraintDesc;

namespace Sc
{
	class ShapeInteraction;
}
	
namespace Dy
{




/**
\brief Batched SOA contact data. Note, we don't support batching with extended contacts for the simple reason that handling multiple articulations would be complex.
*/
struct SolverContactHeader4
{
	enum
	{
		eHAS_MAX_IMPULSE = 1 << 0,
		eHAS_TARGET_VELOCITY = 1 << 1
	};

	PxU8	type;					//Note: mType should be first as the solver expects a type in the first byte.
	PxU8	numNormalConstr;
	PxU8	numFrictionConstr;
	PxU8	flag;

	PxU8	flags[4];
	//These counts are the max of the 4 sets of data.
	//When certain pairs have fewer patches/contacts than others, they are padded with 0s so that no work is performed but 
	//calculations are still shared (afterall, they're computationally free because we're doing 4 things at a time in SIMD)

	//KS - used for write-back only
	PxU8	numNormalConstr0, numNormalConstr1, numNormalConstr2, numNormalConstr3;
	PxU8	numFrictionConstr0, numFrictionConstr1, numFrictionConstr2, numFrictionConstr3;			

	Vec4V	restitution;																			
	Vec4V   staticFriction;
	Vec4V	dynamicFriction;
	//Technically, these mass properties could be pulled out into a new structure and shared. For multi-manifold contacts,
	//this would save 64 bytes per-manifold after the cost of the first manifold
	Vec4V	invMass0D0;
	Vec4V	invMass1D1;
	Vec4V	angDom0;
	Vec4V	angDom1;
	//Normal is shared between all contacts in the batch. This will save some memory!
	Vec4V normalX;
	Vec4V normalY;
	Vec4V normalZ;

	Sc::ShapeInteraction* shapeInteraction[4];		//192 or 208
}; 

#if !PX_P64_FAMILY
PX_COMPILE_TIME_ASSERT(sizeof(SolverContactHeader4) == 192);
#else
PX_COMPILE_TIME_ASSERT(sizeof(SolverContactHeader4) == 208);
#endif


/**
\brief This represents a batch of 4 contacts with static rolled into a single structure
*/
struct SolverContactBatchPointBase4
{
	Vec4V raXnX;
	Vec4V raXnY;
	Vec4V raXnZ;
	Vec4V velMultiplier;
	Vec4V scaledBias;
	Vec4V biasedErr;
};
PX_COMPILE_TIME_ASSERT(sizeof(SolverContactBatchPointBase4) == 96);

/**
\brief Contains the additional data required to represent 4 contacts between 2 dynamic bodies
@see SolverContactBatchPointBase4
*/
struct SolverContactBatchPointDynamic4 : public SolverContactBatchPointBase4
{	
	Vec4V rbXnX;
	Vec4V rbXnY;
	Vec4V rbXnZ;
}; 
PX_COMPILE_TIME_ASSERT(sizeof(SolverContactBatchPointDynamic4) == 144);

/**
\brief This represents the shared information of a batch of 4 friction constraints
*/
struct SolverFrictionSharedData4
{
	BoolV broken;
	PxU8* frictionBrokenWritebackByte[4];
	Vec4V normalX[2];
	Vec4V normalY[2];
	Vec4V normalZ[2];
};
#if !PX_P64_FAMILY
PX_COMPILE_TIME_ASSERT(sizeof(SolverFrictionSharedData4) == 128);
#endif


/**
\brief This represents a batch of 4 friction constraints with static rolled into a single structure
*/
struct SolverContactFrictionBase4
{
	Vec4V raXnX;
	Vec4V raXnY;
	Vec4V raXnZ;
	Vec4V scaledBias;
	Vec4V velMultiplier;
	Vec4V targetVelocity;
};
PX_COMPILE_TIME_ASSERT(sizeof(SolverContactFrictionBase4) == 96);

/**
\brief Contains the additional data required to represent 4 friction constraints between 2 dynamic bodies
@see SolverContactFrictionBase4
*/
struct SolverContactFrictionDynamic4 : public SolverContactFrictionBase4
{
	Vec4V rbXnX;
	Vec4V rbXnY;
	Vec4V rbXnZ;
}; 
PX_COMPILE_TIME_ASSERT(sizeof(SolverContactFrictionDynamic4) == 144);

}

}

#endif //DY_SOLVERCONTACT4_H
