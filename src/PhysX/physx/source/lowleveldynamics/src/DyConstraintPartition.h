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


#ifndef DY_CONSTRAINTPARTITION_H
#define DY_CONSTRAINTPARTITION_H

#include "DyDynamics.h"



namespace physx
{

namespace Dy
{
struct ConstraintPartitionArgs
{
	enum
	{
		eMAX_NUM_BODIES = 8192
	};   

	//Input
	PxSolverBody*							mBodies;
	PxU32									mNumBodies;
	ArticulationSolverDesc*				mArticulationPtrs;
	PxU32									mNumArticulationPtrs;
	PxSolverConstraintDesc*				mContactConstraintDescriptors;
	PxU32									mNumContactConstraintDescriptors;
	//output
	PxSolverConstraintDesc*				mOrderedContactConstraintDescriptors;
	PxSolverConstraintDesc*				mTempContactConstraintDescriptors;
	PxU32									mNumSelfConstraintBlocks;
	PxU32									mNumDifferentBodyConstraints;
	PxU32									mNumSelfConstraints;
	Ps::Array<PxU32>*						mConstraintsPerPartition;
	//Ps::Array<PxU32>*						mStartIndices;
	Ps::Array<PxU32>*						mBitField;

	bool									enhancedDeterminism;
};

PxU32 partitionContactConstraints(ConstraintPartitionArgs& args);

} // namespace physx

}



#endif // DY_CONSTRAINTPARTITION_H  

