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



#ifndef DY_ARTICULATION_INTERFACE_H
#define DY_ARTICULATION_INTERFACE_H

#include "DyArticulationUtils.h"

namespace physx
{

class PxcConstraintBlockStream;
class PxcScratchAllocator;
class PxsConstraintBlockManager;
struct PxSolverConstraintDesc;

namespace Dy
{
	
	struct ArticulationSolverDesc;


class ArticulationPImpl
{
public:

	typedef PxU32 (*ComputeUnconstrainedVelocitiesFn)(const ArticulationSolverDesc& desc,
													 PxReal dt,
													 PxConstraintAllocator& allocator,
													 PxSolverConstraintDesc* constraintDesc,
													 PxU32& acCount,
													 const PxVec3& gravity, PxU64 contextID,
													 Cm::SpatialVectorF* Z, Cm::SpatialVectorF* DeltaV);

	typedef void (*UpdateBodiesFn)(const ArticulationSolverDesc& desc,
								   PxReal dt);

	typedef void (*SaveVelocityFn)(const ArticulationSolverDesc &m, Cm::SpatialVectorF* deltaV);

	typedef void(*SaveVelocityTGSFn)(const ArticulationSolverDesc& m, PxReal invDtF32);

	typedef PxU32(*SetupInternalConstraintsTGSFn)(const ArticulationSolverDesc& desc,
		PxcConstraintBlockStream& stream,
		PxTGSSolverConstraintDesc* constraintDesc,
		PxReal dt,
		PxReal invDt,
		PxReal totalDt,
		PxU32& acCount,
		PxsConstraintBlockManager& constraintBlockManager,
		Cm::SpatialVectorF* Z);

	typedef void(*ComputeUnconstrainedVelocitiesTGSFn)(const ArticulationSolverDesc& desc,
		PxReal dt,
		const PxVec3& gravity, PxU64 contextID, Cm::SpatialVectorF* Z, Cm::SpatialVectorF* DeltaV);

	typedef void(*UpdateDeltaMotionFn)(const ArticulationSolverDesc &m, const PxReal dt, Cm::SpatialVectorF* DeltaV);

	typedef void(*DeltaMotionToMotionVelFn)(const ArticulationSolverDesc &m, const PxReal dt);

	static ComputeUnconstrainedVelocitiesFn sComputeUnconstrainedVelocities[2];
	static UpdateBodiesFn sUpdateBodies[2];
	static UpdateBodiesFn sUpdateBodiesTGS[2];
	static SaveVelocityFn sSaveVelocity[2];
	static SaveVelocityTGSFn sSaveVelocityTGS[2];

	static UpdateDeltaMotionFn sUpdateDeltaMotion[2];
	static DeltaMotionToMotionVelFn sDeltaMotionToMotionVel[2];
	static ComputeUnconstrainedVelocitiesTGSFn sComputeUnconstrainedVelocitiesTGS[2];
	static SetupInternalConstraintsTGSFn sSetupInternalConstraintsTGS[2];

	static PxU32 computeUnconstrainedVelocities(const ArticulationSolverDesc& desc,
										   PxReal dt,
										   PxConstraintAllocator& allocator,
										   PxSolverConstraintDesc* constraintDesc,
										   PxU32& acCount,
										   PxcScratchAllocator&,
										   const PxVec3& gravity, PxU64 contextID,
										   Cm::SpatialVectorF* Z, Cm::SpatialVectorF* deltaV)
	{
		PxU32 type = desc.articulation->getType();
		PX_ASSERT(sComputeUnconstrainedVelocities[type]);
		if (sComputeUnconstrainedVelocities[type])
			return (sComputeUnconstrainedVelocities[type])(desc, dt, allocator, constraintDesc, acCount,
				gravity, contextID, Z, deltaV);
		else
			return 0;
	}

	static void	updateBodies(const ArticulationSolverDesc& desc,
						 PxReal dt)
	{
		PxU32 type = desc.articulation->getType();
		PX_ASSERT(sUpdateBodies[type]);
		if (sUpdateBodies[type])
			(*sUpdateBodies[type])(desc, dt);
	}

	static void	updateBodiesTGS(const ArticulationSolverDesc& desc,
		PxReal dt)
	{
		PxU32 type = desc.articulation->getType();
		PX_ASSERT(sUpdateBodiesTGS[type]);
		if (sUpdateBodiesTGS[type])
			(*sUpdateBodiesTGS[type])(desc, dt);
	}

	static void	saveVelocity(const ArticulationSolverDesc& desc, Cm::SpatialVectorF* deltaV)
	{
		PxU32 type = desc.articulation->getType();
		PX_ASSERT(sSaveVelocity[type]);
		if (sSaveVelocity[type])
			(*sSaveVelocity[type])(desc, deltaV);
	}


	static void	saveVelocityTGS(const ArticulationSolverDesc& desc, PxReal invDtF32)
	{
		PX_UNUSED(desc);
		PX_UNUSED(invDtF32);
		PxU32 type = desc.articulation->getType();
		PX_ASSERT(sSaveVelocityTGS[type]);
		if (sSaveVelocityTGS[type])
			(*sSaveVelocityTGS[type])(desc, invDtF32);
	}

	static void computeUnconstrainedVelocitiesTGS(const ArticulationSolverDesc& desc,
		PxReal dt,
		const PxVec3& gravity, PxU64 contextID, Cm::SpatialVectorF* Z, Cm::SpatialVectorF* DeltaV)
	{
		PxU32 type = desc.articulation->getType();
		PX_ASSERT(sComputeUnconstrainedVelocitiesTGS[type]);
		if (sComputeUnconstrainedVelocitiesTGS[type])
		(sComputeUnconstrainedVelocitiesTGS[type])(desc, dt, gravity, contextID, Z, DeltaV);
	}

	static void	updateDeltaMotion(const ArticulationSolverDesc& desc, const PxReal dt, Cm::SpatialVectorF* DeltaV)
	{
		PxU32 type = desc.articulation->getType();
		PX_ASSERT(sUpdateDeltaMotion[type]);
		if (sUpdateDeltaMotion[type])
			(*sUpdateDeltaMotion[type])(desc, dt, DeltaV);
	}

	static void	deltaMotionToMotionVel(const ArticulationSolverDesc& desc, const PxReal invDt)
	{
		PxU32 type = desc.articulation->getType();
		PX_ASSERT(sDeltaMotionToMotionVel[type]);
		if (sDeltaMotionToMotionVel[type])
			(*sDeltaMotionToMotionVel)(desc, invDt);
	}

	static PxU32 setupSolverInternalConstraintsTGS(const ArticulationSolverDesc& desc,
		PxcConstraintBlockStream& stream,
		PxTGSSolverConstraintDesc* constraintDesc,
		PxReal dt,
		PxReal invDt,
		PxReal totalDt,
		PxU32& acCount,
		PxsConstraintBlockManager& constraintBlockManager,
		Cm::SpatialVectorF* Z)
	{
		PxU32 type = desc.articulation->getType();
		PX_ASSERT(sSetupInternalConstraintsTGS[type]);
		if (sSetupInternalConstraintsTGS[type])
			return sSetupInternalConstraintsTGS[type](desc, stream, constraintDesc, dt, invDt, totalDt, acCount, constraintBlockManager, Z);
		return 0;

	}
};


}
}
#endif //DY_ARTICULATION_INTERFACE_H

