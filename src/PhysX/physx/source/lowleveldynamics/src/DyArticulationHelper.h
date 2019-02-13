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


#ifndef DY_ARTICULATION_HELPER_H
#define DY_ARTICULATION_HELPER_H


#include "DyArticulation.h"

namespace physx
{
struct PxsBodyCore;

class PxcConstraintBlockStream;
class PxcRigidBody;
class PxsConstraintBlockManager;
struct PxSolverConstraintDesc;

namespace Dy
{
	struct FsInertia;
	struct SolverConstraint1DExt;
	struct ArticulationJointCore;
	struct ArticulationSolverDesc;
	struct SolverConstraint1DExtStep;
	struct PxcFsScratchAllocator;
	struct PxTGSSolverConstraintDesc;


struct ArticulationJointTransforms
{
	PxTransform		cA2w;				// joint parent frame in world space 
	PxTransform		cB2w;				// joint child frame in world space
	PxTransform		cB2cA;				// joint relative pose in world space
};

class ArticulationHelper
{
public:

	static void		getImpulseResponse(	const FsData& matrix, 
										PxU32 linkID,
										const Cm::SpatialVectorV& impulse,
										Cm::SpatialVectorV& deltaV);


	static PX_FORCE_INLINE 
			void	getImpulseResponse(	const FsData& matrix, 
										PxU32 linkID,
										const Cm::SpatialVector& impulse,
										Cm::SpatialVector& deltaV)
	{
		getImpulseResponse(matrix, linkID, reinterpret_cast<const Cm::SpatialVectorV&>(impulse), reinterpret_cast<Cm::SpatialVectorV&>(deltaV));
	}

	static void		getImpulseSelfResponse(const FsData& matrix, 
										   PxU32 linkID0, 
										   const Cm::SpatialVectorV& impulse0,
										   Cm::SpatialVectorV& deltaV0,
										   PxU32 linkID1,
										   const Cm::SpatialVectorV& impulse1,
										   Cm::SpatialVectorV& deltaV1);

	//static void		flushVelocity(FsData& matrix);

	static void		saveVelocity(const ArticulationSolverDesc& m, Cm::SpatialVectorF* deltaV);

	static void		saveVelocityTGS(const ArticulationSolverDesc& m, const PxReal invDt);

	//static void		recordDeltaMotion(const ArticulationSolverDesc& m, PxReal dt);

	//static void		deltaMotionToMotionVelocity(const ArticulationSolverDesc& m, PxReal invDt);

	//static void		getDataSizes(PxU32 linkCount, PxU32 &solverDataSize, PxU32& totalSize, PxU32& scratchSize);

	/*static void		initializeDriveCache(Articulation& articulation, 
										FsData &data,
										 PxU16 linkCount,
										 const ArticulationLink* links,
										 PxReal compliance,
										 PxU32 iterations,
										 char* scratchMemory,
										 PxU32 scratchMemorySize);*/

	//static PxU32	getDriveCacheLinkCount(const FsData& cache);

	/*static void		applyImpulses(const FsData& matrix,
								  Cm::SpatialVectorV* Z,
								  Cm::SpatialVectorV* V);*/

//private:
	static PxU32	getLtbDataSize(PxU32 linkCount);
	static PxU32	getFsDataSize(PxU32 linkCount);

	//static void		prepareDataBlock(ArticulationV& articualtion,
	//								 const ArticulationLink* links,
	//								 PxU16 linkCount,	
	//								 PxTransform* poses,
	//							 	 FsInertia *baseInertia,
	//								 ArticulationJointTransforms* jointTransforms,
	//								 PxU32 expectedSize);

	//static void		setInertia(FsInertia& inertia,
	//						   const PxsBodyCore& body,
	//						   const PxTransform& pose);

	//static void		setJointTransforms(ArticulationJointTransforms& transforms,
	//								   const PxTransform& parentPose,
	//								   const PxTransform& childPose,
	//								   const ArticulationJointCore& joint);

	/*static void		prepareLtbMatrix(FsData& fsData,
									 const FsInertia* baseInertia,
									 const PxTransform* poses,
									 const ArticulationJointTransforms* jointTransforms,
									 PxReal recipDt);*/

	//static void		prepareFsData(FsData& fsData,
	//							  const ArticulationLink* links);

	//static PX_FORCE_INLINE PxReal getResistance(PxReal compliance);


	static void		createHardLimit(const FsData& fsData,
									const ArticulationLink* links,
									PxU32 linkIndex,
									SolverConstraint1DExt& s, 
									const PxVec3& axis, 
									PxReal err,
									PxReal recipDt);

	static void		createTangentialSpring(const FsData& fsData,
										   const ArticulationLink* links,
										   PxU32 linkIndex,
										   SolverConstraint1DExt& s, 
										   const PxVec3& axis, 
										   PxReal stiffness,
										   PxReal damping,
										   PxReal dt);

	static void		createHardLimitTGS(const FsData& fsData,
		const ArticulationLink* links,
		PxU32 linkIndex,
		SolverConstraint1DExtStep& s,
		const PxVec3& axis,
		PxReal err,
		PxReal recipDt);

	static void		createTangentialSpringTGS(const FsData& fsData,
		const ArticulationLink* links,
		PxU32 linkIndex,
		SolverConstraint1DExtStep& s,
		const PxVec3& axis,
		PxReal stiffness,
		PxReal damping,
		PxReal dt);

	static PxU32 setupSolverConstraints(Articulation& articulation, PxU32 solverDataSize,
													PxConstraintAllocator& allocator,
													PxSolverConstraintDesc* constraintDesc,
													const ArticulationLink* links,
													const ArticulationJointTransforms* jointTransforms,
													PxReal dt,
													PxU32& acCount);

	/*static void	computeUnconstrainedVelocitiesInternal(const ArticulationSolverDesc& desc,
		PxReal dt,
		const PxVec3& gravity, PxU64 contextID,
		FsInertia* PX_RESTRICT baseInertia,
		ArticulationJointTransforms* PX_RESTRICT jointTransforms,
		PxcFsScratchAllocator& allocator);*/

	/*static void		computeJointDrives(FsData& fsData,
									   Ps::aos::Vec3V* drives, 
									   const ArticulationLink* links,
									   const PxTransform* poses, 
									   const ArticulationJointTransforms* transforms, 
									   const Ps::aos::Mat33V* loads, 
									   PxReal dt);*/

};

}

}

#endif //DY_ARTICULATION_HELPER_H
