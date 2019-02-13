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


#ifndef PXDV_ARTICULATION_H
#define PXDV_ARTICULATION_H

#include "foundation/PxVec3.h"
#include "foundation/PxQuat.h"
#include "foundation/PxTransform.h"
#include "PsVecMath.h"
#include "CmUtils.h"
#include "CmSpatialVector.h"
#include "PxArticulationJoint.h"
#include "PxArticulation.h"
#include "foundation/PxMemory.h"
#include "DyArticulationCore.h"
#include "DyArticulationJointCore.h"


namespace physx
{

	class PxcRigidBody;
	struct PxsBodyCore;

	class PxcConstraintBlockStream;
	class PxcRigidBody;
	class PxsConstraintBlockManager;
	struct PxSolverConstraintDesc;
	

	namespace Sc
	{
		class ArticulationSim;
	}


	namespace Dy
	{

		struct PxcFsScratchAllocator;
		struct PxTGSSolverConstraintDesc;

		static const size_t DY_ARTICULATION_MAX_SIZE = 64;
		struct ArticulationJointTransforms;
		class ArticulationJointCoreData;
		struct Constraint;
		class Context;

		struct ArticulationJointCore : public ArticulationJointCoreBase
		{
			//= ATTENTION! =====================================================================================
			// Changing the data layout of this class breaks the binary serialization format.  See comments for 
			// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
			// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
			// accordingly.
			//==================================================================================================

			// drive model
			PxQuat			targetPosition;
			PxVec3			targetVelocity;

			PxReal			spring;//old
			PxReal			damping;//old

			PxReal			internalCompliance;//old
			PxReal			externalCompliance;//old

			// limit model

			PxReal			swingLimitContactDistance;//old

			PxReal			tangentialStiffness;//old
			PxReal			tangentialDamping;//old

			bool			swingLimited;//old
			bool			twistLimited;//old

			PxU8			driveType; //both

			PxReal			twistLimitContactDistance; //old

			PxReal			tanQSwingY;//old
			PxReal			tanQSwingZ;//old
			PxReal			tanQSwingPad;//old
			PxReal			tanQTwistHigh;//old
			PxReal			tanQTwistLow;//old
			PxReal			tanQTwistPad;//old

			ArticulationJointCore()
			{
				//Cm::markSerializedMem(this, sizeof(ArticulationJointCore));
				parentPose = PxTransform(PxIdentity);
				childPose = PxTransform(PxIdentity);
				internalCompliance = 0;
				externalCompliance = 0;
				swingLimitContactDistance = 0.05f;
				twistLimitContactDistance = 0.05f;

				driveType = PxArticulationJointDriveType::eTARGET;

				jointType = PxArticulationJointType::eFIX;

				for (PxU32 i = 0; i < 6; ++i)
				{
					motion[i] = PxArticulationMotion::eLOCKED;
				}

				dirtyFlag = ArticulationJointCoreDirtyFlag::eMOTION;

				//initial parent to child
				relativeQuat = PxQuat(PxIdentity);

				jointOffset = 0;

			}

			ArticulationJointCore(const PxTransform& parentFrame,
				const PxTransform& childFrame)
			{
				parentPose = parentFrame;
				childPose = childFrame;

				//the old articulation is eTARGET
				driveType = PxArticulationJointDriveType::eTARGET;

				spring = 0.0f;
				damping = 0.0f;

				internalCompliance = 1.0f;
				externalCompliance = 1.0f;

				for (PxU32 i = 0; i < 6; ++i)
				{
					limits[i].low = 0.f;
					limits[i].high = 0.f;
					drives[i].stiffness = 0.f;
					drives[i].damping = 0.f;
					drives[i].maxForce = 0.f;
					targetP[i] = 0.f;
					targetV[i] = 0.f;
				}


				PxReal swingYLimit = PxPi / 4;
				PxReal swingZLimit = PxPi / 4;

				limits[PxArticulationAxis::eSWING1].low = swingYLimit;
				limits[PxArticulationAxis::eSWING2].low = swingZLimit;

				swingLimitContactDistance = 0.05f;
				swingLimited = false;
				tangentialStiffness = 0.0f;
				tangentialDamping = 0.0f;

				PxReal twistLimitLow = -PxPi / 4;
				PxReal twistLimitHigh = PxPi / 4;
				limits[PxArticulationAxis::eTWIST].low = twistLimitLow;
				limits[PxArticulationAxis::eTWIST].high = twistLimitHigh;
				twistLimitContactDistance = 0.05f;
				twistLimited = false;


				tanQSwingY = PxTan(swingYLimit / 4);
				tanQSwingZ = PxTan(swingZLimit / 4);
				tanQSwingPad = PxTan(swingLimitContactDistance / 4);

				tanQTwistHigh = PxTan(twistLimitHigh / 4);
				tanQTwistLow = PxTan(twistLimitLow / 4);
				tanQTwistPad = PxTan(twistLimitContactDistance / 4);

				

				prismaticLimited = false;

				frictionCoefficient = 0.f;

				for (PxU32 i = 0; i < 6; ++i)
				{
					motion[i] = PxArticulationMotion::eLOCKED;
				}

				dirtyFlag = ArticulationJointCoreDirtyFlag::eMOTION;

				//initial parent to child
				relativeQuat = (childFrame.q * (parentFrame.q.getConjugate())).getNormalized();

				jointOffset = 0;

			}

			void setJointPose(ArticulationJointCoreData& jointDatum);

			// PX_SERIALIZATION
			ArticulationJointCore(const PxEMPTY&) {}
			//~PX_SERIALIZATION

		};

		struct ArticulationLoopConstraint
		{
		public:
			PxU32 linkIndex0;
			PxU32 linkIndex1;
			Dy::Constraint* constraint;

		};

#define DY_ARTICULATION_LINK_NONE 0xffffffff

		typedef PxU64 ArticulationBitField;

		struct ArticulationLink
		{
			ArticulationBitField			children;		// child bitmap
			ArticulationBitField			pathToRoot;		// path to root, including link and root
			PxcRigidBody*					body;
			PxsBodyCore*					bodyCore;
			ArticulationJointCore*			inboundJoint;
			PxU32							parent;
		};

		typedef size_t ArticulationLinkHandle;

		class ArticulationV;

		struct ArticulationSolverDesc
		{
			ArticulationV*				articulation;
			ArticulationLink*			links;
			Cm::SpatialVectorV*			motionVelocity;
			Cm::SpatialVector*			acceleration;
			PxTransform*				poses;
			PxQuat*						deltaQ;
			physx::shdfnd::aos::Mat33V* externalLoads;
			physx::shdfnd::aos::Mat33V* internalLoads;
			const ArticulationCore*		core;
			char*						scratchMemory;
			PxU16						totalDataSize;
			PxU16						solverDataSize;
			PxU8						linkCount;
			PxU8						numInternalConstraints;
			PxU16						scratchMemorySize;
		};

		struct PxcFsScratchAllocator
		{
			char*   base;
			size_t	size;
			size_t	taken;
			PxcFsScratchAllocator(char* p, size_t s) : base(p), size(s), taken(0) {}

			template<typename T>
			static size_t sizeof16()
			{
				return (sizeof(T) + 15)&~15;
			}

			template<class T> T* alloc(PxU32 count)
			{
				size_t s = sizeof16<T>();
				PX_ASSERT(taken + s*count <= size);
				T* result = reinterpret_cast<T*>(base + taken);
				taken += s*count;
				return result;
			}
		};


		static const size_t DY_ARTICULATION_IDMASK = DY_ARTICULATION_MAX_SIZE - 1;

#if PX_VC 
#pragma warning(push)   
#pragma warning( disable : 4324 ) // Padding was added at the end of a structure because of a __declspec(align) value.
#endif
		PX_ALIGN_PREFIX(64)
		class ArticulationV
		{
		public:

			// public interface

			ArticulationV(Sc::ArticulationSim* sim, PxArticulationBase::Enum type) : mArticulationSim(sim),
				mContext(NULL),
				mType(type),
				mUpdateSolverData(true),
				mDirty(false),
				mMaxDepth(0)
			{}

			virtual ~ArticulationV() {}

			virtual	void onUpdateSolverDesc()
			{
			}

			virtual bool resize(const PxU32 linkCount);

			virtual void addBody()
			{
				mAcceleration.pushBack(Cm::SpatialVector(PxVec3(0.f), PxVec3(0.f)));
				mUpdateSolverData = true;    
			}

			virtual void removeBody()
			{
				mUpdateSolverData = true;
			}

			bool updateSolverData() {return mUpdateSolverData;}

			void setDirty(const bool dirty) { mDirty = dirty; }
			bool getDirty() { return mDirty; }

			PX_FORCE_INLINE PxU32	getMaxDepth() { return mMaxDepth; }
			PX_FORCE_INLINE void	setMaxDepth(const PxU32	depth) { mMaxDepth = depth; }

			// solver methods
			PX_FORCE_INLINE PxU32					getLinkIndex(ArticulationLinkHandle handle)	const { return PxU32(handle&DY_ARTICULATION_IDMASK); }
			PX_FORCE_INLINE PxU32					getBodyCount()									const { return mSolverDesc.linkCount; }
			PX_FORCE_INLINE PxU32					getSolverDataSize()								const { return mSolverDesc.solverDataSize; }
			PX_FORCE_INLINE PxU32					getTotalDataSize()								const { return mSolverDesc.totalDataSize; }
			PX_FORCE_INLINE void					getSolverDesc(ArticulationSolverDesc& d)		const { d = mSolverDesc; }
			PX_FORCE_INLINE ArticulationSolverDesc& getSolverDesc()										  { return mSolverDesc; }

			PX_FORCE_INLINE const ArticulationCore*	getCore()										const { return mSolverDesc.core; }
			PX_FORCE_INLINE PxU16					getIterationCounts()							const { return mSolverDesc.core->solverIterationCounts; }

			PX_FORCE_INLINE Sc::ArticulationSim*	getArticulationSim()							const { return mArticulationSim; }

			PX_FORCE_INLINE PxU32			getType() const { return mType; }

			// get data sizes for allocation at higher levels
			virtual void		getDataSizes(PxU32 linkCount,
				PxU32 &solverDataSize,
				PxU32& totalSize,
				PxU32& scratchSize) = 0;

			virtual PxU32	getDofs() { return 0; }

			virtual PxU32	getDof(const PxU32 /*linkID*/) { return 0;  }

			virtual void	applyCache(PxArticulationCache& /*cache*/, const PxArticulationCacheFlags /*flag*/) {}

			virtual void	copyInternalStateToCache(PxArticulationCache&/* cache*/, const PxArticulationCacheFlags /*flag*/) {}

			virtual	void	packJointData(const PxReal* /*maximum*/, PxReal* /*reduced*/) {}

			virtual void	unpackJointData(const PxReal* /*reduced*/, PxReal* /*maximum*/) {}

			virtual void	initializeCommonData() {}

			virtual void	getGeneralizedGravityForce(const PxVec3& /*gravity*/, PxArticulationCache& /*cache*/) {}

			virtual void	getCoriolisAndCentrifugalForce(PxArticulationCache& /*cache*/) {}

			virtual void	getGeneralizedExternalForce(PxArticulationCache& /*cache*/) {}

			virtual void	getJointAcceleration(const PxVec3& /*gravity*/, PxArticulationCache& /*cache*/){}

			virtual void	getJointForce(PxArticulationCache& /*cache*/){}

			virtual void	getKinematicJacobian(const PxU32 /*linkID*/, PxArticulationCache& /*cache*/){}

			virtual void	getCoefficentMatrix(const PxReal /*dt*/, const PxU32 /*linkID*/, const PxContactJoint* /*joints*/, const PxU32 /*nbContacts*/, PxArticulationCache& /*cache*/){}

			virtual void	getCoefficentMatrixWithLoopJoints(ArticulationLoopConstraint* /*lConstraints*/, const PxU32 /*nbJoints*/, PxArticulationCache& /*cache*/) {}
			
			virtual bool	getLambda(ArticulationLoopConstraint* /*lConstraints*/, const PxU32 /*nbJoints*/, 
				PxArticulationCache& /*cache*/, PxArticulationCache& /*initialState*/, const PxReal* /*jointTorque*/, 
				const PxVec3& /*gravity*/, const PxU32 /*maxIter*/) { return false;  }

			virtual void	getGeneralizedMassMatrix(PxArticulationCache& /*cache*/){}
			virtual void	getGeneralizedMassMatrixCRB(PxArticulationCache& /*cache*/){}

			virtual void	teleportRootLink(){}

			virtual	void	getImpulseResponse(
				PxU32 linkID,
				Cm::SpatialVectorF* Z,
				const Cm::SpatialVector& impulse,
				Cm::SpatialVector& deltaV) const = 0;

			virtual	void	getImpulseSelfResponse(
				PxU32 linkID0,
				PxU32 linkID1,
				Cm::SpatialVectorF* Z,
				const Cm::SpatialVector& impulse0,
				const Cm::SpatialVector& impulse1,
				Cm::SpatialVector& deltaV0,
				Cm::SpatialVector& deltaV1) const = 0;


			virtual Cm::SpatialVectorV getLinkVelocity(const PxU32 linkID) const = 0;

			virtual Cm::SpatialVectorV getLinkMotionVector(const PxU32 linkID) const = 0;

			virtual PxReal getLinkMaxPenBias(const PxU32 linkID) const = 0;

			virtual void pxcFsApplyImpulse(PxU32 linkID, Ps::aos::Vec3V linear, 
				Ps::aos::Vec3V angular, Cm::SpatialVectorF* Z, Cm::SpatialVectorF* deltaV) = 0;

			virtual void pxcFsApplyImpulses(PxU32 linkID, const Ps::aos::Vec3V& linear,
				const Ps::aos::Vec3V& angular, PxU32 linkID2, const Ps::aos::Vec3V& linear2,
				const Ps::aos::Vec3V& angular2, Cm::SpatialVectorF* Z, Cm::SpatialVectorF* deltaV) = 0;

			virtual void solveInternalConstraints(const PxReal dt, const PxReal invDt, Cm::SpatialVectorF* impulses, Cm::SpatialVectorF* DeltaV,
				bool velIteration) = 0;

			virtual Cm::SpatialVectorV pxcFsGetVelocity(PxU32 linkID) = 0;

			virtual void pxcFsGetVelocities(PxU32 linkID, PxU32 linkID1, Cm::SpatialVectorV& v0, Cm::SpatialVectorV& v1) = 0;

			virtual Cm::SpatialVectorV pxcFsGetVelocityTGS(PxU32 linkID) = 0;

			virtual const PxTransform& getCurrentTransform(PxU32 linkID) const= 0;
			
			virtual const PxQuat& getDeltaQ(PxU32 linkID) const = 0;

			//this is called by island gen to determine whether the articulation should be awake or sleep
			virtual Cm::SpatialVector getMotionVelocity(const PxU32 linkID) const = 0;

			void	setDyContext(Dy::Context* context) { mContext = context;  }
			//These variables are used in the constraint partition
			PxU16 maxSolverFrictionProgress;
			PxU16 maxSolverNormalProgress;
			PxU32 solverProgress;
			PxU8  numTotalConstraints;

		protected:
			Sc::ArticulationSim*				mArticulationSim;
			Dy::Context*						mContext;
			PxU32								mType;
			ArticulationSolverDesc				mSolverDesc;

			Ps::Array<Cm::SpatialVector>		mAcceleration;		// supplied by Sc-layer to feed into articulations

			bool								mUpdateSolverData;
			bool								mDirty; //any of links update configulations, the boolean will be set to true
			PxU32								mMaxDepth;
			
			private:
				PX_NOCOPY(ArticulationV)
		} PX_ALIGN_SUFFIX(64);

#if PX_VC 
#pragma warning(pop) 
#endif

		PX_FORCE_INLINE ArticulationV* getArticulation(ArticulationLinkHandle handle)
		{
			return reinterpret_cast<ArticulationV*>(handle & ~DY_ARTICULATION_IDMASK);
		}

		PX_FORCE_INLINE bool isArticulationRootLink(ArticulationLinkHandle handle)
		{
			return !(handle & DY_ARTICULATION_IDMASK);
		}


	}

}

#endif
