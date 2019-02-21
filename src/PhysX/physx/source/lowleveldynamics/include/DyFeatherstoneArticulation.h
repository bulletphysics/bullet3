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


#ifndef PXD_FEATHERSTONE_ARTICULATION_H
#define PXD_FEATHERSTONE_ARTICULATION_H

#include "foundation/PxVec3.h"
#include "foundation/PxQuat.h"
#include "foundation/PxTransform.h"
#include "PsVecMath.h"
#include "CmUtils.h"
#include "PxArticulationJoint.h"
#include "DyVArticulation.h"
#include "DyFeatherstoneArticulationUtils.h"
#include "DyFeatherstoneArticulationJointData.h"

#ifndef FEATHERSTONE_DEBUG
#define FEATHERSTONE_DEBUG 0
#endif

namespace physx
{
	class PxContactJoint;
	struct PxSolverConstraintDesc;
	class PxcConstraintBlockStream;
	class PxcScratchAllocator;
	class PxsConstraintBlockManager;
	struct SolverConstraint1DExtStep;
	struct PxSolverConstraintPrepDesc;
	struct PxSolverBody;
	struct PxSolverBodyData;
	class PxConstraintAllocator;
	
namespace Dy
{
//#if PX_VC 
//#pragma warning(push)   
//#pragma warning( disable : 4324 ) // Padding was added at the end of a structure because of a __declspec(align) value.
//#endif

	struct PxTGSSolverConstraintDesc;
	//class ArticulationJointCoreData;
	class ArticulationLinkData;
	struct SpatialSubspaceMatrix;
	struct SolverConstraint1DExt;
	struct SolverConstraint1DStep;

	class FeatherstoneArticulation;
	struct SpatialMatrix;
	struct SpatialTransform;
	struct Constraint;

/*	struct DyScratchAllocator
	{
		char*   base;
		size_t	size;
		size_t	taken;

		DyScratchAllocator(char* p, size_t s) : base(p), size(s), taken(0) {}

		PxU32 round16(PxU32 size_) { return (size_ + 15)&(~15); }

		template<class T> T* alloc(PxU32 count)
		{
			const PxU32 sizeReq = round16(sizeof(T)*count);
			PX_ASSERT((taken+ sizeReq) < size);
			T* result = reinterpret_cast<T*>(base + taken);
			taken += sizeReq;
			return result;
		}
	};*/

	//This stuct is used in TGS
/*	class ArticulationTempData
	{
	public:
		Cm::SpatialVectorF					mBaseLinkMotionVelocite;
		Ps::Array<Cm::SpatialVectorF>		mZAForce;
		Ps::Array<PxReal>					mJointPosition;			//	joint position
		Ps::Array<PxReal>					mJointVelocity;			//	joint velocity
		Ps::Array<PxTransform>				mLinkTransform;			//	this is the transform list for links
	};*/

	struct ArticulationInternalConstraint
	{
		//Common/shared directional info between, frictions and drives
		Cm::UnAlignedSpatialVector row0;		//24	24
		Cm::UnAlignedSpatialVector row1;		//24	48
		Cm::UnAlignedSpatialVector deltaVA;		//24	72	
		Cm::UnAlignedSpatialVector deltaVB;		//24	96
		//Response information
		PxReal recipResponse;					//4		100
		PxReal response;						//4		104
		//Joint limit values	
		PxReal lowLimit;						//4		108
		PxReal highLimit;						//4		112
		PxReal lowImpulse;						//4		116		changed
		PxReal highImpulse;						//4		120		changed
		PxReal erp;								//4		124
		//Joint spring drive info
		PxReal driveTargetVel;					//4		128
		PxReal driveBiasCoefficient;			//4		132
		PxReal driveTarget;						//4		136
		PxReal driveVelMultiplier;				//4		140
		PxReal driveImpulseMultiplier;			//4		144
		PxReal maxDriveForce;					//4		148
		PxReal driveForce;						//4		152

		PxReal maxFrictionForce;				//4		156
		PxReal frictionForce;					//4		160
		PxReal frictionForceCoefficient;		//4		164

		bool isLinearConstraint;				//1		165
		PxU8 padding[11];						//11	176
	};

	struct ArticulationInternalLockedAxis
	{
		//How much an impulse will effect angular velocity
		Cm::UnAlignedSpatialVector deltaVA;		//24	24
		Cm::UnAlignedSpatialVector deltaVB;		//24	48
		//Jacobian axis that is locked		
		PxVec3 axis;							//12	60
		//Response information	
		PxReal recipResponse;					//4		64
		//Initial error
		PxReal error;							//4		68
		//Bias scale
		PxReal biasScale;						//4		72

		PxU32 pad[2];							//4		80
	};

	class ArticulationData
	{
	public:

		ArticulationData() :  
			mLinksData(NULL), mJointData(NULL), 
			mDofs(0xffffffff), mLocks(0), mDataDirty(true)
		{
		}

		~ArticulationData();

		PX_FORCE_INLINE	void						init();
						void						resizeLinkData(const PxU32 linkCount);
						void						resizeJointData(const PxU32 dofs);
		
		PX_FORCE_INLINE PxReal*						getJointAccelerations()									{ return mJointAcceleration.begin();		}
		PX_FORCE_INLINE PxReal*						getJointVelocities()									{ return mJointVelocity.begin();			}
		PX_FORCE_INLINE PxReal*						getJointDeltaVelocities()								{ return mJointDeltaVelocity.begin();		}
		PX_FORCE_INLINE PxReal*						getJointPositions()										{ return mJointPosition.begin();			}
		PX_FORCE_INLINE PxReal*						getJointForces()										{ return mJointForce.begin();				}
		//PX_FORCE_INLINE PxReal*					getJointFrictionForces() { return mJointFrictionForce.begin(); }

		PX_FORCE_INLINE ArticulationInternalConstraint&			getInternalConstraint(const PxU32 dofId)		{ return mInternalConstraints[dofId];	}
		PX_FORCE_INLINE const ArticulationInternalConstraint&	getInternalConstraint(const PxU32 dofId) const	{ return mInternalConstraints[dofId];	}
		
		PX_FORCE_INLINE ArticulationInternalLockedAxis&			getInternalLocks(const PxU32 dofId)			{ return mInternalLockedAxes[dofId];		}
		PX_FORCE_INLINE const ArticulationInternalLockedAxis&	getInternalLocks(const PxU32 dofId) const	{ return mInternalLockedAxes[dofId];		}

		PX_FORCE_INLINE Cm::SpatialVectorF*			getMotionVelocities()									{ return mMotionVelocities.begin();			}
		PX_FORCE_INLINE Cm::SpatialVectorF*			getMotionAccelerations()								{ return mMotionAccelerations.begin();		}
		PX_FORCE_INLINE Cm::SpatialVectorF*			getCorioliseVectors()									{ return mCorioliseVectors.begin();			}
		PX_FORCE_INLINE Cm::SpatialVectorF*			getSpatialZAVectors()									{ return mZAForces.begin();					}
		PX_FORCE_INLINE Cm::SpatialVectorF*			getTransmittedForces()									{ return mJointTransmittedForce.begin();	}

		PX_FORCE_INLINE Cm::SpatialVectorF*			getPosIterMotionVelocities()							{ return mPosIterMotionVelocities.begin();	}
		PX_FORCE_INLINE PxReal*						getPosIterJointDeltaVelocities()						{ return mPosIterJointDeltaVelocities.begin(); }
	
		PX_FORCE_INLINE Cm::SpatialVectorF&			getPosIterMotionVelocity(const PxU32 index)				{ return mPosIterMotionVelocities[index];	}
		PX_FORCE_INLINE const Cm::SpatialVectorF&	getMotionVelocity(const PxU32 index)			const	{ return mMotionVelocities[index];			}
		PX_FORCE_INLINE const Cm::SpatialVectorF&	getMotionAcceleration(const PxU32 index)		const	{ return mMotionAccelerations[index];		}
		PX_FORCE_INLINE const Cm::SpatialVectorF&	getCorioliseVector(const PxU32 index)			const	{ return mCorioliseVectors[index];			}
		PX_FORCE_INLINE const Cm::SpatialVectorF&	getSpatialZAVector(const PxU32 index)			const	{ return mZAForces[index];					}
		PX_FORCE_INLINE const Cm::SpatialVectorF&	getTransmittedForce(const PxU32 index)			const	{ return mJointTransmittedForce[index];		}

		PX_FORCE_INLINE Cm::SpatialVectorF&			getMotionVelocity(const PxU32 index)					{ return mMotionVelocities[index];			}
		PX_FORCE_INLINE Cm::SpatialVectorF&			getMotionAcceleration(const PxU32 index)				{ return mMotionAccelerations[index];		}
		PX_FORCE_INLINE Cm::SpatialVectorF&			getCorioliseVector(const PxU32 index)					{ return mCorioliseVectors[index];			}
		PX_FORCE_INLINE Cm::SpatialVectorF&			getSpatialZAVector(const PxU32 index)					{ return mZAForces[index];					}
		PX_FORCE_INLINE Cm::SpatialVectorF&			getTransmittedForce(const PxU32 index)					{ return mJointTransmittedForce[index];		}

		//PX_FORCE_INLINE Dy::SpatialMatrix*			getTempSpatialMatrix() { mTempSpatialMatrix.begin(); }

		PX_FORCE_INLINE PxTransform&				getPreTransform(const PxU32 index)						{ return mPreTransform[index];				}
		PX_FORCE_INLINE const PxTransform&			getPreTransform(const PxU32 index)				const	{ return mPreTransform[index];				}
		PX_FORCE_INLINE void						setPreTransform(const PxU32 index, const PxTransform& t){ mPreTransform[index] = t;					}
		PX_FORCE_INLINE PxTransform*				getPreTransform()										{ return mPreTransform.begin();				}

		PX_FORCE_INLINE const Cm::SpatialVectorF&	getDeltaMotionVector(const PxU32 index) const			{ return mDeltaMotionVector[index];			}
		PX_FORCE_INLINE void						setDeltaMotionVector(const PxU32 index, const Cm::SpatialVectorF& vec) { mDeltaMotionVector[index] = vec; }
		PX_FORCE_INLINE Cm::SpatialVectorF*			getDeltaMotionVector()									{ return mDeltaMotionVector.begin();		}

		PX_FORCE_INLINE ArticulationLink*			getLinks()										const	{ return mLinks;							}
		PX_FORCE_INLINE PxU32						getLinkCount()									const	{ return mLinkCount;						}

		PX_FORCE_INLINE ArticulationLink&			getLink(PxU32 index)							const	{ return mLinks[index];						}

		PX_FORCE_INLINE ArticulationLinkData*		getLinkData()									const	{ return mLinksData;						}
						ArticulationLinkData&		getLinkData(PxU32 index)						const;

		PX_FORCE_INLINE ArticulationJointCoreData*	getJointData()									const	{ return mJointData;						}
						ArticulationJointCoreData&	getJointData(PxU32 index)						const	{ return mJointData[index];					}

		PX_FORCE_INLINE const ArticulationCore*		getCore()										const	{ return mCore;								}
		PX_FORCE_INLINE Cm::SpatialVector*			getExternalAccelerations()								{ return mExternalAcceleration;				}

		PX_FORCE_INLINE PxU32						getSolverDataSize()								const	{ return mSolverDataSize;					}
		PX_FORCE_INLINE Cm::SpatialVector&			getExternalAcceleration(const PxU32 linkID)				{ return mExternalAcceleration[linkID];		}

		PX_FORCE_INLINE PxReal						getDt()											const	{ return mDt;								}
		PX_FORCE_INLINE void						setDt(const PxReal dt)									{ mDt = dt;									}

		PX_FORCE_INLINE bool						getDataDirty()									const	{ return mDataDirty;						}
		PX_FORCE_INLINE void						setDataDirty(const bool dirty)							{ mDataDirty = dirty;						}

		PX_FORCE_INLINE	PxU32						getDofs()										const	{ return mDofs;								}
		PX_FORCE_INLINE	void						setDofs(const PxU32 dof)								{ mDofs = dof;								}

		PX_FORCE_INLINE	PxU32						getLocks()										const	{ return mLocks;							}
		PX_FORCE_INLINE	void						setLocks(const PxU32 locks)								{ mLocks = locks;							}

		PX_FORCE_INLINE FeatherstoneArticulation*	getArticulation()										{ return mArticulation;						}
		PX_FORCE_INLINE void						setArticulation(FeatherstoneArticulation* articulation)	{ mArticulation = articulation;				}
		
		PX_FORCE_INLINE const SpatialMatrix&		getBaseInvSpatialArticulatedInertia()			const	{ return mBaseInvSpatialArticulatedInertia;	}

		PX_FORCE_INLINE PxTransform*				getAccumulatedPoses()									{ return mAccumulatedPoses.begin();			}
		PX_FORCE_INLINE const PxTransform*			getAccumulatedPoses()							const	{ return mAccumulatedPoses.begin();			}

	private:
		Ps::Array<PxReal>							mJointAcceleration;		//	joint acceleration
		Ps::Array<PxReal>							mJointVelocity;			//	joint velocity
		Ps::Array<PxReal>							mJointDeltaVelocity;	//	joint velocity change due to contacts
		Ps::Array<PxReal>							mJointPosition;			//	joint position
		Ps::Array<PxReal>							mJointForce;			//	joint force
		//Ps::Array<PxReal>							mJointFrictionForce;	//	joint friction force
	
		Ps::Array<PxReal>							mPosIterJointDeltaVelocities;	//joint delta velocity after postion iternation before velocity iteration
		Ps::Array<Cm::SpatialVectorF>				mPosIterMotionVelocities;	//link motion velocites after position iteration before velocity iteration
		Ps::Array<Cm::SpatialVectorF>				mMotionVelocities;		//link motion velocites
		Ps::Array<Cm::SpatialVectorF>				mMotionAccelerations;	//link motion accelerations
		Ps::Array<Cm::SpatialVectorF>				mCorioliseVectors;		//link coriolise vector
		Ps::Array<Cm::SpatialVectorF>				mZAForces;				//link spatial zero acceleration force/ spatial articulated force
		Ps::Array<Cm::SpatialVectorF>				mJointTransmittedForce; 
		Ps::Array<ArticulationInternalConstraint>	mInternalConstraints;
		Ps::Array<ArticulationInternalLockedAxis>	mInternalLockedAxes;

		Ps::Array<Cm::SpatialVectorF>				mDeltaMotionVector; //this is for TGS solver
		Ps::Array<PxTransform>						mPreTransform; //this is the previous transform list for links

		//Ps::Array<SpatialMatrix>					mTempSpatialMatrix;

		ArticulationLink*							mLinks;
		PxU32										mLinkCount;
		ArticulationLinkData*						mLinksData;
		ArticulationJointCoreData*					mJointData;
		PxReal										mDt;
		PxU32										mDofs;
		PxU32										mLocks;
		const ArticulationCore*						mCore;
		Cm::SpatialVector*							mExternalAcceleration;
		PxU32										mSolverDataSize;
		bool										mDataDirty; //this means we need to call commonInit()
		bool										mJointDirty; //this means joint delta velocity has been changed by contacts so we need to update joint velocity/joint acceleration 
		FeatherstoneArticulation*					mArticulation;

		Ps::Array<PxTransform>						mAccumulatedPoses;
		Ps::Array<PxQuat>							mDeltaQ;
		PxReal										mAccumulatedDt;

		//ArticulationTempData						mTempData;
		SpatialMatrix								mBaseInvSpatialArticulatedInertia;

		friend class FeatherstoneArticulation;
	};

	void ArticulationData::init()
	{
		//zero delta motion vector for TGS solver
		PxMemZero(getDeltaMotionVector(), sizeof(Cm::SpatialVectorF) * mLinkCount);

		//zero joint velocity delta, which will be changed in pxcFsApplyImpulse if there are any contact with links
		PxMemZero(getJointDeltaVelocities(), sizeof(PxReal) * mDofs);
		mJointDirty = false;
	}

	struct ScratchData
	{
	public:
		ScratchData()
		{
			motionVelocities = NULL;
			motionAccelerations = NULL;
			coriolisVectors = NULL;
			spatialZAVectors = NULL;
			externalAccels = NULL;
			compositeSpatialInertias = NULL;

			jointVelocities = NULL;
			jointAccelerations = NULL;
			jointForces = NULL;
			jointPositions = NULL;
			jointFrictionForces = NULL;
		}

		Cm::SpatialVectorF* motionVelocities;
		Cm::SpatialVectorF* motionAccelerations;
		Cm::SpatialVectorF* coriolisVectors;
		Cm::SpatialVectorF* spatialZAVectors;
		Cm::SpatialVector*	externalAccels;
		Dy::SpatialMatrix*	compositeSpatialInertias;

		PxReal*				jointVelocities;
		PxReal*				jointAccelerations;
		PxReal*				jointForces;
		PxReal*				jointPositions;
		PxReal*				jointFrictionForces;
	};

#if PX_VC 
#pragma warning(push)   
#pragma warning( disable : 4324 ) // Padding was added at the end of a structure because of a __declspec(align) value.
#endif

	PX_ALIGN_PREFIX(64)
		class FeatherstoneArticulation : public ArticulationV
	{
	public:
		// public interface

		FeatherstoneArticulation(Sc::ArticulationSim*);
		~FeatherstoneArticulation();

		// get data sizes for allocation at higher levels
		virtual void		getDataSizes(PxU32 linkCount, PxU32& solverDataSize, PxU32& totalSize, PxU32& scratchSize);

		virtual bool		resize(const PxU32 linkCount);

		virtual	void		onUpdateSolverDesc();

		virtual PxU32		getDofs();

		virtual PxU32		getDof(const PxU32 linkID);

		virtual void		applyCache(PxArticulationCache& cache, const PxArticulationCacheFlags flag);

		virtual void		copyInternalStateToCache(PxArticulationCache& cache, const PxArticulationCacheFlags flag);

		virtual	void		packJointData(const PxReal* maximum, PxReal* reduced);

		virtual void		unpackJointData(const PxReal* reduced, PxReal* maximum);

		virtual void		initializeCommonData();

		//gravity as input, joint force as output
		virtual void		getGeneralizedGravityForce(const PxVec3& gravity, PxArticulationCache& cache);

		//joint velocity as input, generalised force(coriolis and centrigugal force) as output
		virtual void		getCoriolisAndCentrifugalForce(PxArticulationCache& cache);

		//external force as input, joint force as output
		virtual void		getGeneralizedExternalForce(PxArticulationCache& /*cache*/);

		//joint force as input, joint acceleration as output
		virtual void		getJointAcceleration(const PxVec3& gravity, PxArticulationCache& cache);

		//joint acceleration as input, joint force as out
		virtual void		getJointForce(PxArticulationCache& cache);

		virtual void		getKinematicJacobian(const PxU32 linkID, PxArticulationCache& cache);

		//These two functions are for closed loop system
		void				getKMatrix(ArticulationJointCore* loopJoint, const PxU32 parentIndex, const PxU32 childIndex, PxArticulationCache& cache);

		virtual void		getCoefficentMatrix(const PxReal dt, const PxU32 linkID, const PxContactJoint* contactJoints, const PxU32 nbContacts, PxArticulationCache& cache);

		virtual void		getCoefficentMatrixWithLoopJoints(ArticulationLoopConstraint* lConstraints, const PxU32 nbJoints, PxArticulationCache& cache);

		virtual bool		getLambda(ArticulationLoopConstraint* lConstraints, const PxU32 nbJoints, PxArticulationCache& cache, PxArticulationCache& rollBackCache, 
			const PxReal* jointTorque, const PxVec3& gravity, const PxU32 maxIter);

		virtual void		getGeneralizedMassMatrix(PxArticulationCache& cache);

		virtual void		getGeneralizedMassMatrixCRB(PxArticulationCache& cache);

		virtual void		teleportRootLink();

		virtual	void		getImpulseResponse(
			PxU32 linkID,
			Cm::SpatialVectorF* Z,
			const Cm::SpatialVector& impulse,
			Cm::SpatialVector& deltaV) const;

		virtual void		getImpulseSelfResponse(
			PxU32 linkID0,
			PxU32 linkID1,
			Cm::SpatialVectorF* Z,
			const Cm::SpatialVector& impulse0,
			const Cm::SpatialVector& impulse1,
			Cm::SpatialVector& deltaV0,
			Cm::SpatialVector& deltaV1) const;

		virtual Cm::SpatialVectorV getLinkVelocity(const PxU32 linkID) const;

		virtual Cm::SpatialVectorV getLinkMotionVector(const PxU32 linkID) const;

		//this is called by island gen to determine whether the articulation should be awake or sleep
		virtual Cm::SpatialVector getMotionVelocity(const PxU32 linkID) const;

		virtual PxReal getLinkMaxPenBias(const PxU32 linkID) const;

		static PxU32 computeUnconstrainedVelocities(
			const ArticulationSolverDesc& desc,
			PxReal dt,
			PxConstraintAllocator& allocator,
			PxSolverConstraintDesc* constraintDesc,
			PxU32& acCount,
			const PxVec3& gravity, PxU64 contextID,
			Cm::SpatialVectorF* Z, Cm::SpatialVectorF* deltaV);

		static void computeUnconstrainedVelocitiesTGS(
			const ArticulationSolverDesc& desc,
			PxReal dt, const PxVec3& gravity,
			PxU64 contextID, Cm::SpatialVectorF* Z, Cm::SpatialVectorF* deltaV);

		static PxU32 setupSolverConstraintsTGS(const ArticulationSolverDesc& articDesc,
			PxcConstraintBlockStream& stream,
			PxTGSSolverConstraintDesc* constraintDesc,
			PxReal dt,
			PxReal invDt,
			PxReal totalDt,
			PxU32& acCount,
			PxsConstraintBlockManager& constraintBlockManager,
			Cm::SpatialVectorF* Z);

		static void saveVelocity(const ArticulationSolverDesc& d, Cm::SpatialVectorF* deltaV);

		static void saveVelocityTGS(const ArticulationSolverDesc& d, PxReal invDtF32);

		static void updateBodies(const ArticulationSolverDesc& desc, PxReal dt);

		static void updateBodiesTGS(const ArticulationSolverDesc& desc, PxReal dt);

		static void updateBodies(const ArticulationSolverDesc& desc, PxReal dt, bool integrateJointPosition);

		static void recordDeltaMotion(const ArticulationSolverDesc& desc, const PxReal dt, Cm::SpatialVectorF* deltaV);

		static void deltaMotionToMotionVelocity(const ArticulationSolverDesc& desc, PxReal invDt);

		virtual void pxcFsApplyImpulse(PxU32 linkID, Ps::aos::Vec3V linear,
			Ps::aos::Vec3V angular, Cm::SpatialVectorF* Z, Cm::SpatialVectorF* deltaV);

		virtual void pxcFsApplyImpulses(PxU32 linkID, const Ps::aos::Vec3V& linear,
			const Ps::aos::Vec3V& angular, PxU32 linkID2, const Ps::aos::Vec3V& linear2,
			const Ps::aos::Vec3V& angular2, Cm::SpatialVectorF* Z, Cm::SpatialVectorF* deltaV);

		virtual void pxcFsApplyImpulses(Cm::SpatialVectorF* Z);

		virtual Cm::SpatialVectorV pxcFsGetVelocity(PxU32 linkID);

		virtual void pxcFsGetVelocities(PxU32 linkID, PxU32 linkID1, Cm::SpatialVectorV& v0, Cm::SpatialVectorV& v1);

		virtual Cm::SpatialVectorV pxcFsGetVelocityTGS(PxU32 linkID);

		virtual const PxTransform& getCurrentTransform(PxU32 linkID) const;

		virtual const PxQuat& getDeltaQ(PxU32 linkID) const;

		//Applies a set of N impulses, all in local space and updates the links' motion and joint velocities
		void applyImpulses(Cm::SpatialVectorF* Z, Cm::SpatialVectorF* deltaV);
		void getDeltaV(Cm::SpatialVectorF* Z, Cm::SpatialVectorF* deltaV);

		//This method calculate the velocity change due to collision/constraint impulse, record joint velocity and acceleration
		static Cm::SpatialVectorF propagateVelocity(const ArticulationLinkData& linkDatum, const ArticulationJointCoreData& jointDatum, const Cm::SpatialVectorF& Z,
			PxReal* jointVelocity, const Cm::SpatialVectorF& hDeltaV);

		//This method calculate the velocity change due to collision/constraint impulse
		static Cm::SpatialVectorF propagateVelocityTestImpulse(const ArticulationLinkData& linkDatum, const ArticulationJointCoreData& jointDatum,
			const Cm::SpatialVectorF& Z, const Cm::SpatialVectorF& hDeltaV);

		//This method calculate zero acceration impulse due to test/actual impluse
		static Cm::SpatialVectorF propagateImpulse(const ArticulationLinkData& linkDatum, const ArticulationJointCoreData& jointDatum, const Cm::SpatialVectorF& Z);

		void applyCacheToDest(ArticulationData& data, PxArticulationCache& cache,
			PxReal* jVelocities, PxReal* jAcceleration, PxReal* jPosition, PxReal* jointForce,
			const PxArticulationCacheFlags flag);

		PX_FORCE_INLINE	ArticulationData&	getArticulationData()			{ return mArticulationData;	}

		PX_FORCE_INLINE	void				setGpuRemapId(const PxU32 id)	{ mGpuRemapId = id;			}
		PX_FORCE_INLINE	PxU32				getGpuRemapId()		const		{ return mGpuRemapId;		}

	private:
		void constraintPrep(ArticulationLoopConstraint* lConstraints, const PxU32 nbJoints,
			Cm::SpatialVectorF* Z, PxSolverConstraintPrepDesc& prepDesc, PxSolverBody& sBody,
			PxSolverBodyData& sBodyData, PxSolverConstraintDesc* desc, PxConstraintAllocator& allocator);

		void updateArticulation(ScratchData& scratchData,
			const PxVec3& gravity,
			Cm::SpatialVectorF* Z,
			Cm::SpatialVectorF* DeltaV);

		PxU32 computeUnconstrainedVelocitiesInternal(
			PxU32& acCount,
			const PxVec3& gravity,
			Cm::SpatialVectorF* Z, Cm::SpatialVectorF* DeltaV);

		void computeUnconstrainedVelocitiesTGSInternal(const PxVec3& gravity,
			Cm::SpatialVectorF* Z, Cm::SpatialVectorF* DeltaV);

		//copy joint data from fromJointData to toJointData
		void copyJointData(ArticulationData& data, PxReal* toJointData, const PxReal* fromJointData);

		void computeDofs();
		//this function calculates motion subspace matrix(s) for all tree joint
		void jcalc(ArticulationData& data);

		//this function calculates loop joint constraint subspace matrix(s) and active force
		//subspace matrix
		void jcalcLoopJointSubspace(ArticulationJointCore* joint, ArticulationJointCoreData& jointDatum, SpatialSubspaceMatrix& T);

		void computeSpatialInertia(ArticulationData& data);

		//compute zero acceleration force
		void computeZ(ArticulationData& data, const PxVec3& gravity, ScratchData& scratchData);
		
		//compute drag force
		void computeD(ArticulationData& data, ScratchData& scratchData,
			Cm::SpatialVectorF* tZ, Cm::SpatialVectorF* tDeltaV);

		void solveInternalConstraints(const PxReal dt, const PxReal invDt, Cm::SpatialVectorF* impulses, Cm::SpatialVectorF* DeltaV,
			bool velocityIteration);

		//compute coriolis force
		void computeC(ArticulationData& data, ScratchData& scratchData);

		//compute relative transform child to parent
		void computeRelativeTransformC2P(ArticulationData& data);
		//compute relative transform child to base
		void computeRelativeTransformC2B(ArticulationData& data);

		void computeLinkVelocities(ArticulationData& data, ScratchData& scratchData);

		void initLinks(ArticulationData& data, const PxVec3& gravity,
			ScratchData& scratchData, Cm::SpatialVectorF* tZ, Cm::SpatialVectorF* tDeltaV);

		void computeIs(ArticulationLinkData& linkDatum, ArticulationJointCoreData& jointDatum);
		SpatialMatrix computePropagateSpatialInertia(const PxU8 jointType, ArticulationLinkData& linkDatum, ArticulationJointCoreData& jointDatum);
		void transformInertia(const SpatialTransform& sTod, SpatialMatrix& inertia);

		void computeArticulatedSpatialInertia(ArticulationData& data);

		void computeArticulatedSpatialZ(ArticulationData& data, ScratchData& scratchData);

		void computeJointAcceleration(ArticulationLinkData& linkDatum, ArticulationJointCoreData& jointDatum, 
			const Cm::SpatialVectorF& pMotionAcceleration, PxReal* jointAcceleration);

		//compute joint acceleration, joint velocity and link acceleration, velocity based
		//on spatial force and spatial articulated inertia tensor
		void computeLinkAcceleration(ArticulationData& data, ScratchData& scratchData);

		//void computeTempLinkAcceleration(ArticulationData& data, ScratchData& scratchData);
		void computeJointTransmittedFrictionForce(ArticulationData& data, ScratchData& scratchData,
			Cm::SpatialVectorF* Z, Cm::SpatialVectorF* DeltaV);
		void computeJointFriction(ArticulationData& data, ScratchData& scratchData);

		//void copyFromBodyCore();
		//void copyToBodyCore();
		void updateBodies();

		void applyExternalImpulse(ArticulationLink* links, const PxU32 linkCount, const bool fixBase,
			ArticulationData& data, Cm::SpatialVectorF* Z, Cm::SpatialVectorF* deltaV, const PxReal dt, const PxVec3& gravity, Cm::SpatialVector* acceleration);

		static Cm::SpatialVectorF getDeltaVWithDeltaJV(const bool fixBase, const PxU32 linkID,
			const ArticulationData& data, Cm::SpatialVectorF* Z,
			PxReal* jointVelocities);

		static Cm::SpatialVectorF getDeltaV(const bool fixBase, const PxU32 linkID,
			const ArticulationData& data, Cm::SpatialVectorF* Z);

		//impulse need to be in the linkID space
		static void getZ(const PxU32 linkID, const ArticulationData& data, 
			Cm::SpatialVectorF* Z, const Cm::SpatialVectorF& impulse);

		////impulse0 and impulse1 are in linkID0 and linkID1 space respectively
		//static void getZ(
		//	const ArticulationData& data,
		//	Cm::SpatialVectorF* Z,
		//	PxU32 linkID0_,
		//	const Cm::SpatialVector& impulse0,
		//	PxU32 linkID1_,
		//	const Cm::SpatialVector& impulse1);

		//This method use in impulse self response. The input impulse is in the link space
		static Cm::SpatialVectorF getImpulseResponse(
			const PxU32 linkID,
			const bool fixBase,
			const ArticulationData& data,
			Cm::SpatialVectorF* Z,
			const Cm::SpatialVectorF& impulse);

		//This method use in impulse self response. The input impulse is in the link space
		static Cm::SpatialVectorF getImpulseResponseWithJ(
			const PxU32 linkID,
			const bool fixBase,
			const ArticulationData& data,
			Cm::SpatialVectorF* Z,
			const Cm::SpatialVectorF& impulse,
			PxReal* jointVelocities);

		void getImpulseSelfResponseInv(const bool fixBase, 
			PxU32 linkID0,
			PxU32 linkID1,
			Cm::SpatialVectorF* Z,
			const Cm::SpatialVector& impulse0,
			const Cm::SpatialVector& impulse1,
			Cm::SpatialVector& deltaV0,
			Cm::SpatialVector& deltaV1,
			PxReal* jointVelocities);

		void getImpulseResponseSlowInv(Dy::ArticulationLink* links,
			const ArticulationData& data,
			PxU32 linkID0_,
			const Cm::SpatialVector& impulse0,
			Cm::SpatialVector& deltaV0,
			PxU32 linkID1_,
			const Cm::SpatialVector& impulse1,
			Cm::SpatialVector& deltaV1,
			PxReal* jointVelocities);

		Cm::SpatialVectorF getImpulseResponseInv(const bool fixBase, 
			const PxU32 linkID, Cm::SpatialVectorF* Z, 
			const Cm::SpatialVector& impulse,
			PxReal* jointVelocites);

		void inverseDynamic(ArticulationData& data, const PxVec3& gravity,
			ScratchData& scratchData);

		void inverseDynamicFloatingBase(ArticulationData& data, const PxVec3& gravity,
			ScratchData& scratchData);

		//compute link body force with motion velocity and acceleration
		void computeZAForceInv(ArticulationData& data, ScratchData& scratchData);
		void initCompositeSpatialInertia(ArticulationData& data, Dy::SpatialMatrix* compositeSpatialInertia);
		void computeCompositeSpatialInertiaAndZAForceInv(ArticulationData& data, ScratchData& scratchData);

		void computeRelativeGeneralizedForceInv(ArticulationData& data, ScratchData& scratchData);

		//provided joint velocity and joint acceleartion, compute link acceleration
		void computeLinkAccelerationInv(ArticulationData& data, ScratchData& scratchData);

		void computeGeneralizedForceInv(ArticulationData& data, ScratchData& scratchData);

		void calculateMassMatrixColInv(ScratchData& scratchData);

		void calculateHFixBase(PxArticulationCache& cache);

		void calculateHFloatingBase(PxArticulationCache& cache);

		//joint limits
		void enforcePrismaticLimits(PxReal* jPosition, ArticulationJointCore* joint);

		public:
		static void getImpulseSelfResponse(ArticulationLink* links,
			const bool fixBase,
			Cm::SpatialVectorF* Z,
			const ArticulationData& data,
			PxU32 linkID0,
			const Cm::SpatialVectorV& impulse0,
			Cm::SpatialVectorV& deltaV0,
			PxU32 linkID1,
			const Cm::SpatialVectorV& impulse1,
			Cm::SpatialVectorV& deltaV1);

		void createHardLimit(
			ArticulationLink* links,
			const bool fixBase,
			Cm::SpatialVectorF* Z,
			ArticulationData& data,
			PxU32 linkIndex,
			SolverConstraint1DExt& s,
			const PxVec3& axis,
			PxReal err,
			PxReal recipDt);

		void createHardLimits(
			SolverConstraint1DExt& s0,
			SolverConstraint1DExt& s1,
			const PxVec3& axis,
			PxReal err0,
			PxReal err1,
			PxReal recipDt,
			const Cm::SpatialVectorV& deltaVA,
			const Cm::SpatialVectorV& deltaVB,
			PxReal recipResponse);

		void createTangentialSpring(
			ArticulationLink* links,
			const bool fixBase,
			Cm::SpatialVectorF* Z,
			ArticulationData& data,
			PxU32 linkIndex,
			SolverConstraint1DExt& s,
			const PxVec3& axis,
			PxReal stiffness,
			PxReal damping,
			PxReal dt);

		PxU32 setupSolverConstraints(
			ArticulationLink* links,
			const PxU32 linkCount,
			const bool fixBase,
			ArticulationData& data,
			Cm::SpatialVectorF* Z,
			PxU32& acCount);

		void setupInternalConstraints(
			ArticulationLink* links,
			const PxU32 linkCount,
			const bool fixBase,
			ArticulationData& data,
			Cm::SpatialVectorF* Z,
			PxReal dt,
			PxReal invDt,
			PxReal erp,
			bool isTGSSolver);

		void createHardLimitTGS(
			SolverConstraint1DExtStep& s,
			const PxVec3& axis,
			PxReal err,
			PxReal recipDt,
			const Cm::SpatialVectorV& deltaVA,
			const Cm::SpatialVectorV& deltaVB,
			PxReal recipResponse);

		void createTangentialSpringTGS(
			ArticulationLink* links,
			const bool fixBase,
			Cm::SpatialVectorF* Z,
			ArticulationData& data,
			PxU32 linkIndex,
			SolverConstraint1DExtStep& s,
			const PxVec3& axis,
			PxReal stiffness,
			PxReal damping,
			PxReal dt);

		//integration
		void propagateLinksDown(ArticulationData& data, PxReal* jointVelocities, PxReal* jointPositions,
			Cm::SpatialVectorF* motionVelocities);

		void computeAndEnforceJointPositions(ArticulationData& data, PxReal* jointPositions);

		//update link position based on joint position provided by the cache
		void teleportLinks(ArticulationData& data);

		PxU8* allocateScratchSpatialData(PxcScratchAllocator* allocator,
			const PxU32 linkCount, ScratchData& scratchData, bool fallBackToHeap = false);

//		void allocateScratchSpatialData(DyScratchAllocator& allocator,
//			const PxU32 linkCount, ScratchData& scratchData);

		//This method calculate the velocity change from parent to child using parent current motion velocity
		PxTransform propagateTransform(const PxU32 linkID, ArticulationLink* links, ArticulationJointCoreData& jointDatum,
			Cm::SpatialVectorF* motionVelocities, const PxReal dt, const PxTransform& pBody2World, const PxTransform& currentTransform,
			PxReal* jointVelocity, PxReal* jointDeltaVelocities, PxReal* jointPosition);

		static void updateRootBody(const Cm::SpatialVectorF& motionVelocity,
			const PxTransform& preTransform, ArticulationData& data, const PxReal dt);

		ArticulationData				mArticulationData;

		Ps::Array<char>					mScratchMemory;
		bool							mHasSphericalJoint;
		//this is the id remap pxgbodysim and pxgariculation. if application delete the articulation, we need to
		//put back this id to the id pool
		PxU32							mGpuRemapId;

	} PX_ALIGN_SUFFIX(64);

#if PX_VC 
#pragma warning(pop) 
#endif

	void PxvRegisterArticulationsReducedCoordinate();

} //namespace Dy

}

#endif
