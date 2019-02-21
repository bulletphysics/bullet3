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


#ifndef PXS_BODYATOM_H
#define PXS_BODYATOM_H

#include "PxcRigidBody.h"
#include "PxvDynamics.h"

namespace physx
{

class PxsRigidBody : public PxcRigidBody
{
	public:
	PX_FORCE_INLINE									PxsRigidBody(PxsBodyCore* core)	: PxcRigidBody(core) {  }
	PX_FORCE_INLINE									~PxsRigidBody() {}

	PX_FORCE_INLINE		const PxTransform&			getPose()								const	{ PX_ASSERT(mCore->body2World.isSane()); return mCore->body2World; }

	//PX_FORCE_INLINE		const Cm::SpatialVector&	getAccelerationV()						const	{ return mAcceleration;		}
	//PX_FORCE_INLINE		void						setAccelerationV(const Cm::SpatialVector& v)	{ mAcceleration = v;		}

	PX_FORCE_INLINE		const PxVec3&				getLinearVelocity()						const	{ PX_ASSERT(mCore->linearVelocity.isFinite()); return mCore->linearVelocity;			}
	PX_FORCE_INLINE		const PxVec3&				getAngularVelocity()					const	{ PX_ASSERT(mCore->angularVelocity.isFinite()); return mCore->angularVelocity;			}
	
	PX_FORCE_INLINE		void	 					setVelocity(const PxVec3& linear,
																const PxVec3& angular)				{ PX_ASSERT(linear.isFinite()); PX_ASSERT(angular.isFinite());
																									  mCore->linearVelocity = linear;
																									  mCore->angularVelocity = angular; }
	PX_FORCE_INLINE		void						setLinearVelocity(const PxVec3& linear)			{ PX_ASSERT(linear.isFinite()); mCore->linearVelocity = linear; }
	PX_FORCE_INLINE		void						setAngularVelocity(const PxVec3& angular)		{ PX_ASSERT(angular.isFinite()); mCore->angularVelocity = angular; }

	PX_FORCE_INLINE		void						constrainLinearVelocity();
	PX_FORCE_INLINE		void						constrainAngularVelocity();

	PX_FORCE_INLINE		PxU32						getIterationCounts()							{ return mCore->solverIterationCounts; }

	PX_FORCE_INLINE		PxReal						getReportThreshold()					const	{ return mCore->contactReportThreshold;	}

	// AP newccd todo: merge into get both velocities, compute inverse transform once, precompute mLastTransform.getInverse()
	PX_FORCE_INLINE		PxVec3						getLinearMotionVelocity(PxReal invDt)		const	{
														// delta(t0(x))=t1(x)
														// delta(t0(t0`(x)))=t1(t0`(x))
														// delta(x)=t1(t0`(x))
														PxVec3 deltaP = mCore->body2World.p - getLastCCDTransform().p;
														return deltaP * invDt;
													}
	PX_FORCE_INLINE		PxVec3						getAngularMotionVelocity(PxReal invDt)		const	{
														PxQuat deltaQ = mCore->body2World.q * getLastCCDTransform().q.getConjugate();
														PxVec3 axis;
														PxReal angle;
														deltaQ.toRadiansAndUnitAxis(angle, axis);
														return axis * angle * invDt;
													}
	PX_FORCE_INLINE		PxVec3						getLinearMotionVelocity(PxReal dt, const PxsBodyCore* PX_RESTRICT bodyCore)		const	{
															// delta(t0(x))=t1(x)
															// delta(t0(t0`(x)))=t1(t0`(x))
															// delta(x)=t1(t0`(x))
															PxVec3 deltaP = bodyCore->body2World.p - getLastCCDTransform().p;
															return deltaP * 1.0f / dt;
													}
	PX_FORCE_INLINE		PxVec3						getAngularMotionVelocity(PxReal dt, const PxsBodyCore* PX_RESTRICT bodyCore)		const	{
															PxQuat deltaQ = bodyCore->body2World.q * getLastCCDTransform().q.getConjugate();
															PxVec3 axis;
															PxReal angle;
															deltaQ.toRadiansAndUnitAxis(angle, axis);
															return axis * angle * 1.0f/dt;
													}

	PX_FORCE_INLINE		const PxTransform&			getLastCCDTransform()					const	{ return mLastTransform;}
	PX_FORCE_INLINE		void						saveLastCCDTransform()							{ mLastTransform = mCore->body2World; }

	PX_FORCE_INLINE		bool						isKinematic()							const	{ return (mCore->inverseMass == 0.0f); }
						
	PX_FORCE_INLINE		void						setPose(const PxTransform& pose)				{ mCore->body2World = pose; }
	PX_FORCE_INLINE		void						setPosition(const PxVec3& position)				{ mCore->body2World.p = position; }
	PX_FORCE_INLINE		PxReal						getInvMass()							const	{ return mCore->inverseMass; }
	PX_FORCE_INLINE		PxVec3						getInvInertia()							const	{ return mCore->inverseInertia; }
	PX_FORCE_INLINE		PxReal						getMass()								const	{ return 1.0f/mCore->inverseMass; }
	PX_FORCE_INLINE		PxVec3						getInertia()							const	{ return PxVec3(1.0f/mCore->inverseInertia.x,
																													1.0f/mCore->inverseInertia.y,
																													1.0f/mCore->inverseInertia.z); }
	PX_FORCE_INLINE		PxsBodyCore&				getCore()										{ return *mCore;					}
	PX_FORCE_INLINE		const PxsBodyCore&			getCore()								const	{ return *mCore;					}

	PX_FORCE_INLINE		PxU32						isActivateThisFrame()					const	{ return PxU32(mInternalFlags & eACTIVATE_THIS_FRAME); }

	PX_FORCE_INLINE		PxU32						isDeactivateThisFrame()					const	{ return PxU32(mInternalFlags & eDEACTIVATE_THIS_FRAME); }

	PX_FORCE_INLINE		PxU32						isFreezeThisFrame()						const	{ return PxU32(mInternalFlags & eFREEZE_THIS_FRAME); }

	PX_FORCE_INLINE		PxU32						isUnfreezeThisFrame()					const	{ return PxU32(mInternalFlags & eUNFREEZE_THIS_FRAME); }

	PX_FORCE_INLINE		void						clearFreezeFlag()								{ mInternalFlags &= ~eFREEZE_THIS_FRAME;	}

	PX_FORCE_INLINE		void						clearUnfreezeFlag()								{ mInternalFlags &= ~eUNFREEZE_THIS_FRAME; }

	PX_FORCE_INLINE		void						clearAllFrameFlags()							{ mInternalFlags &= (eFROZEN | eDISABLE_GRAVITY); }

						void						advanceToToi(PxReal toi, PxReal dt, bool clip);
						void						advancePrevPoseToToi(PxReal toi);
						PxTransform					getAdvancedTransform(PxReal toi) const;
						Cm::SpatialVector			getPreSolverVelocities() const;


};


void PxsRigidBody::constrainLinearVelocity()
{
	const PxU32 lockFlags = mCore->lockFlags;

	if (lockFlags)
	{
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_X)
			mCore->linearVelocity.x = 0.f;
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_Y)
			mCore->linearVelocity.y = 0.f;
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_Z)
			mCore->linearVelocity.z = 0.f;
	}
}

void PxsRigidBody::constrainAngularVelocity()
{
	const PxU32 lockFlags = mCore->lockFlags;

	if (lockFlags)
	{
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_X)
			mCore->angularVelocity.x = 0.f;
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y)
			mCore->angularVelocity.y = 0.f;
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z)
			mCore->angularVelocity.z = 0.f;
	}
}

}

#endif
