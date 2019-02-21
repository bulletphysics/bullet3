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

#ifndef NP_JOINTCONSTRAINT_H
#define NP_JOINTCONSTRAINT_H

#include "PsAllocator.h"
#include "PsUtilities.h"
#include "PsMathUtils.h"
#include "PxConstraint.h"
#include "PxConstraintExt.h"
#include "PxJoint.h"
#include "PxD6Joint.h"
#include "PxRigidDynamic.h"
#include "PxRigidStatic.h"
#include "PxScene.h"
#include "ExtPvd.h"
#include "PxMetaData.h"
#include "CmRenderOutput.h"
#include "PxPhysics.h"
#include "PsFoundation.h"

#if PX_SUPPORT_PVD
#include "PxScene.h"
#include "PxPvdClient.h"
#include "PxPvdSceneClient.h"
#endif

// PX_SERIALIZATION

namespace physx
{

PxConstraint* resolveConstraintPtr(PxDeserializationContext& v, PxConstraint* old, PxConstraintConnector* connector, PxConstraintShaderTable& shaders);

// ~PX_SERIALIZATION

namespace Ext
{
	PX_FORCE_INLINE float	computeSwingAngle(float swingYZ, float swingW)
	{
		return 4.0f * PxAtan2(swingYZ, 1.0f + swingW);	// tan (t/2) = sin(t)/(1+cos t), so this is the quarter angle
	}

	struct JointData
	{
	//= ATTENTION! =====================================================================================
	// Changing the data layout of this class breaks the binary serialization format.  See comments for 
	// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
	// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
	// accordingly.
	//==================================================================================================
							PxConstraintInvMassScale	invMassScale;		//16
							PxTransform					c2b[2];				//72
							PxU32						pading[2];			//80
	protected:
		                    ~JointData(){}
	};

	template <class Base, class ValueStruct>
	class Joint : public Base, 
				  public PxConstraintConnector, 
				  public Ps::UserAllocated
	{
  
    public:
// PX_SERIALIZATION
						Joint(PxBaseFlags baseFlags) : Base(baseFlags) {}
		virtual	void	requiresObjects(PxProcessPxBaseCallback& c)
		{			
			c.process(*mPxConstraint);
			
			{
				PxRigidActor* a0 = NULL;
				PxRigidActor* a1 = NULL;
				mPxConstraint->getActors(a0,a1);
				
				if (a0)
				{
					c.process(*a0);
				}
				if (a1)
				{
					c.process(*a1);
				}
			}
		}	
//~PX_SERIALIZATION
		
#if PX_SUPPORT_PVD
		// PxConstraintConnector
		virtual bool updatePvdProperties(physx::pvdsdk::PvdDataStream& pvdConnection, const PxConstraint* c, PxPvdUpdateType::Enum updateType) const
		{
			if(updateType == PxPvdUpdateType::UPDATE_SIM_PROPERTIES)
			{
				Ext::Pvd::simUpdate<Base>(pvdConnection, *this);
				return true;
			}
			else if(updateType == PxPvdUpdateType::UPDATE_ALL_PROPERTIES)
			{
				Ext::Pvd::updatePvdProperties<Base, ValueStruct>(pvdConnection, *this);
				return true;
			}
			else if(updateType == PxPvdUpdateType::CREATE_INSTANCE)
			{
				Ext::Pvd::createPvdInstance<Base>(pvdConnection, *c, *this);
				return true;
			}
			else if(updateType == PxPvdUpdateType::RELEASE_INSTANCE)
			{
				Ext::Pvd::releasePvdInstance(pvdConnection, *c, *this);
				return true;
			}
			return false;
		}
#else
		virtual bool updatePvdProperties(physx::pvdsdk::PvdDataStream&, const PxConstraint*, PxPvdUpdateType::Enum) const
		{
			return false;
		}
#endif

		// PxJoint
		virtual void setActors(PxRigidActor* actor0, PxRigidActor* actor1)
		{	
			//TODO SDK-DEV
			//You can get the debugger stream from the NpScene
			//Ext::Pvd::setActors( stream, this, mPxConstraint, actor0, actor1 );
			PX_CHECK_AND_RETURN(actor0 != actor1, "PxJoint::setActors: actors must be different");
			PX_CHECK_AND_RETURN((actor0 && !actor0->is<PxRigidStatic>()) || (actor1 && !actor1->is<PxRigidStatic>()), "PxJoint::setActors: at least one actor must be non-static");

#if PX_SUPPORT_PVD
			PxScene* scene = getScene();
			if(scene)
			{
				//if pvd not connect data stream is NULL
				physx::pvdsdk::PvdDataStream* conn = scene->getScenePvdClient()->getClientInternal()->getDataStream();
				if( conn != NULL )
					Ext::Pvd::setActors(
					*conn,
					*this,
					*mPxConstraint,
					actor0, 
					actor1
					);
			}
#endif
			mPxConstraint->setActors(actor0, actor1);
			mData->c2b[0] = getCom(actor0).transformInv(mLocalPose[0]);
			mData->c2b[1] = getCom(actor1).transformInv(mLocalPose[1]);
			mPxConstraint->markDirty();
		}

		// PxJoint
		virtual	void getActors(PxRigidActor*& actor0, PxRigidActor*& actor1)	const		
		{	
			if ( mPxConstraint ) mPxConstraint->getActors(actor0,actor1);
			else
			{
				actor0 = NULL;
				actor1 = NULL;
			}
		}

		// this is the local pose relative to the actor, and we store internally the local
		// pose relative to the body 

		// PxJoint
		virtual void setLocalPose(PxJointActorIndex::Enum actor, const PxTransform& pose)
		{
			PX_CHECK_AND_RETURN(pose.isSane(), "PxJoint::setLocalPose: transform is invalid");
			PxTransform p = pose.getNormalized();
			mLocalPose[actor] = p;
			mData->c2b[actor] = getCom(actor).transformInv(p); 
			mPxConstraint->markDirty();
		}

		// PxJoint
		virtual	PxTransform	getLocalPose(PxJointActorIndex::Enum actor) const
		{	
			return mLocalPose[actor];
		}

		static	PxTransform	getGlobalPose(const PxRigidActor* actor)
		{
			if(!actor)
				return PxTransform(PxIdentity);
			return actor->getGlobalPose();
		}

		static	void	getActorVelocity(const PxRigidActor* actor, PxVec3& linear, PxVec3& angular)
		{
			if(!actor || actor->is<PxRigidStatic>())
			{
				linear = angular = PxVec3(0.0f);
				return;
			}
			
			linear = static_cast<const PxRigidBody*>(actor)->getLinearVelocity();
			angular = static_cast<const PxRigidBody*>(actor)->getAngularVelocity();
		}

		// PxJoint
		virtual	PxTransform	getRelativeTransform()	const
		{
			PxRigidActor* actor0, * actor1;
			mPxConstraint->getActors(actor0, actor1);
			const PxTransform t0 = getGlobalPose(actor0) * mLocalPose[0];
			const PxTransform t1 = getGlobalPose(actor1) * mLocalPose[1];
			return t0.transformInv(t1);
		}

		// PxJoint
		virtual	PxVec3	getRelativeLinearVelocity()	const
		{
			PxRigidActor* actor0, * actor1;
			PxVec3 l0, a0, l1, a1;
			mPxConstraint->getActors(actor0, actor1);

			PxTransform t0 = getCom(actor0), t1 = getCom(actor1);
			getActorVelocity(actor0, l0, a0);
			getActorVelocity(actor1, l1, a1);

			PxVec3 p0 = t0.q.rotate(mLocalPose[0].p), 
				   p1 = t1.q.rotate(mLocalPose[1].p);
			return t0.transformInv(l1 - a1.cross(p1) - l0 + a0.cross(p0));
		}

		// PxJoint
		virtual	PxVec3	getRelativeAngularVelocity()	const
		{
			PxRigidActor* actor0, * actor1;
			PxVec3 l0, a0, l1, a1;
			mPxConstraint->getActors(actor0, actor1);

			PxTransform t0 = getCom(actor0);
			getActorVelocity(actor0, l0, a0);
			getActorVelocity(actor1, l1, a1);

			return t0.transformInv(a1 - a0);
		}

		// PxJoint
		virtual	void setBreakForce(PxReal force, PxReal torque)
		{
			PX_CHECK_AND_RETURN(PxIsFinite(force) && PxIsFinite(torque), "NpJoint::setBreakForce: invalid float");
			mPxConstraint->setBreakForce(force,torque);
		}

		// PxJoint
		virtual	void getBreakForce(PxReal& force, PxReal& torque)	const
		{
			mPxConstraint->getBreakForce(force,torque);
		}

		// PxJoint
		virtual	void setConstraintFlags(PxConstraintFlags flags)
		{
			mPxConstraint->setFlags(flags);
		}

		// PxJoint
		virtual	void setConstraintFlag(PxConstraintFlag::Enum flag, bool value)
		{
			mPxConstraint->setFlag(flag, value);
		}

		// PxJoint
		virtual	PxConstraintFlags getConstraintFlags()	const
		{
			return mPxConstraint->getFlags();
		}

		// PxJoint
		virtual	void setInvMassScale0(PxReal invMassScale)
		{
			PX_CHECK_AND_RETURN(PxIsFinite(invMassScale) && invMassScale>=0, "PxJoint::setInvMassScale0: scale must be non-negative");
			mData->invMassScale.linear0 = invMassScale;
			mPxConstraint->markDirty();
		}

		// PxJoint
		virtual	PxReal getInvMassScale0() const
		{
			return mData->invMassScale.linear0;
		}

		// PxJoint
		virtual	void setInvInertiaScale0(PxReal invInertiaScale)
		{
			PX_CHECK_AND_RETURN(PxIsFinite(invInertiaScale) && invInertiaScale>=0, "PxJoint::setInvInertiaScale0: scale must be non-negative");
			mData->invMassScale.angular0 = invInertiaScale;
			mPxConstraint->markDirty();
		}

		// PxJoint
		virtual	PxReal getInvInertiaScale0() const
		{
			return mData->invMassScale.angular0;
		}

		// PxJoint
		virtual	void setInvMassScale1(PxReal invMassScale)
		{
			PX_CHECK_AND_RETURN(PxIsFinite(invMassScale) && invMassScale>=0, "PxJoint::setInvMassScale1: scale must be non-negative");
			mData->invMassScale.linear1 = invMassScale;
			mPxConstraint->markDirty();
		}

		// PxJoint
		virtual	PxReal getInvMassScale1() const
		{
			return mData->invMassScale.linear1;
		}

		// PxJoint
		virtual	void setInvInertiaScale1(PxReal invInertiaScale)
		{
			PX_CHECK_AND_RETURN(PxIsFinite(invInertiaScale) && invInertiaScale>=0, "PxJoint::setInvInertiaScale: scale must be non-negative");
			mData->invMassScale.angular1 = invInertiaScale;
			mPxConstraint->markDirty();
		}

		// PxJoint
		virtual	PxReal getInvInertiaScale1() const
		{
			return mData->invMassScale.angular1;
		}

		// PxJoint
		virtual	PxConstraint* getConstraint()	const
		{
			return mPxConstraint;
		}

		// PxJoint
		virtual	void setName(const char* name)
		{
			mName = name;
		}

		// PxJoint
		virtual	const char* getName()	const
		{
			return mName;
		}

		// PxJoint
		virtual	void release()
		{
			mPxConstraint->release();
		}

		// PxJoint
		virtual	PxScene* getScene() const
		{
			return mPxConstraint ? mPxConstraint->getScene() : NULL;
		}

		// PxConstraintConnector
		virtual	void onComShift(PxU32 actor)
		{
			mData->c2b[actor] = getCom(actor).transformInv(mLocalPose[actor]); 
			markDirty();
		}

		// PxConstraintConnector
		virtual	void onOriginShift(const PxVec3& shift)
		{
			PxRigidActor* a[2];
			mPxConstraint->getActors(a[0], a[1]);

			if (!a[0])
			{
				mLocalPose[0].p -= shift;
				mData->c2b[0].p -= shift;
				markDirty();
			}
			else if (!a[1])
			{
				mLocalPose[1].p -= shift;
				mData->c2b[1].p -= shift;
				markDirty();
			}
		}

		// PxConstraintConnector
		virtual	void* prepareData()
		{
			return mData;
		}

		// PxConstraintConnector
		virtual	void* getExternalReference(PxU32& typeID)
		{
			typeID = PxConstraintExtIDs::eJOINT;
			return static_cast<PxJoint*>( this );
		}

		// PxConstraintConnector
		virtual	PxBase* getSerializable()
		{
			return this;
		}

		// PxConstraintConnector
		virtual	void onConstraintRelease()
		{
			PX_FREE_AND_RESET(mData);
			delete this;
		}

		// PxConstraintConnector
		virtual const void* getConstantBlock()	const
		{
			return mData; 
		}

	private:
		PxTransform getCom(PxU32 index) const
		{
			PxRigidActor* a[2];
			mPxConstraint->getActors(a[0],a[1]);
			return getCom(a[index]);
		}

		PxTransform getCom(PxRigidActor* actor) const
		{
			if (!actor)
				return PxTransform(PxIdentity);
			else if (actor->getType() == PxActorType::eRIGID_DYNAMIC || actor->getType() == PxActorType::eARTICULATION_LINK)
				return static_cast<PxRigidBody*>(actor)->getCMassLocalPose();
			else
			{
				PX_ASSERT(actor->getType() == PxActorType::eRIGID_STATIC);
				return static_cast<PxRigidStatic*>(actor)->getGlobalPose().getInverse();
			}
		}

	protected:

		Joint(PxType concreteType, PxBaseFlags baseFlags, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1, PxU32 size, const char* name) :
			Base			(concreteType, baseFlags),
			mName			(NULL),
			mPxConstraint	(NULL)
		{
			PX_UNUSED(name);
			Base::userData = NULL;

			JointData* data = reinterpret_cast<JointData*>(PX_ALLOC(size, name));
			Cm::markSerializedMem(data, size);

			mLocalPose[0]				= localFrame0.getNormalized();
			mLocalPose[1]				= localFrame1.getNormalized();
			data->c2b[0]				= getCom(actor0).transformInv(localFrame0);
			data->c2b[1]				= getCom(actor1).transformInv(localFrame1);
			data->invMassScale.linear0	= 1.0f;
			data->invMassScale.angular0	= 1.0f;
			data->invMassScale.linear1	= 1.0f;
			data->invMassScale.angular1	= 1.0f;

			mData = data;
		}

		virtual ~Joint()
		{
			if(Base::getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
				PX_FREE_AND_RESET(mData);
		}

		PX_FORCE_INLINE	void markDirty()
		{ 
			mPxConstraint->markDirty();
		}

		PX_FORCE_INLINE PxConstraintConnector* getConnector()
		{
			return this;
		}

		PX_FORCE_INLINE PxConstraint* getPxConstraint()
		{
			return mPxConstraint;
		}

		PX_FORCE_INLINE void setPxConstraint(PxConstraint* pxConstraint)
		{
			mPxConstraint = pxConstraint;
		}

		void wakeUpActors()
		{
			PxRigidActor* a[2];
			mPxConstraint->getActors(a[0], a[1]);
			for(PxU32 i = 0; i < 2; i++)
			{
				if(a[i] && a[i]->getScene() && a[i]->getType() == PxActorType::eRIGID_DYNAMIC)
				{
					PxRigidDynamic* rd = static_cast<PxRigidDynamic*>(a[i]);
					if(!(rd->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC))
					{											
						const PxScene* scene = rd->getScene();						
						const PxReal wakeCounterResetValue = scene->getWakeCounterResetValue();

						PxReal wakeCounter = rd->getWakeCounter();

						bool needsWakingUp = rd->isSleeping();
						if (wakeCounter < wakeCounterResetValue)
						{
							wakeCounter = wakeCounterResetValue;
							needsWakingUp = true;
						}

						if (needsWakingUp)
						{
							rd->wakeUp();
							rd->setWakeCounter(wakeCounter);
						}
					}
				}
			}
		}

		PX_FORCE_INLINE	PxQuat	getTwistOrSwing(bool needTwist) const
		{
			const PxQuat q = getRelativeTransform().q;
			// PT: TODO: we don't need to compute both quats here
			PxQuat swing, twist;
			Ps::separateSwingTwist(q, swing, twist);
			return needTwist ? twist : swing;
		}

		PxReal	getTwistAngle_Internal() const
		{
			const PxQuat twist = getTwistOrSwing(true);

			// PT: the angle-axis formulation creates the quat like this:
			//
			//	const float a = angleRadians * 0.5f;
			//	const float s = PxSin(a);
			//	w = PxCos(a);
			//	x = unitAxis.x * s;
			//	y = unitAxis.y * s;
			//	z = unitAxis.z * s;
			//
			// With the twist axis = (1;0;0) this gives:
			//
			//	w = PxCos(angleRadians * 0.5f);
			//	x = PxSin(angleRadians * 0.5f);
			//	y = 0.0f;
			//	z = 0.0f;
			//
			// Thus the quat's "getAngle" function returns:
			//
			//	angle = PxAcos(w) * 2.0f;
			//
			// PxAcos will return an angle between 0 and PI in radians, so "getAngle" will return an angle between 0 and PI*2.

			PxReal angle = twist.getAngle();

			if(twist.x<0.0f)
				angle = -angle;

			return angle;
		}

		PxReal	getSwingYAngle_Internal()	const
		{
			PxQuat swing = getTwistOrSwing(false);

			if(swing.w < 0.0f)		// choose the shortest rotation
				swing = -swing;

			const PxReal angle = computeSwingAngle(swing.y, swing.w);
			PX_ASSERT(angle>-PxPi && angle<=PxPi);				// since |y| < w+1, the atan magnitude is < PI/4
			return angle;
		}

		PxReal	getSwingZAngle_Internal()	const
		{
			PxQuat swing = getTwistOrSwing(false);

			if(swing.w < 0.0f)		// choose the shortest rotation
				swing = -swing;

			const PxReal angle = computeSwingAngle(swing.z, swing.w);
			PX_ASSERT(angle>-PxPi && angle <= PxPi);			// since |y| < w+1, the atan magnitude is < PI/4
			return angle;
		}

		const char*		mName;
		PxTransform		mLocalPose[2];
		PxConstraint*	mPxConstraint;
		JointData*		mData;
	};

	PX_FORCE_INLINE	bool	isLimitActive(const PxJointLimitParameters& limit, PxReal pad, PxReal angle, PxReal low, PxReal high)
	{
		PX_ASSERT(low<high);
		if(limit.isSoft())
			pad = 0.0f;
		bool active = false;
		if(angle < low + pad)
			active = true;
		if(angle > high - pad)
			active = true;
		return active;
	}

} // namespace Ext

}

#endif
