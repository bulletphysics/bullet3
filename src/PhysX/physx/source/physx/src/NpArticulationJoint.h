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


#ifndef PX_PHYSICS_NP_ARTICULATION_JOINT
#define PX_PHYSICS_NP_ARTICULATION_JOINT

#include "PxArticulationJoint.h"
#include "ScbArticulationJoint.h"
#include "NpArticulationTemplate.h"

#if PX_ENABLE_DEBUG_VISUALIZATION
#include "CmRenderOutput.h"
#endif

namespace physx
{

class NpScene;
class NpArticulationLink;

class PxArticulationJointImpl
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================
public:
	Scb::ArticulationJoint			mJoint;
	NpArticulationLink*				mParent;
	NpArticulationLink*				mChild;
	PxArticulationBase::Enum		mType;

	PX_INLINE PxArticulationJointImpl(NpArticulationLink& parent,
		const PxTransform& parentFrame,
		NpArticulationLink& child,
		const PxTransform& childFrame,
		PxArticulationBase::Enum type);

// PX_SERIALIZATION
	PX_INLINE PxArticulationJointImpl(const PxEMPTY) : mJoint(PxEmpty) {}
//~PX_SERIALIZATION

	PX_INLINE NpScene*						getOwnerScene() const; // the scene the user thinks the actor is in, or from which the actor is pending removal

	PX_INLINE void							release();

	PX_INLINE	const Scb::ArticulationJoint&	getScbArticulationJoint() const { return mJoint; }
	PX_INLINE	Scb::ArticulationJoint&			getScbArticulationJoint() { return mJoint; }

	PX_INLINE PxArticulationLink&	getParentArticulationLink() const;
	PX_INLINE PxArticulationLink&	getChildArticulationLink() const;

	PX_INLINE PxTransform			getParentPose() const;
	PX_INLINE void				setParentPose(const PxTransform&);

	PX_INLINE PxTransform			getChildPose() const;
	PX_INLINE void				setChildPose(const PxTransform&);

	PX_INLINE	const NpArticulationLink&		getParent() const { return *mParent; }
	PX_INLINE	NpArticulationLink&				getParent() { return *mParent; }

	PX_INLINE	const NpArticulationLink&		getChild() const { return *mChild; }
	PX_INLINE	NpArticulationLink&				getChild() { return *mChild; }

	PX_INLINE void resolveReferences(PxDeserializationContext& context);
};

template <typename APIClass>
class NpArticulationJointTemplate : public APIClass, public Ps::UserAllocated
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================
public:

// PX_SERIALIZATION
	NpArticulationJointTemplate(PxBaseFlags baseFlags) : APIClass(baseFlags), mImpl(PxEmpty)
	{
		mImpl.getScbArticulationJoint().getScArticulationJoint().setRoot(this);
	}
//~PX_SERIALIZATION

	NpArticulationJointTemplate(NpArticulationLink& parent,
		const PxTransform& parentFrame,
		NpArticulationLink& child,
		const PxTransform& childFrame,
		PxArticulationBase::Enum type = PxArticulationBase::eMaximumCoordinate);

	virtual							~NpArticulationJointTemplate() {}

	//---------------------------------------------------------------------------------
	// Miscellaneous
	//---------------------------------------------------------------------------------
public:
	void				release();

	PX_INLINE	const Scb::ArticulationJoint&	getScbArticulationJoint() const { return mImpl.getScbArticulationJoint(); }
	PX_INLINE	Scb::ArticulationJoint&			getScbArticulationJoint() { return mImpl.getScbArticulationJoint(); }

	virtual     PxArticulationLink&	getParentArticulationLink() const { return mImpl.getParentArticulationLink(); }
	virtual     PxArticulationLink&	getChildArticulationLink() const { return mImpl.getChildArticulationLink(); }

	virtual		PxTransform			getParentPose() const { return mImpl.getParentPose(); }
	virtual		void				setParentPose(const PxTransform& t) { mImpl.setParentPose(t); }

	virtual		PxTransform			getChildPose() const { return mImpl.getChildPose(); }
	virtual		void				setChildPose(const PxTransform& t) { mImpl.setChildPose(t); }

	PX_INLINE	const NpArticulationLink&		getParent() const { return mImpl.getParent(); }
	PX_INLINE	NpArticulationLink&				getParent() { return mImpl.getParent(); }

	PX_INLINE	const NpArticulationLink&		getChild() const { return mImpl.getChild(); }
	PX_INLINE	NpArticulationLink&				getChild() { return mImpl.getChild(); }

	virtual		PxArticulationJointImpl* getImpl() { return &mImpl; }
	virtual		const PxArticulationJointImpl* getImpl() const { return &mImpl; }

protected:

	NpScene*						getOwnerScene() const { return mImpl.getOwnerScene(); }

	PxArticulationJointImpl			mImpl;
};

class NpArticulationJoint : public NpArticulationJointTemplate<PxArticulationJoint>
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================
public:
// PX_SERIALIZATION
									NpArticulationJoint(PxBaseFlags baseFlags) : NpArticulationJointTemplate(baseFlags)	{}
	virtual		void				resolveReferences(PxDeserializationContext& context);
	static		NpArticulationJoint* createObject(PxU8*& address, PxDeserializationContext& context);
	static		void				getBinaryMetaData(PxOutputStream& stream);
				void				exportExtraData(PxSerializationContext&)	{}
				void				importExtraData(PxDeserializationContext&)	{}
	virtual		void				requiresObjects(PxProcessPxBaseCallback&){}	
	virtual		bool			    isSubordinate()  const	 { return true; }           
//~PX_SERIALIZATION
									NpArticulationJoint(NpArticulationLink& parent, 
														const PxTransform& parentFrame,
														NpArticulationLink& child,
														const PxTransform& childFrame);

	virtual							~NpArticulationJoint();

	//---------------------------------------------------------------------------------
	// PxArticulationJoint implementation
	//---------------------------------------------------------------------------------
	// Save
	

	virtual		void				setTargetOrientation(const PxQuat&);
	virtual		PxQuat				getTargetOrientation() const;


	virtual		void				setTargetVelocity(const PxVec3&);
	virtual		PxVec3				getTargetVelocity() const;

	virtual		void				setDriveType(PxArticulationJointDriveType::Enum driveType);
	virtual		PxArticulationJointDriveType::Enum
									getDriveType() const;


	virtual		void				setStiffness(PxReal);
	virtual		PxReal				getStiffness() const;

	virtual		void				setDamping(PxReal);
	virtual		PxReal				getDamping() const;

	virtual		void				setInternalCompliance(PxReal);
	virtual		PxReal				getInternalCompliance() const;

	virtual		void				setExternalCompliance(PxReal);
	virtual		PxReal				getExternalCompliance() const;

	virtual		void				setSwingLimit(PxReal yLimit, PxReal zLimit);
	virtual		void				getSwingLimit(PxReal &yLimit, PxReal &zLimit) const;

	virtual		void				setTangentialStiffness(PxReal spring);
	virtual		PxReal				getTangentialStiffness() const;

	virtual		void				setTangentialDamping(PxReal damping);
	virtual		PxReal				getTangentialDamping() const;

	virtual		void				setSwingLimitEnabled(bool);
	virtual		bool				getSwingLimitEnabled() const;

	virtual		void				setSwingLimitContactDistance(PxReal contactDistance);
	virtual		PxReal				getSwingLimitContactDistance() const;

	virtual		void				setTwistLimit(PxReal lower, PxReal upper);
	virtual		void				getTwistLimit(PxReal &lower, PxReal &upper) const;

	virtual		void				setTwistLimitEnabled(bool);
	virtual		bool				getTwistLimitEnabled() const;

	virtual		void				setTwistLimitContactDistance(PxReal contactDistance);
	virtual		PxReal				getTwistLimitContactDistance() const;

	virtual		void				setJointType(PxArticulationJointType::Enum jointType);
	virtual		PxArticulationJointType::Enum
									getJointType() const;

	virtual		void				setMotion(PxArticulationAxis::Enum axis, PxArticulationMotion::Enum motion);
	virtual		PxArticulationMotion::Enum		
									getMotion(PxArticulationAxis::Enum axis) const;

	virtual		void				setFrictionCoefficient(const PxReal coefficient);

	virtual		PxReal				getFrictionCoefficient() const ;
};

PX_INLINE PxArticulationLink&	PxArticulationJointImpl::getParentArticulationLink() const
{
	return *mParent;
}


PX_INLINE PxArticulationLink&	PxArticulationJointImpl::getChildArticulationLink() const
{
	return *mChild;
}


PX_INLINE PxTransform	PxArticulationJointImpl::getParentPose() const
{
	NP_READ_CHECK(getOwnerScene());
	return mJoint.getParentPose();
}


PX_INLINE void PxArticulationJointImpl::setParentPose(const PxTransform& t)
{
	PX_CHECK_AND_RETURN(t.isSane(), "NpArticulationJoint::setParentPose t is not valid.");

	NP_WRITE_CHECK(getOwnerScene());
	if (mParent == NULL)
		return;

	mJoint.setParentPose(mParent->getCMassLocalPose().transformInv(t.getNormalized()));
}


PX_INLINE PxTransform PxArticulationJointImpl::getChildPose() const
{
	NP_READ_CHECK(getOwnerScene());

	return mChild->getCMassLocalPose().transform(mJoint.getChildPose());
}


PX_INLINE void PxArticulationJointImpl::setChildPose(const PxTransform& t)
{
	PX_CHECK_AND_RETURN(t.isSane(), "NpArticulationJoint::setChildPose t is not valid.");

	NP_WRITE_CHECK(getOwnerScene());

	mJoint.setChildPose(mChild->getCMassLocalPose().transformInv(t.getNormalized()));
}


PX_INLINE NpScene* PxArticulationJointImpl::getOwnerScene() const
{
	return mJoint.getScbScene() ? static_cast<NpScene *>(mJoint.getScbScene()->getPxScene()) : NULL;
}


PX_INLINE PxArticulationJointImpl::PxArticulationJointImpl(NpArticulationLink& parent,
	const PxTransform& parentFrame,
	NpArticulationLink& child,
	const PxTransform& childFrame,
	PxArticulationBase::Enum type)
	: mJoint(parentFrame, childFrame, type)
	, mParent(&parent)
	, mChild(&child)
	, mType(type)
{
	PxArticulationImpl* impl = parent.getRoot().getImpl();
	Scb::Articulation& scbArti = impl->getArticulation();
	mJoint.setScArticulation(&scbArti);
}

PX_INLINE void PxArticulationJointImpl::release()
{
	if (mJoint.getScbSceneForAPI())
		mJoint.getScbSceneForAPI()->removeArticulationJoint(mJoint);

	mJoint.destroy();
}

PX_INLINE void PxArticulationJointImpl::resolveReferences(PxDeserializationContext& context)
{
	context.translatePxBase(mParent);
	context.translatePxBase(mChild);
}

template <typename APIClass>
void NpArticulationJointTemplate<APIClass>::release()
{
	NpPhysics::getInstance().notifyDeletionListenersUserRelease(this, NULL);
	mImpl.release();
}


template <typename APIClass>
NpArticulationJointTemplate<APIClass>::NpArticulationJointTemplate(NpArticulationLink& parent,
	const PxTransform& parentFrame,
	NpArticulationLink& child,
	const PxTransform& childFrame,
	PxArticulationBase::Enum type)
	: APIClass(PxConcreteType::eARTICULATION_JOINT, PxBaseFlag::eOWNS_MEMORY),
	  mImpl(parent, parentFrame, child, childFrame, type)
{
	PX_UNUSED(type);
	mImpl.getScbArticulationJoint().getScArticulationJoint().setRoot(this);
}

}

#endif
