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

#ifndef NP_ARTICULATION_TEMPLATE
#define NP_ARTICULATION_TEMPLATE

#include "CmPhysXCommon.h"

#if PX_ENABLE_DEBUG_VISUALIZATION
#include "CmRenderOutput.h"
#endif

#include "ScbArticulation.h"
#include "NpArticulationLink.h"
#include "NpAggregate.h"

namespace physx
{
class NpArticulationLink;
class NpScene;
class NpAggregate;
class PxAggregate;

class PxArticulationImpl
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================
public:

	~PxArticulationImpl()
	{
	}

	PxArticulationImpl() : 
		mAggregate(NULL), mName(NULL), mCacheVersion(0) {}

	// PX_SERIALIZATION
	PxArticulationImpl(const PxEMPTY) : mArticulation(PxEmpty), mArticulationLinks(PxEmpty) {}
	//~PX_SERIALIZATION


	PX_INLINE PxScene*						getScene() const;

	PX_INLINE void							setSleepThreshold(PxReal threshold);
	PX_INLINE PxReal						getSleepThreshold() const;

	PX_INLINE void							setStabilizationThreshold(PxReal threshold);
	PX_INLINE PxReal						getStabilizationThreshold() const;

	PX_INLINE 	void						setWakeCounter(PxReal wakeCounterValue);
	PX_INLINE PxReal						getWakeCounter() const;

	PX_INLINE void							setSolverIterationCounts(PxU32 positionIters, PxU32 velocityIters);
	PX_INLINE void							getSolverIterationCounts(PxU32 & positionIters, PxU32 & velocityIters) const;

	PX_INLINE bool							isSleeping() const;
	PX_INLINE void							wakeUp();
	PX_INLINE void							putToSleep();

	PX_INLINE PxU32							getNbLinks() const;
	PX_INLINE PxU32							getLinks(PxArticulationLink** userBuffer, PxU32 bufferSize, PxU32 startIndex) const;
		
	PX_INLINE PxBounds3						getWorldBounds(float inflation = 1.01f) const;

	PX_INLINE PxAggregate*					getAggregate() const;

	// Debug name
	PX_INLINE void							setName(const char*);
	PX_INLINE const char*					getName() const;


	//---------------------------------------------------------------------------------
	// Miscellaneous
	//---------------------------------------------------------------------------------

	PX_INLINE		void						addToLinkList(NpArticulationLink& link) { mArticulationLinks.pushBack(&link); }
	PX_INLINE bool								removeLinkFromList(NpArticulationLink& link) { PX_ASSERT(mArticulationLinks.find(&link) != mArticulationLinks.end()); return mArticulationLinks.findAndReplaceWithLast(&link); }
	PX_FORCE_INLINE	NpArticulationLink* const*	getLinks() { return &mArticulationLinks.front(); }

	PX_INLINE NpArticulationLink*				getRoot();

	PX_FORCE_INLINE	const Scb::Articulation&	getArticulation()							const { return mArticulation; }
	PX_FORCE_INLINE	Scb::Articulation&			getArticulation() { return mArticulation; }

	PX_INLINE NpScene*							getAPIScene() const;
	PX_INLINE NpScene*							getOwnerScene() const;		// the scene the user thinks the actor is in, or from which the actor is pending removal

	PX_FORCE_INLINE	void						setAggregate(PxAggregate* a) { mAggregate = static_cast<NpAggregate*>(a); }

	PX_INLINE void								wakeUpInternal(bool forceWakeUp, bool autowake);

	PX_INLINE void								setGlobalPose();

	PX_FORCE_INLINE	Scb::Articulation&			getScbArticulation() { return mArticulation; }
	PX_FORCE_INLINE	const Scb::Articulation&	getScbArticulation() const { return mArticulation; }

	PX_FORCE_INLINE	void						increaseCacheVersion()	{ mCacheVersion++; }

#if PX_ENABLE_DEBUG_VISUALIZATION
public:
	PX_INLINE void						visualize(Cm::RenderOutput& out, NpScene* scene);
#endif

	Scb::Articulation 			mArticulation;
	NpArticulationLinkArray		mArticulationLinks;
	NpAggregate*				mAggregate;
	const char*					mName;
	PxU32						mCacheVersion;
};

template <typename APIClass>
class NpArticulationTemplate : public APIClass, public Ps::UserAllocated
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================
public:

	virtual										~NpArticulationTemplate();
	NpArticulationTemplate(PxType concreteType, PxBaseFlags baseFlags) : APIClass(concreteType, baseFlags) {}

	// PX_SERIALIZATION
	NpArticulationTemplate(PxBaseFlags baseFlags) : APIClass(baseFlags), mImpl(PxEmpty) {}
	//~PX_SERIALIZATION

	virtual			void						release();

	virtual			PxScene*					getScene() const { return mImpl.getScene(); }

	virtual			void						setSleepThreshold(PxReal threshold) { mImpl.setSleepThreshold(threshold); }
	virtual			PxReal						getSleepThreshold() const { return mImpl.getSleepThreshold();}

	virtual			void						setStabilizationThreshold(PxReal threshold) { mImpl.setStabilizationThreshold(threshold); }
	virtual			PxReal						getStabilizationThreshold() const { return mImpl.getStabilizationThreshold(); }

	virtual			void						setWakeCounter(PxReal wakeCounterValue) { mImpl.setWakeCounter(wakeCounterValue); }
	virtual			PxReal						getWakeCounter() const {return mImpl.getWakeCounter();	}

	virtual			void						setSolverIterationCounts(PxU32 positionIters, PxU32 velocityIters) { mImpl.setSolverIterationCounts(positionIters, velocityIters); }
	virtual			void						getSolverIterationCounts(PxU32 & positionIters, PxU32 & velocityIters) const { mImpl.getSolverIterationCounts(positionIters, velocityIters); }

	virtual			bool						isSleeping() const { return mImpl.isSleeping(); }
	virtual			void						wakeUp() { mImpl.wakeUp(); }
	virtual			void						putToSleep() { mImpl.putToSleep(); }
	
	virtual			PxU32						getNbLinks() const { return mImpl.getNbLinks(); }
	virtual			PxU32						getLinks(PxArticulationLink** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
	{
		return mImpl.getLinks(userBuffer, bufferSize, startIndex);
	}

	virtual			PxBounds3					getWorldBounds(float inflation = 1.01f) const { return mImpl.getWorldBounds(inflation); }

	virtual			PxAggregate*				getAggregate() const { return mImpl.getAggregate(); }

	// Debug name
	virtual			void						setName(const char* name) { return mImpl.setName(name); }
	virtual			const char*					getName() const { return mImpl.getName(); }

	virtual PxArticulationLink*	 createLink(PxArticulationLink* parent, const PxTransform& pose);

	//---------------------------------------------------------------------------------
	// Miscellaneous
	//---------------------------------------------------------------------------------
	NpArticulationTemplate();

	virtual const PxArticulationImpl* getImpl() const { return &mImpl; }

	virtual PxArticulationImpl* getImpl() { return &mImpl; }

	virtual PxArticulationBase::Enum getType() const { return PxArticulationBase::Enum(mType); }

#if PX_ENABLE_DEBUG_VISUALIZATION
public:
	void						visualize(Cm::RenderOutput& out, NpScene* scene) { mImpl.visualize(out, scene); }
#endif

public:
	PxU32						mType;
	PxArticulationImpl			mImpl;
};

template<typename APIClass>
NpArticulationTemplate<APIClass>::NpArticulationTemplate()
	: APIClass(PxConcreteType::eARTICULATION, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE)
{
	mType = PxArticulationBase::eMaximumCoordinate;
}

template<typename APIClass>
NpArticulationTemplate<APIClass>::~NpArticulationTemplate()
{
	NpFactory::getInstance().onArticulationRelease(this);
}

template<typename APIClass>
void NpArticulationTemplate<APIClass>::release()
{
	NP_WRITE_CHECK(mImpl.getOwnerScene());

	NpPhysics::getInstance().notifyDeletionListenersUserRelease(this, PxArticulationBase::userData);

	//!!!AL TODO: Order should not matter in this case. Optimize by having a path which does not restrict release to leaf links or
	//      by using a more advanced data structure
	PxU32 idx = 0;
	while (mImpl.mArticulationLinks.size())
	{
		idx = idx % mImpl.mArticulationLinks.size();

		if (mImpl.mArticulationLinks[idx]->getNbChildren() == 0)
		{
			mImpl.mArticulationLinks[idx]->releaseInternal();  // deletes joint, link and removes link from list
		}
		else
		{
			idx++;
		}
	}

	NpScene* npScene = mImpl.getAPIScene();
	if (npScene)
	{
		npScene->getScene().removeArticulation(mImpl.getArticulation());

		npScene->removeFromArticulationList(*this);
	}

	mImpl.mArticulationLinks.clear();

	mImpl.mArticulation.destroy();
}

template<typename APIClass>
PxArticulationLink*	 NpArticulationTemplate<APIClass>::createLink(PxArticulationLink* parent, const PxTransform& pose)
{
	PX_CHECK_AND_RETURN_NULL(pose.isSane(), "NpArticulation::createLink pose is not valid.");
	PX_CHECK_AND_RETURN_NULL(mImpl.mArticulationLinks.size()<64, "NpArticulation::createLink: at most 64 links allowed in an articulation");

	NP_WRITE_CHECK(mImpl.getOwnerScene());

	if (parent && mImpl.mArticulationLinks.empty())
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "Root articulation link must have NULL parent pointer!");
		return NULL;
	}

	if (!parent && !mImpl.mArticulationLinks.empty())
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "Non-root articulation link must have valid parent pointer!");
		return NULL;
	}

	//increase cache version
	mImpl.mCacheVersion++;

	NpArticulationLink* parentLink = static_cast<NpArticulationLink*>(parent);

	NpArticulationLink* link = static_cast<NpArticulationLink*>(NpFactory::getInstance().createArticulationLink(*this, parentLink, pose.getNormalized()));

	if (link)
	{
		NpScene* scene = mImpl.getAPIScene();
		if (scene)
			scene->addArticulationLink(*link);

		mImpl.addToLinkList(*link);
	}
	return link;
}

PxScene* PxArticulationImpl::getScene() const
{
	return getAPIScene();
}

void PxArticulationImpl::setSolverIterationCounts(PxU32 positionIters, PxU32 velocityIters)
{
	NP_WRITE_CHECK(getOwnerScene());
	PX_CHECK_AND_RETURN(positionIters > 0, "Articulation::setSolverIterationCount: positionIters must be more than zero!");
	PX_CHECK_AND_RETURN(positionIters <= 255, "Articulation::setSolverIterationCount: positionIters must be no greater than 255!");
	PX_CHECK_AND_RETURN(velocityIters > 0, "Articulation::setSolverIterationCount: velocityIters must be more than zero!");
	PX_CHECK_AND_RETURN(velocityIters <= 255, "Articulation::setSolverIterationCount: velocityIters must be no greater than 255!");

	getArticulation().setSolverIterationCounts((velocityIters & 0xff) << 8 | (positionIters & 0xff));
}

void PxArticulationImpl::getSolverIterationCounts(PxU32 & positionIters, PxU32 & velocityIters) const
{
	NP_READ_CHECK(getOwnerScene());
	PxU16 x = getArticulation().getSolverIterationCounts();
	velocityIters = PxU32(x >> 8);
	positionIters = PxU32(x & 0xff);
}

void PxArticulationImpl::setGlobalPose()
{
	PX_CHECK_AND_RETURN(getAPIScene(), "PxArticulation::setGlobalPose: object must be in a scene");
	NP_READ_CHECK(getOwnerScene());

	mArticulation.setGlobalPose();

	//This code is force PVD to update other links position
	if (!mArticulation.isBuffering())
	{
		physx::NpArticulationLink*const* links = getLinks();

		const PxU32 nbLinks = getNbLinks();
		for (PxU32 i = 1; i < nbLinks; ++i)
		{
			physx::NpArticulationLink* link = links[i];
			//in the lowlevel articulation, we have already updated bodyCore's body2World
			const PxTransform internalPose = link->getScbBodyFast().getScBody().getBody2World();
			link->getScbBodyFast().setBody2World(internalPose, false);
		}
	}
}

bool PxArticulationImpl::isSleeping() const
{
	NP_READ_CHECK(getOwnerScene());
	PX_CHECK_AND_RETURN_VAL(getAPIScene(), "Articulation::isSleeping: articulation must be in a scene.", true);

	return getArticulation().isSleeping();
}

void PxArticulationImpl::setSleepThreshold(PxReal threshold)
{
	NP_WRITE_CHECK(getOwnerScene());
	getArticulation().setSleepThreshold(threshold);
}

PxReal PxArticulationImpl::getSleepThreshold() const
{
	NP_READ_CHECK(getOwnerScene());
	return getArticulation().getSleepThreshold();
}

void PxArticulationImpl::setStabilizationThreshold(PxReal threshold)
{
	NP_WRITE_CHECK(getOwnerScene());
	getArticulation().setFreezeThreshold(threshold);
}

PxReal PxArticulationImpl::getStabilizationThreshold() const
{
	NP_READ_CHECK(getOwnerScene());
	return getArticulation().getFreezeThreshold();
}

void PxArticulationImpl::setWakeCounter(PxReal wakeCounterValue)
{
	NP_WRITE_CHECK(getOwnerScene());

	for (PxU32 i = 0; i < mArticulationLinks.size(); i++)
	{
		mArticulationLinks[i]->getScbBodyFast().setWakeCounter(wakeCounterValue);
	}

	getArticulation().setWakeCounter(wakeCounterValue);
}

PxReal PxArticulationImpl::getWakeCounter() const
{
	NP_READ_CHECK(getOwnerScene());
	return getArticulation().getWakeCounter();
}

void PxArticulationImpl::wakeUpInternal(bool forceWakeUp, bool autowake)
{
	NpScene* scene = getAPIScene();
	PX_ASSERT(scene);
	PxReal wakeCounterResetValue = scene->getWakeCounterResetValueInteral();

	Scb::Articulation& a = getArticulation();
	PxReal wakeCounter = a.getWakeCounter();

	bool needsWakingUp = isSleeping() && (autowake || forceWakeUp);
	if (autowake && (wakeCounter < wakeCounterResetValue))
	{
		wakeCounter = wakeCounterResetValue;
		needsWakingUp = true;
	}

	if (needsWakingUp)
	{
		for (PxU32 i = 0; i < mArticulationLinks.size(); i++)
		{
			mArticulationLinks[i]->getScbBodyFast().wakeUpInternal(wakeCounter);
		}

		a.wakeUpInternal(wakeCounter);
	}
}

void PxArticulationImpl::wakeUp()
{
	NpScene* scene = getAPIScene();

	NP_WRITE_CHECK(getOwnerScene());  // don't use the API scene, since it will be NULL on pending removal
	PX_CHECK_AND_RETURN(scene, "Articulation::wakeUp: articulation must be in a scene.");

	for (PxU32 i = 0; i < mArticulationLinks.size(); i++)
	{
		mArticulationLinks[i]->getScbBodyFast().wakeUpInternal(scene->getWakeCounterResetValueInteral());
	}

	getArticulation().wakeUp();
}

void PxArticulationImpl::putToSleep()
{
	NP_WRITE_CHECK(getOwnerScene());
	PX_CHECK_AND_RETURN(getAPIScene(), "Articulation::putToSleep: articulation must be in a scene.");

	for (PxU32 i = 0; i < mArticulationLinks.size(); i++)
	{
		mArticulationLinks[i]->getScbBodyFast().putToSleepInternal();
	}

	getArticulation().putToSleep();
}

PxU32 PxArticulationImpl::getNbLinks() const
{
	NP_READ_CHECK(getOwnerScene());
	return mArticulationLinks.size();
}

PxU32 PxArticulationImpl::getLinks(PxArticulationLink** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	NP_READ_CHECK(getOwnerScene());
	return Cm::getArrayOfPointers(userBuffer, bufferSize, startIndex, mArticulationLinks.begin(), mArticulationLinks.size());
}

PxBounds3 PxArticulationImpl::getWorldBounds(float inflation) const
{
	NP_READ_CHECK(getOwnerScene());
	PxBounds3 bounds = PxBounds3::empty();

	for (PxU32 i = 0; i < mArticulationLinks.size(); i++)
	{
		bounds.include(mArticulationLinks[i]->getWorldBounds());
	}
	PX_ASSERT(bounds.isValid());

	// PT: unfortunately we can't just scale the min/max vectors, we need to go through center/extents.
	const PxVec3 center = bounds.getCenter();
	const PxVec3 inflatedExtents = bounds.getExtents() * inflation;
	return PxBounds3::centerExtents(center, inflatedExtents);
}

PxAggregate* PxArticulationImpl::getAggregate() const
{
	NP_READ_CHECK(getOwnerScene());
	return mAggregate;
}

void PxArticulationImpl::setName(const char* debugName)
{
	NP_WRITE_CHECK(getOwnerScene());
	mName = debugName;
}

const char* PxArticulationImpl::getName() const
{
	NP_READ_CHECK(getOwnerScene());
	return mName;
}

NpScene* PxArticulationImpl::getAPIScene() const
{
	return static_cast<NpScene*>(mArticulation.getScbSceneForAPI() ? mArticulation.getScbSceneForAPI()->getPxScene() : NULL);
}

NpScene* PxArticulationImpl::getOwnerScene() const
{
	return static_cast<NpScene*>(mArticulation.getScbScene() ? mArticulation.getScbScene()->getPxScene() : NULL);
}

#if PX_ENABLE_DEBUG_VISUALIZATION

void PxArticulationImpl::visualize(Cm::RenderOutput& out, NpScene* scene)
{
	for (PxU32 i = 0; i<mArticulationLinks.size(); i++)
		mArticulationLinks[i]->visualize(out, scene);
}
#endif  // PX_ENABLE_DEBUG_VISUALIZATION

NpArticulationLink* PxArticulationImpl::getRoot()
{
	if (!mArticulationLinks.size())
		return NULL;

	PX_ASSERT(mArticulationLinks[0]->getInboundJoint() == NULL);
	return mArticulationLinks[0];
}

}

#endif //NP_ARTICULATION_TEMPLATE

