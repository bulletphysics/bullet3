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

#include "NpCast.h"
#include "ScbScene.h"
#include "ScbRigidStatic.h"
#include "ScbBody.h"
#include "ScbShape.h"
#include "ScbConstraint.h"
#include "ScbArticulation.h"
#include "ScbArticulationJoint.h"
#include "ScbNpDeps.h"
#include "ScbAggregate.h"

#include "PsFoundation.h"
#include "PxArticulation.h"
#include "common/PxProfileZone.h"

namespace physx
{
	class NpMaterial;
}

using namespace physx;

// constants to make boolean template parameters more readable
static const bool tSimRunning = true;
static const bool tAdd = true;
static const bool tDynamic = true;
static const bool tNonSimObject = true;
static const bool tSyncOnRemove = true;
static const bool tWakeOnLostTouchCheck = true;

void Scb::ObjectTracker::scheduleForInsert(Scb::Base& element)
{
	ControlState::Enum state = element.getControlState();
	PxU32 flags = element.getControlFlags();
	PX_ASSERT(!(flags & ControlFlag::eIS_RELEASED));
	PX_ASSERT(state == ControlState::eNOT_IN_SCENE || state == ControlState::eREMOVE_PENDING);

	if(state == ControlState::eREMOVE_PENDING)
	{
		element.setControlState(ControlState::eIN_SCENE);
		if(!(flags & ControlFlag::eIS_UPDATED))
			remove(element);
	}
	else
	{
		PX_ASSERT(!(flags & ControlFlag::eIS_UPDATED));
		element.setControlState(ControlState::eINSERT_PENDING);
		insert(element);
	}
}

void Scb::ObjectTracker::scheduleForRemove(Scb::Base& element)
{
	ControlState::Enum state = element.getControlState();
	PxU32 flags = element.getControlFlags();

	PX_ASSERT(!(flags & ControlFlag::eIS_RELEASED));

	if(state == ControlState::eINSERT_PENDING)
	{
		// if it's inserted this frame, just remove it - it can't be dirty
		//ML: this assert wont' work because buffered insert raises this flag. We have a unit test which called TEST_F(ObserverTest, OnRelease) to verify it
		//PX_ASSERT(!(flags & ControlFlag::eIS_UPDATED));
		element.setControlState(ControlState::eNOT_IN_SCENE);
		remove(element);
	}
	else if(state == ControlState::eIN_SCENE)
	{
		element.setControlState(ControlState::eREMOVE_PENDING);
		if(!(flags & ControlFlag::eIS_UPDATED))
			insert(element);
	} 
	else
	{
		PX_ALWAYS_ASSERT_MESSAGE("Trying to remove element not in scene.");
	}
}

void Scb::ObjectTracker::scheduleForUpdate(Scb::Base& element)
{
	ControlState::Enum state = element.getControlState();
	PxU32 flags = element.getControlFlags();

	PX_ASSERT(!(flags & ControlFlag::eIS_RELEASED));
	PX_ASSERT(state == ControlState::eIN_SCENE || state == ControlState::eREMOVE_PENDING || state == ControlState::eINSERT_PENDING);

	if(!(flags & ControlFlag::eIS_UPDATED))
	{
		element.setControlFlag(ControlFlag::eIS_UPDATED);
		if(state == ControlState::eIN_SCENE)
			insert(element);
	}
}
	
void Scb::ObjectTracker::clear()							
{	
	Scb::Base *const * elements = mBuffered.getEntries();
	for(PxU32 i=0;i<mBuffered.size();i++)
	{
		ControlState::Enum state = elements[i]->getControlState();
		PxU32 flags = elements[i]->getControlFlags();

		if(state == ControlState::eIN_SCENE || state == ControlState::eINSERT_PENDING)
			elements[i]->resetControl(ControlState::eIN_SCENE);
		else
		{
			elements[i]->resetControl(ControlState::eNOT_IN_SCENE);
			elements[i]->setScbScene(NULL);
		}

		if(flags & ControlFlag::eIS_RELEASED)
			NpDestroy(*elements[i]);
	}
	mBuffered.clear();
}		

void Scb::ObjectTracker::insert(Scb::Base& element)
{
	PX_ASSERT(!mBuffered.contains(&element));
	mBuffered.insert(&element);
}

void Scb::ObjectTracker::remove(Scb::Base& element)
{
	mBuffered.erase(&element);
}

///////////////////////////////////////////////////////////////////////////////

template <bool TSimRunning, bool TAdd, bool TIsDynamic, bool TIsNonSimObject, class T>
PX_FORCE_INLINE static void addOrRemoveRigidObject(Sc::Scene& s, T& rigidObject, bool wakeOnLostTouch, PxBounds3* uninflatedBounds, const Gu::BVHStructure* bvhStructure);

template <typename T>struct ScSceneFns {};

template<> struct ScSceneFns<Scb::Articulation>
{
	static PX_FORCE_INLINE void insert(Sc::Scene& s, Scb::Articulation& v, PxBounds3*, const Gu::BVHStructure*)
	{ 
		Scb::Body* b = NpArticulationGetRootFromScb(v);
		s.addArticulation(v.getScArticulation(), b->getScBody());
	}
	static PX_FORCE_INLINE void remove(Sc::Scene& s, Scb::Articulation& v, bool wakeOnLostTouch)
	{
		PX_UNUSED(wakeOnLostTouch);

		v.clearBufferedSleepStateChange();  // see comment in remove code of Scb::Body

		s.removeArticulation(v.getScArticulation());
	}
};

template<> struct ScSceneFns<Scb::ArticulationJoint>
{
	static PX_FORCE_INLINE void insert(Sc::Scene& s, Scb::ArticulationJoint& v, PxBounds3*, const Gu::BVHStructure*)	
	{ 
		Scb::Body* scb0, * scb1;
		NpArticulationJointGetBodiesFromScb(v, scb0, scb1);
		s.addArticulationJoint(v.getScArticulationJoint(), scb0->getScBody(), scb1->getScBody());		
	}
	static PX_FORCE_INLINE void remove(Sc::Scene& s, Scb::ArticulationJoint& v, bool wakeOnLostTouch)
	{
		PX_UNUSED(wakeOnLostTouch);
		s.removeArticulationJoint(v.getScArticulationJoint()); 
	}
};

template<> struct ScSceneFns<Scb::Constraint>
{
	static PX_FORCE_INLINE void insert(Sc::Scene& s, Scb::Constraint& v, PxBounds3*, const Gu::BVHStructure*)	
	{ 
		Scb::RigidObject* scb0, * scb1;
		NpConstraintGetRigidObjectsFromScb(v, scb0, scb1);

		PX_ASSERT((!scb0) || (!(scb0->getActorFlags() & PxActorFlag::eDISABLE_SIMULATION)));
		PX_ASSERT((!scb1) || (!(scb1->getActorFlags() & PxActorFlag::eDISABLE_SIMULATION)));
		
		s.addConstraint(v.getScConstraint(), scb0 ? &scb0->getScRigidCore() : NULL, scb1 ? &scb1->getScRigidCore() : NULL);
	}
	static PX_FORCE_INLINE void remove(Sc::Scene& s, Scb::Constraint& v, bool wakeOnLostTouch)
	{
		PX_UNUSED(wakeOnLostTouch);
		s.removeConstraint(v.getScConstraint());
	}
};

template<> struct ScSceneFns<Scb::RigidStatic>
{
	static PX_FORCE_INLINE void insert(Sc::Scene& s, Scb::RigidStatic& v, PxBounds3* uninflatedBounds, const Gu::BVHStructure* bvhStructure)
	{
		// important to use the buffered flags because for a pending insert those describe the end state the
		// user expects.
		
		if (!(v.getActorFlags() & PxActorFlag::eDISABLE_SIMULATION))
			addOrRemoveRigidObject<!tSimRunning, tAdd, !tDynamic, !tNonSimObject>(s, v, false, uninflatedBounds, bvhStructure);
		else
			addOrRemoveRigidObject<!tSimRunning, tAdd, !tDynamic, tNonSimObject>(s, v, false, uninflatedBounds, bvhStructure);
	}
	static PX_FORCE_INLINE void remove(Sc::Scene& s, Scb::RigidStatic& v, bool wakeOnLostTouch)
	{
		// important to use the original flags because for a pending removal those describe the original state that needs
		// to get cleaned up.
		
		if (!v.isSimDisabledInternally())
			addOrRemoveRigidObject<!tSimRunning, !tAdd, !tDynamic, !tNonSimObject>(s, v, wakeOnLostTouch, NULL, NULL);
		else
			addOrRemoveRigidObject<!tSimRunning, !tAdd, !tDynamic, tNonSimObject>(s, v, false, NULL, NULL);
	}
};

template<> struct ScSceneFns<Scb::Body>
{
	static PX_FORCE_INLINE void insert(Sc::Scene& s, Scb::Body& v, PxBounds3* uninflatedBounds, const Gu::BVHStructure* bvhStructure)
	{
		// see comments in rigid static case
		if (!(v.getActorFlags() & PxActorFlag::eDISABLE_SIMULATION))
			addOrRemoveRigidObject<!tSimRunning, tAdd, tDynamic, !tNonSimObject>(s, v, false, uninflatedBounds, bvhStructure);
		else
			addOrRemoveRigidObject<!tSimRunning, tAdd, tDynamic, tNonSimObject>(s, v, false, uninflatedBounds, bvhStructure);
	}
	static PX_FORCE_INLINE void remove(Sc::Scene& s, Scb::Body& v, bool wakeOnLostTouch)	
	{
		// strictly speaking, the following is only necessary for pending removes but it does not have a
		// functional side effect if applied all the time.
		// When an object gets removed from the scene, pending wakeUp/putToSleep events should be ignored because
		// the internal sleep state for a free standing object is specified as sleeping. All the other parameter changes
		// that go along with a sleep state change should still get processed though (zero vel, zero wake counter on
		// putToSleep for example). Those are not affected because they are tracked through buffered updates
		// of the velocity and wake counter.
		// The clearing happens here because only here we are sure that the object does get removed for real. At earlier
		// stages someone might remove and then re-insert and object and for such cases it is important to keep the
		// sleep state change buffered.
		v.clearBufferedSleepStateChange();

		// see comments in rigid static case
		if (!v.isSimDisabledInternally())
			addOrRemoveRigidObject<!tSimRunning, !tAdd, tDynamic, !tNonSimObject>(s, v, wakeOnLostTouch, NULL, NULL);
		else
			addOrRemoveRigidObject<!tSimRunning, !tAdd, tDynamic, tNonSimObject>(s, v, false, NULL, NULL);
	}
};

///////////////////////////////////////////////////////////////////////////////
#if PX_SUPPORT_PVD
template<typename T> struct PvdFns 
{	
	// PT: in the following functions, checkPvdDebugFlag() is done by the callers to save time when functions are called from a loop.

	static void createInstance(Scb::Scene& scene, Vd::ScbScenePvdClient& d, T* v)
	{
		PX_PROFILE_ZONE("PVD.createPVDInstance", scene.getContextId());
		PX_UNUSED(scene);
		d.createPvdInstance(v);
	}

	static void updateInstance(Scb::Scene& scene, Vd::ScbScenePvdClient& d, T* v) 
	{ 
		PX_UNUSED(scene);
		if(((v->getControlFlags() & Scb::ControlFlag::eIS_RELEASED) == 0) && (v->getControlState() != Scb::ControlState::eREMOVE_PENDING))
		{ 
			PX_PROFILE_ZONE("PVD.updatePVDProperties", scene.getContextId());
			d.updatePvdProperties(v); 
		} 
	}

	static void releaseInstance(Scb::Scene& scene, Vd::ScbScenePvdClient& d, T* v) 
	{ 
		PX_UNUSED(scene);
		PX_PROFILE_ZONE("PVD.releasePVDInstance", scene.getContextId());
		d.releasePvdInstance(v); 
	} 
};

	#define CREATE_PVD_INSTANCE(obj) \
	{ \
		if(mScenePvdClient.checkPvdDebugFlag()) \
		{ \
			PX_PROFILE_ZONE("PVD.createPVDInstance", getContextId());\
			mScenePvdClient.createPvdInstance(obj); \
		} \
	}
	#define RELEASE_PVD_INSTANCE(obj) \
	{ \
		if(mScenePvdClient.checkPvdDebugFlag()) \
		{ \
			PX_PROFILE_ZONE("PVD.releasePVDInstance", getContextId());\
			mScenePvdClient.releasePvdInstance(obj); \
		} \
	}
	#define UPDATE_PVD_PROPERTIES(obj) \
	{ \
		if(mScenePvdClient.checkPvdDebugFlag()) \
		{ \
			PX_PROFILE_ZONE("PVD.updatePVDProperties", getContextId());\
			mScenePvdClient.updatePvdProperties(obj); \
		} \
	}
	#define PVD_ORIGIN_SHIFT(shift) \
	{ \
		if(mScenePvdClient.checkPvdDebugFlag()) \
		{ \
			PX_PROFILE_ZONE("PVD.originShift", getContextId());\
			mScenePvdClient.originShift(shift); \
		} \
	}
#else
	#define CREATE_PVD_INSTANCE(obj) {}
	#define RELEASE_PVD_INSTANCE(obj) {}
	#define UPDATE_PVD_PROPERTIES(obj) {}
	#define PVD_ORIGIN_SHIFT(shift){}
#endif

///////////////////////////////////////////////////////////////////////////////

Scb::Scene::Scene(const PxSceneDesc& desc, PxU64 contextID) :
	mScene					(desc, contextID),
	mSimulationRunning		(false),
	mIsBuffering			(false),
	mStream					(16384),
	mShapeMaterialBuffer	(PX_DEBUG_EXP("shapeMaterialBuffer")),
	mShapePtrBuffer			(PX_DEBUG_EXP("shapePtrBuffer")),
	mActorPtrBuffer			(PX_DEBUG_EXP("actorPtrBuffer")),
#if PX_SUPPORT_PVD
	mScenePvdClient			(*this),
#endif
	mWakeCounterResetValue	(desc.wakeCounterResetValue),
	mBufferFlags			(0)
{
}

void Scb::Scene::release()
{
#if PX_SUPPORT_PVD
	mScenePvdClient.releasePvdInstance();
#endif
	mScene.release();
	mShapeMaterialBuffer.clear();
	mShapePtrBuffer.clear();
	mActorPtrBuffer.clear();
	mStream.clear();
}

// PT: TODO: inline this
PxScene* Scb::Scene::getPxScene()
{ 
	return const_cast<NpScene*>(getNpScene(this));
}

///////////////////////////////////////////////////////////////////////////////
template<typename T>
void Scb::Scene::add(T& v, ObjectTracker &tracker, PxBounds3* uninflatedBounds, const Gu::BVHStructure* bvhStructure)
{
	v.setScbScene(this);

	if (!mIsBuffering)
	{
		v.resetControl(ControlState::eIN_SCENE);
		ScSceneFns<T>::insert(mScene, v, uninflatedBounds, bvhStructure);
#if PX_SUPPORT_PVD
		if(mScenePvdClient.checkPvdDebugFlag())
			PvdFns<T>::createInstance(*this, mScenePvdClient, &v);
#endif
	}
	else
		tracker.scheduleForInsert(v);
}

template<typename T>
void Scb::Scene::remove(T& v, ObjectTracker &tracker, bool wakeOnLostTouch)
{
	if (!mIsBuffering)
	{
		ScSceneFns<T>::remove(mScene, v, wakeOnLostTouch);
#if PX_SUPPORT_PVD
		if(mScenePvdClient.checkPvdDebugFlag())	
		   PvdFns<T>::releaseInstance(*this, mScenePvdClient, &v);
#endif
		v.resetControl(ControlState::eNOT_IN_SCENE);
		v.setScbScene(NULL);
	}
	else
	{
		tracker.scheduleForRemove(v);
	}
}


///////////////////////////////////////////////////////////////////////////////

template<bool TIsDynamic, class T>
void Scb::Scene::addRigidNoSim(T& v, ObjectTracker &tracker, const Gu::BVHStructure* bvhStructure)
{
	PX_ASSERT(v.getActorFlags() & PxActorFlag::eDISABLE_SIMULATION);
	v.setScbScene(this);

	if (!mIsBuffering)
	{
		v.resetControl(ControlState::eIN_SCENE);
#if PX_SUPPORT_PVD
		if(mScenePvdClient.checkPvdDebugFlag())
			PvdFns<T>::createInstance(*this, mScenePvdClient, &v);		
#endif
		addOrRemoveRigidObject<!tSimRunning, tAdd, TIsDynamic, tNonSimObject>(mScene, v, false, NULL, bvhStructure);
	}
	else
	{
		tracker.scheduleForInsert(v);
		addOrRemoveRigidObject<tSimRunning, tAdd, TIsDynamic, tNonSimObject>(mScene, v, false, NULL, bvhStructure);
	}
}


template<bool TIsDynamic, class T>
void Scb::Scene::removeRigidNoSim(T& v, ObjectTracker &tracker)
{
	PX_ASSERT(v.isSimDisabledInternally());

	if (!mIsBuffering)
	{
		addOrRemoveRigidObject<!tSimRunning, !tAdd, TIsDynamic, tNonSimObject>(mScene, v, false, NULL, NULL);
#if PX_SUPPORT_PVD
		if(mScenePvdClient.checkPvdDebugFlag())
			PvdFns<T>::releaseInstance(*this, mScenePvdClient, &v);
#endif
		v.resetControl(ControlState::eNOT_IN_SCENE);
		v.setScbScene(NULL);
	}
	else
	{
		tracker.scheduleForRemove(v);
		addOrRemoveRigidObject<tSimRunning, !tAdd, TIsDynamic, tNonSimObject>(mScene, v, false, NULL, NULL);
	}
}


void Scb::Scene::switchRigidToNoSim(Scb::RigidObject& r, bool isDynamic)
{
	PX_ASSERT(!mIsBuffering);

	// when a simulation objects has a pending remove and then gets switched to a non-simulation object,
	// we must not process the code below. On sync the object will get removed before this call.
	if (r.getControlState() == ControlState::eIN_SCENE)
	{
		size_t ptrOffset = -Scb::Shape::getScOffset();
		Ps::InlineArray<const Sc::ShapeCore*, 64> scShapes;

		if (isDynamic)
			mScene.removeBody(static_cast<Sc::BodyCore&>(r.getScRigidCore()), scShapes, true);
		else
			mScene.removeStatic(static_cast<Sc::StaticCore&>(r.getScRigidCore()), scShapes, true);

		// not in simulation anymore -> decrement shape ref-counts
		void* const* shapes = reinterpret_cast<void*const*>(const_cast<Sc::ShapeCore*const*>(scShapes.begin()));
		for(PxU32 i=0; i < scShapes.size(); i++)
		{
			Scb::Shape& scbShape = *Ps::pointerOffset<Scb::Shape*>(shapes[i], ptrdiff_t(ptrOffset));
			NpShapeDecRefCount(scbShape);
		}
	}
}


void Scb::Scene::switchRigidFromNoSim(Scb::RigidObject& r, bool isDynamic)
{
	PX_ASSERT(!mIsBuffering);

	// when a non-simulation objects has a pending remove and then gets switched to a simulation object,
	// we must not process the code below. On sync the object will get removed before this call.
	if (r.getControlState() == ControlState::eIN_SCENE)
	{
		void* const* shapes;
		size_t shapePtrOffset = NpShapeGetScPtrOffset();
		size_t ptrOffset = shapePtrOffset - Scb::Shape::getScOffset();

		PxU32 nbShapes;
		if (isDynamic)
		{
			bool isCompound;
			nbShapes = NpRigidDynamicGetShapes(static_cast<Scb::Body&>(r), shapes, &isCompound);
			mScene.addBody(static_cast<Sc::BodyCore&>(r.getScRigidCore()), shapes, nbShapes, shapePtrOffset, NULL, isCompound);
		}
		else
		{
			nbShapes = NpRigidStaticGetShapes(static_cast<Scb::RigidStatic&>(r), shapes);
			mScene.addStatic(static_cast<Sc::StaticCore&>(r.getScRigidCore()), shapes, nbShapes, shapePtrOffset, NULL);
		}

		// add to simulation -> increment shape ref-counts
		for(PxU32 i=0; i < nbShapes; i++)
		{
			Scb::Shape& scbShape = *Ps::pointerOffset<Scb::Shape*>(shapes[i], ptrdiff_t(ptrOffset));
			NpShapeIncRefCount(scbShape);
		}
	}
}


///////////////////////////////////////////////////////////////////////////////

// PT: TODO:
// - consider making these templates not scene member functions

template<bool TIsDynamic, class T>
PX_FORCE_INLINE void Scb::Scene::addActorT(T& actor, Scb::ObjectTracker& tracker, bool noSim, PxBounds3* uninflatedBounds, const Gu::BVHStructure* bvhStructure)
{
	PX_PROFILE_ZONE("API.addActorToSim", getContextId());
	if(!noSim)
	{
		add<T>(actor, tracker, uninflatedBounds, bvhStructure);
		actor.initBufferedState();

		// copy buffer control state from rigid object to shapes and set scene
		if(mIsBuffering)
			addOrRemoveRigidObject<tSimRunning, tAdd, TIsDynamic, !tNonSimObject>(mScene, actor, false, NULL, bvhStructure);
	}
	else
	{
		addRigidNoSim<TIsDynamic>(actor, tracker, bvhStructure);
		actor.initBufferedState();
	}
}

void Scb::Scene::addActor(Scb::RigidStatic& rigidStatic, bool noSim, PxBounds3* uninflatedBounds, const Gu::BVHStructure* bvhStructure)
{
	addActorT<false>(rigidStatic, mRigidStaticManager, noSim, uninflatedBounds, bvhStructure);
}

void Scb::Scene::addActor(Scb::Body& body, bool noSim, PxBounds3* uninflatedBounds, const Gu::BVHStructure* bvhStructure)
{
	addActorT<true>(body, mBodyManager, noSim, uninflatedBounds, bvhStructure);
}

///////////////////////////////////////////////////////////////////////////////

void Scb::Scene::removeActor(Scb::RigidStatic& rigidStatic, bool wakeOnLostTouch, bool noSim)
{
	PX_PROFILE_ZONE("API.removeActorFromSim", getContextId());
	if (!noSim)
	{
		remove<Scb::RigidStatic>(rigidStatic, mRigidStaticManager, wakeOnLostTouch);

		// copy buffer control state from rigid object to shapes and set scene
		if (mIsBuffering)
		{
			if (wakeOnLostTouch)
				rigidStatic.scheduleForWakeTouching();
			addOrRemoveRigidObject<tSimRunning, !tAdd, !tDynamic, !tNonSimObject>(mScene, rigidStatic, wakeOnLostTouch, NULL, NULL);
		}
	}
	else
	{
		removeRigidNoSim<!tDynamic>(rigidStatic, mRigidStaticManager);
	}

	rigidStatic.clearBufferedState();
}

void Scb::Scene::removeActor(Scb::Body& body, bool wakeOnLostTouch, bool noSim)
{
	PX_PROFILE_ZONE("API.removeActorFromSim", getContextId());
	if (!noSim)
	{
		body.clearSimStateDataForPendingInsert();

		remove<Scb::Body>(body, mBodyManager, wakeOnLostTouch);
		body.clearBufferedState();

		// copy buffer control state from rigid object to shapes and set scene
		if (mIsBuffering)
		{
			if (wakeOnLostTouch)
				body.scheduleForWakeTouching();
			addOrRemoveRigidObject<tSimRunning, !tAdd, tDynamic, !tNonSimObject>(mScene, body, wakeOnLostTouch, NULL, NULL);
		}
	}
	else
	{
		removeRigidNoSim<tDynamic>(body, mBodyManager);

		// note: "noSim" refers to the internal state here. The following asserts only apply if the bufferd state has not switched to "sim".
		PX_ASSERT(!(body.getActorFlags() & PxActorFlag::eDISABLE_SIMULATION) || body.isSleeping());
		PX_ASSERT(!(body.getActorFlags() & PxActorFlag::eDISABLE_SIMULATION) || !body.isBuffered(BodyBuffer::BF_KinematicTarget | BodyBuffer::BF_Acceleration | BodyBuffer::BF_DeltaVelocity));
		// What about velocity, wakeCounter, ...?
		// Those are not allowed on a no-sim object, however, they might still be necessary due to complex buffering scenarios:
		// Imagine the following operation flow (all buffered):
		// - dynamic sim object awake with velocities
		// - switch to no-sim -> needs to clear velocities, wake counter, put to sleep, ...
		// - switch back to sim -> the velocities, wake counter, ... still need to get cleared and it needs to be asleep (that would be the non-buffered behavior of the operations)

		body.clearBufferedState();	// this also covers the buffered case where a noSim object gets switched to a sim object, followed by a wakeUp() call and then a remove. 
									// If we checked whether the buffered object is still a noSim object then only body.RigidObject::clearBufferedState() would be necessary.
	}
}

///////////////////////////////////////////////////////////////////////////////

void Scb::Scene::addConstraint(Scb::Constraint& constraint)
{
	add<Scb::Constraint>(constraint, mConstraintManager, NULL, NULL);
}

void Scb::Scene::removeConstraint(Scb::Constraint& constraint)
{
	if (!mIsBuffering)
	{
		mScene.removeConstraint(constraint.getScConstraint());
	
		// Release pvd constraint immediately since delayed removal with already released ext::joints does not work, can't call callback.
		if(constraint.getControlState() != ControlState::eINSERT_PENDING)
			RELEASE_PVD_INSTANCE(&constraint)

		constraint.resetControl(ControlState::eNOT_IN_SCENE);
		constraint.setScbScene(NULL);
	}
	else
	{
		mConstraintManager.scheduleForRemove(constraint);
	}
}

///////////////////////////////////////////////////////////////////////////////

void Scb::Scene::addArticulation(Scb::Articulation& articulation)
{
	add<Scb::Articulation>(articulation, mArticulationManager, NULL, NULL);
	articulation.initBufferedState();
}

void Scb::Scene::removeArticulation(Scb::Articulation& articulation)
{
	remove<Scb::Articulation>(articulation, mArticulationManager);
	articulation.clearBufferedState();
}

///////////////////////////////////////////////////////////////////////////////

void Scb::Scene::addArticulationJoint(Scb::ArticulationJoint& joint)
{
	add<Scb::ArticulationJoint>(joint, mArticulationJointManager, NULL, NULL);
}

void Scb::Scene::removeArticulationJoint(Scb::ArticulationJoint& joint)
{
	remove<Scb::ArticulationJoint>(joint, mArticulationJointManager);
}

///////////////////////////////////////////////////////////////////////////////

void Scb::Scene::addAggregate(Scb::Aggregate& agg)
{
	agg.setScbScene(this);

	if (!mIsBuffering)
	{
		PxU32 aggregateID = mScene.createAggregate(agg.mPxAggregate, agg.getSelfCollide());
		agg.setAggregateID(aggregateID);
		agg.resetControl(ControlState::eIN_SCENE);	
#if PX_SUPPORT_PVD
		//Sending pvd events after all aggregates's actors are inserted into scene
		mScenePvdClient.createPvdInstance(&agg);
#endif
	}
	else
		mAggregateManager.scheduleForInsert(agg);
}


void Scb::Scene::removeAggregate(Scb::Aggregate& agg)
{
	if (!mIsBuffering)
	{
		mScene.deleteAggregate(agg.getAggregateID());
		agg.resetControl(ControlState::eNOT_IN_SCENE);
		agg.setScbScene(NULL);
#if PX_SUPPORT_PVD
		mScenePvdClient.releasePvdInstance(&agg);
#endif
	}
	else
	{
		mAggregateManager.scheduleForRemove(agg);
	}
}


void Scb::Scene::addMaterial(const Sc::MaterialCore& material)
{
	Ps::Mutex::ScopedLock lock(mSceneMaterialBufferLock);

	mSceneMaterialBuffer.pushBack(MaterialEvent(material.getMaterialIndex(), MATERIAL_ADD));
	
	CREATE_PVD_INSTANCE(&material)
}

void Scb::Scene::updateMaterial(const Sc::MaterialCore& material)
{
	Ps::Mutex::ScopedLock lock(mSceneMaterialBufferLock);

	mSceneMaterialBuffer.pushBack(MaterialEvent(material.getMaterialIndex(), MATERIAL_UPDATE));

	UPDATE_PVD_PROPERTIES(&material)
}

void Scb::Scene::removeMaterial(const Sc::MaterialCore& material)
{
	if(material.getMaterialIndex() == MATERIAL_INVALID_HANDLE)
		return;

	Ps::Mutex::ScopedLock lock(mSceneMaterialBufferLock);

	mSceneMaterialBuffer.pushBack(MaterialEvent(material.getMaterialIndex(), MATERIAL_REMOVE));

	RELEASE_PVD_INSTANCE(&material);
}

///////////////////////////////////////////////////////////////////////////////

void Scb::Scene::updateLowLevelMaterial(NpMaterial** masterMaterial)
{
	Ps::Mutex::ScopedLock lock(mSceneMaterialBufferLock);

	//sync all the material events
	PxsMaterialManager& manager = mScene.getMaterialManager();
	for(PxU32 i=0; i< mSceneMaterialBuffer.size(); ++i)
	{
		const MaterialEvent& event = mSceneMaterialBuffer[i];
		const NpMaterial* masMat = masterMaterial[event.mHandle];
		switch(event.mType)
		{
		case MATERIAL_ADD:
			if(masMat)
			{
				Sc::MaterialCore* materialCore = &masterMaterial[event.mHandle]->getScMaterial();
				manager.setMaterial(materialCore);
				mScene.registerMaterialInNP(*materialCore);
			}
			break;
		case MATERIAL_UPDATE:
			if(masMat)
			{
				Sc::MaterialCore* materialCore = &masterMaterial[event.mHandle]->getScMaterial();
				manager.updateMaterial(materialCore);
				mScene.updateMaterialInNP(*materialCore);
			}
			break;
		case MATERIAL_REMOVE:
			if (event.mHandle < manager.getMaxSize())	// materials might get added and then removed again immediately. However, the add does not get processed (see case MATERIAL_ADD above),
			{											// so the remove might end up reading out of bounds memory unless checked.
				PxsMaterialCore* materialCore = manager.getMaterial(event.mHandle);
				if (materialCore->getMaterialIndex() == event.mHandle)
				{
					mScene.unregisterMaterialInNP(*materialCore);
					manager.removeMaterial(materialCore);
				}
			}
			break;
		};
	}

	mSceneMaterialBuffer.resize(0);
}

//--------------------------------------------------------------
//
// Data synchronization
//
//--------------------------------------------------------------
void Scb::Scene::syncState()
{
	//process client creation -- must be done before BF_CLIENT_BEHAVIOR_FLAGS processing in the below block:
	while(mBufferedData.mNumClientsCreated)
	{
		mScene.createClient();
		mBufferedData.mNumClientsCreated--;
	}

	if(mBufferFlags)
	{
		if(isBuffered(BF_GRAVITY))
			mScene.setGravity(mBufferedData.mGravity);

		if(isBuffered(BF_BOUNCETHRESHOLDVELOCITY))
			mScene.setBounceThresholdVelocity(mBufferedData.mBounceThresholdVelocity);

		if(isBuffered(BF_FLAGS))
			mScene.setPublicFlags(mBufferedData.mFlags);

		if(isBuffered(BF_DOMINANCE_PAIRS))
			mBufferedData.syncDominancePairs(mScene);

		if(isBuffered(BF_SOLVER_BATCH_SIZE))
			mScene.setSolverBatchSize(mBufferedData.mSolverBatchSize);

		if(isBuffered(BF_VISUALIZATION))
		{
			for(PxU32 i=0; i<PxVisualizationParameter::eNUM_VALUES; i++)
			{
				if(mBufferedData.mVisualizationParamChanged[i])
					mScene.setVisualizationParameter(PxVisualizationParameter::Enum(i), mBufferedData.mVisualizationParam[i]);
			}

			mBufferedData.clearVisualizationParams();
		}

		if(isBuffered(BF_CULLING_BOX))
			mScene.setVisualizationCullingBox(mBufferedData.mVisualizationCullingBox);

#if PX_SUPPORT_PVD
		if(mScenePvdClient.checkPvdDebugFlag())
			mScenePvdClient.updatePvdProperties();
#endif
		mBufferFlags = 0;
	}
}

template<typename T>
void Scb::Scene::processUserUpdates(ObjectTracker &tracker)
{
#if PX_SUPPORT_PVD
	bool isPvdValid = mScenePvdClient.checkPvdDebugFlag();
#endif
	Base*const * buffered = tracker.getBuffered();
	for(PxU32 i=0; i < tracker.getBufferedCount(); i++)
	{
		T& v = *static_cast<T*>(buffered[i]);
		if (v.getControlState() == ControlState::eINSERT_PENDING)
		{
			ScSceneFns<T>::insert(mScene, v, NULL, NULL);
#if PX_SUPPORT_PVD
			if(isPvdValid)
   				PvdFns<T>::createInstance(*this, mScenePvdClient, &v);
#endif
		}
		else if(v.getControlFlags() & ControlFlag::eIS_UPDATED)
		{
			v.syncState();
#if PX_SUPPORT_PVD
		if(isPvdValid)
   			PvdFns<T>::updateInstance(*this, mScenePvdClient, &v);
#endif
		}
	}
}

template<typename T, typename S>
void Scb::Scene::processSimUpdates(S*const * scObjects, PxU32 nbObjects)
{
#if PX_SUPPORT_PVD
	bool isPvdValid = mScenePvdClient.checkPvdDebugFlag();
#endif
	for(PxU32 i=0;i<nbObjects;i++)
	{
		T& v = T::fromSc(*scObjects[i]);
		
		if(!(v.getControlFlags() & ControlFlag::eIS_UPDATED)) // else the data will be synced further below
		{
			v.syncState();
#if PX_SUPPORT_PVD
		if(isPvdValid) 
   			PvdFns<T>::updateInstance(*this, mScenePvdClient, &v);
#endif
		}
	}
}

#define ENABLE_PVD_ORIGINSHIFT_EVENT
void Scb::Scene::shiftOrigin(const PxVec3& shift)
{ 
	PX_ASSERT(!isPhysicsBuffering()); 
	mScene.shiftOrigin(shift); 
	
#ifdef ENABLE_PVD_ORIGINSHIFT_EVENT
	PVD_ORIGIN_SHIFT(shift);
#endif
}

void Scb::Scene::syncWriteThroughProperties()
{
	mStream.lock();

	Base*const * buffered = mBodyManager.getBuffered();
	PxU32 count = mBodyManager.getBufferedCount();
	for(PxU32 i=0; i < count; i++)
	{
		Scb::Body& bufferedBody = *static_cast<Scb::Body*>(buffered[i]);
		bufferedBody.syncCollisionWriteThroughState();
	}

	mStream.unlock();
}

void Scb::Scene::syncEntireScene()
{
	PX_PROFILE_ZONE("Sim.syncState", getContextId());

	setPhysicsBuffering(false);  // Clear the buffering flag to allow buffered writes to execute immediately. Once collision detection is running, buffering is automatically forced on

	mStream.lock();
	syncState();
	//
	// Process aggregates (needs to be done before adding actors because the actor's aggregateID needs to get set)
	//

	for(PxU32 i=0; i < mAggregateManager.getBufferedCount(); i++)
	{
		Aggregate* a = static_cast<Aggregate*>(mAggregateManager.getBuffered()[i]);
		if (a->getControlState() == ControlState::eINSERT_PENDING)
		{
			PxU32 aggregateID = mScene.createAggregate(a->mPxAggregate, a->getSelfCollide());
			a->setAggregateID(aggregateID);
#if PX_SUPPORT_PVD
			mScenePvdClient.createPvdInstance(a);
#endif
			a->syncState(*this);  // Necessary to set the aggregate ID for all actors of the aggregate
		}
		else if(a->getControlFlags() & ControlFlag::eIS_UPDATED)
		{
			a->syncState(*this);
		}
	}
	mAggregateManager.clear();
	mActorPtrBuffer.clear();


	// rigid statics
	processUserUpdates<Scb::RigidStatic>(mRigidStaticManager);
	mRigidStaticManager.clear();


	// rigid dynamics and articulation links
	//
	// 1) Sync simulation changed data
	{
		PX_PROFILE_ZONE("SyncActiveBodies", getContextId());
		Sc::BodyCore*const* activeBodies = mScene.getActiveBodiesArray();
		PxU32 nbActiveBodies = mScene.getNumActiveBodies();
		while(nbActiveBodies--)
		{
			Sc::BodyCore* bodyCore = *activeBodies++;
			Scb::Body& bufferedBody = Scb::Body::fromSc(*bodyCore);
			if (!(bufferedBody.getControlFlags() & ControlFlag::eIS_UPDATED))  // Else the data will be synced further below
				bufferedBody.syncState();
		}
	}

	// 2) Sync data of rigid dynamics which were put to sleep by the simulation

	PxU32 nbSleepingBodies;
	Sc::BodyCore* const* sleepingBodies = mScene.getSleepBodiesArray(nbSleepingBodies);
	processSimUpdates<Scb::Body, Sc::BodyCore>(sleepingBodies, nbSleepingBodies);

	// user updates
	processUserUpdates<Scb::Body>(mBodyManager);
	mBodyManager.clear();
	mShapePtrBuffer.clear();


	// rigid body shapes
	//
	// IMPORTANT: This has to run after the material update
	//
	// Sync user changed data. Inserts and removes are handled in actor sync
	for(PxU32 i=0; i < mShapeManager.getBufferedCount(); i++)
	{
		Scb::Shape* s = static_cast<Scb::Shape*>(mShapeManager.getBuffered()[i]);

		if(s->getControlFlags() & ControlFlag::eIS_UPDATED)
		{
			s->syncState();
			UPDATE_PVD_PROPERTIES(s)
		}
	}

	mShapeManager.clear();
	mShapeMaterialBuffer.clear();

	// constraints (get break force and broken status from sim)

	processSimUpdates<Scb::Constraint, Sc::ConstraintCore>(mScene.getConstraints(), mScene.getNbConstraints());
	processUserUpdates<Scb::Constraint>(mConstraintManager);
	mConstraintManager.clear();

	// articulations (get sleep state from sim)
	processSimUpdates<Scb::Articulation, Sc::ArticulationCore>(mScene.getArticulations(), mScene.getNbArticulations());
	processUserUpdates<Scb::Articulation>(mArticulationManager);
	mArticulationManager.clear();


	// Process articulation joints
	processUserUpdates<Scb::ArticulationJoint>(mArticulationJointManager);
	mArticulationJointManager.clear();


	mStream.clearNotThreadSafe();
	mStream.unlock();
}



template<typename T, bool syncOnRemove, bool wakeOnLostTouchCheck> 
void Scb::Scene::processRemoves(ObjectTracker& tracker)
{	
#if PX_SUPPORT_PVD
	bool isPvdValid = mScenePvdClient.checkPvdDebugFlag();
#endif
	typedef ScSceneFns<T> Fns;
	for(PxU32 i=0; i < tracker.getBufferedCount(); i++)
	{
		T* v = static_cast<T*>(tracker.getBuffered()[i]);
		if(v->getControlState() == ControlState::eREMOVE_PENDING)
		{
			bool wakeOnLostTouch = false;
			if (wakeOnLostTouchCheck)
			{
				PX_ASSERT(	(v->getScbType() == ScbType::eBODY) ||
							(v->getScbType() == ScbType::eBODY_FROM_ARTICULATION_LINK) ||
							(v->getScbType() == ScbType::eRIGID_STATIC) );
				wakeOnLostTouch = (v->Base::isBuffered(RigidObjectBuffer::BF_WakeTouching) != 0);  // important to use Scb::Base::isBuffered() because Scb::Body, for example, has a shadowed implementation of this method
			}

			Fns::remove(mScene, *v, wakeOnLostTouch);

			// if no object param has been updated, the state sync still needs to be processed to write simulation results
			// back to the permanently buffered params.
			if (syncOnRemove && !(v->getControlFlags() & ControlFlag::eIS_UPDATED))
				v->syncState();
#if PX_SUPPORT_PVD
		   if(isPvdValid) 
     			PvdFns<T>::releaseInstance(*this, mScenePvdClient, v);
#endif
		}
	}
}

template<typename T> 
void Scb::Scene::processShapeRemoves(ObjectTracker& tracker)
{	
	for(PxU32 i=0; i < tracker.getBufferedCount(); i++)
	{
		T* v = static_cast<T*>(tracker.getBuffered()[i]);
		v->processShapeRemoves();
	}
}

void Scb::Scene::processPendingRemove()
{
	processShapeRemoves<Scb::RigidStatic>(mRigidStaticManager);
	processShapeRemoves<Scb::Body>(mBodyManager);

	processRemoves<Scb::Constraint, tSyncOnRemove, !tWakeOnLostTouchCheck>(mConstraintManager);

	Scb::Base *const * buffered = mConstraintManager.getBuffered();
	for(PxU32 i=0; i < mConstraintManager.getBufferedCount(); i++)
	{
		Scb::Constraint* constraint = static_cast<Scb::Constraint*>(buffered[i]);
		
		if(constraint->getControlFlags() & ControlFlag::eIS_UPDATED)
			constraint->prepareForActorRemoval();	// see comments in Scb::Constraint		
	}

	processRemoves<Scb::ArticulationJoint,  !tSyncOnRemove,	!tWakeOnLostTouchCheck>	(mArticulationJointManager);
	processRemoves<Scb::RigidStatic,		!tSyncOnRemove,	tWakeOnLostTouchCheck>	(mRigidStaticManager);
	processRemoves<Scb::Body,				tSyncOnRemove,	tWakeOnLostTouchCheck>	(mBodyManager);
	processRemoves<Scb::Articulation,		tSyncOnRemove,	!tWakeOnLostTouchCheck>	(mArticulationManager);

	// Do after actors have been removed (coumpound can only be removed after all its elements are gone)
	for(PxU32 i=0; i < mAggregateManager.getBufferedCount(); i++)
	{
		Aggregate* a = static_cast<Aggregate*>(mAggregateManager.getBuffered()[i]);

		if(a->getControlState() == ControlState::eREMOVE_PENDING)
		{
			a->syncState(*this);  // Clears the aggregate ID for all actors of the aggregate
			mScene.deleteAggregate(a->getAggregateID());

#if PX_SUPPORT_PVD
			mScenePvdClient.releasePvdInstance(a);
#endif
		}
	}
}

void Scb::Scene::scheduleForUpdate(Scb::Base& object)
{
	switch(object.getScbType())
	{
		case ScbType::eSHAPE_EXCLUSIVE:
		case ScbType::eSHAPE_SHARED:				{ mShapeManager.scheduleForUpdate(object);				}break;
		case ScbType::eBODY:						{ mBodyManager.scheduleForUpdate(object);				}break;
		case ScbType::eBODY_FROM_ARTICULATION_LINK:	{ mBodyManager.scheduleForUpdate(object);				}break;
		case ScbType::eRIGID_STATIC:				{ mRigidStaticManager.scheduleForUpdate(object);		}break;
		case ScbType::eCONSTRAINT:					{ mConstraintManager.scheduleForUpdate(object);			}break;
		case ScbType::eARTICULATION:				{ mArticulationManager.scheduleForUpdate(object);		}break;
		case ScbType::eARTICULATION_JOINT:			{ mArticulationJointManager.scheduleForUpdate(object);	}break;
		case ScbType::eAGGREGATE:					{ mAggregateManager.scheduleForUpdate(object);			}break;
		case ScbType::eUNDEFINED:
		case ScbType::eTYPE_COUNT:
			PX_ALWAYS_ASSERT_MESSAGE( "scheduleForUpdate: missing type!");
			break;
	}
}

PxU8* Scb::Scene::getStream(ScbType::Enum type)
{
	PxU8* memory = NULL;
	switch(type)
	{
		case ScbType::eSHAPE_EXCLUSIVE:
		case ScbType::eSHAPE_SHARED:				{ memory = reinterpret_cast<PxU8*>(mStream.allocateNotThreadSafe(sizeof(Scb::ShapeBuffer)));				new (memory) Scb::ShapeBuffer;				}break;
		case ScbType::eBODY:						{ memory = reinterpret_cast<PxU8*>(mStream.allocateNotThreadSafe(sizeof(Scb::BodyBuffer)));					new (memory) Scb::BodyBuffer;				}break;
		case ScbType::eBODY_FROM_ARTICULATION_LINK:	{ memory = reinterpret_cast<PxU8*>(mStream.allocateNotThreadSafe(sizeof(Scb::BodyBuffer)));					new (memory) Scb::BodyBuffer;				}break;
		case ScbType::eRIGID_STATIC:				{ memory = reinterpret_cast<PxU8*>(mStream.allocateNotThreadSafe(sizeof(Scb::RigidStaticBuffer)));			new (memory) Scb::RigidStaticBuffer;		}break;
		case ScbType::eCONSTRAINT:					{ memory = reinterpret_cast<PxU8*>(mStream.allocateNotThreadSafe(sizeof(Scb::ConstraintBuffer)));			new (memory) Scb::ConstraintBuffer;			}break;
		case ScbType::eARTICULATION:				{ memory = reinterpret_cast<PxU8*>(mStream.allocateNotThreadSafe(sizeof(Scb::ArticulationBuffer)));			new (memory) Scb::ArticulationBuffer;		}break;
		case ScbType::eARTICULATION_JOINT:			{ memory = reinterpret_cast<PxU8*>(mStream.allocateNotThreadSafe(sizeof(Scb::ArticulationJointBuffer)));	new (memory) Scb::ArticulationJointBuffer;	}break;
		case ScbType::eAGGREGATE:					{ memory = reinterpret_cast<PxU8*>(mStream.allocateNotThreadSafe(sizeof(Scb::AggregateBuffer)));			new (memory) Scb::AggregateBuffer;			}break;
		case ScbType::eUNDEFINED:
		case ScbType::eTYPE_COUNT:
			PX_ALWAYS_ASSERT_MESSAGE("getStream: missing type!");
			return NULL;
	}
	return memory;
}

///////////////////////////////////////////////////////////////////////////////

PxBroadPhaseType::Enum Scb::Scene::getBroadPhaseType() const
{
	return mScene.getBroadPhaseType();
}

bool Scb::Scene::getBroadPhaseCaps(PxBroadPhaseCaps& caps) const
{
	return mScene.getBroadPhaseCaps(caps);
}

PxU32 Scb::Scene::getNbBroadPhaseRegions() const
{
	return mScene.getNbBroadPhaseRegions();
}

PxU32 Scb::Scene::getBroadPhaseRegions(PxBroadPhaseRegionInfo* userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	return mScene.getBroadPhaseRegions(userBuffer, bufferSize, startIndex);
}

PxU32 Scb::Scene::addBroadPhaseRegion(const PxBroadPhaseRegion& region, bool populateRegion)
{
	if(!isPhysicsBuffering())
		return mScene.addBroadPhaseRegion(region, populateRegion);
	else
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "PxScene::addBroadPhaseRegion() not allowed while simulation is running. Call will be ignored.");
	return 0xffffffff;
}

bool Scb::Scene::removeBroadPhaseRegion(PxU32 handle)
{
	if(!isPhysicsBuffering())
		return mScene.removeBroadPhaseRegion(handle);
	else
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, "PxScene::removeBroadPhaseRegion() not allowed while simulation is running. Call will be ignored.");
	return false;
}

//////////////////////////////////////////////////////////////////////////

//
// To avoid duplication of a lot of similar code, the following templated method was introduced. Its main purpose is to
// take care of all the operations related to adding/removing a rigid object to/from the scene. Depending on the type
// of rigid object and the simulation state, there are slight changes to the code flow necessary. Among the operations are:
//
// - Add/remove rigid object to/from scene
// - Add/remove shapes from PVD
// - Adjust buffer control state of shapes
// - Adjust ref-count of shapes
//
template <bool TSimRunning, bool TAdd, bool TIsDynamic, bool TIsNonSimObject, class T>
PX_FORCE_INLINE static void addOrRemoveRigidObject(Sc::Scene& s, T& rigidObject, bool wakeOnLostTouch, PxBounds3* uninflatedBounds, const Gu::BVHStructure* bvhStructure)
{
	PX_ASSERT(TIsDynamic || (rigidObject.getScbType() == ScbType::eRIGID_STATIC));
	if (TSimRunning && TIsNonSimObject && TAdd)
		PX_ASSERT(rigidObject.getActorFlags() & PxActorFlag::eDISABLE_SIMULATION);
	if (TSimRunning && TIsNonSimObject&& !TAdd)
		PX_ASSERT(rigidObject.isSimDisabledInternally());
	if (!TSimRunning && TIsNonSimObject)
		PX_ASSERT(rigidObject.isSimDisabledInternally());  // when the simulation flag gets cleared on an object with pending removal, only the core flag knows that internally it is still a non-simulation object.
	PX_ASSERT(!uninflatedBounds || (TAdd && !TSimRunning && !TIsNonSimObject));

	Ps::InlineArray<const Sc::ShapeCore*, 64> localShapes;
	Ps::InlineArray<const Sc::ShapeCore*, 64>& scShapes = s.getBatchRemove() ? s.getBatchRemove()->removedShapes : localShapes;

	PxActor* pxActor = NULL;
	void* const* shapes;
	PxU32 nbShapes;	
	size_t shapePtrOffset = NpShapeGetScPtrOffset();

	Scb::Body& dynamicObject = reinterpret_cast<Scb::Body&>(rigidObject);
	Scb::RigidStatic& staticObject = reinterpret_cast<Scb::RigidStatic&>(rigidObject);

	if (!TSimRunning)
	{
		if (TIsDynamic)
			pxActor = dynamicObject.getScBody().getPxActor();
		else
			pxActor = staticObject.getScStatic().getPxActor();
	}
	PX_UNUSED(pxActor);
	PX_UNUSED(bvhStructure);
	size_t ptrOffset;
	if (TAdd || TSimRunning || TIsNonSimObject)
	{
		// Np buffers are still valid when the object gets removed while the sim is running.
		// Furthermore, for non-simulation objects, there exists no shape buffer in the simulation controller
		// and we need to fetch from Np all the time.

		ptrOffset = shapePtrOffset - Scb::Shape::getScOffset();

		if (TIsDynamic)		
			nbShapes = NpRigidDynamicGetShapes(dynamicObject, shapes);
		else
			nbShapes = NpRigidStaticGetShapes(staticObject, shapes);
	}

	if ((!TSimRunning) && (!TIsNonSimObject))
	{
		if (TAdd)
		{
			if (TIsDynamic)
			{
				const bool isCompound = bvhStructure ? true : false;
				s.addBody(dynamicObject.getScBody(), shapes, nbShapes, shapePtrOffset, uninflatedBounds, isCompound);
			}
			else
				s.addStatic(staticObject.getScStatic(), shapes, nbShapes, shapePtrOffset, uninflatedBounds);
		}
		else
		{
			ptrOffset = -Scb::Shape::getScOffset();

			if (TIsDynamic)
				s.removeBody(dynamicObject.getScBody(), scShapes, wakeOnLostTouch);
			else
				s.removeStatic(staticObject.getScStatic(), scShapes, wakeOnLostTouch);

			shapes = reinterpret_cast<void*const*>(const_cast<Sc::ShapeCore*const*>(scShapes.begin()));
			nbShapes = scShapes.size();
		}
	}

	Scb::Scene* scbScene = rigidObject.getScbScene();
	Scb::Scene* shapeScenePtr = scbScene;
	Scb::ControlState::Enum controlState = rigidObject.getControlState();

	if (!TSimRunning)
	{
		// hacky: in the non-buffered case the rigid objects might not have been updated properly at this point, so it's done explicitly.

		if (TAdd)
		{
			PX_ASSERT(shapeScenePtr == scbScene);
			controlState = Scb::ControlState::eIN_SCENE;
		}
		else
		{
			shapeScenePtr = NULL;
			controlState = Scb::ControlState::eNOT_IN_SCENE;
		}
	}

	for(PxU32 i=0; i < nbShapes; i++)
	{
		Scb::Shape& scbShape = *Ps::pointerOffset<Scb::Shape*>(shapes[i], ptrdiff_t(ptrOffset));

		if (!TSimRunning)
		{
			PX_ASSERT(pxActor);
			PX_ASSERT(scbScene);

			if (TAdd)
			{
				scbShape.setControlStateIfExclusive(shapeScenePtr, controlState);

				if (!TIsNonSimObject)
					NpShapeIncRefCount(scbShape);  // simulation increases the refcount to avoid that shapes get destroyed while the sim is running

#if PX_SUPPORT_PVD
				scbScene->getScenePvdClient().createPvdInstance(&scbShape, *pxActor); 
#endif
			}
			else
			{
				scbShape.checkUpdateOnRemove<true>(scbScene);

#if PX_SUPPORT_PVD
				scbScene->getScenePvdClient().releasePvdInstance(&scbShape, *pxActor);
#endif
				scbShape.setControlStateIfExclusive(shapeScenePtr, controlState);

				if (!TIsNonSimObject)
					NpShapeDecRefCount(scbShape);  // see comment in the "TAdd" section above
			}
		}
		else
			scbShape.setControlStateIfExclusive(shapeScenePtr, controlState);
	}
}
