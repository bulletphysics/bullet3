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

#ifndef PX_PHYSICS_SCB_RIGID_OBJECT
#define PX_PHYSICS_SCB_RIGID_OBJECT

#include "../../simulationcontroller/include/ScRigidCore.h"
#include "ScbScene.h"
#include "ScbActor.h"
#include "ScbShape.h"
#include "PsInlineArray.h"

namespace physx
{

// base class for dynamic and static rigid objects, so that shapes can have something to refer to

namespace Scb
{
struct RemovedShape
{
	RemovedShape() : mShape(NULL), mWakeTouching(0) {}
	RemovedShape(Scb::Shape* s, PxU8 wakeTouching) : mShape(s), mWakeTouching(wakeTouching) {}

	PX_FORCE_INLINE	bool operator == (const RemovedShape& other) const
	{
		return (mShape == other.mShape);
	}

	PX_FORCE_INLINE	bool operator != (const RemovedShape& other) const
	{
		return (mShape != other.mShape);
	}

	Scb::Shape*		mShape;
	PxU8			mWakeTouching;
};

struct RigidObjectBuffer : public ActorBuffer		//once RigidObject has its own buffered elements, derive from that instead
{
	RigidObjectBuffer(): mResetFilterShape(0), mResetFilterShapeCount(0) {}

	// TODO(dsequeira): ideally we would use an allocator that allocates from the buffered memory stream
	Ps::InlineArray<Scb::Shape*, 4>			mAddedShapes;
	Ps::InlineArray<Scb::RemovedShape, 4>	mRemovedShapes;
	union
	{
		PxU32								mResetFilterShapesIdx;
		Scb::Shape*							mResetFilterShape;
	};
	PxU32									mResetFilterShapeCount;

	enum { BF_Base = ActorBuffer::AttrCount };

	enum
	{
		BF_Shapes			= 1<<BF_Base,
		BF_WakeTouching		= 1<<(BF_Base+1),
		BF_ResetFiltering	= 1<<(BF_Base+2)
	};

	enum { AttrCount = ActorBuffer::AttrCount+3 }; 
};


class RigidObject : public Scb::Actor
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================

	typedef RigidObjectBuffer Buf;
	typedef Sc::RigidCore Core;

public:
// PX_SERIALIZATION
										RigidObject()										{}
										RigidObject(const PxEMPTY) : Scb::Actor(PxEmpty)	{}
	static			void				getBinaryMetaData(PxOutputStream& stream);
//~PX_SERIALIZATION
	
	//---------------------------------------------------------------------------------
	// Wrapper for Sc::RigidCore interface
	//---------------------------------------------------------------------------------

	PX_INLINE void resetFiltering(Scb::Shape*const* shapes, PxU32 shapeCount);


	//---------------------------------------------------------------------------------
	// Data synchronization
	//---------------------------------------------------------------------------------

	// the fetchResults order is to process removes, do callbacks, and then do the other synchronization. So we need to split sync'ing of
	// adds and removes
	// Note:  The array of removed shapes must be reset here to avoid memory leaks: even if the control state means we don't process the 
	// array of removed shapes, we still need to clear the array.
	PX_INLINE void processShapeRemoves()
	{
		if(getBufferFlags() & Buf::BF_Shapes)
		{
			RigidObjectBuffer* b = getBuffer();

			if(getControlState() == ControlState::eIN_SCENE)
			{
#if PX_SUPPORT_PVD
				PxActor& pxActor = *getScRigidCore().getPxActor();
#endif
				for(PxU32 i=0;i<b->mRemovedShapes.size();i++)
				{
					RemovedShape& rs = b->mRemovedShapes[i];
					Shape& shape = *rs.mShape;
					shape.setControlStateIfExclusive(NULL, Scb::ControlState::eNOT_IN_SCENE);

					Sc::RigidCore& rc = getScRigidCore();
					Scb::Scene* scene = getScbScene();
#if PX_SUPPORT_PVD
					scene->getScenePvdClient().releasePvdInstance(&shape, pxActor);
#endif
					if(!isSimDisabledInternally())
					{
						rc.removeShapeFromScene(shape.getScShape(), (rs.mWakeTouching != 0));

						shape.checkUpdateOnRemove<true>(scene);
						
						NpShapeDecRefCount(shape);
					}
				}
			}

			// The array of removed shapes must be reset to avoid memory leaks.
			b->mRemovedShapes.reset();
		}
	}

	PX_INLINE	void syncState()
	{
		const PxU32 bufferFlags = getBufferFlags();

		if(bufferFlags & Buf::BF_ResetFiltering)
		{
			PX_ASSERT(getControlState() != ControlState::eREMOVE_PENDING);  // removing the actor should have cleared BF_ResetFiltering

			Scb::Scene* scene = getScbScene();
			Sc::RigidCore& scCore = getScRigidCore();
			RigidObjectBuffer* b = getBuffer();
			Scb::Shape* const* shapes = (b->mResetFilterShapeCount == 1) ? &b->mResetFilterShape : scene->getShapeBuffer(b->mResetFilterShapesIdx);
			for(PxU32 i=0; i<b->mResetFilterShapeCount; i++)
			{
				Sc::ShapeCore& scShape = shapes[i]->getScShape();

				// do not process the call if the shape will not be a broadphase shape any longer
				if(shapes[i]->getFlags() & (PxShapeFlag::eSIMULATION_SHAPE | PxShapeFlag::eTRIGGER_SHAPE))
					scCore.onShapeChange(scShape, Sc::ShapeChangeNotifyFlag::eRESET_FILTERING, PxShapeFlags());
			}
		}

		if(bufferFlags & Buf::BF_Shapes)
		{
			RigidObjectBuffer* b = getBuffer();
			ControlState::Enum cs = getControlState();
#if PX_SUPPORT_PVD
			PxActor& pxActor = *getScRigidCore().getPxActor();
#endif
			for(PxU32 i=0;i<b->mAddedShapes.size();i++)
			{
				Shape& shape = *b->mAddedShapes[i];

				// it can happen that a shape gets attached while the sim is running but then the actor is removed from the scene,
				// so we need to distinguish those two cases
				if(cs != ControlState::eREMOVE_PENDING)
				{
					shape.setControlStateIfExclusive(getScbScene(), Scb::ControlState::eIN_SCENE);

					if(!(getActorFlags() & PxActorFlag::eDISABLE_SIMULATION))  // important to use the buffered flags since we want the new state.
					{
						getScRigidCore().addShapeToScene(shape.getScShape());
						NpShapeIncRefCount(shape);
					}
#if PX_SUPPORT_PVD
					getScbScene()->getScenePvdClient().createPvdInstance(&shape, pxActor); 
#endif
				}
				else
					shape.setControlStateIfExclusive(getScbScene(), Scb::ControlState::eNOT_IN_SCENE);
			}

			// reset the arrays, because destructors don't run on buffers
			b->mAddedShapes.reset();
		}

		Actor::syncState();
	}

	PX_FORCE_INLINE void scheduleForWakeTouching()
	{
		PX_ASSERT(getScbScene() && getScbScene()->isPhysicsBuffering());
		setBufferFlag(RigidObjectBuffer::BF_WakeTouching);
	}

	//---------------------------------------------------------------------------------
	// Miscellaneous
	//---------------------------------------------------------------------------------
public:
	PX_INLINE const Sc::RigidCore&		getScRigidCore()	const	{	return static_cast<const Sc::RigidCore&>(getActorCore()); }  // Only use if you know what you're doing! 
	PX_INLINE Sc::RigidCore&			getScRigidCore()			{	return static_cast<Sc::RigidCore&>(getActorCore()); }  // Only use if you know what you're doing! 

	PX_INLINE void						onShapeAttach(Scb::Shape& shape)
	{
		// there are two things to do here: add the shape to the sim (if unbuffered) or set it up for
		// * if unbuffered, add the shape to the sim and PVD and increment its refcount, else set it up for buffered insertion, 
		// * if the shape is exclusive, set its Scb control state appropriately.

		ControlState::Enum cs = getControlState();
		if(cs==ControlState::eNOT_IN_SCENE)		
			return;

		Scene* scbScene = getScbScene();
		if(!scbScene->isPhysicsBuffering())
		{
			if(!(getActorFlags() & PxActorFlag::eDISABLE_SIMULATION))
			{
				NpShapeIncRefCount(shape);
				getScRigidCore().addShapeToScene(shape.getScShape());
			}

#if PX_SUPPORT_PVD
			scbScene->getScenePvdClient().createPvdInstance(&shape, *getScRigidCore().getPxActor()); 
#endif
			shape.setControlStateIfExclusive(scbScene, ControlState::eIN_SCENE);
			return;
		}
		else if(cs == ControlState::eINSERT_PENDING)
		{
			shape.setControlStateIfExclusive(scbScene, ControlState::eINSERT_PENDING);
			return;
		}

		RigidObjectBuffer* b = getBuffer();
		if(!b->mRemovedShapes.findAndReplaceWithLast(RemovedShape(&shape, 0)))
			b->mAddedShapes.pushBack(&shape);
		markUpdated(Buf::BF_Shapes);

		shape.setControlStateIfExclusive(scbScene, ControlState::eINSERT_PENDING);
	}

	PX_INLINE void onShapeDetach(Scb::Shape& shape, bool wakeOnLostTouch, bool toBeReleased)
	{
		// see comments in onShapeAttach
		ControlState::Enum cs = getControlState();
		if(cs==ControlState::eNOT_IN_SCENE)		
			return;

		Scene* scbScene = getScbScene();
		if(!scbScene->isPhysicsBuffering())
		{
#if PX_SUPPORT_PVD
			scbScene->getScenePvdClient().releasePvdInstance(&shape, *getScRigidCore().getPxActor());
#endif
			if(!(getActorFlags() & PxActorFlag::eDISABLE_SIMULATION))
			{
				getScRigidCore().removeShapeFromScene(shape.getScShape(), wakeOnLostTouch);
				NpShapeDecRefCount(shape);
			}

			shape.setControlStateIfExclusive(NULL, ControlState::eNOT_IN_SCENE);
			return;
		}
		else if(cs == ControlState::eINSERT_PENDING)
		{
			shape.setControlStateIfExclusive(NULL, ControlState::eNOT_IN_SCENE);
			return;
		}

		RigidObjectBuffer* b = getBuffer();

		// remove from the resetFiltering list
		const PxU32 bufferFlags = getBufferFlags();
		if(bufferFlags & Buf::BF_ResetFiltering)
		{
			if(b->mResetFilterShapeCount == 1)
			{
				if(b->mResetFilterShape == &shape)
				{
					b->mResetFilterShapeCount = 0;
					b->mResetFilterShape = 0;
					resetBufferFlag(Buf::BF_ResetFiltering);
				}
			}
			else
			{
				Scb::Shape** shapes = scbScene->getShapeBuffer(b->mResetFilterShapesIdx);
				PxU32 idx = 0;
				PxU32 lastIdx = b->mResetFilterShapeCount;
				for(PxU32 k=0; k < b->mResetFilterShapeCount; k++)  // need to iterate over whole list, same shape can be in there multiple times
				{
					if(shapes[idx] != &shape)
						idx++;
					else
					{
						lastIdx--;
						shapes[idx] = shapes[lastIdx];
					}
				}
				b->mResetFilterShapeCount = idx;
				if(idx == 0)
				{
					b->mResetFilterShape = 0;
					resetBufferFlag(Buf::BF_ResetFiltering);
				}
				else if(idx == 1)
					b->mResetFilterShape = shapes[0];
			}
		}

		if(b->mAddedShapes.findAndReplaceWithLast(&shape))
			shape.setControlStateIfExclusive(scbScene, ControlState::eIN_SCENE);
		else
		{
			if(!isSimDisabledInternally())
			{
				b->mRemovedShapes.pushBack(RemovedShape(&shape, PxU8(wakeOnLostTouch ? 1 : 0)));
			}
			else
			{
				PX_ASSERT(scbScene);
				PX_ASSERT(scbScene->isPhysicsBuffering());
				if(toBeReleased)
				{
					shape.checkUpdateOnRemove<false>(scbScene);
#if PX_SUPPORT_PVD
					scbScene->getScenePvdClient().releasePvdInstance(&shape, *getScRigidCore().getPxActor());
#endif
				}
				else
					b->mRemovedShapes.pushBack(RemovedShape(&shape, 0));
			}
			shape.setControlStateIfExclusive(scbScene, ControlState::eREMOVE_PENDING);
		}
		markUpdated(Buf::BF_Shapes);
	}

	PX_INLINE		bool				isAddedShape(Scb::Shape&);  // check whether the specified shape is pending for insertion. Only call this method if you know that there are pending shape adds/removes.

	PX_FORCE_INLINE void				switchToNoSim(bool isDynamic);
	PX_FORCE_INLINE void				switchFromNoSim(bool isDynamic);
	PX_FORCE_INLINE void				syncNoSimSwitch(const Buf& buf, Sc::RigidCore& rc, bool isDynamic);

	// IMPORTANT: This is the non-buffered state, for the case where it is important to know what the current internal state is.
	// Reading is fine even if the sim is running because actor flags are read-only internally.
	PX_FORCE_INLINE bool				isSimDisabledInternally() const { return getScRigidCore().getActorFlags().isSet(PxActorFlag::eDISABLE_SIMULATION); }

	PX_FORCE_INLINE	void				clearBufferedState() { resetBufferFlag(Buf::BF_ResetFiltering); }

	PX_FORCE_INLINE static const RigidObject&	fromSc(const Sc::RigidCore& a)	{ return static_cast<const RigidObject&>(Actor::fromSc(a)); }
	PX_FORCE_INLINE static RigidObject&			fromSc(Sc::RigidCore &a)		{ return static_cast<RigidObject&>(Actor::fromSc(a)); }
protected:
	                                    ~RigidObject()	{}
private:
	Buf* getBuffer() { return reinterpret_cast<Buf*>(getStream()); }

	PX_FORCE_INLINE void				copyResetFilterShapes(Scb::Shape** shapePtrs, Scb::Shape*const* oldShapes, PxU32 oldShapeCount, Scb::Shape*const* newShapes, PxU32 newShapeCount);
};

PX_INLINE void RigidObject::resetFiltering(Scb::Shape*const* shapes, PxU32 shapeCount)
{
	PX_ASSERT(!(getActorFlags() & PxActorFlag::eDISABLE_SIMULATION));

	if(!isBuffering())
	{
		for(PxU32 i=0; i < shapeCount; i++)
			getScRigidCore().onShapeChange(shapes[i]->getScShape(), Sc::ShapeChangeNotifyFlag::eRESET_FILTERING, PxShapeFlags());
	}
	else
	{
		RigidObjectBuffer* b = getBuffer();

		if(b->mResetFilterShapeCount == 0)
		{
			if(shapeCount == 1)
			{
				b->mResetFilterShape = shapes[0];
				b->mResetFilterShapeCount = 1;
				markUpdated(Buf::BF_ResetFiltering);
			}
			else
			{
				PxU32 bufferIdx;
				Scb::Shape** shapePtrs = getScbScene()->allocShapeBuffer(shapeCount, bufferIdx);
				if(shapePtrs)
				{
					for(PxU32 i=0; i < shapeCount; i++)
						shapePtrs[i] = shapes[i];
					b->mResetFilterShapesIdx = bufferIdx;
					b->mResetFilterShapeCount = shapeCount;
					markUpdated(Buf::BF_ResetFiltering);
				}
			}
		}
		else
		{
			PxU32 newCount = b->mResetFilterShapeCount + shapeCount;
			PxU32 bufferIdx;
			Scb::Shape** shapePtrs = getScbScene()->allocShapeBuffer(newCount, bufferIdx);
			if(shapePtrs)
			{
				if(b->mResetFilterShapeCount == 1)
					copyResetFilterShapes(shapePtrs, &b->mResetFilterShape, 1, shapes, shapeCount);
				else
					copyResetFilterShapes(shapePtrs, getScbScene()->getShapeBuffer(b->mResetFilterShapesIdx), b->mResetFilterShapeCount, shapes, shapeCount);
				b->mResetFilterShapesIdx = bufferIdx;
				b->mResetFilterShapeCount = newCount;
				markUpdated(Buf::BF_ResetFiltering);
			}
		}
	}
}

PX_INLINE bool RigidObject::isAddedShape(Scb::Shape& shape)
{
	PX_ASSERT(isBuffered(Buf::BF_Shapes));

	if(shape.isExclusive())
	{ 
		return (shape.getControlState() == Scb::ControlState::eINSERT_PENDING);
	}
	else
	{
		// For shared shapes it is not clear from the shape alone whether it has been added while the simulation was running.

		RigidObjectBuffer* buf = getBuffer();
		PX_ASSERT(buf);
		const PxU32 addedShapeCount = buf->mAddedShapes.size();
		for(PxU32 k=0; k < addedShapeCount; k++)
		{
			if(&shape == buf->mAddedShapes[k])
				return true;
		}
		return false;
	}
}

PX_FORCE_INLINE void RigidObject::switchToNoSim(bool isDynamic)
{
	Scb::Scene* scene = getScbScene();

	if(scene && (!scene->isPhysicsBuffering()))
		scene->switchRigidToNoSim(*this, isDynamic);
}

PX_FORCE_INLINE void RigidObject::switchFromNoSim(bool isDynamic)
{
	Scb::Scene* scene = getScbScene();

	if(scene && (!scene->isPhysicsBuffering()))
		scene->switchRigidFromNoSim(*this, isDynamic);
}

PX_FORCE_INLINE void RigidObject::syncNoSimSwitch(const Buf& buf, Sc::RigidCore& rc, bool isDynamic)
{
	const PxActorFlags oldFlags = rc.getActorFlags();
	const bool oldNoSim = oldFlags.isSet(PxActorFlag::eDISABLE_SIMULATION);
	const bool newNoSim = buf.mActorFlags.isSet(PxActorFlag::eDISABLE_SIMULATION);

	if(oldNoSim && (!newNoSim))
		getScbScene()->switchRigidFromNoSim(*this, isDynamic);
	else if((!oldNoSim) && newNoSim)
		getScbScene()->switchRigidToNoSim(*this, isDynamic);
}

PX_FORCE_INLINE void RigidObject::copyResetFilterShapes(Scb::Shape** shapePtrs, Scb::Shape*const* oldShapes, PxU32 oldShapeCount, Scb::Shape*const* newShapes, PxU32 newShapeCount)
{
	for(PxU32 i=0; i < oldShapeCount; i++)
		shapePtrs[i] = oldShapes[i];
	for(PxU32 i=0; i < newShapeCount; i++)
		shapePtrs[i+oldShapeCount] = newShapes[i];
}

}  // namespace Scb

}

#endif
