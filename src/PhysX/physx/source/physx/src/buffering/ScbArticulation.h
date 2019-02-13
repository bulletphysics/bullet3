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

#ifndef PX_PHYSICS_SCB_ARTICULATION
#define PX_PHYSICS_SCB_ARTICULATION

#include "ScArticulationCore.h"
#include "ScbBase.h"
#include "ScbDefs.h"

namespace physx
{
namespace Scb
{

struct ArticulationBuffer
{
#ifdef USE_NEW_SYSTEM
	PxU32	mInternalDriveIterations;
	PxU32	mExternalDriveIterations;
	PxU32	mMaxProjectionIterations;
	PxReal	mSeparationTolerance;
	PxReal	mSleepThreshold;
	PxU16	mSolverIterationCounts;
	PxReal	mFreezeThreshold;
#else
	template <PxU32 I, PxU32 Dummy> struct Fns {};
	typedef Sc::ArticulationCore Core;
	typedef ArticulationBuffer Buf;

	SCB_REGULAR_ATTRIBUTE(0, PxU32,		InternalDriveIterations)
	SCB_REGULAR_ATTRIBUTE(1, PxU32,		ExternalDriveIterations)
	SCB_REGULAR_ATTRIBUTE(2, PxU32,		MaxProjectionIterations)
	SCB_REGULAR_ATTRIBUTE(3, PxReal,	SeparationTolerance)
	SCB_REGULAR_ATTRIBUTE(4, PxReal,	SleepThreshold)
	SCB_REGULAR_ATTRIBUTE(5, PxU16,		SolverIterationCounts)
	SCB_REGULAR_ATTRIBUTE(6, PxReal,	FreezeThreshold)
#endif
	enum { BF_WakeCounter	= 1<<7 };
	enum { BF_PutToSleep	= 1<<8 };
	enum { BF_WakeUp		= 1<<9 };
	enum { BF_ArticulationFlags = 1 << 19 };
	enum { BF_ArticulationPoseUpdate = 1 << 11 };

};

class Articulation : public Base
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================

	typedef ArticulationBuffer Buf;
	typedef Sc::ArticulationCore Core;

// PX_SERIALIZATION
public:
												Articulation(const PxEMPTY) : Base(PxEmpty), mArticulation(PxEmpty), mBufferedIsSleeping(1) {}
	static			void						getBinaryMetaData(PxOutputStream& stream);
//~PX_SERIALIZATION

private:

public:
	PX_INLINE									Articulation();
	PX_INLINE									~Articulation() {}

	//---------------------------------------------------------------------------------
	// Wrapper for Sc::Articulation interface
	//---------------------------------------------------------------------------------

#ifdef USE_NEW_SYSTEM
	SCB_MEMBER(Articulation, mArticulation, InternalDriveIterations, PxU32, 0)
	SCB_MEMBER(Articulation, mArticulation, ExternalDriveIterations, PxU32, 1)
	SCB_MEMBER(Articulation, mArticulation, MaxProjectionIterations, PxU32, 2)
	SCB_MEMBER(Articulation, mArticulation, SeparationTolerance, PxReal, 3)
	SCB_MEMBER(Articulation, mArticulation, SleepThreshold, PxReal, 4)
	SCB_MEMBER(Articulation, mArticulation, SolverIterationCounts, PxU16, 5)
	SCB_MEMBER(Articulation, mArticulation, FreezeThreshold, PxReal, 6)
#else
	PX_INLINE		PxU32		getInternalDriveIterations() const				{ return read<Buf::BF_InternalDriveIterations>();	}	
	PX_INLINE		void		setInternalDriveIterations(const PxU32 v)		{ write<Buf::BF_InternalDriveIterations>(v);		}

	PX_INLINE		PxU32		getExternalDriveIterations() const				{ return read<Buf::BF_ExternalDriveIterations>();	}	
	PX_INLINE		void		setExternalDriveIterations(const PxU32 v)		{ write<Buf::BF_ExternalDriveIterations>(v);		}

	PX_INLINE		PxU32		getMaxProjectionIterations() const				{ return read<Buf::BF_MaxProjectionIterations>();	}
	PX_INLINE		void		setMaxProjectionIterations(const PxU32 v)		{ write<Buf::BF_MaxProjectionIterations>(v);		}

	PX_INLINE		PxU16		getSolverIterationCounts() const				{ return read<Buf::BF_SolverIterationCounts>();		}
	PX_INLINE		void		setSolverIterationCounts(PxU16 v)				{ write<Buf::BF_SolverIterationCounts>(v);			}

	PX_INLINE		PxReal		getSeparationTolerance() const					{ return read<Buf::BF_SeparationTolerance>();		}
	PX_INLINE		void		setSeparationTolerance(const PxReal v)			{ write<Buf::BF_SeparationTolerance>(v);			}

	PX_INLINE		PxReal		getSleepThreshold() const						{ return read<Buf::BF_SleepThreshold>();			}
	PX_INLINE		void		setSleepThreshold(const PxReal v)				{ write<Buf::BF_SleepThreshold>(v);					}

	PX_INLINE		PxReal		getFreezeThreshold() const						{ return read<Buf::BF_FreezeThreshold>();			}
	PX_INLINE		void		setFreezeThreshold(const PxReal v)				{ write<Buf::BF_FreezeThreshold>(v);				}
#endif
	PX_INLINE		PxReal		getWakeCounter() const									{ return mBufferedWakeCounter;						}
	PX_INLINE		void		setWakeCounter(const PxReal v);

	PX_INLINE		PxArticulationFlags		getArticulationFlags() const { return mBufferedArticulationFlags; }
	PX_INLINE		void					setArticulationFlags(PxArticulationFlags flags);

	PX_INLINE		PxArticulationBase::Enum		getArticulationType() const { return mArticulation.getArticulationType(); }
	PX_INLINE		void							setArticulationType(PxArticulationBase::Enum type) { mArticulation.setArticulationType(type); }

	PX_FORCE_INLINE	void		wakeUp();
	PX_FORCE_INLINE	void		putToSleep();
	PX_FORCE_INLINE	bool		isSleeping() const								{ return (mBufferedIsSleeping != 0); }

	//---------------------------------------------------------------------------------
	// Data synchronization
	//---------------------------------------------------------------------------------
	PX_INLINE		void						syncState();

	PX_FORCE_INLINE const Sc::ArticulationCore&	getScArticulation()			const	{ return mArticulation; }  // Only use if you know what you're doing!
	PX_FORCE_INLINE Sc::ArticulationCore&		getScArticulation()					{ return mArticulation; }  // Only use if you know what you're doing!

	PX_FORCE_INLINE static Articulation&		fromSc(Core &a)					{ return *reinterpret_cast<Articulation*>(reinterpret_cast<PxU8*>(&a)-getScOffset()); }
	PX_FORCE_INLINE static const Articulation&	fromSc(const Core &a)			{ return *reinterpret_cast<const Articulation*>(reinterpret_cast<const PxU8*>(&a)-getScOffset()); }

	static size_t getScOffset()	{ return reinterpret_cast<size_t>(&reinterpret_cast<Articulation*>(0)->mArticulation);	}

	PX_FORCE_INLINE	void		wakeUpInternal(PxReal wakeCounter);

	PX_FORCE_INLINE void		setGlobalPose();

	PX_FORCE_INLINE	void		initBufferedState();
	PX_FORCE_INLINE	void		clearBufferedState();
	PX_FORCE_INLINE	void		clearBufferedSleepStateChange();

private:
	Sc::ArticulationCore		mArticulation;
	PxReal						mBufferedWakeCounter;
	PxU8						mBufferedIsSleeping;
	PxArticulationFlags			mBufferedArticulationFlags;
	
	PX_FORCE_INLINE	const Buf*	getArticulationBuffer()	const	{ return reinterpret_cast<const Buf*>(getStream()); }
	PX_FORCE_INLINE	Buf*		getArticulationBuffer()			{ return reinterpret_cast<Buf*>(getStream()); }

#ifndef USE_NEW_SYSTEM
	//---------------------------------------------------------------------------------
	// Infrastructure for regular attributes
	//---------------------------------------------------------------------------------

	struct Access: public BufferedAccess<Buf, Core, Articulation> {};
	template<PxU32 f> PX_FORCE_INLINE typename Buf::Fns<f,0>::Arg read() const		{	return Access::read<Buf::Fns<f,0> >(*this, mArticulation);	}
	template<PxU32 f> PX_FORCE_INLINE void write(typename Buf::Fns<f,0>::Arg v)		{	Access::write<Buf::Fns<f,0> >(*this, mArticulation, v);		}
	template<PxU32 f> PX_FORCE_INLINE void flush(const Buf& buf)					{	Access::flush<Buf::Fns<f,0> >(*this, mArticulation, buf);	}
#endif
};

Articulation::Articulation()
{
	setScbType(ScbType::eARTICULATION);
	mBufferedWakeCounter = mArticulation.getWakeCounter();
	mBufferedIsSleeping = 1;  // this is the specified value for free standing objects
}

PX_INLINE void Articulation::setWakeCounter(PxReal counter)
{
	mBufferedWakeCounter = counter;

	if(!isBuffering())
	{
		if(getScbScene() && (counter > 0.0f))
			mBufferedIsSleeping = 0;

		mArticulation.setWakeCounter(counter);
		UPDATE_PVD_PROPERTIES_OBJECT()
	}
	else
	{
		if(counter > 0.0f)
		{
			mBufferedIsSleeping = 0;
			markUpdated(Buf::BF_WakeUp | Buf::BF_WakeCounter);
			resetBufferFlag(Buf::BF_PutToSleep);
		}
		else
			markUpdated(Buf::BF_WakeCounter);
	}
}

PX_FORCE_INLINE void Articulation::setArticulationFlags(PxArticulationFlags flags)
{
	mBufferedArticulationFlags = flags;
	if (!isBuffering())
	{
		mArticulation.setArticulationFlags(flags);
		UPDATE_PVD_PROPERTIES_OBJECT()
	}
	else
	{
		markUpdated(Buf::BF_ArticulationFlags);
	}
}

PX_FORCE_INLINE void Articulation::wakeUp()
{
	Scene* scene = getScbScene();
	PX_ASSERT(scene);  // only allowed for an object in a scene

	wakeUpInternal(scene->getWakeCounterResetValue());
}

PX_FORCE_INLINE void Articulation::wakeUpInternal(PxReal wakeCounter)
{
	PX_ASSERT(getScbScene());

	mBufferedWakeCounter = wakeCounter;

	mBufferedIsSleeping = 0;
	if(!isBuffering())
	{
		mArticulation.wakeUp(wakeCounter);
	}
	else
	{
		markUpdated(Buf::BF_WakeUp | Buf::BF_WakeCounter);
		resetBufferFlag(Buf::BF_PutToSleep);
	}
}

PX_INLINE void Articulation::setGlobalPose()
{
	if (!isBuffering())
	{
		mArticulation.setGlobalPose();
	}
	else
	{
		markUpdated(Buf::BF_ArticulationPoseUpdate);
	}
}

PX_FORCE_INLINE void Articulation::putToSleep()
{
	mBufferedWakeCounter = 0.0f;

	mBufferedIsSleeping = 1;
	if(!isBuffering())
	{
		mArticulation.putToSleep();
	}
	else
	{
		markUpdated(Buf::BF_PutToSleep | Buf::BF_WakeCounter);
		resetBufferFlag(Buf::BF_WakeUp);
	}
}

PX_FORCE_INLINE	void Articulation::initBufferedState()
{
	PX_ASSERT(mBufferedIsSleeping);  // this method is only meant to get called when an object is added to the scene
	
	if(getWakeCounter() == 0.0f)
		mBufferedIsSleeping = 1;
	else
		mBufferedIsSleeping = 0;
	// note: to really know whether the articulation is awake/asleep on insertion, we need to go through every link and check whether any of them has
	//       a parameter setup that keeps it alive. However, the links get added after the articulation, so we do not test those here. After the links
	//       are added, an additional check will wake the articulation up if necessary.
}

PX_FORCE_INLINE	void Articulation::clearBufferedState()
{
	mBufferedIsSleeping = 1;  // the expected state when an object gets removed from the scene
}

PX_FORCE_INLINE	void Articulation::clearBufferedSleepStateChange()
{
	resetBufferFlag(Buf::BF_WakeUp | Buf::BF_PutToSleep);
}

//--------------------------------------------------------------
//
// Data synchronization
//
//--------------------------------------------------------------

PX_INLINE void Articulation::syncState()
{
	// see comments in Body::syncState
	PX_ASSERT(	(getControlState() != ControlState::eREMOVE_PENDING) || 
				(mBufferedIsSleeping && (!isBuffered(Buf::BF_WakeUp | Buf::BF_PutToSleep))) );

	const PxU32 flags = getBufferFlags();

	//----

	if((flags & Buf::BF_WakeCounter) == 0)
		mBufferedWakeCounter = mArticulation.getWakeCounter();
	else if (!(flags & (Buf::BF_WakeUp | Buf::BF_PutToSleep)))	// if there has been at least one buffered sleep state transition, then there is no use in adjusting the wake counter separately because it will
																// get done in the sleep state update.
	{
		PX_ASSERT(mBufferedWakeCounter == 0.0f);  // a wake counter change is always connected to a sleep state change, except one case: if setWakeCounter(0.0f) was called

		mArticulation.setWakeCounter(mBufferedWakeCounter);
	}

	//----

	if((flags & (Buf::BF_WakeUp | Buf::BF_PutToSleep)) == 0)
	{
		const bool isSimObjectSleeping = mArticulation.isSleeping();
		if(getControlState() != ControlState::eREMOVE_PENDING)  // we do not want the simulation sleep state to take effect if the object was removed (free standing objects have buffered state sleeping)
			mBufferedIsSleeping = PxU8(isSimObjectSleeping);
		else
			PX_ASSERT(mBufferedIsSleeping);  // this must get set immediately at remove
	}
	else
	{
		PX_ASSERT(flags & Buf::BF_WakeCounter);  // sleep state transitions always mark the wake counter dirty
		PX_ASSERT(getControlState() != ControlState::eREMOVE_PENDING);  // removing an object should clear pending wakeUp/putToSleep operations since the state for a free standing object gets set according to specification.

		if(flags & Buf::BF_PutToSleep)
		{
			PX_ASSERT(mBufferedIsSleeping);
			PX_ASSERT(!(flags & Buf::BF_WakeUp));
			PX_ASSERT(mBufferedWakeCounter == 0.0f);
			mArticulation.putToSleep();
		}
		else
		{
			PX_ASSERT(!mBufferedIsSleeping);
			PX_ASSERT(flags & Buf::BF_WakeUp);

			mArticulation.wakeUp(mBufferedWakeCounter);
		}
	}

	//----

	if(flags&~(Buf::BF_WakeCounter|Buf::BF_WakeUp|Buf::BF_PutToSleep))  // Optimization to avoid all the if-statements below if possible
	{
#ifdef USE_NEW_SYSTEM
		syncExternalDriveIterations();
		syncInternalDriveIterations();
		syncMaxProjectionIterations();
		syncSeparationTolerance();
		syncSleepThreshold();
		syncSolverIterationCounts();
		syncFreezeThreshold();
#else
		const Buf* PX_RESTRICT buffer =  getArticulationBuffer();

		flush<Buf::BF_ExternalDriveIterations>(*buffer);
		flush<Buf::BF_InternalDriveIterations>(*buffer);
		flush<Buf::BF_MaxProjectionIterations>(*buffer);
		flush<Buf::BF_SeparationTolerance>(*buffer);
		flush<Buf::BF_SleepThreshold>(*buffer);
		flush<Buf::BF_SolverIterationCounts>(*buffer);
		flush<Buf::BF_FreezeThreshold>(*buffer);
#endif
	}

	//----
	if (flags&(Buf::BF_ArticulationPoseUpdate))
	{
		//the root link global pose has been changed. We need to update
		//other links's global pose
		mArticulation.setGlobalPose();
	}

	// --------------
	// postconditions
	//
	PX_ASSERT((getControlState() != ControlState::eREMOVE_PENDING) || mBufferedIsSleeping);  // nothing in this method should change this
	// 
	// postconditions
	// --------------

	postSyncState();
}

}  // namespace Scb

}

#endif
