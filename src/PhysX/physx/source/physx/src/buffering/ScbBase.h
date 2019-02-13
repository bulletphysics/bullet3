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

#ifndef PX_PHYSICS_SCB_BASE
#define PX_PHYSICS_SCB_BASE

#include "CmPhysXCommon.h"
#include "ScbScene.h"

namespace physx
{
#if PX_SUPPORT_PVD
	// PT: updatePvdProperties() is overloaded and the compiler needs to know 'this' type to do the right thing.
	// Thus we can't just move this as an inlined Base function.
	#define UPDATE_PVD_PROPERTIES_OBJECT()	 {						\
		 Scb::Scene* scene_ = getScbSceneForAPI();					\
		 if(scene_ && !insertPending()  )	                        \
			scene_->getScenePvdClient().updatePvdProperties(this); }
#else
	#define UPDATE_PVD_PROPERTIES_OBJECT() {}
#endif

namespace Scb
{
	struct ControlState
	{
		enum Enum
		{
			/**
			\brief The object is not in the scene.

			The object has not been added to a scene yet. This is the default when an object gets created.

			In this state...
			\li ...user changes get written to Core directly.
			\li ...the object can not be in the list of dirty objects.
			\li ...the object can not be marked as eIS_UPDATED or eIS_RELEASED.
			*/
			eNOT_IN_SCENE	= 0,

			/**
			\brief The object waits to get inserted into the scene internally.

			The object has been added to the scene while the simulation is running and is waiting to get fully registered in all layers.

			In this state...
			\li ...user changes get written to Core directly (since the object has not yet registered in the inner layers, hence, will not get accessed internally).
			\li ...the object is in the list of dirty objects.
			\li ...the object can not be marked as eIS_UPDATED or eIS_RELEASED.
			*/
			eINSERT_PENDING	= 1,

			/**
			\brief The object is registered in the scene internally.

			The object has been added to the scene and is fully registered in all layers.

			In this state...
			\li ...user changes get written to a buffer object and get synced with Core at a sync point.
			\li ...the object can be in the list of dirty objects.
			\li ...the object can be marked as eIS_UPDATED but not eIS_RELEASED.
			*/
			eIN_SCENE		= 2,

			/**
			\brief The object waits to get removed from the scene internally.

			The object is in the scene and fully registered in all layers but has been removed while the simulation is running and is now 
			waiting to get unregistered from all layers.

			In this state...
			\li ...user changes get written to a buffer object and get synced with Core at a sync point.
			\li ...the object is in the list of dirty objects.
			\li ...the object can be marked as eIS_UPDATED or eIS_RELEASED.
			*/
			eREMOVE_PENDING	= 3
		};
	};

	struct ControlFlag
	{
		enum Enum
		{
			/**
			\brief An object property/state has been changed.

			A property/state of the object has been changed and needs to get synced to Core. Insertion & removal don't count.
			*/
			eIS_UPDATED		= 1,

			/**
			\brief The object has been released.

			The object has not just been removed from the scene it has been released as well. The object will get destroyed after the sync has been completed.
			*/
			eIS_RELEASED	= 2
		};
	};

	/**
	\brief Base class for objects that should support buffering.

	This class has members to track the buffering related object state and mark which properties have been changed and need to get synced at sync points.
	*/
	class Base
	{
	//= ATTENTION! =====================================================================================
	// Changing the data layout of this class breaks the binary serialization format.  See comments for 
	// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
	// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
	// accordingly.
	//==================================================================================================

											PX_NOCOPY(Base)
	public:
// PX_SERIALIZATION
											Base(const PxEMPTY) :
			  									mScene		(NULL),
												mStreamPtr	(NULL)
											{
												resetAllBufferFlags();
												resetControl(ControlState::eNOT_IN_SCENE);
											}
		static			void				getBinaryMetaData(PxOutputStream& stream);
//~PX_SERIALIZATION
											Base() :
												mScene		(NULL),
												mStreamPtr	(NULL)
											{
#if PX_DEBUG
												setScbType(ScbType::eUNDEFINED);
												resetControl(ControlState::eNOT_IN_SCENE);
												resetAllBufferFlags();
												PX_ASSERT(mControlState==0);
#endif
												mControlState = 0;
											}

		PX_INLINE		bool				isBuffering()					const
											{
												const ControlState::Enum state = getControlState();
												return state == ControlState::eREMOVE_PENDING || // pending remove not possible if not buffered
													(state == ControlState::eIN_SCENE && mScene->isPhysicsBuffering());
											}

		PX_FORCE_INLINE	Ps::IntBool			insertPending()					const	{ return getControlState() == ControlState::eINSERT_PENDING;		}
		PX_FORCE_INLINE	ScbType::Enum		getScbType()					const	{ return ScbType::Enum((mControlState&eTYPE_MASK)>>eTYPE_SHIFT);	}
		PX_FORCE_INLINE	void				setScbType(ScbType::Enum type)			{ mControlState = (mControlState&~eTYPE_MASK)|(type<<eTYPE_SHIFT);	}


		// the scene value field set if the object is either inserted, in simulation, or waiting for removal. If any of these things
		// is true, it can't be added to a different scene
		PX_FORCE_INLINE void				setScbScene(Scb::Scene* scene)			{ mScene = scene;	}
		PX_FORCE_INLINE	Scb::Scene*			getScbScene()					const	{ return mScene;	}
				
		/**
		\brief Get scene pointer from a users perspective.

		When the user removes an object from the scene while the simulation is running, the scene pointer does not get set to NULL immediately. This will only happen
		at the next sync point. However, from an API point of view, a NULL pointer is expected after a removal. This method provides the correct answer in such a case.

		\return The scene pointer as it should be visible from the users perspective.
		*/
		PX_FORCE_INLINE Scb::Scene*			getScbSceneForAPI()				const
											{
												const ControlState::Enum state = getControlState();
												return state == ControlState::eINSERT_PENDING || state == ControlState::eIN_SCENE ? mScene : NULL;
											}

		PX_FORCE_INLINE bool				hasUpdates()					const	{ return getControlFlags() & ControlFlag::eIS_UPDATED;							}
		PX_FORCE_INLINE PxU32				getControlFlags()				const	{ return (mControlState&eCONTROLFLAG_MASK)>>eCONTROLFLAG_SHIFT;					}
		PX_FORCE_INLINE void				setControlFlag(ControlFlag::Enum f)		{ mControlState |= (f<<eCONTROLFLAG_SHIFT);										}	
		PX_FORCE_INLINE void				resetControlFlag(ControlFlag::Enum f)	{ mControlState &=~(f<<eCONTROLFLAG_SHIFT);										}

		PX_FORCE_INLINE ControlState::Enum	getControlState()				const	{ return ControlState::Enum((mControlState&PxU32(eCONTROLSTATE_MASK))>>eCONTROLSTATE_SHIFT);}
		PX_FORCE_INLINE void				setControlState(ControlState::Enum s)	{ mControlState = (mControlState&~eCONTROLSTATE_MASK)|(s<<eCONTROLSTATE_SHIFT);	}
		PX_FORCE_INLINE void				resetControl(ControlState::Enum s)		{ mControlState = (mControlState&~eCONTROL_MASK) | (s<<eCONTROLSTATE_SHIFT);	}

		PX_FORCE_INLINE	void				scheduleForUpdate()						{ mScene->scheduleForUpdate(*this);												}

		/**
		\brief Destroys the object.

		If the simulation is not running, this will release the object, else it will just mark the object as eIS_RELEASED and thus it will get deleted
		when the next sync point is reached. This method expects that the object has been removed from the scene first (or waits for removal).
		*/
						void				destroy();

		/**
		\brief Test if a property has been changed by the user.

		\param[in] flag The flag of the property to test for. Has to be within the bounds of eBUFFERFLAG_MASK.
		\return Positive value if the property has been changed by the user.
		*/
		PX_FORCE_INLINE	Ps::IntBool			isBuffered(PxU32 flag)			const
											{
												PX_ASSERT((flag & eBUFFERFLAG_MASK) == flag); 
												return Ps::IntBool(mControlState & flag);	
											}

		PX_FORCE_INLINE	PxU8*				getStream()								{ return mStreamPtr ? mStreamPtr : mStreamPtr = mScene->getStream(getScbType());	}
		PX_FORCE_INLINE	const PxU8*			getStream()						const	{ PX_ASSERT(mStreamPtr); return mStreamPtr;	}

		/**
		\brief Helper method to trigger object tracking after a property change.

		This method will flag the marked property as changed and will add the object to the list of updated objects if it is not
		registered already.

		\param[in] flag The flag of the changed property. Has to be within the bounds of eBUFFERFLAG_MASK.
		*/
		PX_FORCE_INLINE	void				markUpdated(PxU32 flag)
											{
												PX_ASSERT((flag & eBUFFERFLAG_MASK) == flag);
												scheduleForUpdate();
												mControlState |= flag;
											}
	protected:
											~Base(){}

		PX_FORCE_INLINE PxU32				getBufferFlags()				const	{ return mControlState & eBUFFERFLAG_MASK; }
		PX_FORCE_INLINE void				setBufferFlag(PxU32 flag)				{ PX_ASSERT((flag & eBUFFERFLAG_MASK) == flag); mControlState |= flag; }
		PX_FORCE_INLINE void				resetBufferFlag(PxU32 flag)				{ PX_ASSERT((flag & eBUFFERFLAG_MASK) == flag); mControlState &= ~flag; }
		PX_FORCE_INLINE	void				resetAllBufferFlags()					{ mControlState &=~eBUFFERFLAG_MASK; }

		/**
		\brief Cleanup method after the object has been synced.

		Every buffering object should implement a syncState() method where the buffered user changes get synced with Core. Call this method at the end of the
		syncState() method to clear all buffer flags and references.
		*/
		PX_FORCE_INLINE	void				postSyncState()
											{
												// DS: this can get called even when mScene == NULL, by removeAggregate (see AggregateFreeStandingCreateDelete test) 
												// TODO(dsequeira): investigate that when the dust settles on shape refactoring
												PX_ASSERT(getControlState()!=ControlState::eNOT_IN_SCENE || mScene == NULL);
												PX_ASSERT(getScbType()!=ScbType::eUNDEFINED);

												mStreamPtr = NULL;
												resetAllBufferFlags();
//												resetControlFlag(ControlFlag::eIS_UPDATED);
											}
	private:
						enum {	eBUFFERFLAG_MASK		= (1<<24) - 1,
								eTYPE_MASK				= 15<<24,
								eCONTROLFLAG_MASK		=  3<<28,
								eCONTROLSTATE_MASK		=  3<<30,
								eCONTROL_MASK			= 15<<28};

						enum {	eTYPE_SHIFT				= 24,
								eCONTROLFLAG_SHIFT		= 28,
								eCONTROLSTATE_SHIFT		= 30};

						/**
						\brief Scene pointer.

						The scene pointer get set as soon as the user adds an object to the scene. However, it does not get cleared until the object has been
						removed from the scene internally, i.e., removing an object while the simulation is running will not set this pointer to NULL immediately.
						*/
						Scb::Scene*			mScene;

						/**
						\brief Mix of buffering related states/flags.

						highest                                                    lowest
						| 2            | 2           | 4       | 24                     |
						| ControlState | ControlFlag | ScbType | buffer attribute flags |

						The buffer attribute flags mark which of the properties have been updated. The specific implementation of this class defines those flags.
						*/
						PxU32				mControlState;

						/**
						\brief Data buffer to store property/state changes temporarily.

						Pointer to a temporary struct where user changes made while the simulation is running are buffered. The structure is currently as large as necessary to hold all 
						properties of a buffered object. Even if only a single property of an object gets changed, the whole structure is assigned. The motivation for this was to keep 
						the implementation complexity low based on the assumption that users will not change all objects in a scene every frame. The temporary buffer gets assigned on demand
						and is returned to pools after the data has been synced to Core. The pointer is then set to NULL again.
						This kind of buffer can not be used for properties that get written by the simulation (for example, pose, velocity, sleep state). Those need to be permanently buffered 
						in the specific implementation of this class, else the user might get an inconsistent picture of the scene object state.

						@see postSyncState()
						*/
						PxU8*				mStreamPtr;
	};

}  // namespace Scb

}

#endif
