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



#ifndef PX_PHYSICS_CCT_CONTROLLER
#define PX_PHYSICS_CCT_CONTROLLER
/** \addtogroup character
  @{
*/

#include "characterkinematic/PxCharacter.h"
#include "characterkinematic/PxExtended.h"
#include "characterkinematic/PxControllerObstacles.h"
#include "PxQueryFiltering.h"
#include "foundation/PxErrorCallback.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/**
\brief The type of controller, eg box, sphere or capsule.
*/
struct PxControllerShapeType
{
	enum Enum
	{
		/**
		\brief A box controller.

		@see PxBoxController PxBoxControllerDesc
		*/
		eBOX,

		/**
		\brief A capsule controller

		@see PxCapsuleController PxCapsuleControllerDesc
		*/
		eCAPSULE,

		eFORCE_DWORD = 0x7fffffff
	};
};

class PxShape;
class PxScene;
class PxController;
class PxRigidDynamic;
class PxMaterial;
struct PxFilterData;
class PxQueryFilterCallback;
class PxControllerBehaviorCallback;
class PxObstacleContext;
class PxObstacle;

/**
\brief specifies how a CCT interacts with non-walkable parts.

This is only used when slopeLimit is non zero. It is currently enabled for static actors only, and not supported for spheres or capsules.
*/
struct PxControllerNonWalkableMode
{
	enum Enum
	{
		ePREVENT_CLIMBING,						//!< Stops character from climbing up non-walkable slopes, but doesn't move it otherwise
		ePREVENT_CLIMBING_AND_FORCE_SLIDING		//!< Stops character from climbing up non-walkable slopes, and forces it to slide down those slopes
	};
};

/**
\brief specifies which sides a character is colliding with.
*/
struct PxControllerCollisionFlag
{
	enum Enum
	{
		eCOLLISION_SIDES	= (1<<0),	//!< Character is colliding to the sides.
		eCOLLISION_UP		= (1<<1),	//!< Character has collision above.
		eCOLLISION_DOWN		= (1<<2)	//!< Character has collision below.
	};
};

/**
\brief Bitfield that contains a set of raised flags defined in PxControllerCollisionFlag.

@see PxControllerCollisionFlag
*/
typedef PxFlags<PxControllerCollisionFlag::Enum, PxU8> PxControllerCollisionFlags;
PX_FLAGS_OPERATORS(PxControllerCollisionFlag::Enum, PxU8)


/**
\brief Describes a controller's internal state.
*/
struct PxControllerState
{
	PxVec3			deltaXP;			//!< delta position vector for the object the CCT is standing/riding on. Not always match the CCT delta when variable timesteps are used.
	PxShape*		touchedShape;		//!< Shape on which the CCT is standing
	PxRigidActor*	touchedActor;		//!< Actor owning 'touchedShape'
	ObstacleHandle	touchedObstacleHandle;	// Obstacle on which the CCT is standing
	PxU32			collisionFlags;		//!< Last known collision flags (PxControllerCollisionFlag)
	bool			standOnAnotherCCT;	//!< Are we standing on another CCT?
	bool			standOnObstacle;	//!< Are we standing on a user-defined obstacle?
	bool			isMovingUp;			//!< is CCT moving up or not? (i.e. explicit jumping)
};

/**
\brief Describes a controller's internal statistics.
*/
struct PxControllerStats
{
	PxU16			nbIterations;
	PxU16			nbFullUpdates;
	PxU16			nbPartialUpdates;
	PxU16			nbTessellation;
};

/**
\brief Describes a generic CCT hit.
*/
struct PxControllerHit
{
	PxController*	controller;		//!< Current controller
	PxExtendedVec3	worldPos;		//!< Contact position in world space
	PxVec3			worldNormal;	//!< Contact normal in world space
	PxVec3			dir;			//!< Motion direction
	PxF32			length;			//!< Motion length
};

/**
\brief Describes a hit between a CCT and a shape. Passed to onShapeHit()

@see PxUserControllerHitReport.onShapeHit()
*/
struct PxControllerShapeHit : public PxControllerHit
{
	PxShape*		shape;			//!< Touched shape
	PxRigidActor*	actor;			//!< Touched actor
	PxU32			triangleIndex;	//!< touched triangle index (only for meshes/heightfields)
};

/**
\brief Describes a hit between a CCT and another CCT. Passed to onControllerHit().

@see PxUserControllerHitReport.onControllerHit()
*/
struct PxControllersHit : public PxControllerHit
{
	PxController*	other;			//!< Touched controller
};

/**
\brief Describes a hit between a CCT and a user-defined obstacle. Passed to onObstacleHit().

@see PxUserControllerHitReport.onObstacleHit() PxObstacleContext
*/
struct PxControllerObstacleHit : public PxControllerHit
{
	const void*		userData;
};

/**
\brief User callback class for character controller events.

\note Character controller hit reports are only generated when move is called.

@see PxControllerDesc.callback
*/
class PxUserControllerHitReport
{
public:

	/**
	\brief Called when current controller hits a shape.

	This is called when the CCT moves and hits a shape. This will not be called when a moving shape hits a non-moving CCT.

	\param[in] hit Provides information about the hit.

	@see PxControllerShapeHit
	*/
	virtual void onShapeHit(const PxControllerShapeHit& hit) = 0;

	/**
	\brief Called when current controller hits another controller.

	\param[in] hit Provides information about the hit.

	@see PxControllersHit
	*/
	virtual void onControllerHit(const PxControllersHit& hit) = 0;

	/**
	\brief Called when current controller hits a user-defined obstacle.

	\param[in] hit Provides information about the hit.

	@see PxControllerObstacleHit PxObstacleContext
	*/
	virtual void onObstacleHit(const PxControllerObstacleHit& hit) = 0;

protected:
	virtual ~PxUserControllerHitReport(){}
};


/**
\brief Dedicated filtering callback for CCT vs CCT.

This controls collisions between CCTs (one CCT vs anoter CCT).

To make each CCT collide against all other CCTs, just return true - or simply avoid defining a callback.
To make each CCT freely go through all other CCTs, just return false.
Otherwise create a custom filtering logic in this callback.

@see PxControllerFilters
*/
class PxControllerFilterCallback
{
public:
	virtual ~PxControllerFilterCallback(){}

	/**
	\brief Filtering method for CCT-vs-CCT.

	\param[in] a	First CCT
	\param[in] b	Second CCT
	\return true to keep the pair, false to filter it out
	*/
	virtual bool filter(const PxController& a, const PxController& b) = 0;
};

/**
\brief Filtering data for "move" call.

This class contains all filtering-related parameters for the PxController::move() call.

Collisions between a CCT and the world are filtered using the mFilterData, mFilterCallback and mFilterFlags
members. These parameters are internally passed to PxScene::overlap() to find objects touched by the CCT.
Please refer to the PxScene::overlap() documentation for details.

Collisions between a CCT and another CCT are filtered using the mCCTFilterCallback member. If this filter
callback is not defined, none of the CCT-vs-CCT collisions are filtered, and each CCT will collide against
all other CCTs.

\note PxQueryFlag::eANY_HIT and PxQueryFlag::eNO_BLOCK are ignored in mFilterFlags.

@see PxController.move() PxControllerFilterCallback
*/
class PxControllerFilters
{
	public:

	PX_INLINE					PxControllerFilters(const PxFilterData* filterData=NULL, PxQueryFilterCallback* cb=NULL, PxControllerFilterCallback* cctFilterCb=NULL) :
									mFilterData			(filterData),
									mFilterCallback		(cb),
									mFilterFlags		(PxQueryFlag::eSTATIC|PxQueryFlag::eDYNAMIC|PxQueryFlag::ePREFILTER),
									mCCTFilterCallback	(cctFilterCb)
								{}

	// CCT-vs-shapes:
	const PxFilterData*			mFilterData;			//!< Data for internal PxQueryFilterData structure. Passed to PxScene::overlap() call.
														//!< This can be NULL, in which case a default PxFilterData is used.
	PxQueryFilterCallback*		mFilterCallback;		//!< Custom filter logic (can be NULL). Passed to PxScene::overlap() call.
	PxQueryFlags				mFilterFlags;			//!< Flags for internal PxQueryFilterData structure. Passed to PxScene::overlap() call.
	// CCT-vs-CCT:
	PxControllerFilterCallback*	mCCTFilterCallback;		//!< CCT-vs-CCT filter callback. If NULL, all CCT-vs-CCT collisions are kept.
};

/**
\brief Descriptor class for a character controller.

@see PxBoxController PxCapsuleController
*/
class PxControllerDesc
{
public:

	/**
	\brief returns true if the current settings are valid

	\return True if the descriptor is valid.
	*/
	PX_INLINE virtual	bool						isValid()		const;

	/**
	\brief Returns the character controller type

	\return The controllers type.

	@see PxControllerType PxCapsuleControllerDesc PxBoxControllerDesc
	*/
	PX_INLINE			PxControllerShapeType::Enum		getType()		const	{ return mType;		}

	/**
	\brief The position of the character

	\note The character's initial position must be such that it does not overlap the static geometry.

	<b>Default:</b> Zero
	*/
	PxExtendedVec3				position;

	/**
	\brief Specifies the 'up' direction

	In order to provide stepping functionality the SDK must be informed about the up direction.

	<b>Default:</b> (0, 1, 0)

	*/
	PxVec3						upDirection;

	/**
	\brief The maximum slope which the character can walk up.

	In general it is desirable to limit where the character can walk, in particular it is unrealistic
	for the character to be able to climb arbitary slopes.

	The limit is expressed as the cosine of desired limit angle. A value of 0 disables this feature.

	\warning It is currently enabled for static actors only (not for dynamic/kinematic actors), and not supported for spheres or capsules.

	<b>Default:</b> 0.707

	@see upDirection invisibleWallHeight maxJumpHeight
	*/
	PxF32						slopeLimit;

	/**
	\brief Height of invisible walls created around non-walkable triangles

	The library can automatically create invisible walls around non-walkable triangles defined
	by the 'slopeLimit' parameter. This defines the height of those walls. If it is 0.0, then
	no extra triangles are created.

	<b>Default:</b> 0.0

	@see upDirection slopeLimit maxJumpHeight
	*/
	PxF32						invisibleWallHeight;

	/**
	\brief Maximum height a jumping character can reach

	This is only used if invisible walls are created ('invisibleWallHeight' is non zero).

	When a character jumps, the non-walkable triangles he might fly over are not found
	by the collision queries (since the character's bounding volume does not touch them).
	Thus those non-walkable triangles do not create invisible walls, and it is possible
	for a jumping character to land on a non-walkable triangle, while he wouldn't have
	reached that place by just walking.

	The 'maxJumpHeight' variable is used to extend the size of the collision volume
	downward. This way, all the non-walkable triangles are properly found by the collision
	queries and it becomes impossible to 'jump over' invisible walls.

	If the character in your game can not jump, it is safe to use 0.0 here. Otherwise it
	is best to keep this value as small as possible, since a larger collision volume
	means more triangles to process.

	<b>Default:</b> 0.0

	@see upDirection slopeLimit invisibleWallHeight
	*/
	PxF32						maxJumpHeight;

	/**
	\brief The contact offset used by the controller.

	Specifies a skin around the object within which contacts will be generated.
	Use it to avoid numerical precision issues.

	This is dependant on the scale of the users world, but should be a small, positive 
	non zero value.

	<b>Default:</b> 0.1
	*/
	PxF32						contactOffset;

	/**
	\brief Defines the maximum height of an obstacle which the character can climb.

	A small value will mean that the character gets stuck and cannot walk up stairs etc, 
	a value which is too large will mean that the character can climb over unrealistically 
	high obstacles.

	<b>Default:</b> 0.5

	@see upDirection 
	*/
	PxF32						stepOffset;

	/**
	\brief Density of underlying kinematic actor

	The CCT creates a PhysX's kinematic actor under the hood. This controls its density.

	<b>Default:</b> 10.0
	*/
	PxF32						density;

	/**
	\brief Scale coefficient for underlying kinematic actor

	The CCT creates a PhysX's kinematic actor under the hood. This controls its scale factor.
	This should be a number a bit smaller than 1.0.

	<b>Default:</b> 0.8
	*/
	PxF32						scaleCoeff;

	/**
	\brief Cached volume growth

	Amount of space around the controller we cache to improve performance. This is a scale factor
	that should be higher than 1.0f but not too big, ideally lower than 2.0f.

	<b>Default:</b> 1.5
	*/
	PxF32						volumeGrowth;

	/**
	\brief Specifies a user report callback.

	This report callback is called when the character collides with shapes and other characters.

	Setting this to NULL disables the callback.

	<b>Default:</b> NULL

	@see PxUserControllerHitReport
	*/
	PxUserControllerHitReport*	reportCallback;

	/**
	\brief Specifies a user behavior callback.

	This behavior callback is called to customize the controller's behavior w.r.t. touched shapes.

	Setting this to NULL disables the callback.

	<b>Default:</b> NULL

	@see PxControllerBehaviorCallback
	*/
	PxControllerBehaviorCallback*	behaviorCallback;

	/**
	\brief The non-walkable mode controls if a character controller slides or not on a non-walkable part.

	This is only used when slopeLimit is non zero.

	<b>Default:</b> PxControllerNonWalkableMode::ePREVENT_CLIMBING

	@see PxControllerNonWalkableMode
	*/
	PxControllerNonWalkableMode::Enum	nonWalkableMode;

	/**
	\brief The material for the actor associated with the controller.
	
	The controller internally creates a rigid body actor. This parameter specifies the material of the actor.

	<b>Default:</b> NULL

	@see PxMaterial
	*/
	PxMaterial*					material;

	/**
	\brief Use a deletion listener to get informed about released objects and clear internal caches if needed.

	If a character controller registers a deletion listener, it will get informed about released objects. That allows the
	controller to invalidate cached data that connects to a released object. If a deletion listener is not
	registered, PxController::invalidateCache has to be called manually after objects have been released.

	@see PxController::invalidateCache

	<b>Default:</b> true
	*/
	bool						registerDeletionListener;

	/**
	\brief User specified data associated with the controller.

	<b>Default:</b> NULL
	*/
	void*						userData;

protected:
	const PxControllerShapeType::Enum mType;		//!< The type of the controller. This gets set by the derived class' ctor, the user should not have to change it.

	/**
	\brief constructor sets to default.
	*/
	PX_INLINE										PxControllerDesc(PxControllerShapeType::Enum);
	PX_INLINE virtual								~PxControllerDesc();

	/**
	\brief copy constructor.
	*/
	PX_INLINE										PxControllerDesc(const PxControllerDesc&);

	/**
	\brief assignment operator.
	*/
	PX_INLINE PxControllerDesc&						operator=(const PxControllerDesc&);

	PX_INLINE void									copy(const PxControllerDesc&);
};

PX_INLINE PxControllerDesc::PxControllerDesc(PxControllerShapeType::Enum t) : mType(t)
{
	upDirection			= PxVec3(0.0f, 1.0f, 0.0f);
	slopeLimit			= 0.707f;
	contactOffset		= 0.1f;
	stepOffset			= 0.5f;
	density				= 10.0f;
	scaleCoeff			= 0.8f;
	volumeGrowth		= 1.5f;
	reportCallback		= NULL;
	behaviorCallback	= NULL;
	userData			= NULL;
	nonWalkableMode		= PxControllerNonWalkableMode::ePREVENT_CLIMBING;
	position.x			= PxExtended(0.0);
	position.y			= PxExtended(0.0);
	position.z			= PxExtended(0.0);
	material			= NULL;
	invisibleWallHeight	= 0.0f;
	maxJumpHeight		= 0.0f;
	registerDeletionListener = true;
}

PX_INLINE PxControllerDesc::PxControllerDesc(const PxControllerDesc& other) : mType(other.mType)
{
	copy(other);
}

PX_INLINE PxControllerDesc& PxControllerDesc::operator=(const PxControllerDesc& other)
{
	copy(other);
	return *this;
}

PX_INLINE void PxControllerDesc::copy(const PxControllerDesc& other)
{
	upDirection			= other.upDirection;
	slopeLimit			= other.slopeLimit;
	contactOffset		= other.contactOffset;
	stepOffset			= other.stepOffset;
	density				= other.density;
	scaleCoeff			= other.scaleCoeff;
	volumeGrowth		= other.volumeGrowth;
	reportCallback		= other.reportCallback;
	behaviorCallback	= other.behaviorCallback;
	userData			= other.userData;
	nonWalkableMode		= other.nonWalkableMode;
	position.x			= other.position.x;
	position.y			= other.position.y;
	position.z			= other.position.z;
	material			= other.material;
	invisibleWallHeight	= other.invisibleWallHeight;
	maxJumpHeight		= other.maxJumpHeight;
	registerDeletionListener = other.registerDeletionListener;
}

PX_INLINE PxControllerDesc::~PxControllerDesc()
{
}

PX_INLINE bool PxControllerDesc::isValid() const
{
	if(		mType!=PxControllerShapeType::eBOX
		&&	mType!=PxControllerShapeType::eCAPSULE)
		return false;
	if(scaleCoeff<0.0f)		return false;
	if(volumeGrowth<1.0f)	return false;
	if(density<0.0f)		return false;
	if(slopeLimit<0.0f)		return false;
	if(stepOffset<0.0f)		return false;
	if(contactOffset<=0.0f)	return false;
	if(!material)			return false;

	return true;
}


/**
\brief Base class for character controllers.

@see PxCapsuleController PxBoxController
*/
class PxController
{
public:
	//*********************************************************************
	// DEPRECATED FUNCTIONS:
	//
	//	PX_DEPRECATED virtual	void						setInteraction(PxCCTInteractionMode::Enum flag)	= 0;
	//	PX_DEPRECATED virtual	PxCCTInteractionMode::Enum	getInteraction()					const		= 0;
	//	PX_DEPRECATED virtual	void						setGroupsBitmask(PxU32 bitmask)					= 0;
	//	PX_DEPRECATED virtual	PxU32						getGroupsBitmask()					const		= 0;
	//
	//	=> replaced with:
	//
	//	PxControllerFilters::mCCTFilterCallback. Please define a PxControllerFilterCallback object and emulate the old interaction mode there.
	//
	//*********************************************************************

	/**
	\brief Return the type of controller

	@see PxControllerType
	*/
	virtual		PxControllerShapeType::Enum	getType()		const			= 0;

	/**
	\brief Releases the controller.
	*/
	virtual		void					release() = 0;

	/**
	\brief Moves the character using a "collide-and-slide" algorithm.

	\param[in] disp	Displacement vector
	\param[in] minDist The minimum travelled distance to consider. If travelled distance is smaller, the character doesn't move.
	This is used to stop the recursive motion algorithm when remaining distance to travel is small.
	\param[in] elapsedTime Time elapsed since last call
	\param[in] filters User-defined filters for this move
	\param[in] obstacles Potential additional obstacles the CCT should collide with.
	\return Collision flags, collection of ::PxControllerCollisionFlags
	*/
	virtual		PxControllerCollisionFlags	move(const PxVec3& disp, PxF32 minDist, PxF32 elapsedTime, const PxControllerFilters& filters, const PxObstacleContext* obstacles=NULL) = 0;

	/**
	\brief Sets controller's position.

	The position controlled by this function is the center of the collision shape.

	\warning This is a 'teleport' function, it doesn't check for collisions.
	\warning The character's position must be such that it does not overlap the static geometry.

	To move the character under normal conditions use the #move() function.

	\param[in] position The new (center) positon for the controller.
	\return Currently always returns true.

	@see PxControllerDesc.position getPosition() getFootPosition() setFootPosition() move()
	*/
	virtual		bool					setPosition(const PxExtendedVec3& position) = 0;

	/**
	\brief Retrieve the raw position of the controller.

	The position retrieved by this function is the center of the collision shape. To retrieve the bottom position of the shape,
	a.k.a. the foot position, use the getFootPosition() function.

	The position is updated by calls to move(). Calling this method without calling
	move() will return the last position or the initial position of the controller.

	\return The controller's center position

	@see PxControllerDesc.position setPosition() getFootPosition() setFootPosition() move()
	*/
	virtual		const PxExtendedVec3&	getPosition()			const	= 0;

	/**
	\brief Set controller's foot position.

	The position controlled by this function is the bottom of the collision shape, a.k.a. the foot position.

	\note The foot position takes the contact offset into account

	\warning This is a 'teleport' function, it doesn't check for collisions.

	To move the character under normal conditions use the #move() function.

	\param[in] position The new (bottom) positon for the controller.
	\return Currently always returns true.

	@see PxControllerDesc.position setPosition() getPosition() getFootPosition() move()
	*/
	virtual		bool					setFootPosition(const PxExtendedVec3& position) = 0;

	/**
	\brief Retrieve the "foot" position of the controller, i.e. the position of the bottom of the CCT's shape.

	\note The foot position takes the contact offset into account

	\return The controller's foot position

	@see PxControllerDesc.position setPosition() getPosition() setFootPosition() move()
	*/
	virtual		PxExtendedVec3			getFootPosition()		const	= 0;

	/**
	\brief Get the rigid body actor associated with this controller (see PhysX documentation). 
	The behavior upon manually altering this actor is undefined, you should primarily 
	use it for reading const properties.

	\return the actor associated with the controller.
	*/
	virtual		PxRigidDynamic*			getActor()				const	= 0;

	/**
	\brief The step height.

	\param[in] offset The new step offset for the controller.

	@see PxControllerDesc.stepOffset
	*/
	virtual	    void					setStepOffset(const PxF32 offset) =0;

	/**
	\brief Retrieve the step height.

	\return The step offset for the controller.

	@see setStepOffset()
	*/
	virtual	    PxF32					getStepOffset()						const		=0;

	/**
	\brief Sets the non-walkable mode for the CCT.

	\param[in] flag The new value of the non-walkable mode.

	\see PxControllerNonWalkableMode
	*/
	virtual		void						setNonWalkableMode(PxControllerNonWalkableMode::Enum flag)	= 0;

	/**
	\brief Retrieves the non-walkable mode for the CCT.

	\return The current non-walkable mode.

	\see PxControllerNonWalkableMode
	*/
	virtual		PxControllerNonWalkableMode::Enum	getNonWalkableMode()				const		= 0;

	/**
	\brief Retrieve the contact offset.

	\return The contact offset for the controller.

	@see PxControllerDesc.contactOffset
	*/
	virtual	    PxF32					getContactOffset()					const		=0;

	/**
	\brief Sets the contact offset.

	\param[in] offset	The contact offset for the controller.

	@see PxControllerDesc.contactOffset
	*/
	virtual	    void					setContactOffset(PxF32 offset)					=0;

	/**
	\brief Retrieve the 'up' direction.

	\return The up direction for the controller.

	@see PxControllerDesc.upDirection
	*/
	virtual		PxVec3					getUpDirection()					const		=0;

	/**
	\brief Sets the 'up' direction.

	\param[in] up The up direction for the controller.

	@see PxControllerDesc.upDirection
	*/
	virtual		void					setUpDirection(const PxVec3& up)				=0;

	/**
	\brief Retrieve the slope limit.

	\return The slope limit for the controller.

	@see PxControllerDesc.slopeLimit
	*/
	virtual	    PxF32					getSlopeLimit()						const		=0;

	/**
	\brief Sets the slope limit.

	\note	This feature can not be enabled at runtime, i.e. if the slope limit is zero when creating the CCT
	(which disables the feature) then changing the slope limit at runtime will not have any effect, and the call
	will be ignored.

	\param[in]	slopeLimit	The slope limit for the controller.

	@see PxControllerDesc.slopeLimit
	*/
	virtual	    void					setSlopeLimit(PxF32 slopeLimit)					=0;

	/**
	\brief Flushes internal geometry cache.
	
	The character controller uses caching in order to speed up collision testing. The cache is
	automatically flushed when a change to static objects is detected in the scene. For example when a
	static shape is added, updated, or removed from the scene, the cache is automatically invalidated.
	
	However there may be situations that cannot be automatically detected, and those require manual
	invalidation of the cache. Currently the user must call this when the filtering behavior changes (the
	PxControllerFilters parameter of the PxController::move call).  While the controller in principle 
	could detect a change in these parameters, it cannot detect a change in the behavior of the filtering 
	function.

	@see PxController.move
	*/
	virtual		void					invalidateCache()			= 0;

	/**
	\brief Retrieve the scene associated with the controller.

	\return The physics scene
	*/
	virtual		PxScene*				getScene()						= 0;

	/**
	\brief Returns the user data associated with this controller.

	\return The user pointer associated with the controller.

	@see PxControllerDesc.userData
	*/
	virtual		void*					getUserData()		const		= 0;

	/**
	\brief Sets the user data associated with this controller.

	\param[in] userData The user pointer associated with the controller.

	@see PxControllerDesc.userData
	*/
	virtual		void					setUserData(void* userData)		= 0;

	/**
	\brief Returns information about the controller's internal state.

	\param[out] state The controller's internal state

	@see PxControllerState
	*/
	virtual		void					getState(PxControllerState& state)	const		= 0;

	/**
	\brief Returns the controller's internal statistics.

	\param[out] stats The controller's internal statistics

	@see PxControllerStats
	*/
	virtual		void					getStats(PxControllerStats& stats)	const		= 0;

	/**
	\brief Resizes the controller.

	This function attempts to resize the controller to a given size, while making sure the bottom
	position of the controller remains constant. In other words the function modifies both the
	height and the (center) position of the controller. This is a helper function that can be used
	to implement a 'crouch' functionality for example.

	\param[in] height Desired controller's height
	*/
	virtual		void					resize(PxReal height)	= 0;

protected:
	PX_INLINE							PxController()					{}
	virtual								~PxController()					{}
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
