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


#ifndef PX_PHYSICS_NX_SCENE
#define PX_PHYSICS_NX_SCENE
/** \addtogroup physics
@{
*/

#include "PxVisualizationParameter.h"
#include "PxSceneDesc.h"
#include "PxSimulationStatistics.h"
#include "PxQueryReport.h"
#include "PxQueryFiltering.h"
#include "PxClient.h"
#include "task/PxTask.h"

#include "pvd/PxPvdSceneClient.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxRigidStatic;
class PxRigidDynamic;
class PxConstraint;
class PxMaterial;
class PxSimulationEventCallback;
class PxPhysics;
class PxBatchQueryDesc;
class PxBatchQuery;
class PxAggregate;
class PxRenderBuffer;

class PxSphereGeometry;
class PxBoxGeometry;
class PxCapsuleGeometry;

class PxPruningStructure;
class PxBVHStructure;
struct PxContactPairHeader;

typedef PxU8 PxDominanceGroup;

class PxPvdSceneClient;

/**
\brief Expresses the dominance relationship of a contact.
For the time being only three settings are permitted:

(1, 1), (0, 1), and (1, 0).

@see getDominanceGroup() PxDominanceGroup PxScene::setDominanceGroupPair()
*/	
struct PxDominanceGroupPair
{
	PxDominanceGroupPair(PxU8 a, PxU8 b) 
		: dominance0(a), dominance1(b) {}
	PxU8 dominance0;
	PxU8 dominance1;
};


/**
\brief Identifies each type of actor for retrieving actors from a scene.

\note #PxArticulationLink objects are not supported. Use the #PxArticulation object to retrieve all its links.

@see PxScene::getActors(), PxScene::getNbActors()
*/
struct PxActorTypeFlag
{
	enum Enum
	{
		/**
		\brief A static rigid body
		@see PxRigidStatic
		*/
		eRIGID_STATIC		= (1 << 0),

		/**
		\brief A dynamic rigid body
		@see PxRigidDynamic
		*/
		eRIGID_DYNAMIC		= (1 << 1)
	};
};

/**
\brief Collection of set bits defined in PxActorTypeFlag.

@see PxActorTypeFlag
*/
typedef PxFlags<PxActorTypeFlag::Enum,PxU16> PxActorTypeFlags;
PX_FLAGS_OPERATORS(PxActorTypeFlag::Enum,PxU16)

/**
\brief single hit cache for scene queries.

If a cache object is supplied to a scene query, the cached actor/shape pair is checked for intersection first.
\note Filters are not executed for the cached shape.
\note If intersection is found, the hit is treated as blocking.
\note Typically actor and shape from the last PxHitCallback.block query result is used as a cached actor/shape pair.
\note Using past touching hits as cache will produce incorrect behavior since the cached hit will always be treated as blocking.
\note Cache is only used if no touch buffer was provided, for single nearest blocking hit queries and queries using eANY_HIT flag.
\note if non-zero touch buffer was provided, cache will be ignored

\note It is the user's responsibility to ensure that the shape and actor are valid, so care must be taken
when deleting shapes to invalidate cached references.

The faceIndex field is an additional hint for a mesh or height field which is not currently used.

@see PxScene.raycast
*/
struct PxQueryCache
{
	/**
	\brief constructor sets to default 
	*/
	PX_INLINE PxQueryCache() : shape(NULL), actor(NULL), faceIndex(0xffffffff) {}

	/**
	\brief constructor to set properties
	*/
	PX_INLINE PxQueryCache(PxShape* s, PxU32 findex) : shape(s), actor(NULL), faceIndex(findex) {}

	PxShape*		shape;			//!< Shape to test for intersection first
	PxRigidActor*	actor;			//!< Actor to which the shape belongs
	PxU32			faceIndex;		//!< Triangle index to test first - NOT CURRENTLY SUPPORTED
};

/** 
 \brief A scene is a collection of bodies and constraints which can interact.

 The scene simulates the behavior of these objects over time. Several scenes may exist 
 at the same time, but each body or constraint is specific to a scene 
 -- they may not be shared.

 @see PxSceneDesc PxPhysics.createScene() release()
*/
class PxScene
{
	protected:
	
	/************************************************************************************************/

	/** @name Basics
	*/
	//@{
	
								PxScene(): userData(0)	{}
	virtual						~PxScene()	{}

	public:

	/**
	\brief Deletes the scene.

	Removes any actors and constraint shaders from this scene
	(if the user hasn't already done so).

	Be sure	to not keep a reference to this object after calling release.
	Avoid release calls while the scene is simulating (in between simulate() and fetchResults() calls).
	
	@see PxPhysics.createScene() 
	*/
	virtual		void			release() = 0;

	/**
	\brief Sets a scene flag. You can only set one flag at a time.

	\note Not all flags are mutable and changing some will result in an error. Please check #PxSceneFlag to see which flags can be changed.

	@see PxSceneFlag
	*/
	virtual		void			setFlag(PxSceneFlag::Enum flag, bool value) = 0;

	/**
	\brief Get the scene flags.

	\return The scene flags. See #PxSceneFlag

	@see PxSceneFlag
	*/
	virtual		PxSceneFlags	getFlags() const = 0;


	/**
	\brief Set new scene limits. 

	\note Increase the maximum capacity of various data structures in the scene. The new capacities will be 
	at least as large as required to deal with the objects currently in the scene. Further, these values 
	are for preallocation and do not represent hard limits.

	\param[in] limits Scene limits.
	@see PxSceneLimits
	*/
	virtual void				setLimits(const PxSceneLimits& limits) = 0;

	/**
	\brief Get current scene limits.
	\return Current scene limits.
	@see PxSceneLimits
	*/
	virtual PxSceneLimits		getLimits() const = 0;


	/**
	\brief Call this method to retrieve the Physics SDK.

	\return The physics SDK this scene is associated with.

	@see PxPhysics
	*/
	virtual	PxPhysics&			getPhysics() = 0;

	/**
	\brief Retrieves the scene's internal timestamp, increased each time a simulation step is completed.

	\return scene timestamp
	*/
	virtual	PxU32				getTimestamp()	const	= 0;

	
	//@}
	/************************************************************************************************/

	/** @name Add/Remove Contained Objects
	*/
	//@{
	/**
	\brief Adds an articulation to this scene.

	\note If the articulation is already assigned to a scene (see #PxArticulation::getScene), the call is ignored and an error is issued.

	\param[in] articulation Articulation to add to scene. See #PxArticulation

	@see PxArticulation
	*/
	virtual	void				addArticulation(PxArticulationBase& articulation) = 0;

	/**
	\brief Removes an articulation from this scene.

	\note If the articulation is not part of this scene (see #PxArticulation::getScene), the call is ignored and an error is issued. 
	
	\note If the articulation is in an aggregate it will be removed from the aggregate.

	\param[in] articulation Articulation to remove from scene. See #PxArticulation
	\param[in] wakeOnLostTouch Specifies whether touching objects from the previous frame should get woken up in the next frame. Only applies to PxArticulation and PxRigidActor types.

	@see PxArticulation, PxAggregate
	*/
	virtual	void				removeArticulation(PxArticulationBase& articulation, bool wakeOnLostTouch = true) = 0;


	//@}
	/************************************************************************************************/


	/**
	\brief Adds an actor to this scene.
	
	\note If the actor is already assigned to a scene (see #PxActor::getScene), the call is ignored and an error is issued.
	\note If the actor has an invalid constraint, in checked builds the call is ignored and an error is issued.

	\note You can not add individual articulation links (see #PxArticulationLink) to the scene. Use #addArticulation() instead.

	\note If the actor is a PxRigidActor then each assigned PxConstraint object will get added to the scene automatically if
	it connects to another actor that is part of the scene already. 

	\note When BVHStructure is provided the actor shapes are grouped together. 
	The scene query pruning structure inside PhysX SDK will store/update one
	bound per actor. The scene queries against such an actor will query actor
	bounds and then make a local space query against the provided BVH structure, which is in
	actor's local space.

	\param[in] actor Actor to add to scene.
	\param[in] bvhStructure BVHStructure for actor shapes.

	@see PxActor, PxConstraint::isValid(), PxBVHStructure
	*/
	virtual	void				addActor(PxActor& actor, const PxBVHStructure* bvhStructure = NULL) = 0;

	/**
	\brief Adds actors to this scene.	

	\note If one of the actors is already assigned to a scene (see #PxActor::getScene), the call is ignored and an error is issued.

	\note You can not add individual articulation links (see #PxArticulationLink) to the scene. Use #addArticulation() instead.

	\note If an actor in the array contains an invalid constraint, in checked builds the call is ignored and an error is issued.
	\note If an actor in the array is a PxRigidActor then each assigned PxConstraint object will get added to the scene automatically if
	it connects to another actor that is part of the scene already.

	\note this method is optimized for high performance, and does not support buffering. It may not be called during simulation.

	\param[in] actors Array of actors to add to scene.
	\param[in] nbActors Number of actors in the array.

	@see PxActor, PxConstraint::isValid()
	*/
	virtual	void				addActors(PxActor*const* actors, PxU32 nbActors) = 0;

	/**
	\brief Adds a pruning structure together with its actors to this scene.	

	\note If an actor in the pruning structure contains an invalid constraint, in checked builds the call is ignored and an error is issued.
	\note For all actors in the pruning structure each assigned PxConstraint object will get added to the scene automatically if
	it connects to another actor that is part of the scene already.

	\note This method is optimized for high performance, and does not support buffering. It may not be called during simulation.

	\note Merging a PxPruningStructure into an active scene query optimization AABB tree might unbalance the tree. A typical use case for 
	PxPruningStructure is a large world scenario where blocks of closely positioned actors get streamed in. The merge process finds the 
	best node in the active scene query optimization AABB tree and inserts the PxPruningStructure. Therefore using PxPruningStructure 
	for actors scattered throughout the world will result in an unbalanced tree.

	\param[in] pruningStructure Pruning structure for a set of actors.

	@see PxPhysics::createPruningStructure, PxPruningStructure
	*/
	virtual	void				addActors(const PxPruningStructure& pruningStructure) = 0;

	/**
	\brief Removes an actor from this scene.

	\note If the actor is not part of this scene (see #PxActor::getScene), the call is ignored and an error is issued.

	\note You can not remove individual articulation links (see #PxArticulationLink) from the scene. Use #removeArticulation() instead.

	\note If the actor is a PxRigidActor then all assigned PxConstraint objects will get removed from the scene automatically.

	\note If the actor is in an aggregate it will be removed from the aggregate.

	\param[in] actor Actor to remove from scene.
	\param[in] wakeOnLostTouch Specifies whether touching objects from the previous frame should get woken up in the next frame. Only applies to PxArticulation and PxRigidActor types.

	@see PxActor, PxAggregate
	*/
	virtual	void				removeActor(PxActor& actor, bool wakeOnLostTouch = true) = 0;

	/**
	\brief Removes actors from this scene.

	\note If some actor is not part of this scene (see #PxActor::getScene), the actor remove is ignored and an error is issued.

	\note You can not remove individual articulation links (see #PxArticulationLink) from the scene. Use #removeArticulation() instead.

	\note If the actor is a PxRigidActor then all assigned PxConstraint objects will get removed from the scene automatically.

	\param[in] actors Array of actors to add to scene.
	\param[in] nbActors Number of actors in the array.
	\param[in] wakeOnLostTouch Specifies whether touching objects from the previous frame should get woken up in the next frame. Only applies to PxArticulation and PxRigidActor types.

	@see PxActor
	*/
	virtual	void				removeActors(PxActor*const* actors, PxU32 nbActors, bool wakeOnLostTouch = true) = 0;

	/**
	\brief Adds an aggregate to this scene.
	
	\note If the aggregate is already assigned to a scene (see #PxAggregate::getScene), the call is ignored and an error is issued.
	\note If the aggregate contains an actor with an invalid constraint, in checked builds the call is ignored and an error is issued.

	\note If the aggregate already contains actors, those actors are added to the scene as well.

	\param[in] aggregate Aggregate to add to scene.
	
	@see PxAggregate, PxConstraint::isValid()
	*/
    virtual	void				addAggregate(PxAggregate& aggregate)	= 0;

	/**
	\brief Removes an aggregate from this scene.

	\note If the aggregate is not part of this scene (see #PxAggregate::getScene), the call is ignored and an error is issued.

	\note If the aggregate contains actors, those actors are removed from the scene as well.

	\param[in] aggregate Aggregate to remove from scene.
	\param[in] wakeOnLostTouch Specifies whether touching objects from the previous frame should get woken up in the next frame. Only applies to PxArticulation and PxRigidActor types.

	@see PxAggregate
	*/
	virtual	void				removeAggregate(PxAggregate& aggregate, bool wakeOnLostTouch = true)	= 0;

	/**
	\brief Adds objects in the collection to this scene.

	This function adds the following types of objects to this scene: PxActor, PxAggregate, PxArticulation. 
	This method is typically used after deserializing the collection in order to populate the scene with deserialized objects.

	\note If the collection contains an actor with an invalid constraint, in checked builds the call is ignored and an error is issued.

	\param[in] collection Objects to add to this scene. See #PxCollection

	@see PxCollection, PxConstraint::isValid()
	*/
	virtual	void				addCollection(const PxCollection& collection) = 0;
	//@}
	/************************************************************************************************/

	/** @name Contained Object Retrieval
	*/
	//@{

	/**
	\brief Retrieve the number of actors of certain types in the scene.

	\param[in] types Combination of actor types.
	\return the number of actors.

	@see getActors()
	*/
	virtual	PxU32				getNbActors(PxActorTypeFlags types) const = 0;

	/**
	\brief Retrieve an array of all the actors of certain types in the scene.

	\param[in] types Combination of actor types to retrieve.
	\param[out] userBuffer The buffer to receive actor pointers.
	\param[in] bufferSize Size of provided user buffer.
	\param[in] startIndex Index of first actor pointer to be retrieved
	\return Number of actors written to the buffer.

	@see getNbActors()
	*/
	virtual	PxU32				getActors(PxActorTypeFlags types, PxActor** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const	= 0;

	/**
	\brief Queries the PxScene for a list of the PxActors whose transforms have been 
	updated during the previous simulation step

	\note PxSceneFlag::eENABLE_ACTIVE_ACTORS must be set.

	\note Do not use this method while the simulation is running. Calls to this method while the simulation is running will be ignored and NULL will be returned.

	\param[out] nbActorsOut The number of actors returned.

	\return A pointer to the list of active PxActors generated during the last call to fetchResults().

	@see PxActor
	*/
	virtual PxActor**		getActiveActors(PxU32& nbActorsOut) = 0;

	/**
	\brief Returns the number of articulations in the scene.

	\return the number of articulations in this scene.

	@see getArticulations()
	*/
	virtual PxU32				getNbArticulations() const = 0;

	/**
	\brief Retrieve all the articulations in the scene.

	\param[out] userBuffer The buffer to receive articulations pointers.
	\param[in] bufferSize Size of provided user buffer.
	\param[in] startIndex Index of first articulations pointer to be retrieved
	\return Number of articulations written to the buffer.

	@see getNbArticulations()
	*/
	virtual	PxU32				getArticulations(PxArticulationBase** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const = 0;

	/**
	\brief Returns the number of constraint shaders in the scene.

	\return the number of constraint shaders in this scene.

	@see getConstraints()
	*/
	virtual PxU32				getNbConstraints()	const	= 0;

	/**
	\brief Retrieve all the constraint shaders in the scene.

	\param[out] userBuffer The buffer to receive constraint shader pointers.
	\param[in] bufferSize Size of provided user buffer.
	\param[in] startIndex Index of first constraint pointer to be retrieved
	\return Number of constraint shaders written to the buffer.

	@see getNbConstraints()
	*/
	virtual	PxU32				getConstraints(PxConstraint** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const = 0;


	/**
	\brief Returns the number of aggregates in the scene.

	\return the number of aggregates in this scene.

	@see getAggregates()
	*/
	virtual			PxU32		getNbAggregates()	const	= 0;

	/**
	\brief Retrieve all the aggregates in the scene.

	\param[out] userBuffer The buffer to receive aggregates pointers.
	\param[in] bufferSize Size of provided user buffer.
	\param[in] startIndex Index of first aggregate pointer to be retrieved
	\return Number of aggregates written to the buffer.

	@see getNbAggregates()
	*/
	virtual			PxU32		getAggregates(PxAggregate** userBuffer, PxU32 bufferSize, PxU32 startIndex=0)	const	= 0;

	//@}
	/************************************************************************************************/

	/** @name Dominance
	*/
	//@{

	/**
	\brief Specifies the dominance behavior of contacts between two actors with two certain dominance groups.
	
	It is possible to assign each actor to a dominance groups using #PxActor::setDominanceGroup().

	With dominance groups one can have all contacts created between actors act in one direction only. This is useful, for example, if you
	want an object to push debris out of its way and be unaffected,while still responding physically to forces and collisions
	with non-debris objects.
	
	Whenever a contact between two actors (a0, a1) needs to be solved, the groups (g0, g1) of both
	actors are retrieved. Then the PxDominanceGroupPair setting for this group pair is retrieved with getDominanceGroupPair(g0, g1).
	
	In the contact, PxDominanceGroupPair::dominance0 becomes the dominance setting for a0, and 
	PxDominanceGroupPair::dominance1 becomes the dominance setting for a1. A dominanceN setting of 1.0f, the default, 
	will permit aN to be pushed or pulled by a(1-N) through the contact. A dominanceN setting of 0.0f, will however 
	prevent aN to be pushed by a(1-N) via the contact. Thus, a PxDominanceGroupPair of (1.0f, 0.0f) makes 
	the interaction one-way.
	
	
	The matrix sampled by getDominanceGroupPair(g1, g2) is initialised by default such that:
	
	if g1 == g2, then (1.0f, 1.0f) is returned
	if g1 <  g2, then (0.0f, 1.0f) is returned
	if g1 >  g2, then (1.0f, 0.0f) is returned
	
	In other words, we permit actors in higher groups to be pushed around by actors in lower groups by default.
		
	These settings should cover most applications, and in fact not overriding these settings may likely result in higher performance.
	
	It is not possible to make the matrix asymetric, or to change the diagonal. In other words: 
	
	* it is not possible to change (g1, g2) if (g1==g2)	
	* if you set 
	
	(g1, g2) to X, then (g2, g1) will implicitly and automatically be set to ~X, where:
	
	~(1.0f, 1.0f) is (1.0f, 1.0f)
	~(0.0f, 1.0f) is (1.0f, 0.0f)
	~(1.0f, 0.0f) is (0.0f, 1.0f)
	
	These two restrictions are to make sure that contacts between two actors will always evaluate to the same dominance
	setting, regardless of the order of the actors.
	
	Dominance settings are currently specified as floats 0.0f or 1.0f because in the future we may permit arbitrary 
	fractional settings to express 'partly-one-way' interactions.
		
	<b>Sleeping:</b> Does <b>NOT</b> wake actors up automatically.

	@see getDominanceGroupPair() PxDominanceGroup PxDominanceGroupPair PxActor::setDominanceGroup() PxActor::getDominanceGroup()
	*/
	virtual void				setDominanceGroupPair(
									PxDominanceGroup group1, PxDominanceGroup group2, const PxDominanceGroupPair& dominance) = 0;

	/**
	\brief Samples the dominance matrix.

	@see setDominanceGroupPair() PxDominanceGroup PxDominanceGroupPair PxActor::setDominanceGroup() PxActor::getDominanceGroup()
	*/
	virtual PxDominanceGroupPair getDominanceGroupPair(PxDominanceGroup group1, PxDominanceGroup group2) const = 0;

	//@}
	/************************************************************************************************/

	/** @name Dispatcher
	*/
	//@{

	/**
	\brief Return the cpu dispatcher that was set in PxSceneDesc::cpuDispatcher when creating the scene with PxPhysics::createScene

	@see PxSceneDesc::cpuDispatcher, PxPhysics::createScene
	*/
	virtual PxCpuDispatcher* getCpuDispatcher() const = 0;

	/**
	\brief Return the gpu dispatcher that was set in PxSceneDesc::gpuDispatcher when creating the scene with PxPhysics::createScene

	<b>Platform specific:</b> Applies to PC GPU only.

	@see PxSceneDesc::gpuDispatcher, PxPhysics::createScene
	*/
	virtual PxGpuDispatcher* getGpuDispatcher() const = 0;

	//@}
	/************************************************************************************************/
	/** @name Multiclient
	*/
	//@{
	/**
	\brief Reserves a new client ID.
	
	PX_DEFAULT_CLIENT is always available as the default clientID.
	Additional clients are returned by this function. Clients cannot be released once created. 
	An error is reported when more than a supported number of clients (currently 128) are created. 

	@see PxClientID
	*/
	virtual PxClientID			createClient() = 0;

	//@}

	/************************************************************************************************/

	/** @name Callbacks
	*/
	//@{

	/**
	\brief Sets a user notify object which receives special simulation events when they occur.

	\note Do not set the callback while the simulation is running. Calls to this method while the simulation is running will be ignored.

	\param[in] callback User notification callback. See #PxSimulationEventCallback.

	@see PxSimulationEventCallback getSimulationEventCallback
	*/
	virtual void				setSimulationEventCallback(PxSimulationEventCallback* callback) = 0;

	/**
	\brief Retrieves the simulationEventCallback pointer set with setSimulationEventCallback().

	\return The current user notify pointer. See #PxSimulationEventCallback.

	@see PxSimulationEventCallback setSimulationEventCallback()
	*/
	virtual PxSimulationEventCallback*	getSimulationEventCallback() const = 0;

	/**
	\brief Sets a user callback object, which receives callbacks on all contacts generated for specified actors.

	\note Do not set the callback while the simulation is running. Calls to this method while the simulation is running will be ignored.

	\param[in] callback Asynchronous user contact modification callback. See #PxContactModifyCallback.
	*/
	virtual void				setContactModifyCallback(PxContactModifyCallback* callback) = 0;

	/**
	\brief Sets a user callback object, which receives callbacks on all CCD contacts generated for specified actors.

	\note Do not set the callback while the simulation is running. Calls to this method while the simulation is running will be ignored.

	\param[in] callback Asynchronous user contact modification callback. See #PxCCDContactModifyCallback.
	*/
	virtual void				setCCDContactModifyCallback(PxCCDContactModifyCallback* callback) = 0;

	/**
	\brief Retrieves the PxContactModifyCallback pointer set with setContactModifyCallback().

	\return The current user contact modify callback pointer. See #PxContactModifyCallback.

	@see PxContactModifyCallback setContactModifyCallback()
	*/
	virtual PxContactModifyCallback*	getContactModifyCallback() const = 0;

	/**
	\brief Retrieves the PxCCDContactModifyCallback pointer set with setContactModifyCallback().

	\return The current user contact modify callback pointer. See #PxContactModifyCallback.

	@see PxContactModifyCallback setContactModifyCallback()
	*/
	virtual PxCCDContactModifyCallback*	getCCDContactModifyCallback() const = 0;

	/**
	\brief Sets a broad-phase user callback object.

	\note Do not set the callback while the simulation is running. Calls to this method while the simulation is running will be ignored.

	\param[in] callback	Asynchronous broad-phase callback. See #PxBroadPhaseCallback.
	*/
	virtual void				setBroadPhaseCallback(PxBroadPhaseCallback* callback) = 0;

	/**
	\brief Retrieves the PxBroadPhaseCallback pointer set with setBroadPhaseCallback().

	\return The current broad-phase callback pointer. See #PxBroadPhaseCallback.

	@see PxBroadPhaseCallback setBroadPhaseCallback()
	*/
	virtual PxBroadPhaseCallback* getBroadPhaseCallback()	const = 0;

	//@}
	/************************************************************************************************/

	/** @name Collision Filtering
	*/
	//@{

	/**
	\brief Sets the shared global filter data which will get passed into the filter shader.

	\note It is the user's responsibility to ensure that changing the shared global filter data does not change the filter output value for existing pairs. 
	      If the filter output for existing pairs does change nonetheless then such a change will not take effect until the pair gets refiltered. 
		  resetFiltering() can be used to explicitly refilter the pairs of specific objects.

	\note The provided data will get copied to internal buffers and this copy will be used for filtering calls.

	\note Do not use this method while the simulation is running. Calls to this method while the simulation is running will be ignored.

	\param[in] data The shared global filter shader data.
	\param[in] dataSize Size of the shared global filter shader data (in bytes).

	@see getFilterShaderData() PxSceneDesc.filterShaderData PxSimulationFilterShader
	*/
	virtual void				setFilterShaderData(const void* data, PxU32 dataSize) = 0;

	/**
	\brief Gets the shared global filter data in use for this scene.

	\note The reference points to a copy of the original filter data specified in #PxSceneDesc.filterShaderData or provided by #setFilterShaderData().

	\return Shared filter data for filter shader.

	@see getFilterShaderDataSize() setFilterShaderData() PxSceneDesc.filterShaderData PxSimulationFilterShader
	*/
	virtual	const void*			getFilterShaderData() const = 0;

	/**
	\brief Gets the size of the shared global filter data (#PxSceneDesc.filterShaderData)

	\return Size of shared filter data [bytes].

	@see getFilterShaderData() PxSceneDesc.filterShaderDataSize PxSimulationFilterShader
	*/
	virtual	PxU32				getFilterShaderDataSize() const = 0;

	/**
	\brief Gets the custom collision filter shader in use for this scene.

	\return Filter shader class that defines the collision pair filtering.

	@see PxSceneDesc.filterShader PxSimulationFilterShader
	*/
	virtual	PxSimulationFilterShader
								getFilterShader() const = 0;

	/**
	\brief Gets the custom collision filter callback in use for this scene.

	\return Filter callback class that defines the collision pair filtering.

	@see PxSceneDesc.filterCallback PxSimulationFilterCallback
	*/
	virtual	PxSimulationFilterCallback*
								getFilterCallback() const = 0;

	/**
	\brief Marks the object to reset interactions and re-run collision filters in the next simulation step.
	
	This call forces the object to remove all existing collision interactions, to search anew for existing contact
	pairs and to run the collision filters again for found collision pairs.

	\note The operation is supported for PxRigidActor objects only.

	\note All persistent state of existing interactions will be lost and can not be retrieved even if the same collison pair
	is found again in the next step. This will mean, for example, that you will not get notified about persistent contact
	for such an interaction (see #PxPairFlag::eNOTIFY_TOUCH_PERSISTS), the contact pair will be interpreted as newly found instead.

	\note Lost touch contact reports will be sent for every collision pair which includes this shape, if they have
	been requested through #PxPairFlag::eNOTIFY_TOUCH_LOST or #PxPairFlag::eNOTIFY_THRESHOLD_FORCE_LOST.

	\note This is an expensive operation, don't use it if you don't have to.

	\note Can be used to retrieve collision pairs that were killed by the collision filters (see #PxFilterFlag::eKILL)

	\note It is invalid to use this method if the actor has not been added to a scene already.

	\note It is invalid to use this method if PxActorFlag::eDISABLE_SIMULATION is set.

	<b>Sleeping:</b> Does wake up the actor.

	\param[in] actor The actor for which to re-evaluate interactions.

	@see PxSimulationFilterShader PxSimulationFilterCallback
	*/
	virtual void				resetFiltering(PxActor& actor) = 0;

	/**
	\brief Marks the object to reset interactions and re-run collision filters for specified shapes in the next simulation step.
	
	This is a specialization of the resetFiltering(PxActor& actor) method and allows to reset interactions for specific shapes of
	a PxRigidActor.

	<b>Sleeping:</b> Does wake up the actor.

	\param[in] actor The actor for which to re-evaluate interactions.
	\param[in] shapes The shapes for which to re-evaluate interactions.
	\param[in] shapeCount Number of shapes in the list.

	@see PxSimulationFilterShader PxSimulationFilterCallback
	*/
	virtual void				resetFiltering(PxRigidActor& actor, PxShape*const* shapes, PxU32 shapeCount) = 0;

	//@}
	/************************************************************************************************/

	/** @name Simulation
	*/
	//@{
	/**
 	\brief Advances the simulation by an elapsedTime time.
	
	\note Large elapsedTime values can lead to instabilities. In such cases elapsedTime
	should be subdivided into smaller time intervals and simulate() should be called
	multiple times for each interval.

	Calls to simulate() should pair with calls to fetchResults():
 	Each fetchResults() invocation corresponds to exactly one simulate()
 	invocation; calling simulate() twice without an intervening fetchResults()
 	or fetchResults() twice without an intervening simulate() causes an error
 	condition.
 
 	scene->simulate();
 	...do some processing until physics is computed...
 	scene->fetchResults();
 	...now results of run may be retrieved.


	\param[in] elapsedTime Amount of time to advance simulation by. The parameter has to be larger than 0, else the resulting behavior will be undefined. <b>Range:</b> (0, PX_MAX_F32)
	\param[in] completionTask if non-NULL, this task will have its refcount incremented in simulate(), then
	decremented when the scene is ready to have fetchResults called. So the task will not run until the
	application also calls removeReference().
	\param[in] scratchMemBlock a memory region for physx to use for temporary data during simulation. This block may be reused by the application
	after fetchResults returns. Must be aligned on a 16-byte boundary
	\param[in] scratchMemBlockSize the size of the scratch memory block. Must be a multiple of 16K.
	\param[in] controlSimulation if true, the scene controls its PxTaskManager simulation state. Leave
    true unless the application is calling the PxTaskManager start/stopSimulation() methods itself.

	@see fetchResults() checkResults()
	*/
	virtual	void				simulate(PxReal elapsedTime, physx::PxBaseTask* completionTask = NULL,
									void* scratchMemBlock = 0, PxU32 scratchMemBlockSize = 0, bool controlSimulation = true) = 0;


	/**
 	\brief Performs dynamics phase of the simulation pipeline.
	
	\note Calls to advance() should follow calls to fetchCollision(). An error message will be issued if this sequence is not followed.

	\param[in] completionTask if non-NULL, this task will have its refcount incremented in advance(), then
	decremented when the scene is ready to have fetchResults called. So the task will not run until the
	application also calls removeReference().

	*/
	virtual	void				advance(physx::PxBaseTask* completionTask = 0) = 0;

	/**
	\brief Performs collision detection for the scene over elapsedTime
	
	\note Calls to collide() should be the first method called to simulate a frame.


	\param[in] elapsedTime Amount of time to advance simulation by. The parameter has to be larger than 0, else the resulting behavior will be undefined. <b>Range:</b> (0, PX_MAX_F32)
	\param[in] completionTask if non-NULL, this task will have its refcount incremented in collide(), then
	decremented when the scene is ready to have fetchResults called. So the task will not run until the
	application also calls removeReference().
	\param[in] scratchMemBlock a memory region for physx to use for temporary data during simulation. This block may be reused by the application
	after fetchResults returns. Must be aligned on a 16-byte boundary
	\param[in] scratchMemBlockSize the size of the scratch memory block. Must be a multiple of 16K.
	\param[in] controlSimulation if true, the scene controls its PxTaskManager simulation state. Leave
    true unless the application is calling the PxTaskManager start/stopSimulation() methods itself.

	*/
	virtual	void				collide(PxReal elapsedTime, physx::PxBaseTask* completionTask = 0, void* scratchMemBlock = 0,
									PxU32 scratchMemBlockSize = 0, bool controlSimulation = true) = 0;  
	
	/**
	\brief This checks to see if the simulation run has completed.

	This does not cause the data available for reading to be updated with the results of the simulation, it is simply a status check.
	The bool will allow it to either return immediately or block waiting for the condition to be met so that it can return true
	
	\param[in] block When set to true will block until the condition is met.
	\return True if the results are available.

	@see simulate() fetchResults()
	*/
	virtual	bool				checkResults(bool block = false) = 0;

	/**
	This method must be called after collide() and before advance(). It will wait for the collision phase to finish. If the user makes an illegal simulation call, the SDK will issue an error
	message.

	\param[in] block When set to true will block until the condition is met, which is collision must finish running.
	*/

	virtual	bool				fetchCollision(bool block = false)	= 0;			

	/**
	This is the big brother to checkResults() it basically does the following:
	
	\code
	if ( checkResults(block) )
	{
		fire appropriate callbacks
		swap buffers
		return true
	}
	else
		return false

	\endcode

	\param[in] block When set to true will block until results are available.
	\param[out] errorState Used to retrieve hardware error codes. A non zero value indicates an error.
	\return True if the results have been fetched.

	@see simulate() checkResults()
	*/
	virtual	bool				fetchResults(bool block = false, PxU32* errorState = 0)	= 0;


	/**
	This call performs the first section of fetchResults (callbacks fired before swapBuffers), and returns a pointer to a 
	to the contact streams output by the simulation. It can be used to process contact pairs in parallel, which is often a limiting factor
	for fetchResults() performance. 

	After calling this function and processing the contact streams, call fetchResultsFinish(). Note that writes to the simulation are not
	permitted between the start of fetchResultsStart() and the end of fetchResultsFinish().

	\param[in] block When set to true will block until results are available.
	\param[out] contactPairs an array of pointers to contact pair headers
	\param[out] nbContactPairs the number of contact pairs
	\return True if the results have been fetched.

	@see simulate() checkResults() fetchResults() fetchResultsFinish()
	*/
	virtual	bool				fetchResultsStart(const PxContactPairHeader*& contactPairs, PxU32& nbContactPairs, bool block = false) = 0;


	/**
	This call processes all event callbacks in parallel. It takes a continuation task, which will be executed once all callbacks have been processed.

	This is a utility function to make it easier to process callbacks in parallel using the PhysX task system. It can only be used in conjunction with 
	fetchResultsStart(...) and fetchResultsFinish(...)

	\param[in] continuation The task that will be executed once all callbacks have been processed.
	*/
	virtual void				processCallbacks(physx::PxBaseTask* continuation) = 0;


	/**
	This call performs the second section of fetchResults: the buffer swap and subsequent callbacks.

	It must be called after fetchResultsStart() returns and contact reports have been processed.

	Note that once fetchResultsFinish() has been called, the contact streams returned in fetchResultsStart() will be invalid.

	\param[out] errorState Used to retrieve hardware error codes. A non zero value indicates an error.

	@see simulate() checkResults() fetchResults() fetchResultsStart()
	*/
	virtual	void				fetchResultsFinish(PxU32* errorState = 0) = 0;


	/**
	\brief Clear internal buffers and free memory.

	This method can be used to clear buffers and free internal memory without having to destroy the scene. Can be useful if
	the physics data gets streamed in and a checkpoint with a clean state should be created.

	\note It is not allowed to call this method while the simulation is running. The call will fail.
	
	\param[in] sendPendingReports When set to true pending reports will be sent out before the buffers get cleaned up (for instance lost touch contact/trigger reports due to deleted objects).
	*/
	virtual	void				flushSimulation(bool sendPendingReports = false) = 0;
	
	/**
	\brief Sets a constant gravity for the entire scene.

	<b>Sleeping:</b> Does <b>NOT</b> wake the actor up automatically.

	\param[in] vec A new gravity vector(e.g. PxVec3(0.0f,-9.8f,0.0f) ) <b>Range:</b> force vector

	@see PxSceneDesc.gravity getGravity()
	*/
	virtual void				setGravity(const PxVec3& vec) = 0;

	/**
	\brief Retrieves the current gravity setting.

	\return The current gravity for the scene.

	@see setGravity() PxSceneDesc.gravity
	*/
	virtual PxVec3				getGravity() const = 0;

	/**
	\brief Set the bounce threshold velocity.  Collision speeds below this threshold will not cause a bounce.

	@see PxSceneDesc::bounceThresholdVelocity, getBounceThresholdVelocity
	*/
	virtual void				setBounceThresholdVelocity(const PxReal t) = 0;

	/**
	\brief Return the bounce threshold velocity.

	@see PxSceneDesc.bounceThresholdVelocity, setBounceThresholdVelocity
	*/
	virtual PxReal				getBounceThresholdVelocity() const = 0;


	/**
	\brief Sets the maximum number of CCD passes

	\param[in] ccdMaxPasses Maximum number of CCD passes

	@see PxSceneDesc.ccdMaxPasses getCCDMaxPasses()

	*/
	virtual void				setCCDMaxPasses(PxU32 ccdMaxPasses) = 0;

	/**
	\brief Gets the maximum number of CCD passes.

	\return The maximum number of CCD passes.

	@see PxSceneDesc::ccdMaxPasses setCCDMaxPasses()

	*/
	virtual PxU32				getCCDMaxPasses() const = 0;	

	/**
	\brief Return the value of frictionOffsetThreshold that was set in PxSceneDesc when creating the scene with PxPhysics::createScene

	@see PxSceneDesc::frictionOffsetThreshold,  PxPhysics::createScene
	*/
	virtual PxReal				getFrictionOffsetThreshold() const = 0;

	/**
	\brief Set the friction model.
	@see PxFrictionType, PxSceneDesc::frictionType
	*/
	virtual void setFrictionType(PxFrictionType::Enum frictionType) = 0;

	/**
	\brief Return the friction model.
	@see PxFrictionType, PxSceneDesc::frictionType
	*/
	virtual PxFrictionType::Enum getFrictionType() const = 0;

	//@}
	/************************************************************************************************/

	/** @name Visualization and Statistics
	*/
	//@{
	/**
	\brief Function that lets you set debug visualization parameters.

	Returns false if the value passed is out of range for usage specified by the enum.

	\param[in] param	Parameter to set. See #PxVisualizationParameter
	\param[in] value	The value to set, see #PxVisualizationParameter for allowable values. Setting to zero disables visualization for the specified property, setting to a positive value usually enables visualization and defines the scale factor.
	\return False if the parameter is out of range.

	@see getVisualizationParameter PxVisualizationParameter getRenderBuffer()
	*/
	virtual bool				setVisualizationParameter(PxVisualizationParameter::Enum param, PxReal value) = 0;

	/**
	\brief Function that lets you query debug visualization parameters.

	\param[in] paramEnum The Parameter to retrieve.
	\return The value of the parameter.

	@see setVisualizationParameter PxVisualizationParameter
	*/
	virtual PxReal				getVisualizationParameter(PxVisualizationParameter::Enum paramEnum) const = 0;


	/**
	\brief Defines a box in world space to which visualization geometry will be (conservatively) culled. Use a non-empty culling box to enable the feature, and an empty culling box to disable it.
	
	\param[in] box the box to which the geometry will be culled. Empty box to disable the feature.
	@see setVisualizationParameter getVisualizationCullingBox getRenderBuffer()
	*/
	virtual void				setVisualizationCullingBox(const PxBounds3& box) = 0;

	/**
	\brief Retrieves the visualization culling box.

	\return the box to which the geometry will be culled.
	@see setVisualizationParameter setVisualizationCullingBox 
	*/
	virtual PxBounds3			getVisualizationCullingBox() const = 0;
	
	/**
	\brief Retrieves the render buffer.
	
	This will contain the results of any active visualization for this scene.

	\note Do not use this method while the simulation is running. Calls to this method while result in undefined behaviour.

	\return The render buffer.

	@see PxRenderBuffer
	*/
	virtual const PxRenderBuffer& getRenderBuffer() = 0;
	
	/**
	\brief Call this method to retrieve statistics for the current simulation step.

	\note Do not use this method while the simulation is running. Calls to this method while the simulation is running will be ignored.

	\param[out] stats Used to retrieve statistics for the current simulation step.

	@see PxSimulationStatistics
	*/
	virtual	void				getSimulationStatistics(PxSimulationStatistics& stats) const = 0;
	
	
	//@}
	/************************************************************************************************/

	/** @name Scene Query
	*/
	//@{

	/**
	\brief Return the value of PxSceneDesc::staticStructure that was set when creating the scene with PxPhysics::createScene

	@see PxSceneDesc::staticStructure, PxPhysics::createScene
	*/
	virtual	PxPruningStructureType::Enum getStaticStructure() const = 0;

	/**
	\brief Return the value of PxSceneDesc::dynamicStructure that was set when creating the scene with PxPhysics::createScene

	@see PxSceneDesc::dynamicStructure, PxPhysics::createScene
	*/
	virtual PxPruningStructureType::Enum getDynamicStructure() const = 0;

	/**
	\brief Flushes any changes in the simulation to the scene query representation.

	This method updates the state of the scene query representation to match changes in the scene state.

	By default, these changes are buffered until the next query is submitted. Calling this function will not change
	the results from scene queries, but can be used to ensure that a query will not perform update work in the course of 
	its execution.
	
	A thread performing updates will hold a write lock on the query structure, and thus stall other querying threads. In multithread
	scenarios it can be useful to explicitly schedule the period where this lock may be held for a significant period, so that
	subsequent queries issued from multiple threads will not block.

	Note that while queries will block during the execution of this method, other read operations on the scene will not.
	*/
	virtual	void				flushQueryUpdates() = 0;

	/**
	\brief Creates a BatchQuery object. 

	Scene queries like raycasts, overlap tests and sweeps are batched in this object and are then executed at once. See #PxBatchQuery.

	\deprecated The batched query feature has been deprecated in PhysX version 3.4

	\param[in] desc The descriptor of scene query. Scene Queries need to register a callback. See #PxBatchQueryDesc.

	@see PxBatchQuery PxBatchQueryDesc
	*/
	PX_DEPRECATED virtual	PxBatchQuery*		createBatchQuery(const PxBatchQueryDesc& desc) = 0;

	/**
	\brief Sets the rebuild rate of the dynamic tree pruning structures.

	\param[in] dynamicTreeRebuildRateHint Rebuild rate of the dynamic tree pruning structures.

	@see PxSceneDesc.dynamicTreeRebuildRateHint getDynamicTreeRebuildRateHint() forceDynamicTreeRebuild()
	*/
	virtual	void				setDynamicTreeRebuildRateHint(PxU32 dynamicTreeRebuildRateHint) = 0;

	/**
	\brief Retrieves the rebuild rate of the dynamic tree pruning structures.

	\return The rebuild rate of the dynamic tree pruning structures.

	@see PxSceneDesc.dynamicTreeRebuildRateHint setDynamicTreeRebuildRateHint() forceDynamicTreeRebuild()
	*/
	virtual PxU32				getDynamicTreeRebuildRateHint() const = 0;

	/**
	\brief Forces dynamic trees to be immediately rebuilt.

	\param[in] rebuildStaticStructure	True to rebuild the dynamic tree containing static objects
	\param[in] rebuildDynamicStructure	True to rebuild the dynamic tree containing dynamic objects

	@see PxSceneDesc.dynamicTreeRebuildRateHint setDynamicTreeRebuildRateHint() getDynamicTreeRebuildRateHint()
	*/
	virtual void				forceDynamicTreeRebuild(bool rebuildStaticStructure, bool rebuildDynamicStructure)	= 0;

	/**
	\brief Sets scene query update mode	

	\param[in] updateMode	Scene query update mode.

	@see PxSceneQueryUpdateMode::Enum
	*/
	virtual void				setSceneQueryUpdateMode(PxSceneQueryUpdateMode::Enum updateMode) = 0;

	/**
	\brief Gets scene query update mode	

	\return Current scene query update mode.

	@see PxSceneQueryUpdateMode::Enum
	*/
	virtual PxSceneQueryUpdateMode::Enum getSceneQueryUpdateMode() const = 0;

	/**
	\brief Executes scene queries update tasks.
	This function will refit dirty shapes within the pruner and will execute a task to build a new AABB tree, which is
	build on a different thread. The new AABB tree is built based on the dynamic tree rebuild hint rate. Once
	the new tree is ready it will be commited in next fetchQueries call, which must be called after.

	\note If PxSceneQueryUpdateMode::eBUILD_DISABLED_COMMIT_DISABLED is used, it is required to update the scene queries
	using this function.

	\param[in] completionTask if non-NULL, this task will have its refcount incremented in sceneQueryUpdate(), then
	decremented when the scene is ready to have fetchQueries called. So the task will not run until the
	application also calls removeReference().
	\param[in] controlSimulation if true, the scene controls its PxTaskManager simulation state. Leave
    true unless the application is calling the PxTaskManager start/stopSimulation() methods itself.

	@see PxSceneQueryUpdateMode::eBUILD_DISABLED_COMMIT_DISABLED
	*/
	virtual void				sceneQueriesUpdate(physx::PxBaseTask* completionTask = NULL, bool controlSimulation = true)	= 0;

	/**
	\brief This checks to see if the scene queries update has completed.

	This does not cause the data available for reading to be updated with the results of the scene queries update, it is simply a status check.
	The bool will allow it to either return immediately or block waiting for the condition to be met so that it can return true
	
	\param[in] block When set to true will block until the condition is met.
	\return True if the results are available.

	@see sceneQueriesUpdate() fetchResults()
	*/
	virtual	bool				checkQueries(bool block = false) = 0;

	/**
	This method must be called after sceneQueriesUpdate. It will wait for the scene queries update to finish. If the user makes an illegal scene queries update call, 
	the SDK will issue an error	message.

	If a new AABB tree build finished, then during fetchQueries the current tree within the pruning structure is swapped with the new tree. 

	\param[in] block When set to true will block until the condition is met, which is tree built task must finish running.
	*/

	virtual	bool				fetchQueries(bool block = false)	= 0;	

	/**
	\brief Performs a raycast against objects in the scene, returns results in a PxRaycastBuffer object
	or via a custom user callback implementation inheriting from PxRaycastCallback.

	\note	Touching hits are not ordered.
	\note	Shooting a ray from within an object leads to different results depending on the shape type. Please check the details in user guide article SceneQuery. User can ignore such objects by employing one of the provided filter mechanisms.

	\param[in] origin		Origin of the ray.
	\param[in] unitDir		Normalized direction of the ray.
	\param[in] distance		Length of the ray. Has to be in the [0, inf) range.
	\param[out] hitCall		Raycast hit buffer or callback object used to report raycast hits.
	\param[in] hitFlags		Specifies which properties per hit should be computed and returned via the hit callback.
	\param[in] filterData	Filtering data passed to the filer shader. See #PxQueryFilterData #PxBatchQueryPreFilterShader, #PxBatchQueryPostFilterShader
	\param[in] filterCall	Custom filtering logic (optional). Only used if the corresponding #PxQueryFlag flags are set. If NULL, all hits are assumed to be blocking.
	\param[in] cache		Cached hit shape (optional). Ray is tested against cached shape first. If no hit is found the ray gets queried against the scene.
							Note: Filtering is not executed for a cached shape if supplied; instead, if a hit is found, it is assumed to be a blocking hit.
							Note: Using past touching hits as cache will produce incorrect behavior since the cached hit will always be treated as blocking.

	\return True if any touching or blocking hits were found or any hit was found in case PxQueryFlag::eANY_HIT was specified.

	@see PxRaycastCallback PxRaycastBuffer PxQueryFilterData PxQueryFilterCallback PxQueryCache PxRaycastHit PxQueryFlag PxQueryFlag::eANY_HIT
	*/
	virtual bool				raycast(
									const PxVec3& origin, const PxVec3& unitDir, const PxReal distance,
									PxRaycastCallback& hitCall, PxHitFlags hitFlags = PxHitFlags(PxHitFlag::eDEFAULT),
									const PxQueryFilterData& filterData = PxQueryFilterData(), PxQueryFilterCallback* filterCall = NULL,
									const PxQueryCache* cache = NULL) const = 0;

	/**
	\brief Performs a sweep test against objects in the scene, returns results in a PxSweepBuffer object
	or via a custom user callback implementation inheriting from PxSweepCallback.
	
	\note	Touching hits are not ordered.
	\note	If a shape from the scene is already overlapping with the query shape in its starting position,
			the hit is returned unless eASSUME_NO_INITIAL_OVERLAP was specified.

	\param[in] geometry		Geometry of object to sweep (supported types are: box, sphere, capsule, convex).
	\param[in] pose			Pose of the sweep object.
	\param[in] unitDir		Normalized direction of the sweep.
	\param[in] distance		Sweep distance. Needs to be in [0, inf) range and >0 if eASSUME_NO_INITIAL_OVERLAP was specified. Will be clamped to PX_MAX_SWEEP_DISTANCE.
	\param[out] hitCall		Sweep hit buffer or callback object used to report sweep hits.
	\param[in] hitFlags		Specifies which properties per hit should be computed and returned via the hit callback.
	\param[in] filterData	Filtering data and simple logic.
	\param[in] filterCall	Custom filtering logic (optional). Only used if the corresponding #PxQueryFlag flags are set. If NULL, all hits are assumed to be blocking.
	\param[in] cache		Cached hit shape (optional). Sweep is performed against cached shape first. If no hit is found the sweep gets queried against the scene.
							Note: Filtering is not executed for a cached shape if supplied; instead, if a hit is found, it is assumed to be a blocking hit.
							Note: Using past touching hits as cache will produce incorrect behavior since the cached hit will always be treated as blocking.
	\param[in] inflation	This parameter creates a skin around the swept geometry which increases its extents for sweeping. The sweep will register a hit as soon as the skin touches a shape, and will return the corresponding distance and normal.
							Note: ePRECISE_SWEEP doesn't support inflation. Therefore the sweep will be performed with zero inflation.	
	
	\return True if any touching or blocking hits were found or any hit was found in case PxQueryFlag::eANY_HIT was specified.
							

	@see PxSweepCallback PxSweepBuffer PxQueryFilterData PxQueryFilterCallback PxSweepHit PxQueryCache
	*/
	virtual bool				sweep(const PxGeometry& geometry, const PxTransform& pose, const PxVec3& unitDir, const PxReal distance,
									PxSweepCallback& hitCall, PxHitFlags hitFlags = PxHitFlags(PxHitFlag::eDEFAULT),
									const PxQueryFilterData& filterData = PxQueryFilterData(), PxQueryFilterCallback* filterCall = NULL,
									const PxQueryCache* cache = NULL, const PxReal inflation = 0.f) const = 0;


	/**
	\brief Performs an overlap test of a given geometry against objects in the scene, returns results in a PxOverlapBuffer object
	or via a custom user callback implementation inheriting from PxOverlapCallback.
	
	\note Filtering: returning eBLOCK from user filter for overlap queries will cause a warning (see #PxQueryHitType).

	\param[in] geometry		Geometry of object to check for overlap (supported types are: box, sphere, capsule, convex).
	\param[in] pose			Pose of the object.
	\param[out] hitCall		Overlap hit buffer or callback object used to report overlap hits.
	\param[in] filterData	Filtering data and simple logic. See #PxQueryFilterData #PxQueryFilterCallback
	\param[in] filterCall	Custom filtering logic (optional). Only used if the corresponding #PxQueryFlag flags are set. If NULL, all hits are assumed to overlap.

	\return True if any touching or blocking hits were found or any hit was found in case PxQueryFlag::eANY_HIT was specified.

	\note eBLOCK should not be returned from user filters for overlap(). Doing so will result in undefined behavior, and a warning will be issued.
	\note If the PxQueryFlag::eNO_BLOCK flag is set, the eBLOCK will instead be automatically converted to an eTOUCH and the warning suppressed.

	@see PxOverlapCallback PxOverlapBuffer PxHitFlags PxQueryFilterData PxQueryFilterCallback
	*/
	virtual bool				overlap(const PxGeometry& geometry, const PxTransform& pose, PxOverlapCallback& hitCall,
									const PxQueryFilterData& filterData = PxQueryFilterData(), PxQueryFilterCallback* filterCall = NULL
									) const = 0;


	/**
	\brief Retrieves the scene's internal scene query timestamp, increased each time a change to the
	static scene query structure is performed.

	\return scene query static timestamp
	*/
	virtual	PxU32	getSceneQueryStaticTimestamp()	const	= 0;
	//@}
	
	/************************************************************************************************/
	/** @name Broad-phase
	*/
	//@{

	/**
	\brief Returns broad-phase type.

	\return Broad-phase type
	*/
	virtual	PxBroadPhaseType::Enum	getBroadPhaseType()								const = 0;

	/**
	\brief Gets broad-phase caps.

	\param[out]	caps	Broad-phase caps
	\return True if success
	*/
	virtual	bool					getBroadPhaseCaps(PxBroadPhaseCaps& caps)			const = 0;

	/**
	\brief Returns number of regions currently registered in the broad-phase.

	\return Number of regions
	*/
	virtual	PxU32					getNbBroadPhaseRegions()							const = 0;

	/**
	\brief Gets broad-phase regions.

	\param[out]	userBuffer	Returned broad-phase regions
	\param[in]	bufferSize	Size of userBuffer
	\param[in]	startIndex	Index of first desired region, in [0 ; getNbRegions()[
	\return Number of written out regions
	*/
	virtual	PxU32					getBroadPhaseRegions(PxBroadPhaseRegionInfo* userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const	= 0;

	/**
	\brief Adds a new broad-phase region.

	Note that by default, objects already existing in the SDK that might touch this region will not be automatically
	added to the region. In other words the newly created region will be empty, and will only be populated with new
	objects when they are added to the simulation, or with already existing objects when they are updated.

	It is nonetheless possible to override this default behavior and let the SDK populate the new region automatically
	with already existing objects overlapping the incoming region. This has a cost though, and it should only be used
	when the game can not guarantee that all objects within the new region will be added to the simulation after the
	region itself.

	\param[in]	region			User-provided region data
	\param[in]	populateRegion	Automatically populate new region with already existing objects overlapping it
	\return Handle for newly created region, or 0xffffffff in case of failure.
	*/
	virtual	PxU32					addBroadPhaseRegion(const PxBroadPhaseRegion& region, bool populateRegion=false)		= 0;

	/**
	\brief Removes a new broad-phase region.

	If the region still contains objects, and if those objects do not overlap any region any more, they are not
	automatically removed from the simulation. Instead, the PxBroadPhaseCallback::onObjectOutOfBounds notification
	is used for each object. Users are responsible for removing the objects from the simulation if this is the
	desired behavior.

	If the handle is invalid, or if a valid handle is removed twice, an error message is sent to the error stream.

	\param[in]	handle	Region's handle, as returned by PxScene::addBroadPhaseRegion.
	\return True if success
	*/
	virtual	bool					removeBroadPhaseRegion(PxU32 handle)				= 0;

	//@}

	/************************************************************************************************/

	/** @name Threads and Memory
	*/
	//@{

	/**
	\brief Get the task manager associated with this scene

	\return the task manager associated with the scene
	*/
	virtual PxTaskManager*			getTaskManager() const = 0;


	/**
	\brief Lock the scene for reading from the calling thread.

	When the PxSceneFlag::eREQUIRE_RW_LOCK flag is enabled lockRead() must be 
	called before any read calls are made on the scene.

	Multiple threads may read at the same time, no threads may read while a thread is writing.
	If a call to lockRead() is made while another thread is holding a write lock 
	then the calling thread will be blocked until the writing thread calls unlockWrite().

	\note Lock upgrading is *not* supported, that means it is an error to
	call lockRead() followed by lockWrite().

	\note Recursive locking is supported but each lockRead() call must be paired with an unlockRead().

	\param file String representing the calling file, for debug purposes
	\param line The source file line number, for debug purposes
	*/
	virtual void lockRead(const char* file=NULL, PxU32 line=0) = 0;

	/** 
	\brief Unlock the scene from reading.

	\note Each unlockRead() must be paired with a lockRead() from the same thread.
	*/
	virtual void unlockRead() = 0;

	/**
	\brief Lock the scene for writing from this thread.

	When the PxSceneFlag::eREQUIRE_RW_LOCK flag is enabled lockWrite() must be 
	called before any write calls are made on the scene.

	Only one thread may write at a time and no threads may read while a thread is writing.
	If a call to lockWrite() is made and there are other threads reading then the 
	calling thread will be blocked until the readers complete.

	Writers have priority. If a thread is blocked waiting to write then subsequent calls to 
	lockRead() from other threads will be blocked until the writer completes.

	\note If multiple threads are waiting to write then the thread that is first
	granted access depends on OS scheduling.

	\note Recursive locking is supported but each lockWrite() call must be paired 
	with an unlockWrite().	

	\note If a thread has already locked the scene for writing then it may call
	lockRead().

	\param file String representing the calling file, for debug purposes
	\param line The source file line number, for debug purposes
	*/
	virtual void lockWrite(const char* file=NULL, PxU32 line=0) = 0;

	/**
	\brief Unlock the scene from writing.

	\note Each unlockWrite() must be paired with a lockWrite() from the same thread.
	*/
	virtual void unlockWrite() = 0;
	

	/**
	\brief set the cache blocks that can be used during simulate(). 
	
	Each frame the simulation requires memory to store contact, friction, and contact cache data. This memory is used in blocks of 16K.
	Each frame the blocks used by the previous frame are freed, and may be retrieved by the application using PxScene::flushSimulation()

	This call will force allocation of cache blocks if the numBlocks parameter is greater than the currently allocated number
	of blocks, and less than the max16KContactDataBlocks parameter specified at scene creation time.

	\param[in] numBlocks The number of blocks to allocate.	

	@see PxSceneDesc.nbContactDataBlocks PxSceneDesc.maxNbContactDataBlocks flushSimulation() getNbContactDataBlocksUsed getMaxNbContactDataBlocksUsed
	*/
	virtual         void				setNbContactDataBlocks(PxU32 numBlocks) = 0;
	

	/**
	\brief get the number of cache blocks currently used by the scene 

	This function may not be called while the scene is simulating

	\return the number of cache blocks currently used by the scene

	@see PxSceneDesc.nbContactDataBlocks PxSceneDesc.maxNbContactDataBlocks flushSimulation() setNbContactDataBlocks() getMaxNbContactDataBlocksUsed()
	*/
	virtual         PxU32				getNbContactDataBlocksUsed() const = 0;

	/**
	\brief get the maximum number of cache blocks used by the scene 

	This function may not be called while the scene is simulating

	\return the maximum number of cache blocks everused by the scene

	@see PxSceneDesc.nbContactDataBlocks PxSceneDesc.maxNbContactDataBlocks flushSimulation() setNbContactDataBlocks() getNbContactDataBlocksUsed()
	*/
	virtual         PxU32				getMaxNbContactDataBlocksUsed() const = 0;


	/**
	\brief Return the value of PxSceneDesc::contactReportStreamBufferSize that was set when creating the scene with PxPhysics::createScene

	@see PxSceneDesc::contactReportStreamBufferSize, PxPhysics::createScene
	*/
	virtual PxU32 getContactReportStreamBufferSize() const = 0;

	
	/**
	\brief Sets the number of actors required to spawn a separate rigid body solver thread.

	\param[in] solverBatchSize Number of actors required to spawn a separate rigid body solver thread.

	@see PxSceneDesc.solverBatchSize getSolverBatchSize()
	*/
	virtual	void						setSolverBatchSize(PxU32 solverBatchSize) = 0;

	/**
	\brief Retrieves the number of actors required to spawn a separate rigid body solver thread.

	\return Current number of actors required to spawn a separate rigid body solver thread.

	@see PxSceneDesc.solverBatchSize setSolverBatchSize()
	*/
	virtual PxU32						getSolverBatchSize() const = 0;
	

	//@}

	/**
	\brief Returns the wake counter reset value.

	\return Wake counter reset value

	@see PxSceneDesc.wakeCounterResetValue
	*/
	virtual	PxReal						getWakeCounterResetValue() const = 0;

	/**
	\brief Shift the scene origin by the specified vector.

	The poses of all objects in the scene and the corresponding data structures will get adjusted to reflect the new origin location
	(the shift vector will get subtracted from all object positions).

	\note It is the user's responsibility to keep track of the summed total origin shift and adjust all input/output to/from PhysX accordingly.

	\note Do not use this method while the simulation is running. Calls to this method while the simulation is running will be ignored.

	\note Make sure to propagate the origin shift to other dependent modules (for example, the character controller module etc.).

	\note This is an expensive operation and we recommend to use it only in the case where distance related precision issues may arise in areas far from the origin.

	\param[in] shift Translation vector to shift the origin by.
	*/
	virtual	void					shiftOrigin(const PxVec3& shift) = 0;

	/**
	\brief Returns the Pvd client associated with the scene.
	\return the client, NULL if no PVD supported.
	*/
	virtual PxPvdSceneClient*		getScenePvdClient() = 0;

	void*	userData;	//!< user can assign this to whatever, usually to create a 1:1 relationship with a user object.
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
