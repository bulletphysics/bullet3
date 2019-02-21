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

#ifndef PX_PHYSICS_NX_PHYSICS
#define PX_PHYSICS_NX_PHYSICS

/** \addtogroup physics
@{
*/

#include "PxPhysXConfig.h"
#include "PxDeletionListener.h"
#include "foundation/PxTransform.h"
#include "PxShape.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxPvd;
class PxPhysicsInsertionCallback;

class PxRigidActor;
class PxConstraintConnector;
struct PxConstraintShaderTable;

class PxGeometry;
class PxFoundation;
class PxSerializationRegistry;

class PxPruningStructure;
class PxBVHStructure;

/**
\brief Abstract singleton factory class used for instancing objects in the Physics SDK.

In addition you can use PxPhysics to set global parameters which will effect all scenes and create 
objects that can be shared across multiple scenes.

You can get an instance of this class by calling PxCreateBasePhysics() or PxCreatePhysics() with pre-registered modules.

@see PxCreatePhysics() PxCreateBasePhysics() PxScene PxVisualizationParameter
*/
class PxPhysics
{
public:

	/** @name Basics
	*/
	//@{
	
	virtual ~PxPhysics() {}
	
	/**	
	\brief Destroys the instance it is called on.

	Use this release method to destroy an instance of this class. Be sure
	to not keep a reference to this object after calling release.
	Avoid release calls while a scene is simulating (in between simulate() and fetchResults() calls).

	Note that this must be called once for each prior call to PxCreatePhysics, as
	there is a reference counter. Also note that you mustn't destroy the allocator or the error callback (if available) until after the
	reference count reaches 0 and the SDK is actually removed.

	Releasing an SDK will also release any scenes, triangle meshes, convex meshes, heightfields and shapes
	created through it, provided the user hasn't already done so.

	\note This function is required to be called to release foundation usage.

	@see PxCreatePhysics()
	*/
	virtual	void release() = 0;

	/**
	\brief Retrieves the Foundation instance.
	\return A reference to the Foundation object.
	*/
	virtual PxFoundation&		getFoundation() = 0;
		
	/**
	\brief Creates an aggregate with the specified maximum size and selfCollision property.

	\param	[in] maxSize				The maximum number of actors that may be placed in the aggregate.
	\param	[in] enableSelfCollision	Whether the aggregate supports self-collision
	\return The new aggregate.

	@see PxAggregate
	*/
	virtual	PxAggregate*		createAggregate(PxU32 maxSize, bool enableSelfCollision)	= 0;

	/**
	\brief Returns the simulation tolerance parameters.  
	\return The current simulation tolerance parameters.  
	*/
	virtual const PxTolerancesScale&		getTolerancesScale() const = 0;

	//@}
	/** @name Meshes
	*/
	//@{

	/**
	\brief Creates a triangle mesh object.

	This can then be instanced into #PxShape objects.

	\param	[in] stream	The triangle mesh stream.
	\return The new triangle mesh.

	@see PxTriangleMesh PxMeshPreprocessingFlag PxTriangleMesh.release() PxInputStream PxTriangleMeshFlag
	*/
	virtual PxTriangleMesh*    createTriangleMesh(PxInputStream& stream) = 0;
	
	/**
	\brief Return the number of triangle meshes that currently exist.

	\return Number of triangle meshes.

	@see getTriangleMeshes()
	*/
	virtual PxU32				getNbTriangleMeshes() const = 0;

	/**
	\brief Writes the array of triangle mesh pointers to a user buffer.
	
	Returns the number of pointers written.

	The ordering of the triangle meshes in the array is not specified.

	\param	[out] userBuffer	The buffer to receive triangle mesh pointers.
	\param	[in]  bufferSize	The number of triangle mesh pointers which can be stored in the buffer.
	\param	[in]  startIndex	Index of first mesh pointer to be retrieved
	\return The number of triangle mesh pointers written to userBuffer, this should be less or equal to bufferSize.

	@see getNbTriangleMeshes() PxTriangleMesh
	*/
	virtual	PxU32				getTriangleMeshes(PxTriangleMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const = 0;

	/**
	\brief Creates a heightfield object from previously cooked stream.

	This can then be instanced into #PxShape objects.

	\param	[in] stream	The heightfield mesh stream.
	\return The new heightfield.

	@see PxHeightField PxHeightField.release() PxInputStream PxRegisterHeightFields
	*/
	virtual PxHeightField*		createHeightField(PxInputStream& stream) = 0;

	/**
	\brief Return the number of heightfields that currently exist.

	\return Number of heightfields.

	@see getHeightFields()
	*/
	virtual PxU32				getNbHeightFields() const = 0;

	/**
	\brief Writes the array of heightfield pointers to a user buffer.
	
	Returns the number of pointers written.

	The ordering of the heightfields in the array is not specified.

	\param	[out] userBuffer	The buffer to receive heightfield pointers.
	\param	[in]  bufferSize	The number of heightfield pointers which can be stored in the buffer.
	\param	[in]  startIndex	Index of first heightfield pointer to be retrieved
	\return The number of heightfield pointers written to userBuffer, this should be less or equal to bufferSize.

	@see getNbHeightFields() PxHeightField
	*/
	virtual	PxU32				getHeightFields(PxHeightField** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const = 0;

	/**
	\brief Creates a convex mesh object.

	This can then be instanced into #PxShape objects.

	\param	[in] stream	The stream to load the convex mesh from.
	\return The new convex mesh.

	@see PxConvexMesh PxConvexMesh.release() PxInputStream createTriangleMesh() PxConvexMeshGeometry PxShape
	*/
	virtual PxConvexMesh*		createConvexMesh(PxInputStream &stream)					= 0;

	/**
	\brief Return the number of convex meshes that currently exist.

	\return Number of convex meshes.

	@see getConvexMeshes()
	*/
	virtual PxU32				getNbConvexMeshes() const = 0;

	/**
	\brief Writes the array of convex mesh pointers to a user buffer.
	
	Returns the number of pointers written.

	The ordering of the convex meshes in the array is not specified.

	\param	[out] userBuffer	The buffer to receive convex mesh pointers.
	\param	[in]  bufferSize	The number of convex mesh pointers which can be stored in the buffer.
	\param	[in]  startIndex	Index of first convex mesh pointer to be retrieved
	\return The number of convex mesh pointers written to userBuffer, this should be less or equal to bufferSize.

	@see getNbConvexMeshes() PxConvexMesh
	*/
	virtual	PxU32				getConvexMeshes(PxConvexMesh** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const = 0;

	/**
	\brief Creates a bounding volume hierarchy structure.
	
	\param	[in] stream	The stream to load the BVH structure from.
	\return The new BVH structure.

	@see PxBVHStructure PxInputStream
	*/
	virtual PxBVHStructure*		createBVHStructure(PxInputStream &stream)					= 0;

	/**
	\brief Return the number of bounding volume hierarchy structures that currently exist.

	\return Number of bounding volume hierarchy structures.

	@see getBVHStructures()
	*/
	virtual PxU32				getNbBVHStructures() const = 0;

	/**
	\brief Writes the array of bounding volume hierarchy structure pointers to a user buffer.
	
	Returns the number of pointers written.

	The ordering of the BVH structures in the array is not specified.

	\param	[out] userBuffer	The buffer to receive BVH structure pointers.
	\param	[in]  bufferSize	The number of BVH structure pointers which can be stored in the buffer.
	\param	[in]  startIndex	Index of first BVH structure pointer to be retrieved
	\return The number of BVH structure pointers written to userBuffer, this should be less or equal to bufferSize.

	@see getNbBVHStructures() PxBVHStructure
	*/
	virtual	PxU32				getBVHStructures(PxBVHStructure** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const = 0;

	//@}
	/** @name Scenes
	*/
	//@{

	/**
	\brief Creates a scene.

	\note Every scene uses a Thread Local Storage slot. This imposes a platform specific limit on the
	number of scenes that can be created.

	\param	[in] sceneDesc	Scene descriptor. See #PxSceneDesc
	\return The new scene object.

	@see PxScene PxScene.release() PxSceneDesc
	*/
	virtual PxScene*			createScene(const PxSceneDesc& sceneDesc) = 0;

	/**
	\brief Gets number of created scenes.

	\return The number of scenes created.

	@see getScene()
	*/
	virtual PxU32				getNbScenes()			const	= 0;

	/**
	\brief Writes the array of scene pointers to a user buffer.
	
	Returns the number of pointers written.

	The ordering of the scene pointers in the array is not specified.

	\param	[out] userBuffer	The buffer to receive scene pointers.
	\param	[in]  bufferSize	The number of scene pointers which can be stored in the buffer.
	\param	[in]  startIndex	Index of first scene pointer to be retrieved
	\return The number of scene pointers written to userBuffer, this should be less or equal to bufferSize.

	@see getNbScenes() PxScene
	*/
	virtual	PxU32				getScenes(PxScene** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const = 0;
	
	//@}
	/** @name Actors
	*/
	//@{

	/**
	\brief Creates a static rigid actor with the specified pose and all other fields initialized
	to their default values.
	
	\param	[in] pose	The initial pose of the actor. Must be a valid transform

	@see PxRigidStatic
	*/
	virtual PxRigidStatic*      createRigidStatic(const PxTransform& pose) = 0;

	/**
	\brief Creates a dynamic rigid actor with the specified pose and all other fields initialized
	to their default values.
	
	\param	[in] pose	The initial pose of the actor. Must be a valid transform

	@see PxRigidDynamic
	*/
	virtual PxRigidDynamic*      createRigidDynamic(const PxTransform& pose) = 0;

	/**
	\brief Creates a pruning structure from actors.

	\note Every provided actor needs at least one shape with the eSCENE_QUERY_SHAPE flag set.
	\note Both static and dynamic actors can be provided.
	\note It is not allowed to pass in actors which are already part of a scene.
	\note Articulation links cannot be provided.

	\param	[in] actors		Array of actors to add to the pruning structure. Must be non NULL.
	\param	[in] nbActors	Number of actors in the array. Must be >0.
	\return Pruning structure created from given actors, or NULL if any of the actors did not comply with the above requirements.
	@see PxActor PxPruningStructure
	*/
	virtual PxPruningStructure*	createPruningStructure(PxRigidActor*const* actors, PxU32 nbActors)	= 0;

	//@}
	/** @name Shapes
	*/
	//@{
	
	/**
	\brief Creates a shape which may be attached to multiple actors
	
	The shape will be created with a reference count of 1.

	\param	[in] geometry		The geometry for the shape
	\param	[in] material		The material for the shape
	\param	[in] isExclusive	Whether this shape is exclusive to a single actor or maybe be shared
	\param	[in] shapeFlags		The PxShapeFlags to be set

	Shared shapes are not mutable when they are attached to an actor

	@see PxShape
	*/
	PX_FORCE_INLINE	PxShape*	createShape(	const PxGeometry& geometry, 
												const PxMaterial& material, 
												bool isExclusive = false, 
												PxShapeFlags shapeFlags = PxShapeFlag::eVISUALIZATION | PxShapeFlag::eSCENE_QUERY_SHAPE | PxShapeFlag::eSIMULATION_SHAPE)
	{
		PxMaterial* materialPtr = const_cast<PxMaterial*>(&material);
		return createShape(geometry, &materialPtr, 1, isExclusive, shapeFlags);
	}

	/**
	\brief Creates a shape which may be attached to multiple actors
	
	The shape will be created with a reference count of 1.

	\param	[in] geometry		The geometry for the shape
	\param	[in] materials		The materials for the shape
	\param	[in] materialCount	The number of materials
	\param	[in] isExclusive	Whether this shape is exclusive to a single actor or may be shared
	\param	[in] shapeFlags		The PxShapeFlags to be set

	Shared shapes are not mutable when they are attached to an actor

	@see PxShape
	*/
	virtual PxShape*			createShape(const PxGeometry& geometry, 
											PxMaterial*const * materials, 
											PxU16 materialCount, 
											bool isExclusive = false,
											PxShapeFlags shapeFlags = PxShapeFlag::eVISUALIZATION | PxShapeFlag::eSCENE_QUERY_SHAPE | PxShapeFlag::eSIMULATION_SHAPE) = 0;

	/**
	\brief Return the number of shapes that currently exist.

	\return Number of shapes.

	@see getShapes()
	*/
	virtual PxU32				getNbShapes() const = 0;

	/**
	\brief Writes the array of shape pointers to a user buffer.
	
	Returns the number of pointers written.

	The ordering of the shapes in the array is not specified.

	\param	[out] userBuffer	The buffer to receive shape pointers.
	\param	[in]  bufferSize	The number of shape pointers which can be stored in the buffer.
	\param	[in]  startIndex	Index of first shape pointer to be retrieved
	\return The number of shape pointers written to userBuffer, this should be less or equal to bufferSize.

	@see getNbShapes() PxShape
	*/
	virtual	PxU32				getShapes(PxShape** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const = 0;

	//@}
	/** @name Constraints and Articulations
	*/
	//@{

	/**
	\brief Creates a constraint shader.

	\note A constraint shader will get added automatically to the scene the two linked actors belong to. Either, but not both, of actor0 and actor1 may
	be NULL to denote attachment to the world.
	
	\param	[in] actor0		The first actor
	\param	[in] actor1		The second actor
	\param	[in] connector	The connector object, which the SDK uses to communicate with the infrastructure for the constraint
	\param	[in] shaders	The shader functions for the constraint
	\param	[in] dataSize	The size of the data block for the shader

	\return The new shader.

	@see PxConstraint
	*/
	virtual PxConstraint*      createConstraint(PxRigidActor* actor0, PxRigidActor* actor1, PxConstraintConnector& connector, const PxConstraintShaderTable& shaders, PxU32 dataSize)		= 0;

	/**
	\brief Creates an articulation with all fields initialized to their default values.
	
	\return the new articulation

	@see PxArticulation, PxRegisterArticulations
	*/
	virtual PxArticulation*						createArticulation() = 0;

	/**
	\brief Creates a reduced coordinate articulation with all fields initialized to their default values.

	\return the new articulation

	@see PxArticulationReducedCoordinate, PxRegisterArticulationsReducedCoordinate
	*/
	virtual PxArticulationReducedCoordinate*	createArticulationReducedCoordinate() = 0;


	//@}
	/** @name Materials
	*/
	//@{

	/**
	\brief Creates a new material with default properties.

	\return The new material.

	\param	[in] staticFriction		The coefficient of static friction
	\param	[in] dynamicFriction	The coefficient of dynamic friction
	\param	[in] restitution		The coefficient of restitution

	@see PxMaterial 
	*/
	virtual PxMaterial*        createMaterial(PxReal staticFriction, PxReal dynamicFriction, PxReal restitution)		= 0;


	/**
	\brief Return the number of materials that currently exist.

	\return Number of materials.

	@see getMaterials()
	*/
	virtual PxU32				getNbMaterials() const = 0;

	/**
	\brief Writes the array of material pointers to a user buffer.
	
	Returns the number of pointers written.

	The ordering of the materials in the array is not specified.

	\param	[out] userBuffer	The buffer to receive material pointers.
	\param	[in]  bufferSize	The number of material pointers which can be stored in the buffer.
	\param	[in]  startIndex	Index of first material pointer to be retrieved
	\return The number of material pointers written to userBuffer, this should be less or equal to bufferSize.

	@see getNbMaterials() PxMaterial
	*/
	virtual	PxU32				getMaterials(PxMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const = 0;

	//@}
	/** @name Deletion Listeners
	*/
	//@{

	/**
	\brief Register a deletion listener. Listeners will be called whenever an object is deleted.

	It is illegal to register or unregister a deletion listener while deletions are being processed.

	\note By default a registered listener will receive events from all objects. Set the restrictedObjectSet parameter to true on registration and use #registerDeletionListenerObjects to restrict the received events to specific objects.

	\note The deletion events are only supported on core PhysX objects. In general, objects in extension modules do not provide this functionality, however, in the case of PxJoint objects, the underlying PxConstraint will send the events.

	\param	[in] observer				Observer object to send notifications to.
	\param	[in] deletionEvents			The deletion event types to get notified of.
	\param	[in] restrictedObjectSet	If false, the deletion listener will get events from all objects, else the objects to receive events from have to be specified explicitly through #registerDeletionListenerObjects.

	@see PxDeletionListener unregisterDeletionListener
	*/
	virtual void registerDeletionListener(PxDeletionListener& observer, const PxDeletionEventFlags& deletionEvents, bool restrictedObjectSet = false) = 0;

	/**
	\brief Unregister a deletion listener. 

	It is illegal to register or unregister a deletion listener while deletions are being processed.

	\param	[in] observer	Observer object to send notifications to

	@see PxDeletionListener registerDeletionListener
	*/
	virtual void unregisterDeletionListener(PxDeletionListener& observer) = 0;

	/**
	\brief Register specific objects for deletion events.

	This method allows for a deletion listener to limit deletion events to specific objects only.

	\note It is illegal to register or unregister objects while deletions are being processed.

	\note The deletion listener has to be registered through #registerDeletionListener() and configured to support restricted objects sets prior to this method being used.

	\param	[in] observer			Observer object to send notifications to.
	\param	[in] observables		List of objects for which to receive deletion events. Only PhysX core objects are supported. In the case of PxJoint objects, the underlying PxConstraint can be used to get the events.
	\param	[in] observableCount	Size of the observables list.

	@see PxDeletionListener unregisterDeletionListenerObjects
	*/
	virtual void registerDeletionListenerObjects(PxDeletionListener& observer, const PxBase* const* observables, PxU32 observableCount) = 0;

	/**
	\brief Unregister specific objects for deletion events.

	This method allows to clear previously registered objects for a deletion listener (see #registerDeletionListenerObjects()).

	\note It is illegal to register or unregister objects while deletions are being processed.

	\note The deletion listener has to be registered through #registerDeletionListener() and configured to support restricted objects sets prior to this method being used.

	\param	[in] observer			Observer object to stop sending notifications to.
	\param	[in] observables		List of objects for which to not receive deletion events anymore.
	\param	[in] observableCount	Size of the observables list.

	@see PxDeletionListener registerDeletionListenerObjects
	*/
	virtual void unregisterDeletionListenerObjects(PxDeletionListener& observer, const PxBase* const* observables, PxU32 observableCount) = 0;

	/**
	\brief Gets PxPhysics object insertion interface. 

	The insertion interface is needed ie. for PxCooking::createTriangleMesh, this allows runtime mesh creation. This is not advised to do, please 
	use offline cooking if possible.

	@see PxCooking::createTriangleMesh PxCooking::createHeightfield
	*/
	virtual PxPhysicsInsertionCallback& getPhysicsInsertionCallback() = 0;

	//@}
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/**
\brief Enables the usage of the articulations feature.  This function is called automatically inside PxCreatePhysics().
On resource constrained platforms, it is possible to call PxCreateBasePhysics() and then NOT call this function 
to save on code memory if your application does not use articulations.  In this case the linker should strip out
the relevant implementation code from the library.  If you need to use articulations but not some other optional 
component, you shoud call PxCreateBasePhysics() followed by this call.
*/
PX_C_EXPORT PX_PHYSX_CORE_API void PX_CALL_CONV PxRegisterArticulations(physx::PxPhysics& physics);


/**
\brief Enables the usage of the reduced coordinate articulations feature.  This function is called automatically inside PxCreatePhysics().
On resource constrained platforms, it is possible to call PxCreateBasePhysics() and then NOT call this function
to save on code memory if your application does not use articulations.  In this case the linker should strip out
the relevant implementation code from the library.  If you need to use articulations but not some other optional
component, you shoud call PxCreateBasePhysics() followed by this call.
*/
PX_C_EXPORT PX_PHYSX_CORE_API void PX_CALL_CONV PxRegisterArticulationsReducedCoordinate(physx::PxPhysics& physics);

/**
\brief Enables the usage of the heightfield feature.  

This call will link the default 'unified' implementation of heightfields which is identical to the narrow phase of triangle meshes.
This function is called automatically inside PxCreatePhysics().

On resource constrained platforms, it is possible to call PxCreateBasePhysics() and then NOT call this function
to save on code memory if your application does not use heightfields.  In this case the linker should strip out
the relevant implementation code from the library.  If you need to use heightfield but not some other optional
component, you shoud call PxCreateBasePhysics() followed by this call.

You must call this function at a time where no ::PxScene instance exists, typically before calling PxPhysics::createScene().
This is to prevent a change to the heightfield implementation code at runtime which would have undefined results.

Calling PxCreateBasePhysics() and then attempting to create a heightfield shape without first calling 
::PxRegisterHeightFields(), will result in an error.
*/
PX_C_EXPORT PX_PHYSX_CORE_API void PX_CALL_CONV PxRegisterHeightFields(physx::PxPhysics& physics);

/**
\brief Creates an instance of the physics SDK with minimal additional components registered

Creates an instance of this class. May not be a class member to avoid name mangling.
Pass the constant #PX_PHYSICS_VERSION as the argument.
There may be only one instance of this class per process. Calling this method after an instance 
has been created already will result in an error message and NULL will be returned.

\param version Version number we are expecting(should be #PX_PHYSICS_VERSION)
\param foundation Foundation instance (see PxFoundation)
\param scale values used to determine default tolerances for objects at creation time
\param trackOutstandingAllocations true if you want to track memory allocations 
			so a debugger connection partway through your physics simulation will get
			an accurate map of everything that has been allocated so far.  This could have a memory
			and performance impact on your simulation hence it defaults to off.
\param pvd When pvd points to a valid PxPvd instance (PhysX Visual Debugger), a connection to the specified PxPvd instance is created.
			If pvd is NULL no connection will be attempted.
\return PxPhysics instance on success, NULL if operation failed

@see PxPhysics, PxFoundation, PxTolerancesScale, PxPvd
*/
PX_C_EXPORT PX_PHYSX_CORE_API physx::PxPhysics* PX_CALL_CONV PxCreateBasePhysics(physx::PxU32 version,
																			     physx::PxFoundation& foundation,
																			     const physx::PxTolerancesScale& scale,																				 
																			     bool trackOutstandingAllocations = false,
																				 physx::PxPvd* pvd = NULL);

/**
\brief Creates an instance of the physics SDK.

Creates an instance of this class. May not be a class member to avoid name mangling.
Pass the constant #PX_PHYSICS_VERSION as the argument.
There may be only one instance of this class per process. Calling this method after an instance 
has been created already will result in an error message and NULL will be returned.

Calling this will register all optional code modules (Articulations and HeightFields), preparing them for use.  
If you do not need some of these modules, consider calling PxCreateBasePhysics() instead and registering needed 
modules manually.

\param version Version number we are expecting(should be #PX_PHYSICS_VERSION)
\param foundation Foundation instance (see PxFoundation)
\param scale values used to determine default tolerances for objects at creation time
\param trackOutstandingAllocations true if you want to track memory allocations 
			so a debugger connection partway through your physics simulation will get
			an accurate map of everything that has been allocated so far.  This could have a memory
			and performance impact on your simulation hence it defaults to off.
\param pvd When pvd points to a valid PxPvd instance (PhysX Visual Debugger), a connection to the specified PxPvd instance is created.
			If pvd is NULL no connection will be attempted.
\return PxPhysics instance on success, NULL if operation failed

@see PxPhysics, PxCreateBasePhysics, PxRegisterArticulations, PxRegisterArticulationsReducedCoordinate, PxRegisterHeightFields 
*/
PX_INLINE physx::PxPhysics* PxCreatePhysics(physx::PxU32 version,
											physx::PxFoundation& foundation,
											const physx::PxTolerancesScale& scale,											
											bool trackOutstandingAllocations = false,
											physx::PxPvd* pvd = NULL )
{
	physx::PxPhysics* physics = PxCreateBasePhysics(version, foundation, scale, trackOutstandingAllocations, pvd);
	if(!physics)
		return NULL;

	PxRegisterArticulations(*physics);
	PxRegisterArticulationsReducedCoordinate(*physics);
	PxRegisterHeightFields(*physics);

	return physics;
}


/**
\brief Retrieves the Physics SDK after it has been created.

Before using this function the user must call #PxCreatePhysics().

\note The behavior of this method is undefined if the Physics SDK instance has not been created already.
*/
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
#endif

PX_C_EXPORT PX_PHYSX_CORE_API physx::PxPhysics& PX_CALL_CONV PxGetPhysics();

#ifdef __clang__
#pragma clang diagnostic pop
#endif

/** @} */
#endif
