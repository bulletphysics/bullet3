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

#ifndef PX_PHYSICS_NX_SHAPE
#define PX_PHYSICS_NX_SHAPE
/** \addtogroup physics
@{
*/

#include "PxPhysXConfig.h"
#include "common/PxBase.h"
#include "geometry/PxGeometry.h"
#include "geometry/PxGeometryHelpers.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxBoxGeometry;
class PxSphereGeometry;
class PxCapsuleGeometry;
class PxPlaneGeometry;
class PxConvexMeshGeometry;
class PxTriangleMeshGeometry;
class PxHeightFieldGeometry;
class PxRigidActor;
struct PxFilterData;
struct PxRaycastHit;
struct PxSweepHit;

/**
\brief Flags which affect the behavior of PxShapes.

@see PxShape PxShape.setFlag()
*/
struct PxShapeFlag
{
	enum Enum
	{
		/**
		\brief The shape will partake in collision in the physical simulation.

		\note It is illegal to raise the eSIMULATION_SHAPE and eTRIGGER_SHAPE flags.
		In the event that one of these flags is already raised the sdk will reject any 
		attempt to raise the other.  To raise the eSIMULATION_SHAPE first ensure that 
		eTRIGGER_SHAPE is already lowered.

		\note This flag has no effect if simulation is disabled for the corresponding actor (see #PxActorFlag::eDISABLE_SIMULATION).

		@see PxSimulationEventCallback.onContact() PxScene.setSimulationEventCallback() PxShape.setFlag(), PxShape.setFlags()
		*/
		eSIMULATION_SHAPE				= (1<<0),

		/**
		\brief The shape will partake in scene queries (ray casts, overlap tests, sweeps, ...).
		*/
		eSCENE_QUERY_SHAPE				= (1<<1),

		/**
		\brief The shape is a trigger which can send reports whenever other shapes enter/leave its volume.

		\note Triangle meshes and heightfields can not be triggers. Shape creation will fail in these cases.

		\note Shapes marked as triggers do not collide with other objects. If an object should act both
		as a trigger shape and a collision shape then create a rigid body with two shapes, one being a 
		trigger shape and the other a collision shape. 	It is illegal to raise the eTRIGGER_SHAPE and 
		eSIMULATION_SHAPE flags on a single PxShape instance.  In the event that one of these flags is already 
		raised the sdk will reject any attempt to raise the other.  To raise the eTRIGGER_SHAPE flag first 
		ensure that eSIMULATION_SHAPE flag is already lowered.

		\note Trigger shapes will no longer send notification events for interactions with other trigger shapes.

		\note Shapes marked as triggers are allowed to participate in scene queries, provided the eSCENE_QUERY_SHAPE flag is set. 

		\note This flag has no effect if simulation is disabled for the corresponding actor (see #PxActorFlag::eDISABLE_SIMULATION).

		@see PxSimulationEventCallback.onTrigger() PxScene.setSimulationEventCallback() PxShape.setFlag(), PxShape.setFlags()
		*/
		eTRIGGER_SHAPE					= (1<<2),

		/**
		\brief Enable debug renderer for this shape

		@see PxScene.getRenderBuffer() PxRenderBuffer PxVisualizationParameter
		*/
		eVISUALIZATION					= (1<<3)
	};
};

/**
\brief collection of set bits defined in PxShapeFlag.

@see PxShapeFlag
*/
typedef PxFlags<PxShapeFlag::Enum,PxU8> PxShapeFlags;
PX_FLAGS_OPERATORS(PxShapeFlag::Enum,PxU8)


/**
\brief Abstract class for collision shapes.

Shapes are shared, reference counted objects.

An instance can be created by calling the createShape() method of the PxRigidActor class, or
the createShape() method of the PxPhysics class.

<h3>Visualizations</h3>
\li PxVisualizationParameter::eCOLLISION_AABBS
\li PxVisualizationParameter::eCOLLISION_SHAPES
\li PxVisualizationParameter::eCOLLISION_AXES

@see PxPhysics.createShape() PxRigidActor.createShape() PxBoxGeometry PxSphereGeometry PxCapsuleGeometry PxPlaneGeometry PxConvexMeshGeometry
PxTriangleMeshGeometry PxHeightFieldGeometry
*/
class PxShape : public PxBase
{
public:

	/**
	\brief Decrements the reference count of a shape and releases it if the new reference count is zero.

	Note that in releases prior to PhysX 3.3 this method did not have reference counting semantics and was used to destroy a shape 
	created with PxActor::createShape(). In PhysX 3.3 and above, this usage is deprecated, instead, use PxRigidActor::detachShape() to detach
	a shape from an actor. If the shape to be detached was created with PxActor::createShape(), the actor holds the only counted reference,
	and so when the shape is detached it will also be destroyed. 

	@see PxRigidActor::createShape() PxPhysics::createShape() PxRigidActor::attachShape() PxRigidActor::detachShape()
	*/
	virtual		void					release() = 0;

	/**
	\brief Returns the reference count of the shape.

	At creation, the reference count of the shape is 1. Every actor referencing this shape increments the
	count by 1.	When the reference count reaches 0, and only then, the shape gets destroyed automatically.

	\return the current reference count.
	*/
	virtual		PxU32					getReferenceCount() const = 0;

	/**
	\brief Acquires a counted reference to a shape.

	This method increases the reference count of the shape by 1. Decrement the reference count by calling release()
	*/
	virtual		void					acquireReference() = 0;

	/**
	\brief Get the geometry type of the shape.

	\return Type of shape geometry.

	@see PxGeometryType
	*/
	virtual		PxGeometryType::Enum	getGeometryType() const = 0;

	/**
	\brief Adjust the geometry of the shape.

	\note The type of the passed in geometry must match the geometry type of the shape.
	\note It is not allowed to change the geometry type of a shape.
	\note This function does not guarantee correct/continuous behavior when objects are resting on top of old or new geometry.

	\param[in] geometry New geometry of the shape.

	@see PxGeometry PxGeometryType getGeometryType()
	*/
	virtual		void					setGeometry(const PxGeometry& geometry) = 0;


	/**
	\brief Retrieve the geometry from the shape in a PxGeometryHolder wrapper class.

	\return a PxGeometryHolder object containing the geometry;
	
	@see PxGeometry PxGeometryType getGeometryType() setGeometry()
	*/

	virtual		PxGeometryHolder		getGeometry() const = 0;


	/**
	\brief Fetch the geometry of the shape.

	\note If the type of geometry to extract does not match the geometry type of the shape
	then the method will return false and the passed in geometry descriptor is not modified.

	\param[in] geometry The descriptor to save the shape's geometry data to.
	\return True on success else false

	@see PxGeometry PxGeometryType getGeometryType()
	*/
	virtual		bool					getBoxGeometry(PxBoxGeometry& geometry) const = 0;

	/**
	\brief Fetch the geometry of the shape.

	\note If the type of geometry to extract does not match the geometry type of the shape
	then the method will return false and the passed in geometry descriptor is not modified.

	\param[in] geometry The descriptor to save the shape's geometry data to.
	\return True on success else false

	@see PxGeometry PxGeometryType getGeometryType()
	*/
	virtual		bool					getSphereGeometry(PxSphereGeometry& geometry) const = 0;

	/**
	\brief Fetch the geometry of the shape.

	\note If the type of geometry to extract does not match the geometry type of the shape
	then the method will return false and the passed in geometry descriptor is not modified.

	\param[in] geometry The descriptor to save the shape's geometry data to.
	\return True on success else false

	@see PxGeometry PxGeometryType getGeometryType()
	*/
	virtual		bool					getCapsuleGeometry(PxCapsuleGeometry& geometry) const = 0;

	/**
	\brief Fetch the geometry of the shape.

	\note If the type of geometry to extract does not match the geometry type of the shape
	then the method will return false and the passed in geometry descriptor is not modified.

	\param[in] geometry The descriptor to save the shape's geometry data to.
	\return True on success else false

	@see PxGeometry PxGeometryType getGeometryType()
	*/
	virtual		bool					getPlaneGeometry(PxPlaneGeometry& geometry) const = 0;

	/**
	\brief Fetch the geometry of the shape.

	\note If the type of geometry to extract does not match the geometry type of the shape
	then the method will return false and the passed in geometry descriptor is not modified.

	\param[in] geometry The descriptor to save the shape's geometry data to.
	\return True on success else false

	@see PxGeometry PxGeometryType getGeometryType()
	*/
	virtual		bool					getConvexMeshGeometry(PxConvexMeshGeometry& geometry) const = 0;

	/**
	\brief Fetch the geometry of the shape.

	\note If the type of geometry to extract does not match the geometry type of the shape
	then the method will return false and the passed in geometry descriptor is not modified.

	\param[in] geometry The descriptor to save the shape's geometry data to.
	\return True on success else false

	@see PxGeometry PxGeometryType getGeometryType()
	*/
	virtual		bool					getTriangleMeshGeometry(PxTriangleMeshGeometry& geometry) const = 0;


	/**
	\brief Fetch the geometry of the shape.

	\note If the type of geometry to extract does not match the geometry type of the shape
	then the method will return false and the passed in geometry descriptor is not modified.

	\param[in] geometry The descriptor to save the shape's geometry data to.
	\return True on success else false

	@see PxGeometry PxGeometryType getGeometryType()
	*/
	virtual		bool					getHeightFieldGeometry(PxHeightFieldGeometry& geometry) const = 0;

	/**
	\brief Retrieves the actor which this shape is associated with.

	\return The actor this shape is associated with, if it is an exclusive shape, else NULL

	@see PxRigidStatic, PxRigidDynamic, PxArticulationLink
	*/
	virtual		PxRigidActor*			getActor() const = 0;


/************************************************************************************************/

/** @name Pose Manipulation
*/
//@{

	/**
	\brief Sets the pose of the shape in actor space, i.e. relative to the actors to which they are attached.
	
	This transformation is identity by default.

	The local pose is an attribute of the shape, and so will apply to all actors to which the shape is attached.

	<b>Sleeping:</b> Does <b>NOT</b> wake the associated actor up automatically.

	<i>Note:</i> Does not automatically update the inertia properties of the owning actor (if applicable); use the
	PhysX extensions method #PxRigidBodyExt::updateMassAndInertia() to do this.

	<b>Default:</b> the identity transform

	\param[in] pose	The new transform from the actor frame to the shape frame. <b>Range:</b> rigid body transform

	@see getLocalPose() 
	*/
	virtual		void					setLocalPose(const PxTransform& pose)		= 0;

	/**
	\brief Retrieves the pose of the shape in actor space, i.e. relative to the actor they are owned by.

	This transformation is identity by default.

	\return Pose of shape relative to the actor's frame.

	@see setLocalPose() 
	*/
	virtual		PxTransform				getLocalPose()					const	= 0;

//@}
/************************************************************************************************/

/** @name Collision Filtering
*/
//@{

	/**
	\brief Sets the user definable collision filter data.
	
	<b>Sleeping:</b> Does wake up the actor if the filter data change causes a formerly suppressed
	collision pair to be enabled.

	<b>Default:</b> (0,0,0,0)

	@see getSimulationFilterData() 
	*/
	virtual		void					setSimulationFilterData(const PxFilterData& data)	= 0;

	/**
	\brief Retrieves the shape's collision filter data.

	@see setSimulationFilterData() 
	*/
	virtual		PxFilterData			getSimulationFilterData()					const	= 0;

	/**
	\brief Sets the user definable query filter data.

	<b>Default:</b> (0,0,0,0)

	@see getQueryFilterData() 
	*/
	virtual		void					setQueryFilterData(const PxFilterData& data)	= 0;

	/**
	\brief Retrieves the shape's Query filter data.

	@see setQueryFilterData() 
	*/
	virtual		PxFilterData			getQueryFilterData()					const	= 0;

//@}
/************************************************************************************************/

	/**
	\brief Assigns material(s) to the shape.
	
	<b>Sleeping:</b> Does <b>NOT</b> wake the associated actor up automatically.

	\param[in] materials List of material pointers to assign to the shape. See #PxMaterial
	\param[in] materialCount The number of materials provided.

	@see PxPhysics.createMaterial() getMaterials() 
	*/
	virtual		void					setMaterials(PxMaterial*const* materials, PxU16 materialCount)	= 0;

	/**
	\brief Returns the number of materials assigned to the shape.

	You can use #getMaterials() to retrieve the material pointers.

	\return Number of materials associated with this shape.

	@see PxMaterial getMaterials()
	*/
	virtual		PxU16					getNbMaterials()		const	= 0;

	/**
	\brief Retrieve all the material pointers associated with the shape.

	You can retrieve the number of material pointers by calling #getNbMaterials()

	Note: Removing materials with #PxMaterial::release() will invalidate the pointer of the released material.

	\param[out] userBuffer The buffer to store the material pointers.
	\param[in] bufferSize Size of provided user buffer.
	\param[in] startIndex Index of first material pointer to be retrieved
	\return Number of material pointers written to the buffer.

	@see PxMaterial getNbMaterials() PxMaterial::release()
	*/
	virtual		PxU32					getMaterials(PxMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const	= 0;
	
	/**
	\brief Retrieve material from given triangle index.

	The input index is the internal triangle index as used inside the SDK. This is the index
	returned to users by various SDK functions such as raycasts.
	
	This function is only useful for triangle meshes or heightfields, which have per-triangle
	materials. For other shapes the function returns the single material associated with the
	shape, regardless of the index.

	\param[in] faceIndex The internal triangle index whose material you want to retrieve.
	\return Material from input triangle

	\note If faceIndex value of 0xFFFFffff is passed as an input for mesh and heightfield shapes, this function will issue a warning and return NULL.
	\note Scene queries set the value of PxQueryHit::faceIndex to 0xFFFFffff whenever it is undefined or does not apply.

	@see PxMaterial getNbMaterials() PxMaterial::release()
	*/
	virtual		PxMaterial*				getMaterialFromInternalFaceIndex(PxU32 faceIndex) const = 0;

	/**
	\brief Sets the contact offset.

	Shapes whose distance is less than the sum of their contactOffset values will generate contacts. The contact offset must be positive and
	greater than the rest offset. Having a contactOffset greater than than the restOffset allows the collision detection system to
	predictively enforce the contact constraint even when the objects are slightly separated. This prevents jitter that would occur
	if the constraint were enforced only when shapes were within the rest distance.

	<b>Default:</b> 0.02f * PxTolerancesScale::length

	<b>Sleeping:</b> Does <b>NOT</b> wake the associated actor up automatically.

	\param[in] contactOffset <b>Range:</b> [maximum(0,restOffset), PX_MAX_F32)

	@see getContactOffset PxTolerancesScale setRestOffset
	*/
	virtual		void					setContactOffset(PxReal contactOffset)	= 0;

	/**
	\brief Retrieves the contact offset. 

	\return The contact offset of the shape.

	@see setContactOffset()
	*/
	virtual		PxReal					getContactOffset() const	= 0;

	/**
	\brief Sets the rest offset. 

	Two shapes will come to rest at a distance equal to the sum of their restOffset values. If the restOffset is 0, they should converge to touching 
	exactly.  Having a restOffset greater than zero is useful to have objects slide smoothly, so that they do not get hung up on irregularities of 
	each others' surfaces.

	<b>Default:</b> 0.0f

	<b>Sleeping:</b> Does <b>NOT</b> wake the associated actor up automatically.

	\param[in] restOffset	<b>Range:</b> (-PX_MAX_F32, contactOffset)

	@see getRestOffset setContactOffset
	*/
	virtual		void					setRestOffset(PxReal restOffset)	= 0;

	/**
	\brief Retrieves the rest offset. 

	\return The rest offset of the shape.

	@see setRestOffset()
	*/
	virtual		PxReal					getRestOffset() const	= 0;


	/**
	\brief Sets torsional patch radius.
	
	This defines the radius of the contact patch used to apply torsional friction. If the radius is 0, no torsional friction
	will be applied. If the radius is > 0, some torsional friction will be applied. This is proportional to the penetration depth
	so, if the shapes are separated or penetration is zero, no torsional friction will be applied. It is used to approximate 
	rotational friction introduced by the compression of contacting surfaces.

	\param[in] radius	<b>Range:</b> (0, PX_MAX_F32)

	*/
	virtual			void						setTorsionalPatchRadius(PxReal radius) = 0;

	/**
	\brief Gets torsional patch radius.

	This defines the radius of the contact patch used to apply torsional friction. If the radius is 0, no torsional friction
	will be applied. If the radius is > 0, some torsional friction will be applied. This is proportional to the penetration depth
	so, if the shapes are separated or penetration is zero, no torsional friction will be applied. It is used to approximate
	rotational friction introduced by the compression of contacting surfaces.

	\return The torsional patch radius of the shape.
	*/
	virtual			PxReal						getTorsionalPatchRadius() const = 0;

	/**
	\brief Sets minimum torsional patch radius.

	This defines the minimum radius of the contact patch used to apply torsional friction. If the radius is 0, the amount of torsional friction
	that will be applied will be entirely dependent on the value of torsionalPatchRadius. 
	
	If the radius is > 0, some torsional friction will be applied regardless of the value of torsionalPatchRadius or the amount of penetration.

	\param[in] radius	<b>Range:</b> (0, PX_MAX_F32)

	*/
	virtual			void						setMinTorsionalPatchRadius(PxReal radius) = 0;

	/**
	\brief Gets minimum torsional patch radius.

	This defines the minimum radius of the contact patch used to apply torsional friction. If the radius is 0, the amount of torsional friction
	that will be applied will be entirely dependent on the value of torsionalPatchRadius. 
	
	If the radius is > 0, some torsional friction will be applied regardless of the value of torsionalPatchRadius or the amount of penetration.

	\return The minimum torsional patch radius of the shape.
	*/
	virtual			PxReal						getMinTorsionalPatchRadius() const = 0;


/************************************************************************************************/

	/**
	\brief Sets shape flags

	<b>Sleeping:</b> Does <b>NOT</b> wake the associated actor up automatically.

	\param[in] flag The shape flag to enable/disable. See #PxShapeFlag.
	\param[in] value True to set the flag. False to clear the flag specified in flag.

	<b>Default:</b> PxShapeFlag::eVISUALIZATION | PxShapeFlag::eSIMULATION_SHAPE | PxShapeFlag::eSCENE_QUERY_SHAPE

	@see PxShapeFlag getFlags()
	*/
	virtual		void					setFlag(PxShapeFlag::Enum flag, bool value) = 0;

	/**
	\brief Sets shape flags

	@see PxShapeFlag getFlags()
	*/
	virtual		void					setFlags(PxShapeFlags inFlags) = 0;

	/**
	\brief Retrieves shape flags.

	\return The values of the shape flags.

	@see PxShapeFlag setFlag()
	*/
	virtual		PxShapeFlags			getFlags() const = 0;

	/**
	\brief Returns true if the shape is exclusive to an actor.
	
	@see PxPhysics::createShape()
	*/
	virtual		bool					isExclusive() const	= 0;

	/**
	\brief Sets a name string for the object that can be retrieved with #getName().
	
	This is for debugging and is not used by the SDK.
	The string is not copied by the SDK, only the pointer is stored.

	<b>Default:</b> NULL
	
	\param[in] name The name string to set the objects name to.

	@see getName()
	*/
	virtual		void					setName(const char* name)		= 0;


	/**
	\brief retrieves the name string set with setName().
	\return The name associated with the shape.

	@see setName()
	*/
	virtual		const char*				getName()			const	= 0;


	virtual		const char*				getConcreteTypeName() const	{ return "PxShape"; }

/************************************************************************************************/

				void*					userData;	//!< user can assign this to whatever, usually to create a 1:1 relationship with a user object.

protected:
	PX_INLINE							PxShape(PxBaseFlags baseFlags) : PxBase(baseFlags) {}
	PX_INLINE							PxShape(PxType concreteType, PxBaseFlags baseFlags) : PxBase(concreteType, baseFlags), userData(NULL) {}
	virtual								~PxShape() {}
	virtual		bool					isKindOf(const char* name) const { return !::strcmp("PxShape", name) || PxBase::isKindOf(name); }

};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
