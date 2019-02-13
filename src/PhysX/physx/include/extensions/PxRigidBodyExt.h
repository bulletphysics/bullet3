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


#ifndef PX_PHYSICS_EXTENSIONS_RIGIDBODY_H
#define PX_PHYSICS_EXTENSIONS_RIGIDBODY_H
/** \addtogroup extensions
  @{
*/

#include "PxPhysXConfig.h"
#include "PxRigidBody.h"
#include "PxQueryReport.h"
#include "PxQueryFiltering.h"
#include "extensions/PxMassProperties.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxScene;
struct PxQueryCache;
class PxShape;

/**
\brief utility functions for use with PxRigidBody and subclasses

@see PxRigidBody PxRigidDynamic PxArticulationLink
*/

class PxRigidBodyExt
{
public:
	/**
	\brief Computation of mass properties for a rigid body actor

	To simulate a dynamic rigid actor, the SDK needs a mass and an inertia tensor. 

	This method offers functionality to compute the necessary mass and inertia properties based on the shapes declared in
	the PxRigidBody descriptor and some additionally specified parameters. For each shape, the shape geometry, 
	the shape positioning within the actor and the specified shape density are used to compute the body's mass and 
	inertia properties.

	<ul>
	<li>Shapes without PxShapeFlag::eSIMULATION_SHAPE set are ignored unless includeNonSimShapes is true.</li>
	<li>Shapes with plane, triangle mesh or heightfield geometry and PxShapeFlag::eSIMULATION_SHAPE set are not allowed for PxRigidBody collision.</li>
	</ul>

	This method will set the mass, center of mass, and inertia tensor 

	if no collision shapes are found, the inertia tensor is set to (1,1,1) and the mass to 1

	if massLocalPose is non-NULL, the rigid body's center of mass parameter  will be set 
	to the user provided value (massLocalPose) and the inertia tensor will be resolved at that point.

	\note If all shapes of the actor have the same density then the overloaded method updateMassAndInertia() with a single density parameter can be used instead.

	\param[in,out] body The rigid body.
	\param[in] shapeDensities The per shape densities. There must be one entry for each shape which has the PxShapeFlag::eSIMULATION_SHAPE set (or for all shapes if includeNonSimShapes is set to true). Other shapes are ignored. The density values must be greater than 0.
	\param[in] shapeDensityCount The number of provided density values.
	\param[in] massLocalPose The center of mass relative to the actor frame.  If set to null then (0,0,0) is assumed.
	\param[in] includeNonSimShapes True if all kind of shapes (PxShapeFlag::eSCENE_QUERY_SHAPE, PxShapeFlag::eTRIGGER_SHAPE) should be taken into account.
	\return Boolean. True on success else false.

	@see PxRigidBody::setMassLocalPose PxRigidBody::setMassSpaceInertiaTensor PxRigidBody::setMass
	*/
	static		bool			updateMassAndInertia(PxRigidBody& body, const PxReal* shapeDensities, PxU32 shapeDensityCount, const PxVec3* massLocalPose = NULL, bool includeNonSimShapes = false);


	/**
	\brief Computation of mass properties for a rigid body actor

	See previous method for details.

	\param[in,out] body The rigid body.
	\param[in] density The density of the body. Used to compute the mass of the body. The density must be greater than 0. 
	\param[in] massLocalPose The center of mass relative to the actor frame.  If set to null then (0,0,0) is assumed.
	\param[in] includeNonSimShapes True if all kind of shapes (PxShapeFlag::eSCENE_QUERY_SHAPE, PxShapeFlag::eTRIGGER_SHAPE) should be taken into account.
	\return Boolean. True on success else false.

	@see PxRigidBody::setMassLocalPose PxRigidBody::setMassSpaceInertiaTensor PxRigidBody::setMass
	*/
	static		bool			updateMassAndInertia(PxRigidBody& body, PxReal density, const PxVec3* massLocalPose = NULL, bool includeNonSimShapes = false);
	

	/**
	\brief Computation of mass properties for a rigid body actor

	This method sets the mass, inertia and center of mass of a rigid body. The mass is set to the sum of all user-supplied
	shape mass values, and the inertia and center of mass are computed according to the rigid body's shapes and the per shape mass input values.

	If no collision shapes are found, the inertia tensor is set to (1,1,1)

	\note If a single mass value should be used for the actor as a whole then the overloaded method setMassAndUpdateInertia() with a single mass parameter can be used instead.

	@see updateMassAndInertia for more details.

	\param[in,out] body The rigid body for which to set the mass and centre of mass local pose properties.
	\param[in] shapeMasses The per shape mass values. There must be one entry for each shape which has the PxShapeFlag::eSIMULATION_SHAPE set. Other shapes are ignored. The mass values must be greater than 0.
	\param[in] shapeMassCount The number of provided mass values.
	\param[in] massLocalPose The center of mass relative to the actor frame. If set to null then (0,0,0) is assumed.
	\param[in] includeNonSimShapes True if all kind of shapes (PxShapeFlag::eSCENE_QUERY_SHAPE, PxShapeFlag::eTRIGGER_SHAPE) should be taken into account.
	\return Boolean. True on success else false.

	@see PxRigidBody::setCMassLocalPose PxRigidBody::setMassSpaceInertiaTensor PxRigidBody::setMass
	*/
	static		bool			setMassAndUpdateInertia(PxRigidBody& body, const PxReal* shapeMasses, PxU32 shapeMassCount, const PxVec3* massLocalPose = NULL, bool includeNonSimShapes = false);


	/**
	\brief Computation of mass properties for a rigid body actor

	This method sets the mass, inertia and center of mass of a rigid body. The mass is set to the user-supplied
	value, and the inertia and center of mass are computed according to the rigid body's shapes and the input mass.

	If no collision shapes are found, the inertia tensor is set to (1,1,1)

	@see updateMassAndInertia for more details.

	\param[in,out] body The rigid body for which to set the mass and centre of mass local pose properties.
	\param[in] mass The mass of the body. Must be greater than 0.
	\param[in] massLocalPose The center of mass relative to the actor frame. If set to null then (0,0,0) is assumed.
	\param[in] includeNonSimShapes True if all kind of shapes (PxShapeFlag::eSCENE_QUERY_SHAPE, PxShapeFlag::eTRIGGER_SHAPE) should be taken into account.
	\return Boolean. True on success else false.

	@see PxRigidBody::setCMassLocalPose PxRigidBody::setMassSpaceInertiaTensor PxRigidBody::setMass
	*/
	static		bool			setMassAndUpdateInertia(PxRigidBody& body, PxReal mass, const PxVec3* massLocalPose = NULL, bool includeNonSimShapes = false);


	/**
	\brief Compute the mass, inertia tensor and center of mass from a list of shapes.

	\param[in] shapes The shapes to compute the mass properties from.
	\param[in] shapeCount The number of provided shapes.
	\return The mass properties from the combined shapes.

	@see PxRigidBody::setCMassLocalPose PxRigidBody::setMassSpaceInertiaTensor PxRigidBody::setMass
	*/
	static		PxMassProperties	computeMassPropertiesFromShapes(const PxShape* const* shapes, PxU32 shapeCount);
	

	/**
	\brief Applies a force (or impulse) defined in the global coordinate frame, acting at a particular 
	point in global coordinates, to the actor. 

	Note that if the force does not act along the center of mass of the actor, this
	will also add the corresponding torque. Because forces are reset at the end of every timestep, 
	you can maintain a total external force on an object by calling this once every frame.

	\note if this call is used to apply a force or impulse to an articulation link, only the link is updated, not the entire
	articulation

	::PxForceMode determines if the force is to be conventional or impulsive. Only eFORCE and eIMPULSE are supported, as the 
	force required to produce a given velocity change or acceleration is underdetermined given only the desired change at a
	given point.

	<b>Sleeping:</b> This call wakes the actor if it is sleeping and the wakeup parameter is true (default).

	\param[in] body The rigid body to apply the force to.
	\param[in] force Force/impulse to add, defined in the global frame. <b>Range:</b> force vector
	\param[in] pos Position in the global frame to add the force at. <b>Range:</b> position vector
	\param[in] mode The mode to use when applying the force/impulse(see #PxForceMode). 
	\param[in] wakeup Specify if the call should wake up the actor.

	@see PxForceMode 
	@see addForceAtLocalPos() addLocalForceAtPos() addLocalForceAtLocalPos()
	*/
	static		void			addForceAtPos(PxRigidBody& body, const PxVec3& force, const PxVec3& pos, PxForceMode::Enum mode = PxForceMode::eFORCE, bool wakeup = true);

	/**
	\brief Applies a force (or impulse) defined in the global coordinate frame, acting at a particular 
	point in local coordinates, to the actor. 

	Note that if the force does not act along the center of mass of the actor, this
	will also add the corresponding torque. Because forces are reset at the end of every timestep, you can maintain a
	total external force on an object by calling this once every frame.

	\note if this call is used to apply a force or impulse to an articulation link, only the link is updated, not the entire
	articulation

	::PxForceMode determines if the force is to be conventional or impulsive. Only eFORCE and eIMPULSE are supported, as the 
	force required to produce a given velocity change or acceleration is underdetermined given only the desired change at a
	given point.

	<b>Sleeping:</b> This call wakes the actor if it is sleeping and the wakeup parameter is true (default).

	\param[in] body The rigid body to apply the force to.
	\param[in] force Force/impulse to add, defined in the global frame. <b>Range:</b> force vector
	\param[in] pos Position in the local frame to add the force at. <b>Range:</b> position vector
	\param[in] mode The mode to use when applying the force/impulse(see #PxForceMode). 
	\param[in] wakeup Specify if the call should wake up the actor.

	@see PxForceMode 
	@see addForceAtPos() addLocalForceAtPos() addLocalForceAtLocalPos()
	*/
	static		void			addForceAtLocalPos(PxRigidBody& body, const PxVec3& force, const PxVec3& pos, PxForceMode::Enum mode = PxForceMode::eFORCE, bool wakeup = true);

	/**
	\brief Applies a force (or impulse) defined in the actor local coordinate frame, acting at a 
	particular point in global coordinates, to the actor. 

	Note that if the force does not act along the center of mass of the actor, this
	will also add the corresponding torque. Because forces are reset at the end of every timestep, you can maintain a
	total external force on an object by calling this once every frame.

	\note if this call is used to apply a force or impulse to an articulation link, only the link is updated, not the entire
	articulation

	::PxForceMode determines if the force is to be conventional or impulsive. Only eFORCE and eIMPULSE are supported, as the 
	force required to produce a given velocity change or acceleration is underdetermined given only the desired change at a
	given point.

	<b>Sleeping:</b> This call wakes the actor if it is sleeping and the wakeup parameter is true (default).

	\param[in] body The rigid body to apply the force to.
	\param[in] force Force/impulse to add, defined in the local frame. <b>Range:</b> force vector
	\param[in] pos Position in the global frame to add the force at. <b>Range:</b> position vector
	\param[in] mode The mode to use when applying the force/impulse(see #PxForceMode). 
	\param[in] wakeup Specify if the call should wake up the actor.

	@see PxForceMode 
	@see addForceAtPos() addForceAtLocalPos() addLocalForceAtLocalPos()
	*/
	static		void			addLocalForceAtPos(PxRigidBody& body, const PxVec3& force, const PxVec3& pos, PxForceMode::Enum mode = PxForceMode::eFORCE, bool wakeup = true);

	/**
	\brief Applies a force (or impulse) defined in the actor local coordinate frame, acting at a 
	particular point in local coordinates, to the actor. 

	Note that if the force does not act along the center of mass of the actor, this
	will also add the corresponding torque. Because forces are reset at the end of every timestep, you can maintain a
	total external force on an object by calling this once every frame.

	\note if this call is used to apply a force or impulse to an articulation link, only the link is updated, not the entire
	articulation

	::PxForceMode determines if the force is to be conventional or impulsive. Only eFORCE and eIMPULSE are supported, as the 
	force required to produce a given velocity change or acceleration is underdetermined given only the desired change at a
	given point.

	<b>Sleeping:</b> This call wakes the actor if it is sleeping and the wakeup parameter is true (default).

	\param[in] body The rigid body to apply the force to.
	\param[in] force Force/impulse to add, defined in the local frame. <b>Range:</b> force vector
	\param[in] pos Position in the local frame to add the force at. <b>Range:</b> position vector
	\param[in] mode The mode to use when applying the force/impulse(see #PxForceMode). 
	\param[in] wakeup Specify if the call should wake up the actor.

	@see PxForceMode 
	@see addForceAtPos() addForceAtLocalPos() addLocalForceAtPos()
	*/
	static		void			addLocalForceAtLocalPos(PxRigidBody& body, const PxVec3& force, const PxVec3& pos, PxForceMode::Enum mode = PxForceMode::eFORCE, bool wakeup = true);

	/**
	\brief Computes the velocity of a point given in world coordinates if it were attached to the 
	specified body and moving with it.

	\param[in] body The rigid body the point is attached to.
	\param[in] pos Position we wish to determine the velocity for, defined in the global frame. <b>Range:</b> position vector
	\return The velocity of point in the global frame.

	@see getLocalPointVelocity()
	*/
	static		PxVec3			getVelocityAtPos(const PxRigidBody& body, const PxVec3& pos);

	/**
	\brief Computes the velocity of a point given in local coordinates if it were attached to the 
	specified body and moving with it.

	\param[in] body The rigid body the point is attached to.
	\param[in] pos Position we wish to determine the velocity for, defined in the local frame. <b>Range:</b> position vector
	\return The velocity of point in the local frame.

	@see getLocalPointVelocity()
	*/
	static		PxVec3			getLocalVelocityAtLocalPos(const PxRigidBody& body, const PxVec3& pos);

	/**
	\brief Computes the velocity of a point (offset from the origin of the body) given in world coordinates if it were attached to the 
	specified body and moving with it.

	\param[in] body The rigid body the point is attached to.
	\param[in] pos Position (offset from the origin of the body) we wish to determine the velocity for, defined in the global frame. <b>Range:</b> position vector
	\return The velocity of point (offset from the origin of the body) in the global frame.

	@see getLocalPointVelocity()
	*/
	static		PxVec3			getVelocityAtOffset(const PxRigidBody& body, const PxVec3& pos);


	/**
	\brief Performs a linear sweep through space with the body's geometry objects.

	\note Supported geometries are: box, sphere, capsule, convex. Other geometry types will be ignored.
	\note If eTOUCH is returned from the filter callback, it will trigger an error and the hit will be discarded.

	The function sweeps all shapes attached to a given rigid body through space and reports the nearest
	object in the scene which intersects any of of the shapes swept paths.
	Information about the closest intersection is written to a #PxSweepHit structure.

	\param[in] body The rigid body to sweep.
	\param[in] scene The scene object to process the query.
	\param[in] unitDir Normalized direction of the sweep.
	\param[in] distance Sweep distance. Needs to be larger than 0.
	\param[in] outputFlags Specifies which properties should be written to the hit information.
	\param[out] closestHit Closest hit result.
	\param[out] shapeIndex Index of the body shape that caused the closest hit.
	\param[in] filterData If any word in filterData.data is non-zero then filterData.data will be used for filtering,
							otherwise shape->getQueryFilterData() will be used instead.
	\param[in] filterCall Custom filtering logic (optional). Only used if the corresponding #PxQueryFlag flags are set. If NULL, all hits are assumed to be blocking.
	\param[in] cache		Cached hit shape (optional). Ray is tested against cached shape first then against the scene.
							Note: Filtering is not executed for a cached shape if supplied; instead, if a hit is found, it is assumed to be a blocking hit.
	\param[in] inflation	This parameter creates a skin around the swept geometry which increases its extents for sweeping. The sweep will register a hit as soon as the skin touches a shape, and will return the corresponding distance and normal.

	\return True if a blocking hit was found.

	@see PxScene PxQueryFlags PxFilterData PxBatchQueryPreFilterShader PxBatchQueryPostFilterShader PxSweepHit
	*/
	static		bool			linearSweepSingle(
									PxRigidBody& body, PxScene& scene, const PxVec3& unitDir, const PxReal distance,
									PxHitFlags outputFlags,
									PxSweepHit& closestHit, PxU32& shapeIndex,
									const PxQueryFilterData& filterData = PxQueryFilterData(),
									PxQueryFilterCallback* filterCall = NULL,
									const PxQueryCache* cache = NULL,
									const PxReal inflation=0.0f);

	/**
	\brief Performs a linear sweep through space with the body's geometry objects, returning all overlaps.

	\note Supported geometries are: box, sphere, capsule, convex. Other geometry types will be ignored.

	This function sweeps all shapes attached to a given rigid body through space and reports all
	objects in the scene that intersect any of the shapes' swept paths until there are no more objects to report
	or a blocking hit is encountered.

	\param[in] body The rigid body to sweep.
	\param[in] scene The scene object to process the query.
	\param[in] unitDir Normalized direction of the sweep.
	\param[in] distance Sweep distance. Needs to be larger than 0.
	\param[in] outputFlags		Specifies which properties should be written to the hit information.
	\param[out] touchHitBuffer	Raycast hit information buffer. If the buffer overflows, an arbitrary subset of touch hits
								is returned (typically the query should be restarted with a larger buffer).
	\param[out] touchHitShapeIndices After the query is completed, touchHitShapeIndices[i] will contain the body index that caused the hit stored in hitBuffer[i]
	\param[in] touchHitBufferSize	Size of both touch hit buffers in elements.
	\param[out] block	Closest blocking hit is returned via this reference.
	\param[out] blockingShapeIndex	Set to -1 if if a blocking hit was not found, otherwise set to closest blocking hit shape index. The touching hits are reported separately in hitBuffer.
	\param[out] overflow	Set to true if touchHitBuffer didn't have enough space for all results. Touch hits will be incomplete if overflow occurred. Possible solution is to restart the query with a larger buffer.
	\param[in] filterData	If any word in filterData.data is non-zero then filterData.data will be used for filtering,
							otherwise shape->getQueryFilterData() will be used instead.
	\param[in] filterCall	Custom filtering logic (optional). Only used if the corresponding #PxQueryFlag flags are set. If NULL, all hits are assumed to be blocking.
	\param[in] cache		Cached hit shape (optional). Ray is tested against cached shape first then against the scene.
							Note: Filtering is not executed for a cached shape if supplied; instead, if a hit is found, it is assumed to be a blocking hit.
	\param[in] inflation	This parameter creates a skin around the swept geometry which increases its extents for sweeping. The sweep will register a hit as soon as the skin touches a shape, and will return the corresponding distance and normal.

	\return the number of touching hits. If overflow is set to true, the results are incomplete. In case of overflow there are also no guarantees that all touching hits returned are closer than the blocking hit.

	@see PxScene PxQueryFlags PxFilterData PxBatchQueryPreFilterShader PxBatchQueryPostFilterShader PxSweepHit
	*/
	static		PxU32			linearSweepMultiple(
									PxRigidBody& body, PxScene& scene, const PxVec3& unitDir, const PxReal distance,
									PxHitFlags outputFlags,
									PxSweepHit* touchHitBuffer, PxU32* touchHitShapeIndices, PxU32 touchHitBufferSize,
									PxSweepHit& block, PxI32& blockingShapeIndex, bool& overflow,
									const PxQueryFilterData& filterData = PxQueryFilterData(),
									PxQueryFilterCallback* filterCall = NULL,
									const PxQueryCache* cache = NULL, const PxReal inflation = 0.0f);


	/**
	\brief Compute the change to linear and angular velocity that would occur if an impulsive force and torque were to be applied to a specified rigid body. 
	
	The rigid body is left unaffected unless a subsequent independent call is executed that actually applies the computed changes to velocity and angular velocity.

	\note if this call is used to determine the velocity delta for an articulation link, only the mass properties of the link are taken into account.

	@see PxRigidBody::getLinearVelocity, PxRigidBody::setLinearVelocity,  PxRigidBody::getAngularVelocity, PxRigidBody::setAngularVelocity 

	\param[in] body The body under consideration.
	\param[in] impulsiveForce The impulsive force that would be applied to the specified rigid body.
	\param[in] impulsiveTorque The impulsive torque that would be applied to the specified rigid body.
	\param[out] deltaLinearVelocity The change in linear velocity that would arise if impulsiveForce was to be applied to the specified rigid body.
	\param[out] deltaAngularVelocity The change in angular velocity that would arise if impulsiveTorque was to be applied to the specified rigid body.
	*/
	static		void			computeVelocityDeltaFromImpulse(const PxRigidBody& body, const PxVec3& impulsiveForce, const PxVec3& impulsiveTorque, PxVec3& deltaLinearVelocity, PxVec3& deltaAngularVelocity);

	/**
	\brief Computes the linear and angular velocity change vectors for a given impulse at a world space position taking a mass and inertia scale into account

	This function is useful for extracting the respective linear and angular velocity changes from a contact or joint when the mass/inertia ratios have been adjusted.

	\note if this call is used to determine the velocity delta for an articulation link, only the mass properties of the link are taken into account.

	\param[in] body The rigid body
	\param[in] globalPose The body's world space transform
	\param[in] point The point in world space where the impulse is applied
	\param[in] impulse The impulse vector in world space
	\param[in] invMassScale The inverse mass scale
	\param[in] invInertiaScale The inverse inertia scale
	\param[out] deltaLinearVelocity The linear velocity change
	\param[out] deltaAngularVelocity The angular velocity change
	*/

	static void					computeVelocityDeltaFromImpulse(const PxRigidBody& body, const PxTransform& globalPose, const PxVec3& point, const PxVec3& impulse, const PxReal invMassScale, 
														const PxReal invInertiaScale, PxVec3& deltaLinearVelocity, PxVec3& deltaAngularVelocity);

	/**
	\brief Computes the linear and angular impulse vectors for a given impulse at a world space position taking a mass and inertia scale into account

	This function is useful for extracting the respective linear and angular impulses from a contact or joint when the mass/inertia ratios have been adjusted.

	\param[in] body The rigid body
	\param[in] globalPose The body's world space transform
	\param[in] point The point in world space where the impulse is applied
	\param[in] impulse The impulse vector in world space
	\param[in] invMassScale The inverse mass scale
	\param[in] invInertiaScale The inverse inertia scale
	\param[out] linearImpulse The linear impulse
	\param[out] angularImpulse The angular impulse
	*/
	static void					computeLinearAngularImpulse(const PxRigidBody& body, const PxTransform& globalPose, const PxVec3& point, const PxVec3& impulse, const PxReal invMassScale, 
														const PxReal invInertiaScale, PxVec3& linearImpulse, PxVec3& angularImpulse);


};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
