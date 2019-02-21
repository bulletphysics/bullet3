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


#ifndef PX_PHYSICS_EXTENSIONS_SIMPLE_FACTORY_H
#define PX_PHYSICS_EXTENSIONS_SIMPLE_FACTORY_H
/** \addtogroup extensions
  @{
*/

#include "common/PxPhysXCommonConfig.h"
#include "foundation/PxTransform.h"
#include "foundation/PxPlane.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

	class PxPhysics;
	class PxMaterial;
	class PxRigidActor;
	class PxRigidDynamic;
	class PxRigidStatic;
	class PxGeometry;
	class PxShape;


/** \brief simple method to create a PxRigidDynamic actor with a single PxShape. 

	\param[in] sdk the PxPhysics object
	\param[in] transform the global pose of the new object
	\param[in] geometry the geometry of the new object's shape, which must be a sphere, capsule, box or convex
	\param[in] material the material for the new object's shape
	\param[in] density the density of the new object. Must be greater than zero.
	\param[in] shapeOffset an optional offset for the new shape, defaults to identity

	\return a new dynamic actor with the PxRigidBodyFlag, or NULL if it could 
	not be constructed

	@see PxRigidDynamic PxShapeFlag
*/

PxRigidDynamic*	PxCreateDynamic(PxPhysics& sdk,
								const PxTransform& transform,
								const PxGeometry& geometry,
								PxMaterial& material,
								PxReal density,
								const PxTransform& shapeOffset = PxTransform(PxIdentity));


/** \brief simple method to create a PxRigidDynamic actor with a single PxShape. 

	\param[in] sdk the PxPhysics object
	\param[in] transform the transform of the new object
	\param[in] shape the shape of the new object
	\param[in] density the density of the new object. Must be greater than zero.

	\return a new dynamic actor with the PxRigidBodyFlag, or NULL if it could 
	not be constructed

	@see PxRigidDynamic PxShapeFlag
*/

PxRigidDynamic*	PxCreateDynamic(PxPhysics& sdk,
								const PxTransform& transform,
								PxShape& shape,
								PxReal density);


/** \brief simple method to create a kinematic PxRigidDynamic actor with a single PxShape. 

	\param[in] sdk the PxPhysics object
	\param[in] transform the global pose of the new object
	\param[in] geometry the geometry of the new object's shape
	\param[in] material the material for the new object's shape
	\param[in] density the density of the new object. Must be greater than zero if the object is to participate in simulation.
	\param[in] shapeOffset an optional offset for the new shape, defaults to identity

	\note unlike PxCreateDynamic, the geometry is not restricted to box, capsule, sphere or convex. However, 
	kinematics of other geometry types may not participate in simulation collision and may be used only for
	triggers or scene queries of moving objects under animation control. In this case the density parameter
	will be ignored and the created shape will be set up as a scene query only shape (see #PxShapeFlag::eSCENE_QUERY_SHAPE)

	\return a new dynamic actor with the PxRigidBodyFlag::eKINEMATIC set, or NULL if it could 
	not be constructed

	@see PxRigidDynamic PxShapeFlag
*/

PxRigidDynamic*	PxCreateKinematic(PxPhysics& sdk,
								  const PxTransform& transform,
								  const PxGeometry& geometry,
								  PxMaterial& material,
								  PxReal density,
								  const PxTransform& shapeOffset = PxTransform(PxIdentity));


/** \brief simple method to create a kinematic PxRigidDynamic actor with a single PxShape. 

	\param[in] sdk the PxPhysics object
	\param[in] transform the global pose of the new object
	\param[in] density the density of the new object. Must be greater than zero if the object is to participate in simulation.
	\param[in] shape the shape of the new object

	\note unlike PxCreateDynamic, the geometry is not restricted to box, capsule, sphere or convex. However, 
	kinematics of other geometry types may not participate in simulation collision and may be used only for
	triggers or scene queries of moving objects under animation control. In this case the density parameter
	will be ignored and the created shape will be set up as a scene query only shape (see #PxShapeFlag::eSCENE_QUERY_SHAPE)

	\return a new dynamic actor with the PxRigidBodyFlag::eKINEMATIC set, or NULL if it could 
	not be constructed

	@see PxRigidDynamic PxShapeFlag
*/

PxRigidDynamic*	PxCreateKinematic(PxPhysics& sdk,
								  const PxTransform& transform,
								  PxShape& shape,
								  PxReal density);


/** \brief simple method to create a PxRigidStatic actor with a single PxShape. 

	\param[in] sdk the PxPhysics object
	\param[in] transform the global pose of the new object
	\param[in] geometry the geometry of the new object's shape
	\param[in] material the material for the new object's shape
	\param[in] shapeOffset an optional offset for the new shape, defaults to identity

	\return a new static actor, or NULL if it could not be constructed

	@see PxRigidStatic
*/

PxRigidStatic*	PxCreateStatic(PxPhysics& sdk,
							   const PxTransform& transform,
							   const PxGeometry& geometry,
							   PxMaterial& material,
							   const PxTransform& shapeOffset = PxTransform(PxIdentity));


/** \brief simple method to create a PxRigidStatic actor with a single PxShape. 

	\param[in] sdk the PxPhysics object
	\param[in] transform the global pose of the new object
	\param[in] shape the new object's shape

	\return a new static actor, or NULL if it could not be constructed

	@see PxRigidStatic
*/

PxRigidStatic*	PxCreateStatic(PxPhysics& sdk,
							   const PxTransform& transform,
							   PxShape& shape);


/** \brief simple method to create a PxRigidStatic actor with a single PxShape. 

	\param[in] sdk the PxPhysics object
	\param[in] transform the global pose of the new object
	\param[in] shape the new object's shape

	\return a new static actor, or NULL if it could not be constructed

	@see PxRigidStatic
*/

PxRigidStatic*	PxCreateStatic(PxPhysics& sdk,
							   const PxTransform& transform,
							   PxShape& shape);


/**
\brief create a shape by copying attributes from another shape

The function clones a PxShape. The following properties are copied:
- geometry
- flags
- materials
- actor-local pose
- contact offset
- rest offset
- simulation filter data
- query filter data

The following are not copied and retain their default values:
- name
- user data

\param[in] physicsSDK - the physics SDK used to allocate the shape
\param[in] shape the shape from which to take the attributes.
\param[in] isExclusive whether the new shape should be an exclusive or shared shape.

\return the newly-created rigid static

*/

PxShape* PxCloneShape(PxPhysics& physicsSDK,
					  const PxShape& shape,
					  bool isExclusive);



/**
\brief create a static body by copying attributes from another rigid actor

The function clones a PxRigidDynamic or PxRigidStatic as a PxRigidStatic. A uniform scale is applied. The following properties are copied:
- shapes
- actor flags 
- owner client and client behavior bits

The following are not copied and retain their default values:
- name
- joints or observers
- aggregate or scene membership
- user data

\note Transforms are not copied with bit-exact accuracy.

\param[in] physicsSDK - the physics SDK used to allocate the rigid static
\param[in] actor the rigid actor from which to take the attributes.
\param[in] transform the transform of the new static.

\return the newly-created rigid static

*/

PxRigidStatic* PxCloneStatic(PxPhysics& physicsSDK, 
							 const PxTransform& transform,
							 const PxRigidActor& actor);


/**
\brief create a dynamic body by copying attributes from an existing body

The following properties are copied:
- shapes
- actor flags and rigidDynamic flags
- mass, moment of inertia, and center of mass frame
- linear and angular velocity
- linear and angular damping
- maximum angular velocity
- position and velocity solver iterations
- maximum depenetration velocity
- sleep threshold
- contact report threshold
- dominance group
- owner client and client behavior bits
- name pointer

The following are not copied and retain their default values:
- name
- joints or observers
- aggregate or scene membership
- sleep timer
- user data

\note Transforms are not copied with bit-exact accuracy.

\param[in] physicsSDK PxPhysics - the physics SDK used to allocate the rigid static
\param[in] body the rigid dynamic to clone.
\param[in] transform the transform of the new dynamic

\return the newly-created rigid static

*/

PxRigidDynamic*	PxCloneDynamic(PxPhysics& physicsSDK, 	 
							   const PxTransform& transform,
							   const PxRigidDynamic& body);


/** \brief create a plane actor. The plane equation is n.x + d = 0

	\param[in] sdk the PxPhysics object
	\param[in] plane a plane of the form n.x + d = 0
	\param[in] material the material for the new object's shape

	\return a new static actor, or NULL if it could not be constructed

	@see PxRigidStatic
*/

PxRigidStatic*	PxCreatePlane(PxPhysics& sdk,
							  const PxPlane& plane,
							  PxMaterial& material);


/**
\brief scale a rigid actor by a uniform scale

The geometry and relative positions of the actor are multiplied by the given scale value. If the actor is a rigid body or an
articulation link and the scaleMassProps value is true, the mass properties are scaled assuming the density is constant: the 
center of mass is linearly scaled, the mass is multiplied by the cube of the scale, and the inertia tensor by the fifth power of the scale. 

\param[in] actor a rigid actor
\param[in] scale the scale by which to multiply the actor. Must be >0.
\param[in] scaleMassProps whether to scale the mass properties
*/

void PxScaleRigidActor(PxRigidActor& actor, PxReal scale, bool scaleMassProps = true);


#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
