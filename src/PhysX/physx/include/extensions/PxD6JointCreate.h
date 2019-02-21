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

#ifndef PX_D6JOINT_CREATE_H
#define PX_D6JOINT_CREATE_H

#include "common/PxPhysXCommonConfig.h"

/** \addtogroup extensions
  @{
*/

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxPhysics;
class PxRigidActor;
class PxJoint;

/**
	\brief Helper function to create a fixed joint, using either a PxD6Joint or PxFixedJoint.

	For fixed joints it is important that the joint frames have the same orientation. This helper function uses an identity rotation for both.
	It is also important that the joint frames have an equivalent position in world space. The function does not check this, so it is up to users
	to ensure that this is the case.

	\param[in] physics		The physics SDK
	\param[in] actor0		An actor to which the joint is attached. NULL may be used to attach the joint to a specific point in the world frame
	\param[in] localPos0	The position of the joint relative to actor0
	\param[in] actor1		An actor to which the joint is attached. NULL may be used to attach the joint to a specific point in the world frame
	\param[in] localPos1	The position of the joint relative to actor1 
	\param[in] useD6		True to use a PxD6Joint, false to use a PxFixedJoint;

	\return	The created joint.

	@see PxD6Joint PxFixedJoint
*/
PxJoint* PxD6JointCreate_Fixed(PxPhysics& physics, PxRigidActor* actor0, const PxVec3& localPos0, PxRigidActor* actor1, const PxVec3& localPos1, bool useD6);

/**
	\brief Helper function to create a distance joint, using either a PxD6Joint or PxDistanceJoint.

	This helper function only supports a maximum distance constraint, because PxD6Joint does not support a minimum distance constraint (contrary
	to PxDistanceJoint).

	The distance is computed between the joint frames' world-space positions. The joint frames' orientations are irrelevant here so the function
	sets them to identity.

	\param[in] physics		The physics SDK
	\param[in] actor0		An actor to which the joint is attached. NULL may be used to attach the joint to a specific point in the world frame
	\param[in] localPos0	The position of the joint relative to actor0
	\param[in] actor1		An actor to which the joint is attached. NULL may be used to attach the joint to a specific point in the world frame
	\param[in] localPos1	The position of the joint relative to actor1 
	\param[in] maxDist		The maximum allowed distance
	\param[in] useD6		True to use a PxD6Joint, false to use a PxDistanceJoint;

	\return	The created joint.

	@see PxD6Joint PxDistanceJoint
*/
PxJoint* PxD6JointCreate_Distance(PxPhysics& physics, PxRigidActor* actor0, const PxVec3& localPos0, PxRigidActor* actor1, const PxVec3& localPos1, float maxDist, bool useD6);

/**
	\brief Helper function to create a prismatic joint, using either a PxD6Joint or PxPrismaticJoint.

	This function enforces that the joint frames have the same orientation, which is a local frame whose X is the desired translation axis.
	This orientation is computed by the function, so users only have to define the desired translation axis (typically 1;0;0 or 0;1;0 or 0;0;1).

	The translation can be limited. Limits are enforced if minLimit<maxLimit. If minLimit=maxLimit the axis is locked. If minLimit>maxLimit the
	limits are not enforced and the axis is free. The limit values are computed relative to the position of actor0's joint frame.
	
	The function creates hard limits, and uses PhysX's default contact distance parameter.

	\param[in] physics		The physics SDK
	\param[in] actor0		An actor to which the joint is attached. NULL may be used to attach the joint to a specific point in the world frame
	\param[in] localPos0	The position of the joint relative to actor0
	\param[in] actor1		An actor to which the joint is attached. NULL may be used to attach the joint to a specific point in the world frame
	\param[in] localPos1	The position of the joint relative to actor1 
	\param[in] axis			The axis along which objects are allowed to move, expressed in the actors' local space
	\param[in] minLimit		The minimum allowed position along the axis
	\param[in] maxLimit		The maximum allowed position along the axis
	\param[in] useD6		True to use a PxD6Joint, false to use a PxPrismaticJoint;

	\return	The created joint.

	@see PxD6Joint PxPrismaticJoint
*/
PxJoint* PxD6JointCreate_Prismatic(PxPhysics& physics, PxRigidActor* actor0, const PxVec3& localPos0, PxRigidActor* actor1, const PxVec3& localPos1, const PxVec3& axis, float minLimit, float maxLimit, bool useD6);

/**
	\brief Helper function to create a revolute joint, using either a PxD6Joint or PxRevoluteJoint.

	This function enforces that the joint frames have the same orientation, which is a local frame whose X is the desired rotation axis.
	This orientation is computed by the function, so users only have to define the desired rotation axis (typically 1;0;0 or 0;1;0 or 0;0;1).

	The rotation can be limited. Limits are enforced if minLimit<maxLimit. If minLimit=maxLimit the axis is locked. If minLimit>maxLimit the
	limits are not enforced and the axis is free. The limit values are computed relative to the rotation of actor0's joint frame.
	
	The function creates hard limits, and uses PhysX's default contact distance parameter.

	Limits are expressed in radians. Allowed range is ]-2*PI;+2*PI[

	\param[in] physics		The physics SDK
	\param[in] actor0		An actor to which the joint is attached. NULL may be used to attach the joint to a specific point in the world frame
	\param[in] localPos0	The position of the joint relative to actor0
	\param[in] actor1		An actor to which the joint is attached. NULL may be used to attach the joint to a specific point in the world frame
	\param[in] localPos1	The position of the joint relative to actor1 
	\param[in] axis			The axis around which objects are allowed to move, expressed in the actors' local space
	\param[in] minLimit		The minimum allowed rotation along the axis
	\param[in] maxLimit		The maximum allowed rotation along the axis
	\param[in] useD6		True to use a PxD6Joint, false to use a PxRevoluteJoint;

	\return	The created joint.

	@see PxD6Joint PxRevoluteJoint
*/
PxJoint* PxD6JointCreate_Revolute(PxPhysics& physics, PxRigidActor* actor0, const PxVec3& localPos0, PxRigidActor* actor1, const PxVec3& localPos1, const PxVec3& axis, float minLimit, float maxLimit, bool useD6);

/**
	\brief Helper function to create a spherical joint, using either a PxD6Joint or PxSphericalJoint.

	This function supports a cone limit shape, defined by a cone axis and two angular limit values.

	This function enforces that the joint frames have the same orientation, which is a local frame whose X is the desired cone axis.
	This orientation is computed by the function, so users only have to define the desired cone axis (typically 1;0;0 or 0;1;0 or 0;0;1).

	The rotations can be limited. Limits are enforced if limit1>0 and limit2>0. Otherwise the motion is free. The limit values define an ellipse,
	which is the cross-section of the cone limit shape.
	
	The function creates hard limits, and uses PhysX's default contact distance parameter.

	Limits are expressed in radians. Allowed range is ]0;PI[. Limits are symmetric around the cone axis.

	The cone axis is equivalent to the twist axis for the D6 joint. The twist motion is not limited.

	\param[in] physics		The physics SDK
	\param[in] actor0		An actor to which the joint is attached. NULL may be used to attach the joint to a specific point in the world frame
	\param[in] localPos0	The position of the joint relative to actor0
	\param[in] actor1		An actor to which the joint is attached. NULL may be used to attach the joint to a specific point in the world frame
	\param[in] localPos1	The position of the joint relative to actor1 
	\param[in] axis			The cone axis, expressed in the actors' local space
	\param[in] limit1		Max angular limit for the ellipse along the joint frame's second axis (first axis = cone axis)
	\param[in] limit2		Max angular limit for the ellipse along the joint frame's third axis (first axis = cone axis)
	\param[in] useD6		True to use a PxD6Joint, false to use a PxSphericalJoint;

	\return	The created joint.

	@see PxD6Joint PxSphericalJoint
*/
PxJoint* PxD6JointCreate_Spherical(PxPhysics& physics, PxRigidActor* actor0, const PxVec3& localPos0, PxRigidActor* actor1, const PxVec3& localPos1, const PxVec3& axis, float limit1, float limit2, bool useD6);

/**
	\brief Helper function to create a spherical joint, using either a PxD6Joint or PxSphericalJoint.

	This function supports a cone limit shape, defined by two pairs of angular limit values. This can be used to create an asymmetric cone. If the
	angular limit values are symmetric (i.e. minLimit1=-maxLimit1 and minLimit2=-maxLimit2) then the cone axis is the X axis in actor0's space.
	If the limits are not symmetric, the function rotates the cone axis accordingly so that limits remain symmetric for PhysX. If this happens,
	the initial joint frames will be different for both actors. By default minLimit1/maxLimit1 are limits around the joint's Y axis, and
	minLimit2/maxLimit2 are limits around the joint's Z axis.

	The function creates hard limits, and uses PhysX's default contact distance parameter.

	Limits are expressed in radians. Allowed range is ]-PI;PI[.

	The cone axis is equivalent to the twist axis for the D6 joint. The twist motion is not limited.

	The returned apiroty and apirotz values can later be added to retrieved Y and Z swing angle values (from the joint), to remap
	angle values to the given input range.

	\param[out] apiroty		Amount of rotation around Y used to setup actor0's joint frame
	\param[out] apirotz		Amount of rotation around Z used to setup actor0's joint frame
	\param[in] physics		The physics SDK
	\param[in] actor0		An actor to which the joint is attached. NULL may be used to attach the joint to a specific point in the world frame
	\param[in] localPos0	The position of the joint relative to actor0
	\param[in] actor1		An actor to which the joint is attached. NULL may be used to attach the joint to a specific point in the world frame
	\param[in] localPos1	The position of the joint relative to actor1 
	\param[in] minLimit1	Min angular limit along the joint frame's second axis (first axis = cone axis)
	\param[in] maxLimit1	Max angular limit along the joint frame's second axis (first axis = cone axis)
	\param[in] minLimit2	Min angular limit along the joint frame's third axis (first axis = cone axis)
	\param[in] maxLimit2	Max angular limit along the joint frame's third axis (first axis = cone axis)
	\param[in] useD6		True to use a PxD6Joint, false to use a PxSphericalJoint;

	\return	The created joint.

	@see PxD6Joint PxSphericalJoint
*/
PxJoint* PxD6JointCreate_GenericCone(float& apiroty, float& apirotz, PxPhysics& physics, PxRigidActor* actor0, const PxVec3& localPos0, PxRigidActor* actor1, const PxVec3& localPos1, float minLimit1, float maxLimit1, float minLimit2, float maxLimit2, bool useD6);


/**
	\brief Helper function to create a D6 joint with pyramidal swing limits.

	This function supports a pyramid limit shape, defined by two pairs of angular limit values. This can be used to create an asymmetric pyramid. If the
	angular limit values are symmetric (i.e. minLimit1=-maxLimit1 and minLimit2=-maxLimit2) then the pyramid axis is the X axis in actor0's space.
	By default minLimit1/maxLimit1 are limits around the joint's Y axis, and minLimit2/maxLimit2 are limits around the joint's Z axis.

	The function creates hard limits, and uses PhysX's default contact distance parameter.

	Limits are expressed in radians. Allowed range is ]-PI;PI[.

	The pyramid axis is equivalent to the twist axis for the D6 joint. The twist motion is not limited.

	\param[in] physics		The physics SDK
	\param[in] actor0		An actor to which the joint is attached. NULL may be used to attach the joint to a specific point in the world frame
	\param[in] localPos0	The position of the joint relative to actor0
	\param[in] actor1		An actor to which the joint is attached. NULL may be used to attach the joint to a specific point in the world frame
	\param[in] localPos1	The position of the joint relative to actor1 
	\param[in] axis			The pyramid axis, expressed in the actors' local space
	\param[in] minLimit1	Min angular limit along the joint frame's second axis (first axis = pyramid axis)
	\param[in] maxLimit1	Max angular limit along the joint frame's second axis (first axis = pyramid axis)
	\param[in] minLimit2	Min angular limit along the joint frame's third axis (first axis = pyramid axis)
	\param[in] maxLimit2	Max angular limit along the joint frame's third axis (first axis = pyramid axis)

	\return	The created joint.

	@see PxD6Joint
*/
PxJoint* PxD6JointCreate_Pyramid(PxPhysics& physics, PxRigidActor* actor0, const PxVec3& localPos0, PxRigidActor* actor1, const PxVec3& localPos1, const PxVec3& axis,
										float minLimit1, float maxLimit1, float minLimit2, float maxLimit2);


#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
