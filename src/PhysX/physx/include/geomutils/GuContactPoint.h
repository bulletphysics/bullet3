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

#ifndef GU_CONTACT_POINT_H
#define GU_CONTACT_POINT_H

/** \addtogroup geomutils
@{
*/

#include "foundation/PxVec3.h"

namespace physx
{
namespace Gu
{

struct ContactPoint
{
	/**
	\brief The normal of the contacting surfaces at the contact point.

	For two shapes s0 and s1, the normal points in the direction that s0 needs to move in to resolve the contact with s1.
	*/
	PX_ALIGN(16, PxVec3	normal);
	/**
	\brief The separation of the shapes at the contact point.  A negative separation denotes a penetration.
	*/
	PxReal	separation;

	/**
	\brief The point of contact between the shapes, in world space. 
	*/
	PX_ALIGN(16, PxVec3	point);	

	/**
	\brief The max impulse permitted at this point
	*/
	PxReal maxImpulse;

	PX_ALIGN(16, PxVec3 targetVel);

	/**
	\brief The static friction coefficient
	*/
	PxReal staticFriction;

	/**
	\brief Material flags for this contact (eDISABLE_FRICTION, eDISABLE_STRONG_FRICTION). @see PxMaterialFlag
	*/
	PxU8 materialFlags;

	/**
	\brief internal structure used for internal use only
	*/
	PxU16 forInternalUse;

	/**
	\brief The surface index of shape 1 at the contact point.  This is used to identify the surface material.

	\note This field is only supported by triangle meshes and heightfields, else it will be set to PXC_CONTACT_NO_FACE_INDEX.
	\note This value must be directly after internalFaceIndex0 in memory
	*/

	PxU32   internalFaceIndex1;

	/**
	\brief The dynamic friction coefficient
	*/
	PxReal dynamicFriction;
	/**
	\brief The restitution coefficient
	*/
	PxReal restitution;

};

}

}

/** @} */
#endif
