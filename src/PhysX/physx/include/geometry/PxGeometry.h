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


#ifndef PX_PHYSICS_NX_GEOMETRY
#define PX_PHYSICS_NX_GEOMETRY
/** \addtogroup geomutils
@{
*/

#include "common/PxPhysXCommonConfig.h"
#include "foundation/PxFlags.h"
#include "foundation/PxMath.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/**
\brief A geometry type.

Used to distinguish the type of a ::PxGeometry object.
*/
struct PxGeometryType
{
	enum Enum
	{
		eSPHERE,
		ePLANE,
		eCAPSULE,
		eBOX,
		eCONVEXMESH,
		eTRIANGLEMESH,
		eHEIGHTFIELD,
		eGEOMETRY_COUNT,	//!< internal use only!
		eINVALID = -1		//!< internal use only!
	};
};

/**
\brief A geometry object.

A geometry object defines the characteristics of a spatial object, but without any information
about its placement in the world.

\note This is an abstract class.  You cannot create instances directly.  Create an instance of one of the derived classes instead.
*/
class PxGeometry 
{ 
public:
	/**
	\brief Returns the type of the geometry.
	\return The type of the object.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxGeometryType::Enum getType() const	{ return mType; }	

protected:
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxGeometry(PxGeometryType::Enum type) : mType(type) {}
	PxGeometryType::Enum mType; 
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
