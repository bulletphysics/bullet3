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


#ifndef PX_PHYSICS_NX_HEIGHTFIELD_GEOMETRY
#define PX_PHYSICS_NX_HEIGHTFIELD_GEOMETRY
/** \addtogroup geomutils
@{
*/
#include "geometry/PxTriangleMeshGeometry.h"
#include "common/PxCoreUtilityTypes.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

#define PX_MIN_HEIGHTFIELD_XZ_SCALE 1e-8f
#define PX_MIN_HEIGHTFIELD_Y_SCALE (0.0001f / PxReal(0xFFFF))

class PxHeightField;

/**
\brief Height field geometry class.

This class allows to create a scaled height field geometry instance.

There is a minimum allowed value for Y and XZ scaling - PX_MIN_HEIGHTFIELD_XZ_SCALE, heightfield creation will fail if XZ value is below this value.
*/
class PxHeightFieldGeometry : public PxGeometry 
{
public:
	PX_INLINE PxHeightFieldGeometry() :		
		PxGeometry		(PxGeometryType::eHEIGHTFIELD),
		heightField		(NULL),
		heightScale		(1.0f), 
		rowScale		(1.0f), 
		columnScale		(1.0f), 
		heightFieldFlags(0)
	{}

	PX_INLINE PxHeightFieldGeometry(PxHeightField* hf,
									PxMeshGeometryFlags flags, 
									PxReal heightScale_,
									PxReal rowScale_, 
									PxReal columnScale_) :
		PxGeometry			(PxGeometryType::eHEIGHTFIELD), 
		heightField			(hf) ,
		heightScale			(heightScale_), 
		rowScale			(rowScale_), 
		columnScale			(columnScale_), 
		heightFieldFlags	(flags)
		{
		}

	/**
	\brief Returns true if the geometry is valid.

	\return True if the current settings are valid

	\note A valid height field has a positive scale value in each direction (heightScale > 0, rowScale > 0, columnScale > 0).
	It is illegal to call PxRigidActor::createShape and PxPhysics::createShape with a height field that has zero extents in any direction.

	@see PxRigidActor::createShape, PxPhysics::createShape
	*/
	PX_INLINE bool isValid() const;

public:
	/**
	\brief The height field data.
	*/
	PxHeightField*			heightField;

	/**
	\brief The scaling factor for the height field in vertical direction (y direction in local space).
	*/
	PxReal					heightScale;

	/**
	\brief The scaling factor for the height field in the row direction (x direction in local space).
	*/
	PxReal					rowScale;

	/**
	\brief The scaling factor for the height field in the column direction (z direction in local space).
	*/
	PxReal					columnScale;

	/**
	\brief Flags to specify some collision properties for the height field.
	*/
	PxMeshGeometryFlags		heightFieldFlags;

	PxPadding<3>			paddingFromFlags;	//!< padding for mesh flags.
};


PX_INLINE bool PxHeightFieldGeometry::isValid() const
{
	if (mType != PxGeometryType::eHEIGHTFIELD)
		return false;
	if (!PxIsFinite(heightScale) || !PxIsFinite(rowScale) || !PxIsFinite(columnScale))
		return false;
	if (rowScale < PX_MIN_HEIGHTFIELD_XZ_SCALE || columnScale < PX_MIN_HEIGHTFIELD_XZ_SCALE || heightScale < PX_MIN_HEIGHTFIELD_Y_SCALE)
		return false;
	if (!heightField)
		return false;

	return true;
}

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
