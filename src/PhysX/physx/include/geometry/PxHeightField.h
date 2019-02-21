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


#ifndef PX_PHYSICS_GEOMUTILS_NX_HEIGHTFIELD
#define PX_PHYSICS_GEOMUTILS_NX_HEIGHTFIELD
/** \addtogroup geomutils
  @{
*/

#include "geometry/PxHeightFieldFlag.h"
#include "geometry/PxHeightFieldSample.h"
#include "common/PxBase.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxHeightFieldDesc;

/**
\brief A height field class.

Height fields work in a similar way as triangle meshes specified to act as
height fields, with some important differences:

Triangle meshes can be made of nonuniform geometry, while height fields are
regular, rectangular grids.  This means that with PxHeightField, you sacrifice
flexibility in return for improved performance and decreased memory consumption.

In local space rows extend in X direction, columns in Z direction and height in Y direction.

Like Convexes and TriangleMeshes, HeightFields are referenced by shape instances
(see #PxHeightFieldGeometry, #PxShape).

To avoid duplicating data when you have several instances of a particular
height field differently, you do not use this class to represent a
height field object directly. Instead, you create an instance of this height field
via the PxHeightFieldGeometry and PxShape classes.

<h3>Creation</h3>

To create an instance of this class call PxPhysics::createHeightField() or
PxCooking::createHeightField(const PxHeightFieldDesc&, PxPhysicsInsertionCallback&).
To delete it call release(). This is only possible
once you have released all of its PxHeightFiedShape instances.

<h3>Visualizations:</h3>
\li #PxVisualizationParameter::eCOLLISION_AABBS
\li #PxVisualizationParameter::eCOLLISION_SHAPES
\li #PxVisualizationParameter::eCOLLISION_AXES
\li #PxVisualizationParameter::eCOLLISION_FNORMALS
\li #PxVisualizationParameter::eCOLLISION_EDGES

@see PxHeightFieldDesc PxHeightFieldGeometry PxShape PxPhysics.createHeightField() PxCooking.createHeightField()
*/

class PxHeightField	: public PxBase
{
	public:
	/**
	\brief Decrements the reference count of a height field and releases it if the new reference count is zero.

	@see PxPhysics.createHeightField() PxHeightFieldDesc PxHeightFieldGeometry PxShape
	*/
	PX_PHYSX_COMMON_API virtual		void						release() = 0;

	/**
    \brief Writes out the sample data array.

	The user provides destBufferSize bytes storage at destBuffer.
	The data is formatted and arranged as PxHeightFieldDesc.samples.

	\param[out] destBuffer The destination buffer for the sample data.
	\param[in] destBufferSize The size of the destination buffer.
	\return The number of bytes written.

	@see PxHeightFieldDesc.samples
	*/
    PX_PHYSX_COMMON_API virtual		PxU32						saveCells(void* destBuffer, PxU32 destBufferSize) const = 0;

	/**
    \brief Replaces a rectangular subfield in the sample data array.

	The user provides the description of a rectangular subfield in subfieldDesc.
	The data is formatted and arranged as PxHeightFieldDesc.samples.

	\param[in] startCol First cell in the destination heightfield to be modified. Can be negative.
	\param[in] startRow First row in the destination heightfield to be modified. Can be negative.
	\param[in] subfieldDesc Description of the source subfield to read the samples from.
	\param[in] shrinkBounds If left as false, the bounds will never shrink but only grow. If set to true the bounds will be recomputed from all HF samples at O(nbColums*nbRows) perf cost.
	\return True on success, false on failure. Failure can occur due to format mismatch.

	\note Modified samples are constrained to the same height quantization range as the original heightfield.
	Source samples that are out of range of target heightfield will be clipped with no error.
	PhysX does not keep a mapping from the heightfield to heightfield shapes that reference it.
	Call PxShape::setGeometry on each shape which references the height field, to ensure that internal data structures are updated to reflect the new geometry.
	Please note that PxShape::setGeometry does not guarantee correct/continuous behavior when objects are resting on top of old or new geometry.

	@see PxHeightFieldDesc.samples PxShape.setGeometry
	*/
	PX_PHYSX_COMMON_API virtual		bool						modifySamples(PxI32 startCol, PxI32 startRow, const PxHeightFieldDesc& subfieldDesc, bool shrinkBounds = false) = 0;

	/**
	\brief Retrieves the number of sample rows in the samples array.

	\return The number of sample rows in the samples array.

	@see PxHeightFieldDesc.nbRows
	*/
	PX_PHYSX_COMMON_API virtual		PxU32						getNbRows()					const = 0;

	/**
	\brief Retrieves the number of sample columns in the samples array.

	\return The number of sample columns in the samples array.

	@see PxHeightFieldDesc.nbColumns
	*/
	PX_PHYSX_COMMON_API virtual		PxU32						getNbColumns()				const = 0;

	/**
	\brief Retrieves the format of the sample data.

	\return The format of the sample data.

	@see PxHeightFieldDesc.format PxHeightFieldFormat
	*/
	PX_PHYSX_COMMON_API virtual		PxHeightFieldFormat::Enum	getFormat()					const = 0;

	/**
	\brief Retrieves the offset in bytes between consecutive samples in the array.

	\return The offset in bytes between consecutive samples in the array.

	@see PxHeightFieldDesc.sampleStride
	*/
	PX_PHYSX_COMMON_API virtual		PxU32						getSampleStride()			const = 0;

	/**
	\brief Retrieves the convex edge threshold.

	\return The convex edge threshold.

	@see PxHeightFieldDesc.convexEdgeThreshold
	*/
	PX_PHYSX_COMMON_API virtual		PxReal						getConvexEdgeThreshold()	const = 0;

	/**
	\brief Retrieves the flags bits, combined from values of the enum ::PxHeightFieldFlag.

	\return The flags bits, combined from values of the enum ::PxHeightFieldFlag.

	@see PxHeightFieldDesc.flags PxHeightFieldFlag
	*/
	PX_PHYSX_COMMON_API virtual		PxHeightFieldFlags			getFlags()					const = 0;

	/**
	\brief Retrieves the height at the given coordinates in grid space.

	\return The height at the given coordinates or 0 if the coordinates are out of range.
	*/
	PX_PHYSX_COMMON_API virtual		PxReal						getHeight(PxReal x, PxReal z) const = 0;

	/**
	\brief Returns the reference count for shared heightfields.

	At creation, the reference count of the heightfield is 1. Every shape referencing this heightfield increments the
	count by 1.	When the reference count reaches 0, and only then, the heightfield gets destroyed automatically.

	\return the current reference count.
	*/
	PX_PHYSX_COMMON_API virtual		PxU32						getReferenceCount()			const	= 0;

	/**
	\brief Acquires a counted reference to a heightfield.

	This method increases the reference count of the heightfield by 1. Decrement the reference count by calling release()
	*/
	PX_PHYSX_COMMON_API virtual void							acquireReference()					= 0;

	/**
	\brief Returns material table index of given triangle

	\note This function takes a post cooking triangle index.

	\param[in] triangleIndex (internal) index of desired triangle
	\return Material table index, or 0xffff if no per-triangle materials are used
	*/
	PX_PHYSX_COMMON_API virtual	PxMaterialTableIndex	getTriangleMaterialIndex(PxTriangleID triangleIndex) const = 0;

	/**
	\brief Returns a triangle face normal for a given triangle index

	\note This function takes a post cooking triangle index.

	\param[in] triangleIndex (internal) index of desired triangle
	\return Triangle normal for a given triangle index
	*/
	PX_PHYSX_COMMON_API virtual	PxVec3					getTriangleNormal(PxTriangleID triangleIndex) const = 0;

	/**
	\brief Returns heightfield sample of given row and column	

	\param[in] row Given heightfield row
	\param[in] column Given heightfield column
	\return Heightfield sample
	*/
	PX_PHYSX_COMMON_API virtual	const PxHeightFieldSample&	getSample(PxU32 row, PxU32 column) const = 0;

	/**
	\brief Returns the number of times the heightfield data has been modified
	
	This method returns the number of times modifySamples has been called on this heightfield, so that code that has
	retained state that depends on the heightfield can efficiently determine whether it has been modified.
	
	\return the number of times the heightfield sample data has been modified.
	*/
	PX_PHYSX_COMMON_API virtual		PxU32						getTimestamp()			const	= 0;

	PX_PHYSX_COMMON_API virtual	const char*				getConcreteTypeName() const { return "PxHeightField"; }

protected:
						PX_INLINE						PxHeightField(PxType concreteType, PxBaseFlags baseFlags) : PxBase(concreteType, baseFlags) {}
						PX_INLINE						PxHeightField(PxBaseFlags baseFlags) : PxBase(baseFlags) {}
	PX_PHYSX_COMMON_API virtual							~PxHeightField() {}
	PX_PHYSX_COMMON_API virtual	bool					isKindOf(const char* name) const { return !::strcmp("PxHeightField", name) || PxBase::isKindOf(name); }
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
