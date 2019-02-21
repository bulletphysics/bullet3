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


#ifndef PX_COLLISION_NXHEIGHTFIELDDESC
#define PX_COLLISION_NXHEIGHTFIELDDESC
/** \addtogroup geomutils
@{
*/

#include "common/PxPhysXCommonConfig.h"
#include "geometry/PxHeightFieldFlag.h"
#include "common/PxCoreUtilityTypes.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/**
\brief Descriptor class for #PxHeightField.

\note The heightfield data is *copied* when a PxHeightField object is created from this descriptor. After the call the
user may discard the height data.

@see PxHeightField PxHeightFieldGeometry PxShape PxPhysics.createHeightField() PxCooking.createHeightField()
*/
class PxHeightFieldDesc
{
public:

	/**
	\brief Number of sample rows in the height field samples array.

	\note Local space X-axis corresponds to rows.

	<b>Range:</b> &gt;1<br>
	<b>Default:</b> 0
	*/
	PxU32							nbRows;

	/**
	\brief Number of sample columns in the height field samples array.

	\note Local space Z-axis corresponds to columns.

	<b>Range:</b> &gt;1<br>
	<b>Default:</b> 0
	*/
	PxU32							nbColumns;

	/**
	\brief Format of the sample data.

	Currently the only supported format is PxHeightFieldFormat::eS16_TM:

	<b>Default:</b> PxHeightFieldFormat::eS16_TM

	@see PxHeightFormat PxHeightFieldDesc.samples
	*/
	PxHeightFieldFormat::Enum		format;

	/**
	\brief The samples array.

	It is copied to the SDK's storage at creation time.

	There are nbRows * nbColumn samples in the array,
	which define nbRows * nbColumn vertices and cells,
	of which (nbRows - 1) * (nbColumns - 1) cells are actually used.

	The array index of sample(row, column) = row * nbColumns + column.
	The byte offset of sample(row, column) = sampleStride * (row * nbColumns + column).
	The sample data follows at the offset and spans the number of bytes defined by the format.
	Then there are zero or more unused bytes depending on sampleStride before the next sample.

	<b>Default:</b> NULL

	@see PxHeightFormat
	*/
	PxStridedData					samples;

	/**
	This threshold is used by the collision detection to determine if a height field edge is convex
	and can generate contact points.
	Usually the convexity of an edge is determined from the angle (or cosine of the angle) between
	the normals of the faces sharing that edge.
	The height field allows a more efficient approach by comparing height values of neighboring vertices.
	This parameter offsets the comparison. Smaller changes than 0.5 will not alter the set of convex edges.
	The rule of thumb is that larger values will result in fewer edge contacts.

	This parameter is ignored in contact generation with sphere and capsule primitives.

	<b>Range:</b> [0, PX_MAX_F32)<br>
	<b>Default:</b> 0
	*/
	PxReal					convexEdgeThreshold;

	/**
	\brief Flags bits, combined from values of the enum ::PxHeightFieldFlag.

	<b>Default:</b> 0

	@see PxHeightFieldFlag PxHeightFieldFlags
	*/
	PxHeightFieldFlags		flags;

	/**
	\brief Constructor sets to default.
	*/
	PX_INLINE				PxHeightFieldDesc();

	/**
	\brief (re)sets the structure to the default.
	*/
	PX_INLINE		void	setToDefault();

	/**
	\brief Returns true if the descriptor is valid.
	\return True if the current settings are valid.
	*/
	PX_INLINE		bool	isValid() const;
};

PX_INLINE PxHeightFieldDesc::PxHeightFieldDesc()	//constructor sets to default
{
	nbColumns					= 0;
	nbRows						= 0;
	format						= PxHeightFieldFormat::eS16_TM;
	convexEdgeThreshold			= 0.0f;
	flags						= PxHeightFieldFlags();
}

PX_INLINE void PxHeightFieldDesc::setToDefault()
{
	*this = PxHeightFieldDesc();
}

PX_INLINE bool PxHeightFieldDesc::isValid() const
{
	if (nbColumns < 2)
		return false;
	if (nbRows < 2)
		return false;
	if(format != PxHeightFieldFormat::eS16_TM)
		return false;
	if (samples.stride < 4)
		return false;
	if (convexEdgeThreshold < 0)
		return false;
	if ((flags & PxHeightFieldFlag::eNO_BOUNDARY_EDGES) != flags)
		return false;
	return true;
}

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
