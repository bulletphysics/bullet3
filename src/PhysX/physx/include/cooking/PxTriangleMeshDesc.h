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


#ifndef PX_COLLISION_NXTRIANGLEMESHDESC
#define PX_COLLISION_NXTRIANGLEMESHDESC
/** \addtogroup cooking
@{
*/

#include "PxPhysXConfig.h"
#include "geometry/PxSimpleTriangleMesh.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/**
\brief Descriptor class for #PxTriangleMesh.

Note that this class is derived from PxSimpleTriangleMesh which contains the members that describe the basic mesh.
The mesh data is *copied* when an PxTriangleMesh object is created from this descriptor. After the call the
user may discard the triangle data.

@see PxTriangleMesh PxTriangleMeshGeometry PxShape
*/
class PxTriangleMeshDesc : public PxSimpleTriangleMesh
{
public:

	/**
	Optional pointer to first material index, or NULL. There are PxSimpleTriangleMesh::numTriangles indices in total.
	Caller may add materialIndexStride bytes to the pointer to access the next triangle.

	When a triangle mesh collides with another object, a material is required at the collision point.
	If materialIndices is NULL, then the material of the PxShape instance is used.
	Otherwise, if the point of contact is on a triangle with index i, then the material index is determined as: 
	PxMaterialTableIndex	index = *(PxMaterialTableIndex *)(((PxU8*)materialIndices) + materialIndexStride * i);

	If the contact point falls on a vertex or an edge, a triangle adjacent to the vertex or edge is selected, and its index
	used to look up a material. The selection is arbitrary but consistent over time. 

	<b>Default:</b> NULL

	@see materialIndexStride
	*/
	PxTypedStridedData<PxMaterialTableIndex> materialIndices;

	/**
	\brief Constructor sets to default.
	*/
	PX_INLINE PxTriangleMeshDesc();	

	/**
	\brief (re)sets the structure to the default.	
	*/
	PX_INLINE void setToDefault();

	/**
	\brief Returns true if the descriptor is valid.
	\return true if the current settings are valid
	*/
	PX_INLINE bool isValid() const;
};

PX_INLINE PxTriangleMeshDesc::PxTriangleMeshDesc()	//constructor sets to default
{
	PxSimpleTriangleMesh::setToDefault();	
}

PX_INLINE void PxTriangleMeshDesc::setToDefault()
{
	*this = PxTriangleMeshDesc();
}

PX_INLINE bool PxTriangleMeshDesc::isValid() const
{
	if(points.count < 3) 	//at least 1 trig's worth of points
		return false;
	if ((!triangles.data) && (points.count%3))		// Non-indexed mesh => we must ensure the geometry defines an implicit number of triangles // i.e. numVertices can't be divided by 3
		return false;
	//add more validity checks here
	if (materialIndices.data && materialIndices.stride < sizeof(PxMaterialTableIndex))
		return false;
	return PxSimpleTriangleMesh::isValid();
}

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
