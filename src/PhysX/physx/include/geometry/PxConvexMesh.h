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


#ifndef PX_PHYSICS_GEOMUTILS_NX_CONVEXMESH
#define PX_PHYSICS_GEOMUTILS_NX_CONVEXMESH
/** \addtogroup geomutils
  @{
*/

#include "foundation/Px.h"
#include "common/PxBase.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/**
\brief Polygon data

Plane format: (mPlane[0],mPlane[1],mPlane[2]).dot(x) + mPlane[3] = 0
With the normal outward-facing from the hull.
*/
struct PxHullPolygon
{
	PxReal			mPlane[4];		//!< Plane equation for this polygon
	PxU16			mNbVerts;		//!< Number of vertices/edges in the polygon
	PxU16			mIndexBase;		//!< Offset in index buffer
};

/**
\brief A convex mesh.

Internally represented as a list of convex polygons. The number
of polygons is limited to 256.

To avoid duplicating data when you have several instances of a particular
mesh positioned differently, you do not use this class to represent a
convex object directly. Instead, you create an instance of this mesh via
the PxConvexMeshGeometry and PxShape classes.

<h3>Creation</h3>

To create an instance of this class call PxPhysics::createConvexMesh(),
and PxConvexMesh::release() to delete it. This is only possible
once you have released all of its #PxShape instances.

<h3>Visualizations:</h3>
\li #PxVisualizationParameter::eCOLLISION_AABBS
\li #PxVisualizationParameter::eCOLLISION_SHAPES
\li #PxVisualizationParameter::eCOLLISION_AXES
\li #PxVisualizationParameter::eCOLLISION_FNORMALS
\li #PxVisualizationParameter::eCOLLISION_EDGES

@see PxConvexMeshDesc PxPhysics.createConvexMesh()
*/
class PxConvexMesh	: public PxBase
{
public:

	/**
	\brief Returns the number of vertices.
	\return	Number of vertices.
	@see getVertices()
	*/
	PX_PHYSX_COMMON_API virtual	PxU32				getNbVertices()									const	= 0;

	/**
	\brief Returns the vertices.
	\return	Array of vertices.
	@see getNbVertices()
	*/
	PX_PHYSX_COMMON_API virtual	const PxVec3*		getVertices()									const	= 0;

	/**
	\brief Returns the index buffer.
	\return	Index buffer.
	@see getNbPolygons() getPolygonData()
	*/
	PX_PHYSX_COMMON_API virtual	const PxU8*			getIndexBuffer()								const	= 0;

	/**
	\brief Returns the number of polygons.
	\return	Number of polygons.
	@see getIndexBuffer() getPolygonData()
	*/
	PX_PHYSX_COMMON_API virtual	PxU32				getNbPolygons()									const	= 0;

	/**
	\brief Returns the polygon data.
	\param[in] index	Polygon index in [0 ; getNbPolygons()[.
	\param[out] data	Polygon data.
	\return	True if success.
	@see getIndexBuffer() getNbPolygons()
	*/
	PX_PHYSX_COMMON_API virtual	bool				getPolygonData(PxU32 index, PxHullPolygon& data)	const	= 0;

	/**
	\brief Decrements the reference count of a convex mesh and releases it if the new reference count is zero.	
	
	@see PxPhysics.createConvexMesh() PxConvexMeshGeometry PxShape
	*/
	PX_PHYSX_COMMON_API virtual	void				release()													= 0;

	/**
	\brief Returns the reference count of a convex mesh.

	At creation, the reference count of the convex mesh is 1. Every shape referencing this convex mesh increments the
	count by 1.	When the reference count reaches 0, and only then, the convex mesh gets destroyed automatically.

	\return the current reference count.
	*/
	PX_PHYSX_COMMON_API virtual PxU32				getReferenceCount()									const	= 0;

	/**
	\brief Acquires a counted reference to a convex mesh.

	This method increases the reference count of the convex mesh by 1. Decrement the reference count by calling release()
	*/
	PX_PHYSX_COMMON_API virtual void				acquireReference()											= 0;

	/**
	\brief Returns the mass properties of the mesh assuming unit density.

	The following relationship holds between mass and volume:

		mass = volume * density

	The mass of a unit density mesh is equal to its volume, so this function returns the volume of the mesh.

	Similarly, to obtain the localInertia of an identically shaped object with a uniform density of d, simply multiply the
	localInertia of the unit density mesh by d.

	\param[out] mass The mass of the mesh assuming unit density.
	\param[out] localInertia The inertia tensor in mesh local space assuming unit density.
	\param[out] localCenterOfMass Position of center of mass (or centroid) in mesh local space.
	*/
	PX_PHYSX_COMMON_API virtual void				getMassInformation(PxReal& mass, PxMat33& localInertia, PxVec3& localCenterOfMass)		const	= 0;

	/**
	\brief Returns the local-space (vertex space) AABB from the convex mesh.

	\return	local-space bounds
	*/
	PX_PHYSX_COMMON_API virtual	PxBounds3			getLocalBounds()	const	= 0;

	PX_PHYSX_COMMON_API virtual	const char*			getConcreteTypeName() const	{ return "PxConvexMesh"; }

	/**
	\brief This method decides whether a convex mesh is gpu compatible. If the total number of vertices are more than 64 or any number of vertices in a polygon is more than 32, or
	convex hull data was not cooked with GPU data enabled during cooking or was loaded from a serialized collection, the convex hull is incompatible with GPU collision detection. Otherwise
	it is compatible.

	\return True if the convex hull is gpu compatible
	*/
	PX_PHYSX_COMMON_API virtual bool				isGpuCompatible() const = 0;

protected:
						PX_INLINE					PxConvexMesh(PxType concreteType, PxBaseFlags baseFlags) : PxBase(concreteType, baseFlags) {}
						PX_INLINE					PxConvexMesh(PxBaseFlags baseFlags) : PxBase(baseFlags) {}
	PX_PHYSX_COMMON_API virtual						~PxConvexMesh() {}
	PX_PHYSX_COMMON_API virtual	bool				isKindOf(const char* name) const { return !::strcmp("PxConvexMesh", name) || PxBase::isKindOf(name); }
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
