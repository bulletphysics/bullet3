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

#ifndef PX_PSCOOKING_H
#define PX_PSCOOKING_H

#include "foundation/PxMemory.h"
#include "PxCooking.h"
#include "PsUserAllocated.h"

namespace physx
{
class TriangleMeshBuilder;
class ConvexMeshBuilder;
class ConvexHullLib;

class Cooking: public PxCooking, public Ps::UserAllocated
{
public:
									Cooking(const PxCookingParams& params): mParams(params) {}

	virtual void					release();
	virtual void					setParams(const PxCookingParams& params);
	virtual const PxCookingParams&	getParams() const;
	virtual bool					platformMismatch() const;
	virtual bool					cookTriangleMesh(const PxTriangleMeshDesc& desc, PxOutputStream& stream, PxTriangleMeshCookingResult::Enum* condition = NULL) const;
	virtual PxTriangleMesh*			createTriangleMesh(const PxTriangleMeshDesc& desc, PxPhysicsInsertionCallback& insertionCallback, PxTriangleMeshCookingResult::Enum* condition = NULL) const;
	virtual bool					validateTriangleMesh(const PxTriangleMeshDesc& desc) const;

	virtual bool					cookConvexMesh(const PxConvexMeshDesc& desc, PxOutputStream& stream, PxConvexMeshCookingResult::Enum* condition) const;
	virtual PxConvexMesh*			createConvexMesh(const PxConvexMeshDesc& desc, PxPhysicsInsertionCallback& insertionCallback, PxConvexMeshCookingResult::Enum* condition) const;
	virtual bool					validateConvexMesh(const PxConvexMeshDesc& desc) const;
	virtual bool					computeHullPolygons(const PxSimpleTriangleMesh& mesh, PxAllocatorCallback& inCallback,PxU32& nbVerts, PxVec3*& vertices,
											PxU32& nbIndices, PxU32*& indices, PxU32& nbPolygons, PxHullPolygon*& hullPolygons) const;
	virtual bool					cookHeightField(const PxHeightFieldDesc& desc, PxOutputStream& stream) const;
	virtual PxHeightField*			createHeightField(const PxHeightFieldDesc& desc, PxPhysicsInsertionCallback& insertionCallback) const;
	virtual bool					cookBVHStructure(const PxBVHStructureDesc& desc, PxOutputStream& stream) const;
	virtual PxBVHStructure*			createBVHStructure(const PxBVHStructureDesc& desc, PxPhysicsInsertionCallback& insertionCallback) const;

	PX_FORCE_INLINE static void		gatherStrided(const void* src, void* dst, PxU32 nbElem, PxU32 elemSize, PxU32 stride)
	{
		const PxU8* s = reinterpret_cast<const PxU8*>(src);
		PxU8* d = reinterpret_cast<PxU8*>(dst);
		while(nbElem--)
		{
			PxMemCopy(d, s, elemSize);
			d += elemSize;
			s += stride;
		}
	}

private:
	bool							cookConvexMeshInternal(const PxConvexMeshDesc& desc, ConvexMeshBuilder& meshBuilder, ConvexHullLib* hullLib, PxConvexMeshCookingResult::Enum* condition) const;
	bool							cookTriangleMesh(TriangleMeshBuilder& builder, const PxTriangleMeshDesc& desc, PxOutputStream& stream, PxTriangleMeshCookingResult::Enum* condition) const;

	PxTriangleMesh*					createTriangleMesh(TriangleMeshBuilder& builder, const PxTriangleMeshDesc& desc, PxPhysicsInsertionCallback& insertionCallback, PxTriangleMeshCookingResult::Enum* condition) const;

private:
	PxCookingParams					mParams;

};

}
#endif //#define PX_PSCOOKING_H
