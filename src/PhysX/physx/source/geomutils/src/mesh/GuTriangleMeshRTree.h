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

#ifndef GU_TRIANGLEMESH_RTREE_H
#define GU_TRIANGLEMESH_RTREE_H

#include "GuTriangleMesh.h"

namespace physx
{
class GuMeshFactory;

namespace Gu
{

#if PX_VC
#pragma warning(push)
#pragma warning(disable: 4324)	// Padding was added at the end of a structure because of a __declspec(align) value.
#endif

class RTreeTriangleMesh : public TriangleMesh
{
	public:
						virtual const char*				getConcreteTypeName()	const	{ return "PxBVH33TriangleMesh"; }
// PX_SERIALIZATION
														RTreeTriangleMesh(PxBaseFlags baseFlags) : TriangleMesh(baseFlags), mRTree(PxEmpty) {}
	PX_PHYSX_COMMON_API	virtual void					exportExtraData(PxSerializationContext& ctx);
								void					importExtraData(PxDeserializationContext&);
	PX_PHYSX_COMMON_API	static	TriangleMesh*			createObject(PxU8*& address, PxDeserializationContext& context);
	PX_PHYSX_COMMON_API	static	void					getBinaryMetaData(PxOutputStream& stream);
//~PX_SERIALIZATION
														RTreeTriangleMesh(GuMeshFactory& factory, TriangleMeshData& data);
						virtual							~RTreeTriangleMesh(){}

						virtual	PxMeshMidPhase::Enum	getMidphaseID()			const	{ return PxMeshMidPhase::eBVH33; }

#if PX_ENABLE_DYNAMIC_MESH_RTREE
						virtual PxVec3*					getVerticesForModification();
						virtual PxBounds3				refitBVH();
#endif

	PX_FORCE_INLINE				const Gu::RTree&		getRTree()				const	{ return mRTree; }
	private:
								Gu::RTree				mRTree;								
};

#if PX_VC
#pragma warning(pop)
#endif

} // namespace Gu

}

#endif
