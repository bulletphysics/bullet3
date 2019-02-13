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

#include "GuTriangleMesh.h"
#include "GuTriangleMeshBV4.h"

using namespace physx;

namespace physx
{

Gu::BV4TriangleMesh::BV4TriangleMesh(GuMeshFactory& factory, TriangleMeshData& d)
:	TriangleMesh(factory, d)
{
	PX_ASSERT(d.mType==PxMeshMidPhase::eBVH34);

	BV4TriangleData& bv4Data = static_cast<BV4TriangleData&>(d);
	mMeshInterface = bv4Data.mMeshInterface;
	mBV4Tree = bv4Data.mBV4Tree;
	mBV4Tree.mMeshInterface = &mMeshInterface;
}

Gu::TriangleMesh* Gu::BV4TriangleMesh::createObject(PxU8*& address, PxDeserializationContext& context)
{
	BV4TriangleMesh* obj = new (address) BV4TriangleMesh(PxBaseFlag::eIS_RELEASABLE);
	address += sizeof(BV4TriangleMesh);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}

void Gu::BV4TriangleMesh::exportExtraData(PxSerializationContext& stream)
{
	mBV4Tree.exportExtraData(stream);
	TriangleMesh::exportExtraData(stream);
}

void Gu::BV4TriangleMesh::importExtraData(PxDeserializationContext& context)
{
	mBV4Tree.importExtraData(context);
	TriangleMesh::importExtraData(context);

	if(has16BitIndices())
		mMeshInterface.setPointers(NULL, const_cast<IndTri16*>(reinterpret_cast<const IndTri16*>(getTrianglesFast())), getVerticesFast());
	else
		mMeshInterface.setPointers(const_cast<IndTri32*>(reinterpret_cast<const IndTri32*>(getTrianglesFast())), NULL, getVerticesFast());
	mBV4Tree.mMeshInterface = &mMeshInterface;
}

} // namespace physx
