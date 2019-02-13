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

#include "PsIntrinsics.h"
#include "GuMidphaseInterface.h"
#include "GuSerialize.h"
#include "GuMeshFactory.h"
#include "CmRenderOutput.h"
#include "PxVisualizationParameter.h"
#include "GuBox.h"
#include "PxMeshScale.h"
#include "CmUtils.h"

using namespace physx;

namespace physx
{

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static PxConcreteType::Enum gTable[] = {	PxConcreteType::eTRIANGLE_MESH_BVH33,
											PxConcreteType::eTRIANGLE_MESH_BVH34
										};

Gu::TriangleMesh::TriangleMesh(GuMeshFactory& factory, TriangleMeshData& d)
:	PxTriangleMesh(PxType(gTable[d.mType]), PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE)
,	mNbVertices				(d.mNbVertices)
,	mNbTriangles			(d.mNbTriangles)
,	mVertices				(d.mVertices)
,	mTriangles				(d.mTriangles)
,	mAABB					(d.mAABB)
,	mExtraTrigData			(d.mExtraTrigData)
,	mGeomEpsilon			(d.mGeomEpsilon)
,	mFlags					(d.mFlags)
,	mMaterialIndices		(d.mMaterialIndices)
,	mFaceRemap				(d.mFaceRemap)
,	mAdjacencies			(d.mAdjacencies)

,	mMeshFactory			(&factory)

,	mGRB_triIndices					(d.mGRB_primIndices)

,	mGRB_triAdjacencies				(d.mGRB_primAdjacencies)

,	mGRB_faceRemap					(d.mGRB_faceRemap)
,	mGRB_BV32Tree					(d.mGRB_BV32Tree)
{
	// this constructor takes ownership of memory from the data object
	d.mVertices = 0;
	d.mTriangles = 0;
	d.mExtraTrigData = 0;
	d.mFaceRemap = 0;
	d.mAdjacencies = 0;
	d.mMaterialIndices = 0;

	d.mGRB_primIndices = 0;

	d.mGRB_primAdjacencies = 0;
	d.mGRB_faceRemap = 0;
	d.mGRB_BV32Tree = 0;

	// PT: 'getPaddedBounds()' is only safe if we make sure the bounds member is followed by at least 32bits of data
	PX_COMPILE_TIME_ASSERT(PX_OFFSET_OF(Gu::TriangleMesh, mExtraTrigData)>=PX_OFFSET_OF(Gu::TriangleMesh, mAABB)+4);
	
}

Gu::TriangleMesh::~TriangleMesh() 
{ 	
	if(getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
	{
		PX_FREE_AND_RESET(mExtraTrigData);
		PX_FREE_AND_RESET(mFaceRemap);
		PX_FREE_AND_RESET(mAdjacencies);
		PX_FREE_AND_RESET(mMaterialIndices);
		PX_FREE_AND_RESET(mTriangles);
		PX_FREE_AND_RESET(mVertices);

		PX_FREE_AND_RESET(mGRB_triIndices); 

		PX_FREE_AND_RESET(mGRB_triAdjacencies);
		PX_FREE_AND_RESET(mGRB_faceRemap);

		BV32Tree* bv32Tree = reinterpret_cast<BV32Tree*>(mGRB_BV32Tree);
		PX_DELETE_AND_RESET(bv32Tree);

	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// PT: used to be automatic but making it manual saves bytes in the internal mesh

void Gu::TriangleMesh::exportExtraData(PxSerializationContext& stream)
{
	//PX_DEFINE_DYNAMIC_ARRAY(TriangleMesh, mVertices, PxField::eVEC3, mNbVertices, Ps::PxFieldFlag::eSERIALIZE),
	if(mVertices)
	{
		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(mVertices, mNbVertices * sizeof(PxVec3));
	}

	if(mTriangles)
	{
		const PxU32 triangleSize = mFlags & PxTriangleMeshFlag::e16_BIT_INDICES ? sizeof(PxU16) : sizeof(PxU32);
		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(mTriangles, mNbTriangles * 3 * triangleSize);
	}

	//PX_DEFINE_DYNAMIC_ARRAY(TriangleMesh, mExtraTrigData, PxField::eBYTE, mNbTriangles, Ps::PxFieldFlag::eSERIALIZE),
	if(mExtraTrigData)
	{
		// PT: it might not be needed to 16-byte align this array of PxU8....
		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(mExtraTrigData, mNbTriangles * sizeof(PxU8));
	}

	if(mMaterialIndices)
	{
		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(mMaterialIndices, mNbTriangles * sizeof(PxU16));
	}

	if(mFaceRemap)
	{
		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(mFaceRemap, mNbTriangles * sizeof(PxU32));
	}

	if(mAdjacencies)
	{
		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(mAdjacencies, mNbTriangles * sizeof(PxU32) * 3);
	}
}

void Gu::TriangleMesh::importExtraData(PxDeserializationContext& context)
{
	// PT: vertices are followed by indices, so it will be safe to V4Load vertices from a deserialized binary file
	if(mVertices)
		mVertices = context.readExtraData<PxVec3, PX_SERIAL_ALIGN>(mNbVertices);

	if(mTriangles)
	{
		if(mFlags & PxTriangleMeshFlag::e16_BIT_INDICES)
			mTriangles = context.readExtraData<PxU16, PX_SERIAL_ALIGN>(3*mNbTriangles);
		else
			mTriangles = context.readExtraData<PxU32, PX_SERIAL_ALIGN>(3*mNbTriangles);
	}

	if(mExtraTrigData)
		mExtraTrigData = context.readExtraData<PxU8, PX_SERIAL_ALIGN>(mNbTriangles);

	if(mMaterialIndices)
		mMaterialIndices = context.readExtraData<PxU16, PX_SERIAL_ALIGN>(mNbTriangles);

	if(mFaceRemap)
		mFaceRemap = context.readExtraData<PxU32, PX_SERIAL_ALIGN>(mNbTriangles);

	if(mAdjacencies)
		mAdjacencies = context.readExtraData<PxU32, PX_SERIAL_ALIGN>(3*mNbTriangles);

	mGRB_triIndices = NULL;
	mGRB_triAdjacencies = NULL;
	mGRB_faceRemap = NULL;
	mGRB_BV32Tree = NULL;
}

void Gu::TriangleMesh::onRefCountZero()
{
	if(mMeshFactory->removeTriangleMesh(*this))
	{
		const PxType type = getConcreteType();
		GuMeshFactory* mf = mMeshFactory;
		Cm::deletePxBase(this);
		mf->notifyFactoryListener(this, type);
		return;
	}

	// PT: if we reach this point, we didn't find the mesh in the Physics object => don't delete!
	// This prevents deleting the object twice.
	Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "Gu::TriangleMesh::release: double deletion detected!");
}
//~PX_SERIALIZATION

void Gu::TriangleMesh::release()
{
	decRefCount();
}

#if PX_ENABLE_DYNAMIC_MESH_RTREE
PxVec3* Gu::TriangleMesh::getVerticesForModification()
{
	Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxTriangleMesh::getVerticesForModification() is only supported for meshes with PxMeshMidPhase::eBVH33.");

	return NULL;
}

PxBounds3 Gu::TriangleMesh::refitBVH()
{
	Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxTriangleMesh::refitBVH() is only supported for meshes with PxMeshMidPhase::eBVH33.");

	return PxBounds3(mAABB.getMin(), mAABB.getMax());
}
#endif

} // namespace physx
