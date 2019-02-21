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


#include "foundation/PxErrorCallback.h"
#include "PsFoundation.h"
#include "PsUtilities.h"
#include "PsFPU.h"
#include "CmPhysXCommon.h"
#include "PxPhysXConfig.h"
#include "PxSimpleTriangleMesh.h"
#include "PxTriangleMeshDesc.h"
#include "PxConvexMeshDesc.h"
#include "PxCooking.h"
#include "Cooking.h"
#include "mesh/TriangleMeshBuilder.h"
#include "GuConvexMesh.h"
#include "ConvexMeshBuilder.h"
#include "BVHStructureBuilder.h"
#include "QuickHullConvexHullLib.h"
#include "CmIO.h"
#include "PxHeightFieldDesc.h"
#include "GuHeightField.h"
#include "HeightFieldCooking.h"
#include "common/PxPhysicsInsertionCallback.h"
#include "CmUtils.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

using namespace physx;
using namespace Gu;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Cooking::setParams(const PxCookingParams& params)
{
	mParams = params;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const PxCookingParams& Cooking::getParams() const
{
	return mParams;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Cooking::platformMismatch() const
{
	// Get current endianness (the one for the platform where cooking is performed)
	const PxI8 currentEndian = Ps::littleEndian();

	const bool mismatch = currentEndian!=1;	// The files must be little endian - we don't have big endian platforms anymore.
	return mismatch;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Cooking::release()
{
	delete this;

	Ps::Foundation::decRefCount();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Cooking::validateTriangleMesh(const PxTriangleMeshDesc& desc) const
{
	// cooking code does lots of float bitwise reinterpretation that generates exceptions
	PX_FPU_GUARD;

	if(!desc.isValid())
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "Cooking::validateTriangleMesh: user-provided triangle mesh descriptor is invalid!");
		return false;
	}

	// PT: validation code doesn't look at midphase data, so ideally we wouldn't build the midphase structure at all here.
	BV4TriangleMeshBuilder builder(mParams);
	return builder.loadFromDesc(desc, NULL, true /*doValidate*/);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Cooking::cookTriangleMesh(TriangleMeshBuilder& builder, const PxTriangleMeshDesc& desc, PxOutputStream& stream, PxTriangleMeshCookingResult::Enum* condition) const
{
	// cooking code does lots of float bitwise reinterpretation that generates exceptions
	PX_FPU_GUARD;

	if (condition)
		*condition = PxTriangleMeshCookingResult::eSUCCESS;
	if(!builder.loadFromDesc(desc, condition, false))
	{
		return false;
	}

	builder.save(stream, platformMismatch(), mParams);
	return true;
}

bool Cooking::cookTriangleMesh(const PxTriangleMeshDesc& desc, PxOutputStream& stream, PxTriangleMeshCookingResult::Enum* condition) const
{
	if(mParams.midphaseDesc.getType() == PxMeshMidPhase::eBVH33)
	{
		RTreeTriangleMeshBuilder builder(mParams);
		return cookTriangleMesh(builder, desc, stream, condition);
	}
	else
	{
		BV4TriangleMeshBuilder builder(mParams);
		return cookTriangleMesh(builder, desc, stream, condition);
	}
}

PxTriangleMesh* Cooking::createTriangleMesh(TriangleMeshBuilder& builder, const PxTriangleMeshDesc& desc, PxPhysicsInsertionCallback& insertionCallback, PxTriangleMeshCookingResult::Enum* condition) const
{	
	// cooking code does lots of float bitwise reinterpretation that generates exceptions
	PX_FPU_GUARD;

	if (condition)
		*condition = PxTriangleMeshCookingResult::eSUCCESS;
	if(!builder.loadFromDesc(desc, condition, false))
	{
		return NULL;	
	}

	// check if the indices can be moved from 32bits to 16bits
	if(!(mParams.meshPreprocessParams & PxMeshPreprocessingFlag::eFORCE_32BIT_INDICES))
		builder.checkMeshIndicesSize();

	PxConcreteType::Enum type;
	if(builder.getMidphaseID()==PxMeshMidPhase::eBVH33)
		type = PxConcreteType::eTRIANGLE_MESH_BVH33;
	else
		type = PxConcreteType::eTRIANGLE_MESH_BVH34;

	return static_cast<PxTriangleMesh*>(insertionCallback.buildObjectFromData(type, &builder.getMeshData()));
}

PxTriangleMesh* Cooking::createTriangleMesh(const PxTriangleMeshDesc& desc, PxPhysicsInsertionCallback& insertionCallback, PxTriangleMeshCookingResult::Enum* condition) const
{
	if(mParams.midphaseDesc.getType() == PxMeshMidPhase::eBVH33)
	{
		RTreeTriangleMeshBuilder builder(mParams);
		return createTriangleMesh(builder, desc, insertionCallback, condition);
	}
	else
	{
		BV4TriangleMeshBuilder builder(mParams);
		return createTriangleMesh(builder, desc, insertionCallback, condition);
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// cook convex mesh from given desc, internal function to be shared between create/cook convex mesh
bool Cooking::cookConvexMeshInternal(const PxConvexMeshDesc& desc_, ConvexMeshBuilder& meshBuilder, ConvexHullLib* hullLib,
	PxConvexMeshCookingResult::Enum* condition) const
{
	if (condition)
		*condition = PxConvexMeshCookingResult::eFAILURE;

	if (!desc_.isValid())
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "Cooking::cookConvexMesh: user-provided convex mesh descriptor is invalid!");
		return false;
	}

	if (mParams.areaTestEpsilon <= 0.0f)
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "Cooking::cookConvexMesh: provided cooking parameter areaTestEpsilon is invalid!");
		return false;
	}

	if(mParams.planeTolerance < 0.0f)
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "Cooking::cookConvexMesh: provided cooking parameter planeTolerance is invalid!");
		return false;
	}

	PxConvexMeshDesc desc = desc_;	
	bool polygonsLimitReached = false;

	// the convex will be cooked from provided points
	if (desc_.flags & PxConvexFlag::eCOMPUTE_CONVEX)
	{
		PX_ASSERT(hullLib);

		// clean up the indices information, it could have been set by accident
		desc.flags &= ~PxConvexFlag::e16_BIT_INDICES;
		desc.indices.count = 0;
		desc.indices.data = NULL;
		desc.indices.stride = 0;
		desc.polygons.count = 0;
		desc.polygons.data = NULL;
		desc.polygons.stride = 0;

		PxConvexMeshCookingResult::Enum res = hullLib->createConvexHull();
		if (res == PxConvexMeshCookingResult::eSUCCESS || res == PxConvexMeshCookingResult::ePOLYGONS_LIMIT_REACHED)
		{
			if (res == PxConvexMeshCookingResult::ePOLYGONS_LIMIT_REACHED)
				polygonsLimitReached = true;

			hullLib->fillConvexMeshDesc(desc);
		}
		else
		{
			if (res == PxConvexMeshCookingResult::eZERO_AREA_TEST_FAILED)
			{
				*condition = PxConvexMeshCookingResult::eZERO_AREA_TEST_FAILED;
			}
			
			return false;
		}
	}

	if (desc.points.count >= 256)
	{		
		Ps::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "Cooking::cookConvexMesh: user-provided hull must have less than 256 vertices!");
		return false;
	}
		
	if (!meshBuilder.build(desc, mParams.gaussMapLimit, false, hullLib))
	{	
		return false;
	}

	if (condition)
	{
		*condition = polygonsLimitReached ? PxConvexMeshCookingResult::ePOLYGONS_LIMIT_REACHED : PxConvexMeshCookingResult::eSUCCESS;
	}

	return true;
}

//////////////////////////////////////////////////////////////////////////
// cook convex mesh from given desc, save the results into stream
bool Cooking::cookConvexMesh(const PxConvexMeshDesc& desc_, PxOutputStream& stream, PxConvexMeshCookingResult::Enum* condition) const
{	
	PX_FPU_GUARD;
	// choose cooking library if needed
	ConvexHullLib* hullLib = NULL;
    PxConvexMeshDesc desc = desc_;

	if(desc_.flags & PxConvexFlag::eCOMPUTE_CONVEX)
	{			
		const PxU16 gpuMaxVertsLimit = 64;

		// GRB supports 64 verts max
		if(desc_.flags & PxConvexFlag::eGPU_COMPATIBLE)
		{
			desc.vertexLimit = PxMin(desc.vertexLimit, gpuMaxVertsLimit);
		}

		hullLib = PX_NEW(QuickHullConvexHullLib) (desc, mParams);			
	}

	ConvexMeshBuilder meshBuilder(mParams.buildGPUData);
	if(!cookConvexMeshInternal(desc,meshBuilder,hullLib , condition))
	{
		if(hullLib)
			PX_DELETE(hullLib);
		return false;
	}

	// save the cooked results into stream
	if(!meshBuilder.save(stream, platformMismatch()))
	{		
		if (condition)
		{
			*condition = PxConvexMeshCookingResult::eFAILURE;
		}
		if(hullLib)
			PX_DELETE(hullLib);
		return false;
	}
	
	if(hullLib)
		PX_DELETE(hullLib);
	return true;
}

//////////////////////////////////////////////////////////////////////////
// cook convex mesh from given desc, copy the results into internal convex mesh
// and insert the mesh into PxPhysics
PxConvexMesh* Cooking::createConvexMesh(const PxConvexMeshDesc& desc_, PxPhysicsInsertionCallback& insertionCallback, PxConvexMeshCookingResult::Enum* condition) const
{
	PX_FPU_GUARD;
	// choose cooking library if needed
	ConvexHullLib* hullLib = NULL;	
	PxConvexMeshDesc desc = desc_;

	if(desc.flags & PxConvexFlag::eCOMPUTE_CONVEX)
	{
		const PxU16 gpuMaxVertsLimit = 64;

		// GRB supports 64 verts max
		if(desc_.flags & PxConvexFlag::eGPU_COMPATIBLE)
		{
			desc.vertexLimit = PxMin(desc.vertexLimit, gpuMaxVertsLimit);
		}

		hullLib = PX_NEW(QuickHullConvexHullLib) (desc, mParams);
	}

	// cook the mesh
	ConvexMeshBuilder meshBuilder(mParams.buildGPUData);
	if (!cookConvexMeshInternal(desc, meshBuilder, hullLib, condition))
	{
		if(hullLib)
			PX_DELETE(hullLib);		
		return NULL;
	}
	
	// copy the constructed data into the new mesh

	PxU32 nb = 0;
	Gu::ConvexHullData meshData;
	meshBuilder.copy(meshData, nb);

	// insert into physics
	Gu::ConvexMesh* convexMesh = static_cast<Gu::ConvexMesh*>(insertionCallback.buildObjectFromData(PxConcreteType::eCONVEX_MESH, &meshData));
	if (!convexMesh)
	{
		if(condition)
			*condition = PxConvexMeshCookingResult::eFAILURE;
		if (hullLib)
			PX_DELETE(hullLib);
		return NULL;
	}

	convexMesh->setNb(nb);
	convexMesh->setMass(meshBuilder.getMass());
	convexMesh->setInertia(meshBuilder.getInertia());
	if(meshBuilder.getBigConvexData())
	{
		convexMesh->setBigConvexData(meshBuilder.getBigConvexData());
		meshBuilder.setBigConvexData(NULL);
	}

	if(hullLib)
		PX_DELETE(hullLib);
	return convexMesh;
}

//////////////////////////////////////////////////////////////////////////

bool Cooking::validateConvexMesh(const PxConvexMeshDesc& desc) const
{
	ConvexMeshBuilder mesh(mParams.buildGPUData);
	return mesh.build(desc, mParams.gaussMapLimit, true);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Cooking::computeHullPolygons(const PxSimpleTriangleMesh& mesh, PxAllocatorCallback& inCallback,PxU32& nbVerts, PxVec3*& vertices,
		PxU32& nbIndices, PxU32*& indices, PxU32& nbPolygons, PxHullPolygon*& hullPolygons) const
{
	PxVec3* geometry = reinterpret_cast<PxVec3*>(PxAlloca(sizeof(PxVec3)*mesh.points.count));
	Cooking::gatherStrided(mesh.points.data, geometry, mesh.points.count, sizeof(PxVec3), mesh.points.stride);

	PxU32* topology = reinterpret_cast<PxU32*>(PxAlloca(sizeof(PxU32)*3*mesh.triangles.count));
	if (mesh.flags & PxMeshFlag::e16_BIT_INDICES)
	{
		// conversion; 16 bit index -> 32 bit index & stride
		PxU32* dest = topology;
		const PxU32* pastLastDest = topology + 3*mesh.triangles.count;
		const PxU8* source = reinterpret_cast<const PxU8*>(mesh.triangles.data);
		while (dest < pastLastDest)
		{
			const PxU16 * trig16 = reinterpret_cast<const PxU16*>(source);
			*dest++ = trig16[0];
			*dest++ = trig16[1];
			*dest++ = trig16[2];
			source += mesh.triangles.stride;
		}
	}
	else
	{
		Cooking::gatherStrided(mesh.triangles.data, topology, mesh.triangles.count, sizeof(PxU32) * 3, mesh.triangles.stride);
	}

	ConvexMeshBuilder meshBuilder(mParams.buildGPUData);
	if(!meshBuilder.computeHullPolygons(mesh.points.count,geometry,mesh.triangles.count,topology,inCallback, nbVerts, vertices,nbIndices,indices,nbPolygons,hullPolygons))
		return false;

	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Cooking::cookHeightField(const PxHeightFieldDesc& desc, PxOutputStream& stream) const
{
	PX_FPU_GUARD;

	if(!desc.isValid())
	{
		#if PX_CHECKED
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "Cooking::createHeightField: user-provided heightfield descriptor is invalid!");
		#endif
		return false;
	}
	
	Gu::HeightField hf(NULL);

	if(!hf.loadFromDesc(desc))
	{
		hf.releaseMemory();		
		return false;
	}

	if (!saveHeightField(hf, stream, platformMismatch()))
	{
		hf.releaseMemory();		
		return false;
	}

	hf.releaseMemory();	

	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PxHeightField* Cooking::createHeightField(const PxHeightFieldDesc& desc, PxPhysicsInsertionCallback& insertionCallback) const
{
	PX_FPU_GUARD;

	if(!desc.isValid())
	{
		#if PX_CHECKED
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "Cooking::createHeightField: user-provided heightfield descriptor is invalid!");
		#endif
		return NULL;
	}

	Gu::HeightField* hf;
	PX_NEW_SERIALIZED(hf, Gu::HeightField)(NULL);

	if(!hf->loadFromDesc(desc))
	{
		PX_DELETE(hf);
		return NULL;
	}

	// create heightfield and set the HF data
	Gu::HeightField* heightField = static_cast<Gu::HeightField*>(insertionCallback.buildObjectFromData(PxConcreteType::eHEIGHTFIELD, &hf->mData));
	if(!heightField)
	{
		PX_DELETE(hf);
		return NULL;
	}

	// copy the Gu::HeightField variables
	heightField->mSampleStride = hf->mSampleStride;
	heightField->mNbSamples = hf->mNbSamples;
	heightField->mMinHeight = hf->mMinHeight;
	heightField->mMaxHeight = hf->mMaxHeight;
	heightField->mModifyCount = hf->mModifyCount;

	PX_DELETE(hf);
	return heightField;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Cooking::cookBVHStructure(const PxBVHStructureDesc& desc, PxOutputStream& stream) const
{
	PX_FPU_GUARD;

	if(!desc.isValid())
	{
		#if PX_CHECKED
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "Cooking::createHeightField: user-provided heightfield descriptor is invalid!");
		#endif
		return false;
	}

	BVHStructureBuilder builder;
	if(!builder.loadFromDesc(desc))
	{
		return false;
	}

	builder.save(stream, platformMismatch());
	return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PxBVHStructure* Cooking::createBVHStructure(const PxBVHStructureDesc& desc, PxPhysicsInsertionCallback& insertionCallback) const
{
	PX_FPU_GUARD;

	if(!desc.isValid())
	{
		#if PX_CHECKED
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "Cooking::createHeightField: user-provided heightfield descriptor is invalid!");
		#endif
		return NULL;
	}

	BVHStructureBuilder builder;
	if(!builder.loadFromDesc(desc))
	{
		return NULL;
	}

	Gu::BVHStructureData bvhData;
	builder.moveData(bvhData);
	Gu::BVHStructure* bvhStructure = static_cast<Gu::BVHStructure*>(insertionCallback.buildObjectFromData(PxConcreteType::eBVH_STRUCTURE, &bvhData));

	return bvhStructure;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PxCooking* PxCreateCooking(PxU32 /*version*/, PxFoundation& foundation, const PxCookingParams& params)
{
	PX_ASSERT(static_cast<Ps::Foundation*>(&foundation) == &Ps::Foundation::getInstance());
	PX_UNUSED(foundation);

	Ps::Foundation::incRefCount();

	return PX_NEW(Cooking)(params);
}

