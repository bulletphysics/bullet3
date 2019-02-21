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

#include "PxTriangleMesh.h"
#include "PxvGeometry.h"
#include "PxsMaterialManager.h"
#include "PxcNpThreadContext.h"
#include "GuHeightField.h"
#include "GuTriangleMesh.h"

using namespace physx;
using namespace Gu;

namespace physx
{
	bool PxcGetMaterialShapeMesh(const PxsShapeCore* shape0, const PxsShapeCore* shape1, PxcNpThreadContext& context, PxsMaterialInfo* materialInfo);
	bool PxcGetMaterialMesh(const PxsShapeCore* shape, const PxU32 index,  PxcNpThreadContext& context, PxsMaterialInfo* materialInfo);
}

bool physx::PxcGetMaterialMesh(const PxsShapeCore* shape, const PxU32 index, PxcNpThreadContext& context, PxsMaterialInfo* materialInfo)
{
	PX_ASSERT(index == 1);
	PX_UNUSED(index);
	ContactBuffer& contactBuffer = context.mContactBuffer;
	const PxTriangleMeshGeometryLL& shapeMesh = shape->geometry.get<const PxTriangleMeshGeometryLL>();
	if(shapeMesh.materials.numIndices <= 1)
	{
		for(PxU32 i=0; i< contactBuffer.count; ++i) 
		{
			(&materialInfo[i].mMaterialIndex0)[index] = shape->materialIndex;
		}
	}
	else
	{
		for(PxU32 i=0; i< contactBuffer.count; ++i)
		{

			Gu::ContactPoint& contact = contactBuffer.contacts[i];
			const PxU16* eaMaterialIndices = shapeMesh.materialIndices;

			const PxU32 localMaterialIndex = eaMaterialIndices[contact.internalFaceIndex1];//shapeMesh.triangleMesh->getTriangleMaterialIndex(contact.featureIndex1);
			(&materialInfo[i].mMaterialIndex0)[index] = shapeMesh.materials.indices[localMaterialIndex];
		}
	}
	return true;
}

bool physx::PxcGetMaterialShapeMesh(const PxsShapeCore* shape0, const PxsShapeCore* shape1, PxcNpThreadContext& context, PxsMaterialInfo* materialInfo)
{
	
	ContactBuffer& contactBuffer = context.mContactBuffer;
	const PxTriangleMeshGeometryLL& shapeMesh = shape1->geometry.get<const PxTriangleMeshGeometryLL>();
//	const Gu::TriangleMesh* meshData = shapeMesh.meshData;
	if(shapeMesh.materials.numIndices <= 1)
	{
		for(PxU32 i=0; i< contactBuffer.count; ++i) 
		{
			materialInfo[i].mMaterialIndex0 = shape0->materialIndex;
			materialInfo[i].mMaterialIndex1 = shape1->materialIndex;
		}
	}
	else
	{

		for(PxU32 i=0; i< contactBuffer.count; ++i)
		{

			Gu::ContactPoint& contact = contactBuffer.contacts[i];
			//contact.featureIndex0 = shape0->materialIndex;
			materialInfo[i].mMaterialIndex0 = shape0->materialIndex;
			const PxU16* eaMaterialIndices = shapeMesh.materialIndices;

			const PxU32 localMaterialIndex = eaMaterialIndices[contact.internalFaceIndex1];//shapeMesh.triangleMesh->getTriangleMaterialIndex(contact.featureIndex1);
			//contact.featureIndex1 = shapeMesh.materials.indices[localMaterialIndex];
			materialInfo[i].mMaterialIndex1 = shapeMesh.materials.indices[localMaterialIndex];

		}
	}

	return true;
}
