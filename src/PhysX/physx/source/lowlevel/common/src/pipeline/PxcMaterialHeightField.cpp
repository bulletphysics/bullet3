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

using namespace physx;
using namespace Gu;

namespace physx
{
	bool PxcGetMaterialShapeHeightField(const PxsShapeCore* shape0, const PxsShapeCore* shape1, PxcNpThreadContext& context, PxsMaterialInfo* materialInfo);
	bool PxcGetMaterialHeightField(const PxsShapeCore* shape, const PxU32 index, PxcNpThreadContext& context, PxsMaterialInfo* materialInfo);
	PxU32 GetMaterialIndex(const Gu::HeightFieldData* hfData, PxU32 triangleIndex);
}

physx::PxU32 physx::GetMaterialIndex(const Gu::HeightFieldData* hfData, PxU32 triangleIndex)
{
	const PxU32 sampleIndex = triangleIndex >> 1;
	const bool isFirstTriangle = (triangleIndex & 0x1) == 0;

	//get sample
	const PxHeightFieldSample* hf = &hfData->samples[sampleIndex];
	return isFirstTriangle ? hf->materialIndex0 : hf->materialIndex1;
}

bool physx::PxcGetMaterialHeightField(const PxsShapeCore* shape, const PxU32 index, PxcNpThreadContext& context, PxsMaterialInfo* materialInfo)
{
	PX_ASSERT(index == 1);
	PX_UNUSED(index);
	const ContactBuffer& contactBuffer = context.mContactBuffer;
	const PxHeightFieldGeometryLL& hfGeom = shape->geometry.get<const PxHeightFieldGeometryLL>();
	if(hfGeom.materials.numIndices <= 1)
	{
		for(PxU32 i=0; i< contactBuffer.count; ++i)
		{
			(&materialInfo[i].mMaterialIndex0)[index] = shape->materialIndex;
		}
	}
	else
	{
		const PxU16* materialIndices = hfGeom.materials.indices;
			
		const Gu::HeightFieldData* hf = hfGeom.heightFieldData;
		
		for(PxU32 i=0; i< contactBuffer.count; ++i)
		{
			const Gu::ContactPoint& contact = contactBuffer.contacts[i];
			const PxU32 localMaterialIndex = GetMaterialIndex(hf, contact.internalFaceIndex1);
			(&materialInfo[i].mMaterialIndex0)[index] = materialIndices[localMaterialIndex];
		}
	}
	return true;
}

bool physx::PxcGetMaterialShapeHeightField(const PxsShapeCore* shape0, const PxsShapeCore* shape1, PxcNpThreadContext& context,  PxsMaterialInfo* materialInfo)
{
	const ContactBuffer& contactBuffer = context.mContactBuffer;
	const PxHeightFieldGeometryLL& hfGeom = shape1->geometry.get<const PxHeightFieldGeometryLL>();
	if(hfGeom.materials.numIndices <= 1)
	{
		for(PxU32 i=0; i< contactBuffer.count; ++i)
		{
			materialInfo[i].mMaterialIndex0 = shape0->materialIndex;
			materialInfo[i].mMaterialIndex1 = shape1->materialIndex;
		}
	}
	else
	{
		const PxU16* materialIndices = hfGeom.materials.indices;
			
		const Gu::HeightFieldData* hf = hfGeom.heightFieldData;
		
		for(PxU32 i=0; i< contactBuffer.count; ++i)
		{
			const Gu::ContactPoint& contact = contactBuffer.contacts[i];
			materialInfo[i].mMaterialIndex0 = shape0->materialIndex;
			//contact.featureIndex0 = shape0->materialIndex;
			const PxU32 localMaterialIndex = GetMaterialIndex(hf, contact.internalFaceIndex1);
			//contact.featureIndex1 = materialIndices[localMaterialIndex];
			PX_ASSERT(localMaterialIndex<hfGeom.materials.numIndices);
			materialInfo[i].mMaterialIndex1 = materialIndices[localMaterialIndex];
		}
	}
	return true;
}

