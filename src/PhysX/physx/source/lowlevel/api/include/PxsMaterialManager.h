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


#ifndef PXS_MATERIALMANAGER
#define PXS_MATERIALMANAGER

#include "PxsMaterialCore.h"
#include "PsAlignedMalloc.h"

namespace physx
{
	struct PxsMaterialInfo
	{
		PxU16 mMaterialIndex0;
		PxU16 mMaterialIndex1;
	};

	class PxsMaterialManager 
	{
	public:
		PxsMaterialManager()
		{
			const PxU32 matCount = 128;
			materials = reinterpret_cast<PxsMaterialCore*>(physx::shdfnd::AlignedAllocator<16>().allocate(sizeof(PxsMaterialCore)*matCount,  __FILE__, __LINE__));
			maxMaterials = matCount;
			for(PxU32 i=0; i<matCount; ++i)
			{
				materials[i].setMaterialIndex(MATERIAL_INVALID_HANDLE);
			}
		}

		~PxsMaterialManager()
		{
			physx::shdfnd::AlignedAllocator<16>().deallocate(materials);
		}

		void setMaterial(PxsMaterialCore* mat)
		{
			const PxU32 materialIndex = mat->getMaterialIndex();
			resize(materialIndex+1);
			materials[materialIndex] = *mat;
		}

		void updateMaterial(PxsMaterialCore* mat)
		{
			materials[mat->getMaterialIndex()] =*mat;
		}

		void removeMaterial(PxsMaterialCore* mat)
		{
			mat->setMaterialIndex(MATERIAL_INVALID_HANDLE);
		}

		PX_FORCE_INLINE PxsMaterialCore* getMaterial(const PxU32 index)const
		{
			PX_ASSERT(index <  maxMaterials);
			return &materials[index];
		}

		PxU32 getMaxSize()const 
		{
			return maxMaterials;
		}

		void resize(PxU32 minValueForMax)
		{			
			if(maxMaterials>=minValueForMax)
				return;

			const PxU32 numMaterials = maxMaterials;
			
			maxMaterials = (minValueForMax+31)&~31;
			PxsMaterialCore* mat = reinterpret_cast<PxsMaterialCore*>(physx::shdfnd::AlignedAllocator<16>().allocate(sizeof(PxsMaterialCore)*maxMaterials,  __FILE__, __LINE__));
			for(PxU32 i=0; i<numMaterials; ++i)
			{
				mat[i] = materials[i];
			}
			for(PxU32 i = numMaterials; i < maxMaterials; ++i)
			{
				mat[i].setMaterialIndex(MATERIAL_INVALID_HANDLE);
			}

			physx::shdfnd::AlignedAllocator<16>().deallocate(materials);

			materials = mat;
		}

		PxsMaterialCore* materials;//make sure materials's start address is 16 bytes align
		PxU32 maxMaterials;
		PxU32 mPad[2];
	};

	class PxsMaterialManagerIterator
	{
	
	public:
		PxsMaterialManagerIterator(PxsMaterialManager& manager) : mManager(manager), mIndex(0)
		{
		}

		bool getNextMaterial(PxsMaterialCore*& materialCore)
		{
			const PxU32 maxSize = mManager.getMaxSize();
			PxU32 index = mIndex;
			while(index < maxSize && mManager.getMaterial(index)->getMaterialIndex() == MATERIAL_INVALID_HANDLE)
				index++;
			materialCore = NULL;
			if(index < maxSize)
				materialCore = mManager.getMaterial(index++);
			mIndex = index;
			return materialCore!=NULL;
		}

	private:
		PxsMaterialManagerIterator& operator=(const PxsMaterialManagerIterator&);
		PxsMaterialManager&	mManager;
		PxU32				mIndex;
	};

}

#endif
