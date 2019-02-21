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


#ifndef NP_MATERIALMANAGER
#define NP_MATERIALMANAGER

#include "foundation/PxMemory.h"
#include "NpMaterial.h"
#include "CmIDPool.h"

namespace physx
{
	class NpMaterialManager 
	{
	public:
		NpMaterialManager()
		{
			const PxU32 matCount = 128;
			mMaterials = reinterpret_cast<NpMaterial**>(PX_ALLOC(sizeof(NpMaterial*) * matCount,  "NpMaterialManager::initialise"));
			mMaxMaterials = matCount;
			PxMemZero(mMaterials, sizeof(NpMaterial*)*mMaxMaterials);
		}

		~NpMaterialManager() {}

		void releaseMaterials()
		{
			for(PxU32 i=0; i<mMaxMaterials; ++i)
			{
				if(mMaterials[i])
				{
					const PxU32 handle = mMaterials[i]->getHandle();
					mHandleManager.freeID(handle);
					mMaterials[i]->release();
					mMaterials[i] = NULL;
				}
			}
			PX_FREE(mMaterials);
		}

		bool setMaterial(NpMaterial& mat)
		{
			const PxU32 materialIndex = mHandleManager.getNewID();

			if(materialIndex >= mMaxMaterials)
				resize();

			mMaterials[materialIndex] = &mat;
			mat.setHandle(materialIndex);
			return true;
		}

		void updateMaterial(NpMaterial& mat)
		{
			mMaterials[mat.getHandle()] = &mat;
		}

		PX_FORCE_INLINE PxU32 getNumMaterials()	const
		{
			return mHandleManager.getNumUsedID();
		}

		void removeMaterial(NpMaterial& mat)
		{
			const PxU32 handle = mat.getHandle();
			if(handle != MATERIAL_INVALID_HANDLE)
			{
				mMaterials[handle] = NULL;
				mHandleManager.freeID(handle);
			}
		}

		PX_FORCE_INLINE NpMaterial* getMaterial(const PxU32 index)	const
		{
			PX_ASSERT(index <  mMaxMaterials);
			return mMaterials[index];
		}

		PX_FORCE_INLINE PxU32 getMaxSize()	const 
		{
			return mMaxMaterials;
		}

		PX_FORCE_INLINE NpMaterial** getMaterials() const
		{
			return mMaterials;
		}

	private:
		void resize()
		{
			const PxU32 numMaterials = mMaxMaterials;
			mMaxMaterials = mMaxMaterials*2;

			NpMaterial** mat = reinterpret_cast<NpMaterial**>(PX_ALLOC(sizeof(NpMaterial*)*mMaxMaterials,  "NpMaterialManager::resize"));
			PxMemZero(mat, sizeof(NpMaterial*)*mMaxMaterials);
			for(PxU32 i=0; i<numMaterials; ++i)
				mat[i] = mMaterials[i];

			PX_FREE(mMaterials);

			mMaterials = mat;
		}

		Cm::IDPool		mHandleManager;
		NpMaterial**	mMaterials;
		PxU32			mMaxMaterials;
	};

	class NpMaterialManagerIterator
	{
	public:
		NpMaterialManagerIterator(const NpMaterialManager& manager) : mManager(manager), mIndex(0)
		{
		}

		bool getNextMaterial(NpMaterial*& np)
		{
			const PxU32 maxSize = mManager.getMaxSize();
			PxU32 index = mIndex;
			while(index < maxSize && mManager.getMaterial(index)==NULL)
				index++;
			np = NULL;
			if(index < maxSize)
				np = mManager.getMaterial(index++);
			mIndex = index;
			return np!=NULL;
		}

	private:
		NpMaterialManagerIterator& operator=(const NpMaterialManagerIterator&);
		const NpMaterialManager&	mManager;
		PxU32						mIndex;
	};
}

#endif
