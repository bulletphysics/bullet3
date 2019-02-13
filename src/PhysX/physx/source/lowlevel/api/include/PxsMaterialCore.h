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


#ifndef PXS_MATERIAL_H
#define PXS_MATERIAL_H

#include "foundation/PxVec3.h"
#include "PxMaterial.h"
#include "PsUserAllocated.h"
#include "CmPhysXCommon.h"
#include "PxMetaData.h"
#include "PsUtilities.h"

namespace physx
{

#define	MATERIAL_INVALID_HANDLE	0xffffffff

	//PX_ALIGN_PREFIX(16) struct PxsMaterialData 
	//{
	//	PxReal					dynamicFriction;
	//	PxReal					staticFriction;
	//	PxReal					restitution;
	//	PxReal					dynamicFrictionV;
	//	PxReal					staticFrictionV;
	//	PxVec3					dirOfAnisotropy;//might need to get rid of this
	//	PxCombineMode::Enum		frictionCombineMode;
	//	PxCombineMode::Enum		restitutionCombineMode;

	//	PxMaterialFlags			flags;
	//	PxU8					paddingFromFlags[2];
	//	PxU32					pads;

	//	PxsMaterialData()
	//	:	dynamicFriction(0.0)
	//	,	staticFriction(0.0f)
	//	,	restitution(0.0f)
	//	,	dynamicFrictionV(0.0f)
	//	,	staticFrictionV(0.0f)
	//	,	dirOfAnisotropy(1,0,0)
	//	,	frictionCombineMode(PxCombineMode::eAVERAGE)
	//	,	restitutionCombineMode(PxCombineMode::eAVERAGE)
	//	{}

	//	PX_FORCE_INLINE PxCombineMode::Enum getFrictionCombineMode() const
	//	{
	//		return frictionCombineMode;
	//	}

	//	
	//	PX_FORCE_INLINE PxCombineMode::Enum getRestitutionCombineMode() const
	//	{
	//		return restitutionCombineMode;
	//	}

	//	PX_FORCE_INLINE void setFrictionCombineMode(PxCombineMode::Enum frictionFlags)
	//	{
	//		frictionCombineMode = frictionFlags;
	//	}

	//	PX_FORCE_INLINE void setRestitutionCombineMode(PxCombineMode::Enum restitutionFlags)
	//	{
	//		restitutionCombineMode = restitutionFlags;
	//	}

	//}PX_ALIGN_SUFFIX(16);

	PX_ALIGN_PREFIX(16) struct PxsMaterialData 
	{
		PxReal					dynamicFriction;				//4
		PxReal					staticFriction;					//8
		PxReal					restitution;					//12
		PxMaterialFlags			flags;							//14
		PxU8					fricRestCombineMode;			//15
		PxU8					padding;						//16

		PxsMaterialData()
		:	dynamicFriction(0.0f)
		,	staticFriction(0.0f)
		,	restitution(0.0f)
		,	fricRestCombineMode((PxCombineMode::eAVERAGE << 4) | PxCombineMode::eAVERAGE)
		,	padding(0)
		{}

		PxsMaterialData(const PxEMPTY) {}

		PX_CUDA_CALLABLE PX_FORCE_INLINE PxCombineMode::Enum getFrictionCombineMode() const
		{
			return PxCombineMode::Enum(fricRestCombineMode >> 4);
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE PxCombineMode::Enum getRestitutionCombineMode() const
		{
			return PxCombineMode::Enum(fricRestCombineMode & 0xf);
		}

		PX_FORCE_INLINE void setFrictionCombineMode(PxCombineMode::Enum frictionFlags)
		{
			fricRestCombineMode = Ps::to8((fricRestCombineMode & 0xf) | (frictionFlags << 4));
		}

		PX_FORCE_INLINE void setRestitutionCombineMode(PxCombineMode::Enum restitutionFlags)
		{
			fricRestCombineMode = Ps::to8((fricRestCombineMode & 0xf0) | (restitutionFlags));
		}

	}PX_ALIGN_SUFFIX(16);


	class PxsMaterialCore : public PxsMaterialData, public Ps::UserAllocated
	{
	public:
					
											PxsMaterialCore(const PxsMaterialData& desc): PxsMaterialData(desc), mNxMaterial(0), mMaterialIndex(MATERIAL_INVALID_HANDLE)
											{
											}

											PxsMaterialCore(): mNxMaterial(0), mMaterialIndex(MATERIAL_INVALID_HANDLE)
											{
											}

											PxsMaterialCore(const PxEMPTY) : PxsMaterialData(PxEmpty) {}

											~PxsMaterialCore()
											{
											}

	PX_FORCE_INLINE	void					setNxMaterial(PxMaterial* m)					{ mNxMaterial = m;		}
	PX_FORCE_INLINE	PxMaterial*				getNxMaterial()					const			{ return mNxMaterial;	}
	PX_FORCE_INLINE	void					setMaterialIndex(const PxU32 materialIndex)		{ mMaterialIndex = materialIndex; }
	PX_FORCE_INLINE	PxU32					getMaterialIndex()				const			{ return mMaterialIndex; }

	protected:
					PxMaterial*				mNxMaterial;
					PxU32					mMaterialIndex; //handle assign by the handle manager
	};

} //namespace phyxs

#endif
