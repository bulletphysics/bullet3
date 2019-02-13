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


#ifndef PXS_MATERIALCOMBINER_H
#define PXS_MATERIALCOMBINER_H

#include "PxsMaterialCore.h"

namespace physx
{

	class PxsMaterialCombiner
	{
	public:

		class PxsCombinedMaterial
		{
		public:
			PxReal staFriction;
			PxReal dynFriction;
			PxU32  flags;	//PxMaterialFlag::eDISABLE_FRICTION, PxMaterialFlag::eDISABLE_STRONG_FRICTION.
		};

		static PxReal combineRestitution(const PxsMaterialData& material0, const PxsMaterialData& material1);

		PxsMaterialCombiner(PxReal staticFrictionScaling, PxReal dynamicFrictionScaling);

		PxsCombinedMaterial combineIsotropicFriction(const PxsMaterialData& material0, const PxsMaterialData& material1);

		//ML:: move this function to header file to avoid LHS in Xbox
		PX_FORCE_INLINE void combineIsotropicFriction(const PxsMaterialData& mat0, const PxsMaterialData& mat1, PxReal& dynamicFriction, PxReal& staticFriction, PxU32& flags)
		{
		
			const PxU32 combineFlags= (mat0.flags | mat1.flags); //& (PxMaterialFlag::eDISABLE_STRONG_FRICTION|PxMaterialFlag::eDISABLE_FRICTION);	//eventually set DisStrongFric flag, lower all others.

			if (!(combineFlags & PxMaterialFlag::eDISABLE_FRICTION))
			{
				const PxI32 fictionCombineMode = PxMax(mat0.getFrictionCombineMode(), mat1.getFrictionCombineMode());
				PxReal dynFriction = 0.f;
				PxReal staFriction = 0.f;


				switch (fictionCombineMode)
				{
				case PxCombineMode::eAVERAGE:
					dynFriction = 0.5f * (mat0.dynamicFriction + mat1.dynamicFriction);
					staFriction = 0.5f * (mat0.staticFriction + mat1.staticFriction);
					break;
				case PxCombineMode::eMIN:
					dynFriction = PxMin(mat0.dynamicFriction, mat1.dynamicFriction);
					staFriction = PxMin(mat0.staticFriction, mat1.staticFriction);
					break;
				case PxCombineMode::eMULTIPLY:
					dynFriction = (mat0.dynamicFriction * mat1.dynamicFriction);
					staFriction = (mat0.staticFriction * mat1.staticFriction);
					break;
				case PxCombineMode::eMAX:
					dynFriction = PxMax(mat0.dynamicFriction, mat1.dynamicFriction);
					staFriction = PxMax(mat0.staticFriction, mat1.staticFriction);
					break;
				}   

				dynFriction*=mDynamicFrictionScaling;
				staFriction*=mStaticFrictionScaling;
				//isotropic case
				const PxReal fDynFriction = PxMax(dynFriction, 0.f);

				const PxReal fStaFriction = physx::intrinsics::fsel(staFriction - fDynFriction, staFriction*mStaticFrictionScaling, fDynFriction);
				/*dest.dynFriction = fDynFriction;
				dest.staFriction = fStaFriction;*/

				dynamicFriction = fDynFriction;
				staticFriction = fStaFriction;
				flags = combineFlags;
			}
			else
			{
			/*	dest.flags |= PxMaterialFlag::eDISABLE_STRONG_FRICTION;
				dest.staFriction = 0.0f;
				dest.dynFriction = 0.0f;*/
				flags = (combineFlags | PxMaterialFlag::eDISABLE_STRONG_FRICTION);
				dynamicFriction = 0.f;
				staticFriction = 0.f;
			}

		}


	//private:
	protected:
		static PX_FORCE_INLINE PxReal combineScalars(PxReal a, PxReal b, PxI32 nxCombineMode)
		{
			switch (nxCombineMode)
			{
			case PxCombineMode::eAVERAGE:
				return 0.5f * (a + b);
			case PxCombineMode::eMIN:
				return PxMin(a,b);
			case PxCombineMode::eMULTIPLY:
				return a * b;
			case PxCombineMode::eMAX:
				return PxMax(a,b);
			default:
			/*	Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, 
					"Sc::MaterialCombiner::combineScalars(): unknown combine mode");*/
				return PxReal(0);
			}   
		}

		PxReal mStaticFrictionScaling, mDynamicFrictionScaling;

	};

}

#endif
