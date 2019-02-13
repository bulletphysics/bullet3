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


#ifndef PX_MIDPHASE_DESC_H
#define PX_MIDPHASE_DESC_H
/** \addtogroup cooking
@{
*/

#include "geometry/PxTriangleMesh.h"
#include "cooking/PxBVH33MidphaseDesc.h"
#include "cooking/PxBVH34MidphaseDesc.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/**

\brief Structure describing parameters affecting midphase mesh structure.

@see PxCookingParams, PxBVH33MidphaseDesc, PxBVH34MidphaseDesc
*/
class PxMidphaseDesc
{
public:
	PX_FORCE_INLINE PxMidphaseDesc()	{ setToDefault(PxMeshMidPhase::eBVH33);	}

	/**
	\brief	Returns type of midphase mesh structure.
	\return	PxMeshMidPhase::Enum 

	@see PxMeshMidPhase::Enum
	*/
	PX_FORCE_INLINE PxMeshMidPhase::Enum getType() const { return mType; }

	/**
	\brief	Midphase descriptors union

	@see PxBV33MidphaseDesc, PxBV34MidphaseDesc
	*/
	union {		
		PxBVH33MidphaseDesc  mBVH33Desc;
		PxBVH34MidphaseDesc  mBVH34Desc;
    };

	/**
	\brief	Initialize the midphase mesh structure descriptor
	\param[in] type Midphase mesh structure descriptor

	@see PxBV33MidphaseDesc, PxBV34MidphaseDesc
	*/
	void setToDefault(PxMeshMidPhase::Enum type)
	{
		mType = type;
		if(type==PxMeshMidPhase::eBVH33)
			mBVH33Desc.setToDefault();
		else if(type==PxMeshMidPhase::eBVH34)
			mBVH34Desc.setToDefault();
	}

	/**
	\brief Returns true if the descriptor is valid.
	\return true if the current settings are valid.
	*/
	bool isValid() const
	{		
		if(mType==PxMeshMidPhase::eBVH33)
			return mBVH33Desc.isValid();
		else if(mType==PxMeshMidPhase::eBVH34)
			return mBVH34Desc.isValid();
		return false;
	}

	PX_FORCE_INLINE PxMidphaseDesc&		operator=(PxMeshMidPhase::Enum descType) 
	{ 
		setToDefault(descType);
		return *this; 
	}

protected:	
	PxMeshMidPhase::Enum	mType;
};

#if !PX_DOXYGEN
} // namespace physx
#endif


  /** @} */
#endif // PX_MIDPHASE_DESC_UNION_H
