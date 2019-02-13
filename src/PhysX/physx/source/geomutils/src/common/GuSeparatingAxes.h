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

#ifndef GU_SEPARATINGAXES_H
#define GU_SEPARATINGAXES_H

#include "foundation/PxVec3.h"
#include "PxPhysXCommonConfig.h"

namespace physx
{
namespace Gu
{
	// PT: this is a number of axes. Multiply by sizeof(PxVec3) for size in bytes.
	#define SEP_AXIS_FIXED_MEMORY	256

	// This class holds a list of potential separating axes.
	// - the orientation is irrelevant so V and -V should be the same vector
	// - the scale is irrelevant so V and n*V should be the same vector
	// - a given separating axis should appear only once in the class
#if PX_VC 
    #pragma warning(push)
	#pragma warning( disable : 4251 ) // class needs to have dll-interface to be used by clients of class
#endif
	class PX_PHYSX_COMMON_API SeparatingAxes
	{
	public:
		PX_INLINE SeparatingAxes() : mNbAxes(0)	{}

		bool addAxis(const PxVec3& axis);

		PX_FORCE_INLINE const PxVec3* getAxes() const
		{
			return mAxes;
		}

		PX_FORCE_INLINE PxU32 getNumAxes() const
		{
			return mNbAxes;
		}

		PX_FORCE_INLINE void reset()
		{
			mNbAxes = 0;
		}

	private:
		PxU32	mNbAxes;
		PxVec3	mAxes[SEP_AXIS_FIXED_MEMORY];
	};
#if PX_VC 
     #pragma warning(pop) 
#endif

	enum PxcSepAxisType
	{
		SA_NORMAL0,		// Normal of object 0
		SA_NORMAL1,		// Normal of object 1
		SA_EE			// Cross product of edges
	};

}
}

#endif
