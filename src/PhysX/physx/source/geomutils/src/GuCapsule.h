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

#ifndef GU_CAPSULE_H
#define GU_CAPSULE_H

/** \addtogroup geomutils
@{
*/

#include "GuSegment.h"

namespace physx
{
namespace Gu
{

/**
\brief Represents a capsule.
*/
	class Capsule : public Segment
	{
	public:
		/**
		\brief Constructor
		*/
		PX_INLINE Capsule()
		{
		}

		/**
		\brief Constructor

		\param seg Line segment to create capsule from.
		\param _radius Radius of the capsule.
		*/
		PX_INLINE Capsule(const Segment& seg, PxF32 _radius) : Segment(seg), radius(_radius)
		{
		}

		/**
		\brief Constructor

		\param _p0 First segment point
		\param _p1 Second segment point
		\param _radius Radius of the capsule.
		*/
		PX_INLINE Capsule(const PxVec3& _p0, const PxVec3& _p1, PxF32 _radius) : Segment(_p0, _p1), radius(_radius)
		{           
		}

		/**
		\brief Destructor
		*/
		PX_INLINE ~Capsule()
		{
		}

		PxF32	radius;
	};
}

}

/** @} */
#endif
