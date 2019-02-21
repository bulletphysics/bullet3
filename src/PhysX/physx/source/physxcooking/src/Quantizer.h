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


#ifndef QUANTIZER_H
#define QUANTIZER_H

#include "foundation/Px.h"

namespace physx
{

	//////////////////////////////////////////////////////////////////////////
	// K-means quantization class
	// see http://en.wikipedia.org/wiki/K-means_clustering
	// implementation from John Ratcliff http://codesuppository.blogspot.ch/2010/12/k-means-clustering-algorithm.html
	class Quantizer
	{
	public:
		// quantize the input vertices
		virtual const PxVec3* kmeansQuantize3D(PxU32 vcount,
													const PxVec3 *vertices,
													PxU32 stride,
													bool denormalizeResults,
													PxU32 maxVertices,
													PxU32 &outVertsCount) = 0;

		// returns the denormalized scale
		virtual const PxVec3& getDenormalizeScale(void) const = 0;

		// returns the denormalized center
		virtual const PxVec3& getDenormalizeCenter(void) const = 0;

		// release internal data
		virtual void release(void) = 0;


	protected:
		virtual ~Quantizer(void)
		{

		}
	};

	// creates the quantizer class
	Quantizer * createQuantizer(void);

}

#endif
