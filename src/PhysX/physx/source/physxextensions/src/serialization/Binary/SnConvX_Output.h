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

#ifndef PX_CONVX_OUTPUT_H
#define PX_CONVX_OUTPUT_H

#include "foundation/PxSimpleTypes.h"

namespace physx { namespace Sn {

	struct PxMetaDataEntry;
	class ConvX;
	
	typedef void	(Sn::ConvX::*ConvertCallback)	(const char* src, const PxMetaDataEntry& entry, const PxMetaDataEntry& dstEntry);

	inline_ void flip(PxI16& v)
	{
		PxI8* b = reinterpret_cast<PxI8*>(&v);
		PxI8 temp = b[0];
		b[0] = b[1];
		b[1] = temp;
	}

	inline_ void flip(PxU16& v)
	{
		flip(reinterpret_cast<PxI16&>(v));
	}

	inline_ void flip32(PxI8* b)
	{
		PxI8 temp = b[0];
		b[0] = b[3];
		b[3] = temp;
		temp = b[1];
		b[1] = b[2];
		b[2] = temp;
	}

	inline_ void flip(PxI32& v)
	{
		PxI8* b = reinterpret_cast<PxI8*>(&v);
		flip32(b);		
	}

	inline_ void flip(PxU32& v)
	{
		PxI8* b = reinterpret_cast<PxI8*>(&v);
		flip32(b);
	}

	inline_ void flip(PxI64& v)
	{
		PxI8* b = reinterpret_cast<PxI8*>(&v);

		PxI8 temp = b[0];
		b[0] = b[7];
		b[7] = temp;
		temp = b[1];
		b[1] = b[6];
		b[6] = temp;
		temp = b[2];
		b[2] = b[5];
		b[5] = temp;
		temp = b[3];
		b[3] = b[4];
		b[4] = temp;
	}

	inline_ void flip(PxF32& v)
	{
		PxI8* b = reinterpret_cast<PxI8*>(&v);
		flip32(b);
	}

	inline_ void flip(void*& v)
	{
		PxI8* b = reinterpret_cast<PxI8*>(&v);
		flip32(b);
	}

	inline_ void flip(const PxI8*& v)
	{
		PxI8* b = const_cast<PxI8*>(v);
		flip32(b);
	}
} }

#endif
