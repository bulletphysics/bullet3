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

#include "SnConvX.h"
#include "SnConvX_Align.h"
#include <assert.h>
using namespace physx;

void Sn::ConvX::alignTarget(int alignment)
{
	const int outputSize = getCurrentOutputSize();
	const PxU32 outPadding = getPadding(size_t(outputSize), PxU32(alignment));
	if(outPadding)
	{
		assert(outPadding<CONVX_ZERO_BUFFER_SIZE);
		output(mZeros, int(outPadding));
	}
}

const char* Sn::ConvX::alignStream(const char* buffer, int alignment)
{
	const PxU32 padding = getPadding(size_t(buffer), PxU32(alignment));
	assert(!getPadding(size_t(buffer + padding), PxU32(alignment)));

	const int outputSize = getCurrentOutputSize();
	const PxU32 outPadding = getPadding(size_t(outputSize), PxU32(alignment));
	if(outPadding==padding)
	{
		assert(outPadding<CONVX_ZERO_BUFFER_SIZE);
		output(mZeros, int(outPadding));
	}
	else if(outPadding)
	{
		assert(outPadding<CONVX_ZERO_BUFFER_SIZE);
		output(mZeros, int(outPadding));
	}

	return buffer + padding;
}
