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
#include "PsFPU.h"
#include "float.h"
#include "PsIntrinsics.h"

#if PX_X64 || PX_ARM
#define _MCW_ALL _MCW_DN | _MCW_EM | _MCW_RC
#else
#define _MCW_ALL _MCW_DN | _MCW_EM | _MCW_IC | _MCW_RC | _MCW_PC
#endif

physx::shdfnd::FPUGuard::FPUGuard()
{
// default plus FTZ and DAZ
#if PX_X64 || PX_ARM
	// query current control word state
	_controlfp_s(mControlWords, 0, 0);

	// set both x87 and sse units to default + DAZ
	unsigned int cw;
	_controlfp_s(&cw, _CW_DEFAULT | _DN_FLUSH, _MCW_ALL);
#else
	// query current control word state
	__control87_2(0, 0, mControlWords, mControlWords + 1);

	// set both x87 and sse units to default + DAZ
	unsigned int x87, sse;
	__control87_2(_CW_DEFAULT | _DN_FLUSH, _MCW_ALL, &x87, &sse);
#endif
}

physx::shdfnd::FPUGuard::~FPUGuard()
{
	_clearfp();

#if PX_X64 || PX_ARM
	// reset FP state
	unsigned int cw;
	_controlfp_s(&cw, *mControlWords, _MCW_ALL);
#else

	// reset FP state
	unsigned int x87, sse;
	__control87_2(mControlWords[0], _MCW_ALL, &x87, 0);
	__control87_2(mControlWords[1], _MCW_ALL, 0, &sse);
#endif
}

void physx::shdfnd::enableFPExceptions()
{
	// clear any pending exceptions
	_clearfp();

	// enable all fp exceptions except inexact and underflow (common, benign)
	_controlfp_s(NULL, uint32_t(~_MCW_EM) | _EM_INEXACT | _EM_UNDERFLOW, _MCW_EM);
}

void physx::shdfnd::disableFPExceptions()
{
	_controlfp_s(NULL, _MCW_EM, _MCW_EM);
}
