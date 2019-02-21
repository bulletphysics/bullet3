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
#include "PsFPU.h"

#if !(defined(__CYGWIN__) || PX_ANDROID || PX_PS4)
#include <fenv.h>
PX_COMPILE_TIME_ASSERT(8 * sizeof(uint32_t) >= sizeof(fenv_t));
#endif

#if PX_OSX
// osx defines SIMD as standard for floating point operations.
#include <xmmintrin.h>
#endif

physx::shdfnd::FPUGuard::FPUGuard()
{
#if defined(__CYGWIN__)
#pragma message "FPUGuard::FPUGuard() is not implemented"
#elif PX_ANDROID
// not supported unless ARM_HARD_FLOAT is enabled.
#elif PX_PS4
	// not supported
	PX_UNUSED(mControlWords);
#elif PX_OSX
	mControlWords[0] = _mm_getcsr();
	// set default (disable exceptions: _MM_MASK_MASK) and FTZ (_MM_FLUSH_ZERO_ON), DAZ (_MM_DENORMALS_ZERO_ON: (1<<6))
	_mm_setcsr(_MM_MASK_MASK | _MM_FLUSH_ZERO_ON | (1 << 6));
#elif defined(__EMSCRIPTEN__)
// not supported
#else
	PX_COMPILE_TIME_ASSERT(sizeof(fenv_t) <= sizeof(mControlWords));

	fegetenv(reinterpret_cast<fenv_t*>(mControlWords));
	fesetenv(FE_DFL_ENV);

#if PX_LINUX
	// need to explicitly disable exceptions because fesetenv does not modify
	// the sse control word on 32bit linux (64bit is fine, but do it here just be sure)
	fedisableexcept(FE_ALL_EXCEPT);
#endif

#endif
}

physx::shdfnd::FPUGuard::~FPUGuard()
{
#if defined(__CYGWIN__)
#pragma message "FPUGuard::~FPUGuard() is not implemented"
#elif PX_ANDROID
// not supported unless ARM_HARD_FLOAT is enabled.
#elif PX_PS4
// not supported
#elif PX_OSX
	// restore control word and clear exception flags
	// (setting exception state flags cause exceptions on the first following fp operation)
	_mm_setcsr(mControlWords[0] & ~_MM_EXCEPT_MASK);
#elif defined(__EMSCRIPTEN__)
// not supported
#else
	fesetenv(reinterpret_cast<fenv_t*>(mControlWords));
#endif
}

PX_FOUNDATION_API void physx::shdfnd::enableFPExceptions()
{
#if PX_LINUX && !defined(__EMSCRIPTEN__)
	feclearexcept(FE_ALL_EXCEPT);
	feenableexcept(FE_INVALID | FE_DIVBYZERO | FE_OVERFLOW);
#elif PX_OSX
	// clear any pending exceptions
	// (setting exception state flags cause exceptions on the first following fp operation)
	uint32_t control = _mm_getcsr() & ~_MM_EXCEPT_MASK;

	// enable all fp exceptions except inexact and underflow (common, benign)
	// note: denorm has to be disabled as well because underflow can create denorms
	_mm_setcsr((control & ~_MM_MASK_MASK) | _MM_MASK_INEXACT | _MM_MASK_UNDERFLOW | _MM_MASK_DENORM);

#endif
}

PX_FOUNDATION_API void physx::shdfnd::disableFPExceptions()
{
#if PX_LINUX && !defined(__EMSCRIPTEN__)
	fedisableexcept(FE_ALL_EXCEPT);
#elif PX_OSX
	// clear any pending exceptions
	// (setting exception state flags cause exceptions on the first following fp operation)
	uint32_t control = _mm_getcsr() & ~_MM_EXCEPT_MASK;
	_mm_setcsr(control | _MM_MASK_MASK);
#endif
}
