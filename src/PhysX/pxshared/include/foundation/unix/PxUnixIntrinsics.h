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

#ifndef PXFOUNDATION_PXUNIXINTRINSICS_H
#define PXFOUNDATION_PXUNIXINTRINSICS_H

#include "foundation/Px.h"
#include "foundation/PxSharedAssert.h"

#if !(PX_LINUX || PX_ANDROID || PX_PS4 || PX_APPLE_FAMILY)
#error "This file should only be included by Unix builds!!"
#endif

#if (PX_LINUX || PX_ANDROID) && !defined(__CUDACC__) && !PX_EMSCRIPTEN
    // Linux/android and CUDA compilation does not work with std::isfnite, as it is not marked as CUDA callable
    #include <cmath>
    #ifndef isfinite
        using std::isfinite;
    #endif
#endif

#include <math.h>
#include <float.h>

namespace physx
{
namespace intrinsics
{
//! \brief platform-specific absolute value
PX_CUDA_CALLABLE PX_FORCE_INLINE float abs(float a)
{
	return ::fabsf(a);
}

//! \brief platform-specific select float
PX_CUDA_CALLABLE PX_FORCE_INLINE float fsel(float a, float b, float c)
{
	return (a >= 0.0f) ? b : c;
}

//! \brief platform-specific sign
PX_CUDA_CALLABLE PX_FORCE_INLINE float sign(float a)
{
	return (a >= 0.0f) ? 1.0f : -1.0f;
}

//! \brief platform-specific reciprocal
PX_CUDA_CALLABLE PX_FORCE_INLINE float recip(float a)
{
	return 1.0f / a;
}

//! \brief platform-specific reciprocal estimate
PX_CUDA_CALLABLE PX_FORCE_INLINE float recipFast(float a)
{
	return 1.0f / a;
}

//! \brief platform-specific square root
PX_CUDA_CALLABLE PX_FORCE_INLINE float sqrt(float a)
{
	return ::sqrtf(a);
}

//! \brief platform-specific reciprocal square root
PX_CUDA_CALLABLE PX_FORCE_INLINE float recipSqrt(float a)
{
	return 1.0f / ::sqrtf(a);
}

PX_CUDA_CALLABLE PX_FORCE_INLINE float recipSqrtFast(float a)
{
	return 1.0f / ::sqrtf(a);
}

//! \brief platform-specific sine
PX_CUDA_CALLABLE PX_FORCE_INLINE float sin(float a)
{
	return ::sinf(a);
}

//! \brief platform-specific cosine
PX_CUDA_CALLABLE PX_FORCE_INLINE float cos(float a)
{
	return ::cosf(a);
}

//! \brief platform-specific minimum
PX_CUDA_CALLABLE PX_FORCE_INLINE float selectMin(float a, float b)
{
	return a < b ? a : b;
}

//! \brief platform-specific maximum
PX_CUDA_CALLABLE PX_FORCE_INLINE float selectMax(float a, float b)
{
	return a > b ? a : b;
}

//! \brief platform-specific finiteness check (not INF or NAN)
PX_CUDA_CALLABLE PX_FORCE_INLINE bool isFinite(float a)
{
	//std::isfinite not recommended as of Feb 2017, since it doesn't work with g++/clang's floating point optimization.
    union localU { PxU32 i; float f; } floatUnion;
    floatUnion.f = a;
    return !((floatUnion.i & 0x7fffffff) >= 0x7f800000);
}

//! \brief platform-specific finiteness check (not INF or NAN)
PX_CUDA_CALLABLE PX_FORCE_INLINE bool isFinite(double a)
{
	return !!isfinite(a);
}

/*!
Sets \c count bytes starting at \c dst to zero.
*/
PX_FORCE_INLINE void* memZero(void* dest, uint32_t count)
{
	return memset(dest, 0, count);
}

/*!
Sets \c count bytes starting at \c dst to \c c.
*/
PX_FORCE_INLINE void* memSet(void* dest, int32_t c, uint32_t count)
{
	return memset(dest, c, count);
}

/*!
Copies \c count bytes from \c src to \c dst. User memMove if regions overlap.
*/
PX_FORCE_INLINE void* memCopy(void* dest, const void* src, uint32_t count)
{
	return memcpy(dest, src, count);
}

/*!
Copies \c count bytes from \c src to \c dst. Supports overlapping regions.
*/
PX_FORCE_INLINE void* memMove(void* dest, const void* src, uint32_t count)
{
	return memmove(dest, src, count);
}

/*!
Set 128B to zero starting at \c dst+offset. Must be aligned.
*/
PX_FORCE_INLINE void memZero128(void* dest, uint32_t offset = 0)
{
	PX_SHARED_ASSERT(((size_t(dest) + offset) & 0x7f) == 0);
	memSet(reinterpret_cast<char*>(dest) + offset, 0, 128);
}

} // namespace intrinsics
} // namespace physx

#endif // #ifndef PXFOUNDATION_PXUNIXINTRINSICS_H
