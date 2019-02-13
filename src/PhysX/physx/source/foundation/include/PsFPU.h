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

#ifndef PSFOUNDATION_PSFPU_H
#define PSFOUNDATION_PSFPU_H

#include "Ps.h"
#include "PsIntrinsics.h"

#define PX_IR(x) ((uint32_t&)(x))
#define PX_SIR(x) ((int32_t&)(x))
#define PX_FR(x) ((float&)(x))

// signed integer representation of a floating-point value.

// Floating-point representation of a integer value.

#define PX_SIGN_BITMASK 0x80000000

#define PX_FPU_GUARD shdfnd::FPUGuard scopedFpGuard;
#define PX_SIMD_GUARD shdfnd::SIMDGuard scopedFpGuard;

#define PX_SUPPORT_GUARDS (PX_WINDOWS_FAMILY || PX_XBOXONE || (PX_LINUX && (PX_X86 || PX_X64)) || PX_PS4 || PX_OSX)

namespace physx
{
namespace shdfnd
{
// sets the default SDK state for scalar and SIMD units
class PX_FOUNDATION_API FPUGuard
{
  public:
	FPUGuard();  // set fpu control word for PhysX
	~FPUGuard(); // restore fpu control word
  private:
	uint32_t mControlWords[8];
};

// sets default SDK state for simd unit only, lighter weight than FPUGuard
class SIMDGuard
{
  public:
	PX_INLINE SIMDGuard();  // set simd control word for PhysX
	PX_INLINE ~SIMDGuard(); // restore simd control word
  private:
#if PX_SUPPORT_GUARDS
	uint32_t mControlWord;
#endif
};

/**
\brief Enables floating point exceptions for the scalar and SIMD unit
*/
PX_FOUNDATION_API void enableFPExceptions();

/**
\brief Disables floating point exceptions for the scalar and SIMD unit
*/
PX_FOUNDATION_API void disableFPExceptions();

} // namespace shdfnd
} // namespace physx

#if PX_WINDOWS_FAMILY || PX_XBOXONE
#include "windows/PsWindowsFPU.h"
#elif (PX_LINUX && PX_SSE2) || PX_PS4 || PX_OSX
#include "unix/PsUnixFPU.h"
#else
PX_INLINE physx::shdfnd::SIMDGuard::SIMDGuard()
{
}
PX_INLINE physx::shdfnd::SIMDGuard::~SIMDGuard()
{
}
#endif

#endif // #ifndef PSFOUNDATION_PSFPU_H
