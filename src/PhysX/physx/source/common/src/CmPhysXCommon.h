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


#ifndef PX_PHYSICS_COMMON
#define PX_PHYSICS_COMMON

//! \file Top level internal include file for PhysX SDK

#include "Ps.h"

// Enable debug visualization
#define PX_ENABLE_DEBUG_VISUALIZATION	1

// Enable simulation statistics generation
#define PX_ENABLE_SIM_STATS 1

// PT: typical "invalid" value in various CD algorithms
#define	PX_INVALID_U32		0xffffffff
#define PX_INVALID_U16		0xffff

// PT: this used to be replicated everywhere in the code, causing bugs to sometimes reappear (e.g. TTP 3587).
// It is better to define it in a header and use the same constant everywhere. The original value (1e-05f)
// caused troubles (e.g. TTP 1705, TTP 306).
#define PX_PARALLEL_TOLERANCE	1e-02f

namespace physx
{
	// alias shared foundation to something usable
	namespace Ps = shdfnd;
}

#if PX_CHECKED
	#define PX_CHECK_MSG(exp, msg)				(!!(exp) || (physx::shdfnd::getFoundation().error(physx::PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, msg), 0) )
	#define PX_CHECK(exp)						PX_CHECK_MSG(exp, #exp)
	#define PX_CHECK_AND_RETURN(exp,msg)		{ if(!(exp)) { PX_CHECK_MSG(exp, msg); return; } }
	#define PX_CHECK_AND_RETURN_NULL(exp,msg)	{ if(!(exp)) { PX_CHECK_MSG(exp, msg); return 0; } }
	#define PX_CHECK_AND_RETURN_VAL(exp,msg,r)	{ if(!(exp)) { PX_CHECK_MSG(exp, msg); return r; } }
#else
	#define PX_CHECK_MSG(exp, msg)
	#define PX_CHECK(exp)
	#define PX_CHECK_AND_RETURN(exp,msg)
	#define PX_CHECK_AND_RETURN_NULL(exp,msg)
	#define PX_CHECK_AND_RETURN_VAL(exp,msg,r)
#endif

#if PX_VC
	// VC compiler defines __FUNCTION__ as a string literal so it is possible to concatenate it with another string
	// Example: #define PX_CHECK_VALID(x)	PX_CHECK_MSG(physx::shdfnd::checkValid(x), __FUNCTION__ ": parameter invalid!")
	#define PX_CHECK_VALID(x)				PX_CHECK_MSG(physx::shdfnd::checkValid(x), __FUNCTION__)
#elif PX_GCC_FAMILY
	// GCC compiler defines __FUNCTION__ as a variable, hence, it is NOT possible concatenate an additional string to it
	// In GCC, __FUNCTION__ only returns the function name, using __PRETTY_FUNCTION__ will return the full function definition
	#define PX_CHECK_VALID(x)				PX_CHECK_MSG(physx::shdfnd::checkValid(x), __PRETTY_FUNCTION__)
#else
	// Generic macro for other compilers
	#define PX_CHECK_VALID(x)				PX_CHECK_MSG(physx::shdfnd::checkValid(x), __FUNCTION__)
#endif


#endif
