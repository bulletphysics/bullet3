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


#ifndef PX_PHYSICS_COMMON_NX
#define PX_PHYSICS_COMMON_NX

/** \addtogroup common 
@{ */

#include "foundation/Px.h"

/*Disable support for VS2017 prior version 15.5.1 for windows platform, because of a compiler bug:
https://developercommunity.visualstudio.com/content/problem/66047/possible-compiler-bug.html
*/
#if (PX_VC == 15) && PX_WINDOWS && (_MSC_FULL_VER < 191225830)
#error Visual studio 2017 prior to 15.5.1 is not supported because of a compiler bug.
#endif

// define API function declaration (public API only needed because of extensions)
#if defined PX_PHYSX_STATIC_LIB
	#define PX_PHYSX_CORE_API
#else
	#if PX_WINDOWS
		#if defined PX_PHYSX_CORE_EXPORTS
			#define PX_PHYSX_CORE_API __declspec(dllexport)
		#else
			#define PX_PHYSX_CORE_API __declspec(dllimport)
		#endif
	#elif PX_UNIX_FAMILY
		#define PX_PHYSX_CORE_API PX_UNIX_EXPORT
    #else
		#define PX_PHYSX_CORE_API
    #endif
#endif

#if PX_SUPPORT_GPU_PHYSX
// define API function declaration
#if defined PX_PHYSX_GPU_STATIC
	#define PX_PHYSX_GPU_API
#else
	#if PX_WINDOWS 
	#if defined PX_PHYSX_GPU_EXPORTS
	#define PX_PHYSX_GPU_API __declspec(dllexport)
	#else
	#define PX_PHYSX_GPU_API __declspec(dllimport)
	#endif
	#elif PX_UNIX_FAMILY
	#define PX_PHYSX_GPU_API PX_UNIX_EXPORT
	#else
	#define PX_PHYSX_GPU_API
	#endif
#endif

#else // PX_SUPPORT_GPU_PHYSX
#define PX_PHYSX_GPU_API
#endif // PX_SUPPORT_GPU_PHYSX

#if defined PX_PHYSX_STATIC_LIB
	#define PX_PHYSX_COMMON_API
#else
	#if PX_WINDOWS && !defined(__CUDACC__)
		#if defined PX_PHYSX_COMMON_EXPORTS
			#define PX_PHYSX_COMMON_API __declspec(dllexport)
		#else
			#define PX_PHYSX_COMMON_API __declspec(dllimport)
		#endif
	#elif PX_UNIX_FAMILY
		#define PX_PHYSX_COMMON_API PX_UNIX_EXPORT
	#else
		#define PX_PHYSX_COMMON_API
	#endif
#endif 

// Changing these parameters requires recompilation of the SDK

#if !PX_DOXYGEN
namespace physx
{
#endif
	class PxCollection;
	class PxBase;

	class PxHeightField;
	class PxHeightFieldDesc;

	class PxTriangleMesh;
	class PxConvexMesh;

	typedef PxU32 PxTriangleID;
	typedef PxU16 PxMaterialTableIndex;

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
