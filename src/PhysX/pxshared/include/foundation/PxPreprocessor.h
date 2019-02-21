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

#ifndef PXFOUNDATION_PXPREPROCESSOR_H
#define PXFOUNDATION_PXPREPROCESSOR_H

#include <stddef.h>
#if !defined(PX_GENERATE_META_DATA)
#include <ciso646>  
#endif
/** \addtogroup foundation
  @{
*/

#define PX_STRINGIZE_HELPER(X) #X
#define PX_STRINGIZE(X) PX_STRINGIZE_HELPER(X)

#define PX_CONCAT_HELPER(X, Y) X##Y
#define PX_CONCAT(X, Y) PX_CONCAT_HELPER(X, Y)

/*
The following preprocessor identifiers specify compiler, OS, and architecture.
All definitions have a value of 1 or 0, use '#if' instead of '#ifdef'.
*/

/**
Compiler defines, see http://sourceforge.net/p/predef/wiki/Compilers/
*/
#if defined(_MSC_VER)
#if _MSC_VER >= 1910
#define PX_VC 15
#elif _MSC_VER >= 1900
#define PX_VC 14
#elif _MSC_VER >= 1800
#define PX_VC 12
#elif _MSC_VER >= 1700
#define PX_VC 11
#elif _MSC_VER >= 1600
#define PX_VC 10
#elif _MSC_VER >= 1500
#define PX_VC 9
#else
#error "Unknown VC version"
#endif
#elif defined(__clang__)
#define PX_CLANG 1
#elif defined(__GNUC__) // note: __clang__ implies __GNUC__
#define PX_GCC 1
#else
#error "Unknown compiler"
#endif

/**
Operating system defines, see http://sourceforge.net/p/predef/wiki/OperatingSystems/
*/
#if defined(_XBOX_ONE)
#define PX_XBOXONE 1
#elif defined(WINAPI_FAMILY) && WINAPI_FAMILY == WINAPI_FAMILY_APP
#define PX_UWP 1
#elif defined(_WIN64) // note: _XBOX_ONE implies _WIN64
#define PX_WIN64 1
#elif defined(_WIN32) // note: _M_PPC implies _WIN32
#define PX_WIN32 1
#elif defined(__ANDROID__)
#define PX_ANDROID 1
#elif defined(__linux__) || defined (__EMSCRIPTEN__) // note: __ANDROID__ implies __linux__
#define PX_LINUX 1
#elif defined(__APPLE__) && (defined(__arm__) || defined(__arm64__))
#define PX_IOS 1
#elif defined(__APPLE__)
#define PX_OSX 1
#elif defined(__ORBIS__)
#define PX_PS4 1
#elif defined(__NX__)
#define PX_SWITCH 1
#else
#error "Unknown operating system"
#endif

/**
Architecture defines, see http://sourceforge.net/p/predef/wiki/Architectures/
*/
#if defined(__x86_64__) || defined(_M_X64) // ps4 compiler defines _M_X64 without value
#define PX_X64 1
#elif defined(__i386__) || defined(_M_IX86) || defined (__EMSCRIPTEN__)
#define PX_X86 1
#elif defined(__arm64__) || defined(__aarch64__)
#define PX_A64 1
#elif defined(__arm__) || defined(_M_ARM)
#define PX_ARM 1
#elif defined(__ppc__) || defined(_M_PPC) || defined(__CELLOS_LV2__)
#define PX_PPC 1
#else
#error "Unknown architecture"
#endif

/**
SIMD defines
*/
#if !defined(PX_SIMD_DISABLED)
#if defined(__i386__) || defined(_M_IX86) || defined(__x86_64__) || defined(_M_X64) || (defined (__EMSCRIPTEN__) && defined(__SSE2__))
#define PX_SSE2 1
#endif
#if defined(_M_ARM) || defined(__ARM_NEON__) || defined(__ARM_NEON)
#define PX_NEON 1
#endif
#if defined(_M_PPC) || defined(__CELLOS_LV2__)
#define PX_VMX 1
#endif
#endif

/**
define anything not defined on this platform to 0
*/
#ifndef PX_VC
#define PX_VC 0
#endif
#ifndef PX_CLANG
#define PX_CLANG 0
#endif
#ifndef PX_GCC
#define PX_GCC 0
#endif
#ifndef PX_XBOXONE
#define PX_XBOXONE 0
#endif
#ifndef PX_WIN64
#define PX_WIN64 0
#endif
#ifndef PX_WIN32
#define PX_WIN32 0
#endif
#ifndef PX_ANDROID
#define PX_ANDROID 0
#endif
#ifndef PX_LINUX
#define PX_LINUX 0
#endif
#ifndef PX_IOS
#define PX_IOS 0
#endif
#ifndef PX_OSX
#define PX_OSX 0
#endif
#ifndef PX_PS4
#define PX_PS4 0
#endif
#ifndef PX_SWITCH
#define PX_SWITCH 0
#endif
#ifndef PX_UWP
#define PX_UWP 0
#endif
#ifndef PX_X64
#define PX_X64 0
#endif
#ifndef PX_X86
#define PX_X86 0
#endif
#ifndef PX_A64
#define PX_A64 0
#endif
#ifndef PX_ARM
#define PX_ARM 0
#endif
#ifndef PX_PPC
#define PX_PPC 0
#endif
#ifndef PX_SSE2
#define PX_SSE2 0
#endif
#ifndef PX_NEON
#define PX_NEON 0
#endif
#ifndef PX_VMX
#define PX_VMX 0
#endif

/*
define anything not defined through the command line to 0
*/
#ifndef PX_DEBUG
#define PX_DEBUG 0
#endif
#ifndef PX_CHECKED
#define PX_CHECKED 0
#endif
#ifndef PX_PROFILE
#define PX_PROFILE 0
#endif
#ifndef PX_DEBUG_CRT
#define PX_DEBUG_CRT 0
#endif
#ifndef PX_NVTX
#define PX_NVTX 0
#endif
#ifndef PX_DOXYGEN
#define PX_DOXYGEN 0
#endif

/**
family shortcuts
*/
// compiler
#define PX_GCC_FAMILY (PX_CLANG || PX_GCC)
// os
#define PX_WINDOWS_FAMILY (PX_WIN32 || PX_WIN64 || PX_UWP)
#define PX_MICROSOFT_FAMILY (PX_XBOXONE || PX_WINDOWS_FAMILY)
#define PX_LINUX_FAMILY (PX_LINUX || PX_ANDROID)
#define PX_APPLE_FAMILY (PX_IOS || PX_OSX)                  // equivalent to #if __APPLE__
#define PX_UNIX_FAMILY (PX_LINUX_FAMILY || PX_APPLE_FAMILY) // shortcut for unix/posix platforms
#if defined(__EMSCRIPTEN__)
#define PX_EMSCRIPTEN 1
#else
#define PX_EMSCRIPTEN 0
#endif
// architecture
#define PX_INTEL_FAMILY (PX_X64 || PX_X86)
#define PX_ARM_FAMILY (PX_ARM || PX_A64)
#define PX_P64_FAMILY (PX_X64 || PX_A64) // shortcut for 64-bit architectures

/**
C++ standard library defines
*/
#if defined(_LIBCPP_VERSION) || PX_WIN64 || PX_WIN32 || PX_PS4 || PX_XBOXONE || PX_UWP || PX_EMSCRIPTEN
#define PX_LIBCPP 1
#else
#define PX_LIBCPP 0
#endif

// legacy define for PhysX
#define PX_WINDOWS (PX_WINDOWS_FAMILY && !PX_ARM_FAMILY)

/**
Assert macro
*/
#ifndef PX_ENABLE_ASSERTS
#if PX_DEBUG && !defined(__CUDACC__)
#define PX_ENABLE_ASSERTS 1
#else
#define PX_ENABLE_ASSERTS 0
#endif
#endif

/**
DLL export macros
*/
#ifndef PX_C_EXPORT
#if PX_WINDOWS_FAMILY || PX_LINUX
#define PX_C_EXPORT extern "C"
#else
#define PX_C_EXPORT
#endif
#endif

#if PX_UNIX_FAMILY&& __GNUC__ >= 4
#define PX_UNIX_EXPORT __attribute__((visibility("default")))
#else
#define PX_UNIX_EXPORT
#endif

#if PX_WINDOWS_FAMILY
#define PX_DLL_EXPORT __declspec(dllexport)
#define PX_DLL_IMPORT __declspec(dllimport)
#else
#define PX_DLL_EXPORT PX_UNIX_EXPORT
#define PX_DLL_IMPORT
#endif

/**
Calling convention
*/
#ifndef PX_CALL_CONV
#if PX_MICROSOFT_FAMILY
#define PX_CALL_CONV __cdecl
#else
#define PX_CALL_CONV
#endif
#endif

/**
Pack macros - disabled on SPU because they are not supported
*/
#if PX_VC
#define PX_PUSH_PACK_DEFAULT __pragma(pack(push, 8))
#define PX_POP_PACK __pragma(pack(pop))
#elif PX_GCC_FAMILY
#define PX_PUSH_PACK_DEFAULT _Pragma("pack(push, 8)")
#define PX_POP_PACK _Pragma("pack(pop)")
#else
#define PX_PUSH_PACK_DEFAULT
#define PX_POP_PACK
#endif

/**
Inline macro
*/
#define PX_INLINE inline
#if PX_MICROSOFT_FAMILY
#pragma inline_depth(255)
#endif

/**
Force inline macro
*/
#if PX_VC
#define PX_FORCE_INLINE __forceinline
#elif PX_LINUX // Workaround; Fedora Core 3 do not agree with force inline and PxcPool
#define PX_FORCE_INLINE inline
#elif PX_GCC_FAMILY
#define PX_FORCE_INLINE inline __attribute__((always_inline))
#else
#define PX_FORCE_INLINE inline
#endif

/**
Noinline macro
*/
#if PX_MICROSOFT_FAMILY
#define PX_NOINLINE __declspec(noinline)
#elif PX_GCC_FAMILY
#define PX_NOINLINE __attribute__((noinline))
#else
#define PX_NOINLINE
#endif

/**
Restrict macro
*/
#if defined(__CUDACC__)
#define PX_RESTRICT __restrict__
#else
#define PX_RESTRICT __restrict
#endif

/**
Noalias macro
*/
#if PX_MICROSOFT_FAMILY
#define PX_NOALIAS __declspec(noalias)
#else
#define PX_NOALIAS
#endif

/**
Alignment macros

PX_ALIGN_PREFIX and PX_ALIGN_SUFFIX can be used for type alignment instead of aligning individual variables as follows:
PX_ALIGN_PREFIX(16)
struct A {
...
} PX_ALIGN_SUFFIX(16);
This declaration style is parsed correctly by Visual Assist.

*/
#ifndef PX_ALIGN
#if PX_MICROSOFT_FAMILY
#define PX_ALIGN(alignment, decl) __declspec(align(alignment)) decl
#define PX_ALIGN_PREFIX(alignment) __declspec(align(alignment))
#define PX_ALIGN_SUFFIX(alignment)
#elif PX_GCC_FAMILY
#define PX_ALIGN(alignment, decl) decl __attribute__((aligned(alignment)))
#define PX_ALIGN_PREFIX(alignment)
#define PX_ALIGN_SUFFIX(alignment) __attribute__((aligned(alignment)))
#elif defined __CUDACC__
#define PX_ALIGN(alignment, decl) __align__(alignment) decl
#define PX_ALIGN_PREFIX(alignment)
#define PX_ALIGN_SUFFIX(alignment) __align__(alignment))
#else
#define PX_ALIGN(alignment, decl)
#define PX_ALIGN_PREFIX(alignment)
#define PX_ALIGN_SUFFIX(alignment)
#endif
#endif

/**
Deprecated macro
- To deprecate a function: Place PX_DEPRECATED at the start of the function header (leftmost word).
- To deprecate a 'typedef', a 'struct' or a 'class': Place PX_DEPRECATED directly after the keywords ('typdef',
'struct', 'class').

Use these macro definitions to create warnings for deprecated functions
\#define PX_DEPRECATED __declspec(deprecated) // Microsoft
\#define PX_DEPRECATED __attribute__((deprecated())) // GCC
*/
#define PX_DEPRECATED

/**
General defines
*/

// static assert
#if(defined(__GNUC__) && (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 7))) || (PX_PS4) || (PX_APPLE_FAMILY) || (PX_SWITCH) || (PX_CLANG && PX_ARM)
#define PX_COMPILE_TIME_ASSERT(exp) typedef char PX_CONCAT(PxCompileTimeAssert_Dummy, __COUNTER__)[(exp) ? 1 : -1] __attribute__((unused))
#else
#define PX_COMPILE_TIME_ASSERT(exp) typedef char PxCompileTimeAssert_Dummy[(exp) ? 1 : -1]
#endif

#if PX_GCC_FAMILY
#define PX_OFFSET_OF(X, Y) __builtin_offsetof(X, Y)
#else
#define PX_OFFSET_OF(X, Y) offsetof(X, Y)
#endif

#define PX_OFFSETOF_BASE 0x100 // casting the null ptr takes a special-case code path, which we don't want
#define PX_OFFSET_OF_RT(Class, Member)                                                                                 \
	(reinterpret_cast<size_t>(&reinterpret_cast<Class*>(PX_OFFSETOF_BASE)->Member) - size_t(PX_OFFSETOF_BASE))

// check that exactly one of NDEBUG and _DEBUG is defined
#if !defined(NDEBUG) ^ defined(_DEBUG)
#error Exactly one of NDEBUG and _DEBUG needs to be defined!
#endif

// make sure PX_CHECKED is defined in all _DEBUG configurations as well
#if !PX_CHECKED && PX_DEBUG
#error PX_CHECKED must be defined when PX_DEBUG is defined
#endif

#ifdef __CUDACC__
#define PX_CUDA_CALLABLE __host__ __device__
#else
#define PX_CUDA_CALLABLE
#endif

// avoid unreferenced parameter warning
// preferred solution: omit the parameter's name from the declaration
template <class T>
PX_CUDA_CALLABLE PX_INLINE void PX_UNUSED(T const&)
{
}

// Ensure that the application hasn't tweaked the pack value to less than 8, which would break
// matching between the API headers and the binaries
// This assert works on win32/win64, but may need further specialization on other platforms.
// Some GCC compilers need the compiler flag -malign-double to be set.
// Apparently the apple-clang-llvm compiler doesn't support malign-double.
#if PX_PS4 || PX_APPLE_FAMILY || (PX_CLANG && !PX_ARM)
struct PxPackValidation
{
	char _;
	long a;
};
#elif PX_ANDROID || (PX_CLANG && PX_ARM)
struct PxPackValidation
{
	char _;
	double a;
};
#else
struct PxPackValidation
{
	char _;
	long long a;
};
#endif
// clang (as of version 3.9) cannot align doubles on 8 byte boundary  when compiling for Intel 32 bit target
#if !PX_APPLE_FAMILY && !PX_EMSCRIPTEN && !(PX_CLANG && PX_X86)
PX_COMPILE_TIME_ASSERT(PX_OFFSET_OF(PxPackValidation, a) == 8);
#endif

// use in a cpp file to suppress LNK4221
#if PX_VC
#define PX_DUMMY_SYMBOL                                                                                                \
	namespace                                                                                                          \
	{                                                                                                                  \
	char PxDummySymbol;                                                                                                \
	}
#else
#define PX_DUMMY_SYMBOL
#endif

#if PX_GCC_FAMILY
#define PX_WEAK_SYMBOL __attribute__((weak)) // this is to support SIMD constant merging in template specialization
#else
#define PX_WEAK_SYMBOL
#endif

// Macro for avoiding default assignment and copy, because doing this by inheritance can increase class size on some
// platforms.
#define PX_NOCOPY(Class)                                                                                               \
	\
protected:                                                                                                             \
	Class(const Class&);                                                                                               \
	Class& operator=(const Class&);

#ifndef DISABLE_CUDA_PHYSX
//CUDA is currently supported only on windows 
#define PX_SUPPORT_GPU_PHYSX ((PX_WINDOWS_FAMILY) || (PX_LINUX && PX_X64))
#else
#define PX_SUPPORT_GPU_PHYSX 0
#endif

#define PX_SUPPORT_COMPUTE_PHYSX 0

#ifndef PX_SUPPORT_EXTERN_TEMPLATE
#define PX_SUPPORT_EXTERN_TEMPLATE ((!PX_ANDROID) && (PX_VC != 11))
#else
#define PX_SUPPORT_EXTERN_TEMPLATE 0
#endif

/** @} */
#endif // #ifndef PXFOUNDATION_PXPREPROCESSOR_H
