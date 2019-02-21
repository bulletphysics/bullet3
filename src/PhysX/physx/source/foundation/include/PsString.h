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

#ifndef PSFOUNDATION_PSSTRING_H
#define PSFOUNDATION_PSSTRING_H

#include "foundation/PxPreprocessor.h"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxFoundationConfig.h"
#include <stdarg.h>

namespace physx
{
namespace shdfnd
{

// the following functions have C99 semantics. Note that C99 requires for snprintf and vsnprintf:
// * the resulting string is always NULL-terminated regardless of truncation.
// * in the case of truncation the return value is the number of characters that would have been created.

PX_FOUNDATION_API int32_t sscanf(const char* buffer, const char* format, ...);
PX_FOUNDATION_API int32_t strcmp(const char* str1, const char* str2);
PX_FOUNDATION_API int32_t strncmp(const char* str1, const char* str2, size_t count);
PX_FOUNDATION_API int32_t snprintf(char* dst, size_t dstSize, const char* format, ...);
PX_FOUNDATION_API int32_t vsnprintf(char* dst, size_t dstSize, const char* src, va_list arg);

// strlcat and strlcpy have BSD semantics:
// * dstSize is always the size of the destination buffer
// * the resulting string is always NULL-terminated regardless of truncation
// * in the case of truncation the return value is the length of the string that would have been created

PX_FOUNDATION_API size_t strlcat(char* dst, size_t dstSize, const char* src);
PX_FOUNDATION_API size_t strlcpy(char* dst, size_t dstSize, const char* src);

// case-insensitive string comparison
PX_FOUNDATION_API int32_t stricmp(const char* str1, const char* str2);
PX_FOUNDATION_API int32_t strnicmp(const char* str1, const char* str2, size_t count);

// in-place string case conversion
PX_FOUNDATION_API void strlwr(char* str);
PX_FOUNDATION_API void strupr(char* str);

/**
\brief The maximum supported formatted output string length
(number of characters after replacement).

@see printFormatted()
*/
static const size_t MAX_PRINTFORMATTED_LENGTH = 1024;

/**
\brief Prints the formatted data, trying to make sure it's visible to the app programmer

@see NS_MAX_PRINTFORMATTED_LENGTH
*/
PX_FOUNDATION_API void printFormatted(const char*, ...);

/**
\brief Prints the string literally (does not consume % specifier), trying to make sure it's visible to the app
programmer
*/
PX_FOUNDATION_API void printString(const char*);
}
}
#endif // #ifndef PSFOUNDATION_PSSTRING_H
