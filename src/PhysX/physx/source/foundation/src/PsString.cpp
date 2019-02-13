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

#include "PsString.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#if PX_WINDOWS_FAMILY
#pragma warning(push)
#pragma warning(disable : 4996) // unsafe string functions
#endif

#if PX_PS4 || PX_APPLE_FAMILY
#pragma clang diagnostic push
// error : format string is not a string literal
#pragma clang diagnostic ignored "-Wformat-nonliteral"
#endif

namespace physx
{
namespace shdfnd
{
// cross-platform implementations

int32_t strcmp(const char* str1, const char* str2)
{
	return ::strcmp(str1, str2);
}

int32_t strncmp(const char* str1, const char* str2, size_t count)
{
	return ::strncmp(str1, str2, count);
}

int32_t snprintf(char* dst, size_t dstSize, const char* format, ...)
{
	va_list arg;
	va_start(arg, format);
	int32_t r = shdfnd::vsnprintf(dst, dstSize, format, arg);
	va_end(arg);
	return r;
}

int32_t sscanf(const char* buffer, const char* format, ...)
{
	va_list arg;
	va_start(arg, format);
#if (PX_VC < 12) && !PX_LINUX
	int32_t r = ::sscanf(buffer, format, arg);
#else
	int32_t r = ::vsscanf(buffer, format, arg);
#endif
	va_end(arg);

	return r;
}

size_t strlcpy(char* dst, size_t dstSize, const char* src)
{
	size_t i = 0;
	if(dst && dstSize)
	{
		for(; i + 1 < dstSize && src[i]; i++) // copy up to dstSize-1 bytes
			dst[i] = src[i];
		dst[i] = 0; // always null-terminate
	}

	while(src[i]) // read any remaining characters in the src string to get the length
		i++;

	return i;
}

size_t strlcat(char* dst, size_t dstSize, const char* src)
{
	size_t i = 0, s = 0;
	if(dst && dstSize)
	{
		s = strlen(dst);
		for(; i + s + 1 < dstSize && src[i]; i++) // copy until total is at most dstSize-1
			dst[i + s] = src[i];
		dst[i + s] = 0; // always null-terminate
	}

	while(src[i]) // read any remaining characters in the src string to get the length
		i++;

	return i + s;
}

void strlwr(char* str)
{
	for(; *str; str++)
		if(*str >= 'A' && *str <= 'Z')
			*str += 32;
}

void strupr(char* str)
{
	for(; *str; str++)
		if(*str >= 'a' && *str <= 'z')
			*str -= 32;
}

int32_t vsnprintf(char* dst, size_t dstSize, const char* src, va_list arg)
{

#if PX_VC // MSVC is not C99-compliant...
	int32_t result = dst ? ::vsnprintf(dst, dstSize, src, arg) : -1;
	if(dst && (result == int32_t(dstSize) || result < 0))
		dst[dstSize - 1] = 0; // string was truncated or there wasn't room for the NULL
	if(result < 0)
		result = _vscprintf(src, arg); // work out how long the answer would have been.
#else
	int32_t result = ::vsnprintf(dst, dstSize, src, arg);
#endif
	return result;
}

int32_t stricmp(const char* str, const char* str1)
{
#if PX_VC
	return (::_stricmp(str, str1));
#else
	return (::strcasecmp(str, str1));
#endif
}

int32_t strnicmp(const char* str, const char* str1, size_t n)
{
#if PX_VC
	return (::_strnicmp(str, str1, n));
#else
	return (::strncasecmp(str, str1, n));
#endif
}

void printFormatted(const char* format, ...)
{
	char buf[MAX_PRINTFORMATTED_LENGTH];

	va_list arg;
	va_start(arg, format);
	vsnprintf(buf, MAX_PRINTFORMATTED_LENGTH, format, arg);
	va_end(arg);

	printString(buf);
}
}
}

#if PX_PS4 || PX_APPLE_FAMILY
#pragma clang diagnostic pop
#endif

#if PX_WINDOWS_FAMILY
#pragma warning(pop)
#endif
