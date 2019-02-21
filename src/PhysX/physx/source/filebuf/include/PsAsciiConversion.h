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

#ifndef PSFILEBUFFER_PSASCIICONVERSION_H
#define PSFILEBUFFER_PSASCIICONVERSION_H

/*!
\file
\brief PxAsciiConversion namespace contains string/value helper functions
*/

#include "PxMath.h"
#include "PsString.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <float.h>

namespace physx
{
namespace general_string_parsing2
{
namespace PxAsc
{

const uint32_t PxF32StrLen = 24;
const uint32_t PxF64StrLen = 32;
const uint32_t IntStrLen = 32;

PX_INLINE bool isWhiteSpace(char c);
PX_INLINE const char * skipNonWhiteSpace(const char *scan);
PX_INLINE const char * skipWhiteSpace(const char *scan);

//////////////////////////
// str to value functions
//////////////////////////
PX_INLINE bool strToBool(const char *str, const char **endptr);
PX_INLINE int8_t  strToI8(const char *str, const char **endptr);
PX_INLINE int16_t strToI16(const char *str, const char **endptr);
PX_INLINE int32_t strToI32(const char *str, const char **endptr);
PX_INLINE int64_t strToI64(const char *str, const char **endptr);
PX_INLINE uint8_t  strToU8(const char *str, const char **endptr);
PX_INLINE uint16_t strToU16(const char *str, const char **endptr);
PX_INLINE uint32_t strToU32(const char *str, const char **endptr);
PX_INLINE uint64_t strToU64(const char *str, const char **endptr);
PX_INLINE float strToF32(const char *str, const char **endptr);
PX_INLINE double strToF64(const char *str, const char **endptr);
PX_INLINE void strToF32s(float *v,uint32_t count,const char *str, const char**endptr);


//////////////////////////
// value to str functions
//////////////////////////
PX_INLINE const char * valueToStr( bool val, char *buf, uint32_t n );
PX_INLINE const char * valueToStr( int8_t val, char *buf, uint32_t n );
PX_INLINE const char * valueToStr( int16_t val, char *buf, uint32_t n );
PX_INLINE const char * valueToStr( int32_t val, char *buf, uint32_t n );
PX_INLINE const char * valueToStr( int64_t val, char *buf, uint32_t n );
PX_INLINE const char * valueToStr( uint8_t val, char *buf, uint32_t n );
PX_INLINE const char * valueToStr( uint16_t val, char *buf, uint32_t n );
PX_INLINE const char * valueToStr( uint32_t val, char *buf, uint32_t n );
PX_INLINE const char * valueToStr( uint64_t val, char *buf, uint32_t n );
PX_INLINE const char * valueToStr( float val, char *buf, uint32_t n );
PX_INLINE const char * valueToStr( double val, char *buf, uint32_t n );

#include "PsAsciiConversion.inl"

} // end of namespace
} // end of namespace
using namespace general_string_parsing2;
} // end of namespace


#endif // PSFILEBUFFER_PSASCIICONVERSION_H
