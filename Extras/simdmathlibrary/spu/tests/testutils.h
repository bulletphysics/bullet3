/* Common part for SPU SIMD Math library testsuite
   Copyright (C) 2006, 2007 Sony Computer Entertainment Inc.
   All rights reserved.

   Redistribution and use in source and binary forms,
   with or without modification, are permitted provided that the
   following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Sony Computer Entertainment Inc nor the names
      of its contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
 */




#ifndef _TESTUTILS_H_

#include <spu_intrinsics.h>

#define NANF __builtin_nanf("")

extern unsigned int hide_uint( unsigned int x );
extern int hide_int( int x );
extern float hide_float( float x );
extern double hide_double( double x );

extern float make_float( unsigned int x );
extern unsigned int make_uint( float x );
extern double make_double( unsigned long long x );
extern unsigned long long make_ulonglong( double x );

extern vec_uint4 bitDiff_f4(vec_float4 ref, vec_float4 vals);
extern unsigned int bitDiff_f(float ref, float val);

extern vec_ullong2 bitDiff_d2(vec_double2 ref, vec_double2 vals);
extern unsigned long long bitDiff_d(double ref, double val);

extern vec_uint4 ulpDiff_f4(vec_float4 ref, vec_float4 vals);
extern unsigned int ulpDiff_f(float ref, float val);

extern vec_ullong2 ulpDiff_d2(vec_double2 ref, vec_double2 vals);
extern unsigned long long ulpDiff_d(double ref, double val);

extern vec_ullong2 cmpposzerod2( vec_double2 x );
extern vec_ullong2 cmpnegzerod2( vec_double2 x );
extern int allequal_int4( vec_int4 x, vec_int4 y );
static inline int allequal_uint4( vec_uint4 x, vec_uint4 y )
{
  return allequal_int4 ((vec_int4)x, (vec_int4)y);
}
extern int allequal_llong2( vec_llong2 x, vec_llong2 y );
static inline int allequal_ullong2( vec_ullong2 x, vec_ullong2 y )
{
  return allequal_llong2((vec_llong2)x, (vec_llong2)y);
}
extern int allequal_float4( vec_float4 x, vec_float4 y );
extern int allequal_double2( vec_double2 x, vec_double2 y );
extern int allequal_llroundf4( llroundf4_t x, llroundf4_t y );
extern int allequal_ulps_float4( vec_float4 x, vec_float4 y, int tolerance );
extern int allequal_ulps_double2( vec_double2 x, vec_double2 y, int tolerance );
extern int allequal_bits_float4( vec_float4 x, vec_float4 y, int tolerance );
extern int allequal_bits_double2( vec_double2 x, vec_double2 y, int tolerance );
extern int allposinf_double2( vec_double2 x );
extern int allneginf_double2( vec_double2 x );
extern int allzerodenorm_double2( vec_double2 x );
extern int allposzero_double2( vec_double2 x );
extern int allnegzero_double2( vec_double2 x );
extern int allnan_double2( vec_double2 x );

#endif
