/* Header file for common parts of the testsuite
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


/* FIXME: Don't use altivec style initializers. */

#ifndef _TESTUTILS_H_

#include <altivec.h>
#define vec_uchar16             vector unsigned char
#define vec_char16              vector   signed char
#define vec_ushort8             vector unsigned short
#define vec_short8              vector   signed short
#define vec_uint4               vector unsigned int
#define vec_int4                vector   signed int
#define vec_ullong2             vector unsigned long long
#define vec_llong2              vector   signed long long
#define vec_float4              vector          float
#define vec_double2             vector          double

extern unsigned int hide_uint( unsigned int x );
extern int hide_int( int x );
extern float hide_float( float x );
extern float make_float( unsigned int x );

inline vec_int4 vec_splat_int( int x )
{
  return (vec_int4){x, x, x, x};
}

inline vec_float4 vec_splat_float( float x )
{
  return (vec_float4){x, x, x, x};
}

inline vec_uint4 bitDiff_f4(vec_float4 ref, vec_float4 vals) {
  vec_int4 refi  = (vec_int4)ref;
  vec_int4 valsi = (vec_int4)vals;
  vec_int4 diff  = vec_sub( refi, valsi );
  vec_int4 negdiff = vec_sub( vec_splat_int(0), diff );
  vec_uint4 lz;

  diff = vec_sel( negdiff, diff, vec_cmpgt( diff, vec_splat_int(0) ) );

  lz = vec_sub( (vec_uint4)vec_splat_int(158), vec_sr( (vec_uint4)vec_ctf( diff, 0 ), (vec_uint4)vec_splat_int(23) ) );
  lz = vec_sel( lz, (vec_uint4)vec_splat_int(32), vec_cmpeq( diff, vec_splat_int(0) ) );

  return vec_sub( (vec_uint4)vec_splat_int(32), lz );
}

inline vec_uint4 ulpDiff_f4(vec_float4 ref, vec_float4 vals) {
  vec_int4 refi  = (vec_int4)ref;
  vec_int4 valsi = (vec_int4)vals;
  vec_int4 diff  = vec_sub( refi, valsi );
  vec_int4 negdiff = vec_sub( vec_splat_int(0), diff );

  return (vec_uint4)( vec_sel( negdiff, diff, vec_cmpgt( diff, vec_splat_int(0) ) ) );
}

inline int allequal_int4( vec_int4 x, vec_int4 y )
{
  return ( vec_all_eq( x, y ) );
}

inline int allequal_float4( vec_float4 x, vec_float4 y )
{
  return ( vec_all_eq( x, y ) );
}

inline int allequal_ulps_float4( vec_float4 x, vec_float4 y, int tolerance )
{  
  vec_uint4 vtol = (vec_uint4)vec_splat_int( tolerance );
  vec_uint4 ulps = ulpDiff_f4( x, y );
  return vec_all_le( ulps, vtol );
}

inline int allequal_bits_float4( vec_float4 x, vec_float4 y, int tolerance )
{
  vec_uint4 vtol = (vec_uint4)vec_splat_int( tolerance );
  vec_uint4 bits = bitDiff_f4( x, y );
  return vec_all_le( bits, vtol );
}

#endif
