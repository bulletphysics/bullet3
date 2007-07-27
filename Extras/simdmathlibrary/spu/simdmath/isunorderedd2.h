/* isunorderedd2 - for each element of vector x and y, return a mask of ones if x' is unordered to y', zero otherwise
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

#ifndef ___SIMD_MATH_ISUNORDEREDD2_H___
#define ___SIMD_MATH_ISUNORDEREDD2_H___

#include <simdmath.h>
#include <spu_intrinsics.h>

static inline vector unsigned long long
_isunorderedd2 (vector double x, vector double y)
{
  vec_double2 neg;
  vec_ullong2 cmpgt, cmpeq, cmpnanx, cmpnany;
  vec_uchar16 even = (vec_uchar16)(vec_uint4){ 0x00010203, 0x00010203, 0x08090a0b, 0x08090a0b };
  vec_uchar16 odd = (vec_uchar16)(vec_uint4){ 0x04050607, 0x04050607, 0x0c0d0e0f, 0x0c0d0e0f };
  vec_ullong2 expn = (vec_ullong2)spu_splats(0xfff0000000000000ull);
  vec_ullong2 sign = (vec_ullong2)spu_splats(0x8000000000000000ull);

  //Check if x is nan
  neg = (vec_double2)spu_or( (vec_ullong2)x, sign );
  cmpgt = (vec_ullong2)spu_cmpgt( (vec_uint4)neg, (vec_uint4)expn );
  cmpeq = (vec_ullong2)spu_cmpeq( (vec_uint4)neg, (vec_uint4)expn );

  cmpnanx = spu_or( spu_shuffle( cmpgt, cmpgt, even ),
                    spu_and( spu_shuffle( cmpeq, cmpeq, even ), 
                             spu_shuffle( cmpgt, cmpgt, odd ) ) );

  //Check if y is nan
  neg = (vec_double2)spu_or( (vec_ullong2)y, sign );
  cmpgt = (vec_ullong2)spu_cmpgt( (vec_uint4)neg, (vec_uint4)expn );
  cmpeq = (vec_ullong2)spu_cmpeq( (vec_uint4)neg, (vec_uint4)expn );

  cmpnany = spu_or( spu_shuffle( cmpgt, cmpgt, even ),
                    spu_and( spu_shuffle( cmpeq, cmpeq, even ), 
                             spu_shuffle( cmpgt, cmpgt, odd ) ) );

  return spu_or( cmpnanx, cmpnany );
}

#endif
