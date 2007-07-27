/* islessequald2 - for each of two double slots, if x <= y return mask of ones, else 0
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

#ifndef ___SIMD_MATH_ISLESSEQUALD2_H___
#define ___SIMD_MATH_ISLESSEQUALD2_H___

#include <simdmath.h>
#include <spu_intrinsics.h>

#include <simdmath/isnand2.h>

static inline vector unsigned long long
_islessequald2 (vector double x, vector double y)
{
  vec_uchar16 even = (vec_uchar16)(vec_uint4){ 0x00010203, 0x00010203, 0x08090a0b, 0x08090a0b };
  vec_uchar16 odd = (vec_uchar16)(vec_uint4){ 0x04050607, 0x04050607, 0x0c0d0e0f, 0x0c0d0e0f };
  vec_uchar16 swapEvenOdd = (vec_uchar16)(vec_uint4){ 0x04050607, 0x00010203, 0x0c0d0e0f, 0x08090a0b };
  vec_ullong2 sign = spu_splats(0x8000000000000000ull);
  vec_uint4   cmpgt_i, cmpgt_ui, cmpeq_i, cmpeq_i_even;
  vec_ullong2 cmpgt_ll, cmplt_ll, cmpeq_ll;
  vec_ullong2 bothneg, bothzero;
   
  cmpgt_i = spu_cmpgt( (vec_int4)x, (vec_int4)y );
  cmpeq_i = spu_cmpeq( (vec_int4)x, (vec_int4)y );
  cmpgt_ui = spu_cmpgt( (vec_uint4)x, (vec_uint4)y );

  cmpeq_i_even = spu_shuffle( cmpeq_i, cmpeq_i, even );
  cmpgt_ll = (vec_ullong2)spu_or( spu_shuffle( cmpgt_i, cmpgt_i, even ),
				  spu_and( cmpeq_i_even, spu_shuffle( cmpgt_ui, cmpgt_ui, odd ) ) );
  cmpeq_ll = (vec_ullong2)spu_and( cmpeq_i_even, spu_shuffle( cmpeq_i, cmpeq_i, odd ) );
  cmplt_ll = spu_nor( cmpeq_ll, cmpgt_ll );

  bothzero = spu_andc( spu_or( (vec_ullong2)x, (vec_ullong2)y ), sign );
  bothzero = (vec_ullong2)spu_cmpeq( (vec_uint4)bothzero, 0U );
  bothzero = spu_and( bothzero, spu_shuffle( bothzero, bothzero, swapEvenOdd ) );

  cmpeq_ll = spu_or( cmpeq_ll, bothzero);

  bothneg = spu_and( (vec_ullong2)x, (vec_ullong2)y );
  bothneg = (vec_ullong2)spu_cmpgt( spu_splats(0), (vec_int4)bothneg );
  bothneg = spu_shuffle( bothneg, bothneg, even );

  return spu_andc( spu_or( spu_sel( cmplt_ll, cmpgt_ll, bothneg ), cmpeq_ll), 
		   spu_or( _isnand2 ( x ), _isnand2 ( y ) ) );
}

#endif
