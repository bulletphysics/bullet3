/* rintd2 - Round the input to the nearest integer according to
            the current rounding mode.
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

#ifndef ___SIMD_MATH_RINTD2_H___
#define ___SIMD_MATH_RINTD2_H___

#include <simdmath.h>
#include <spu_intrinsics.h>

static inline vector double 
_rintd2(vector double in)
{
  vec_ullong2 sign = ((vec_ullong2){0x8000000000000000ULL,0x8000000000000000ULL});
  vec_uint4 vec_norm = ((vec_uint4){0x00100000,0,0x00100000,0});
  vec_uint4 vec_bias = ((vec_uint4){0x43300000,0,0x43300000,0});
  vec_double2 addend, xx;
  vec_uint4 abs_x;
  vec_uint4 is_zerox;
  vec_uint4 is_denorm;
  vec_uint4 ofs;

  abs_x     = spu_andc((vec_uint4)in, (vec_uint4)sign);

  // check denormalized
  is_zerox  = spu_cmpeq( abs_x, 0);
  is_denorm = spu_cmpgt( vec_norm, abs_x );
  is_zerox  = spu_and( is_zerox, spu_shuffle(is_zerox,is_zerox, ((vec_uchar16){4,5,6,7,0,1,2,3,12,13,14,15,8,9,10,11})));
  is_denorm = spu_andc(is_denorm, is_zerox);
  ofs       = spu_and( vec_norm, is_denorm);

  xx = spu_or( in, (vec_double2)ofs );

  /* Add 2^53 and then subtract 2^53 to affect a round to be performed by the
   * hardware. Also preserve the input sign so that negative inputs that 
   * round to zero generate a -0.0.
   */
  addend = (vec_double2)spu_and(vec_bias, spu_cmpgt( vec_bias, abs_x ));
  addend = spu_sel(addend, in, sign);

  return spu_sel(spu_sub(spu_add(xx, addend), addend), in, sign);


}

#endif
