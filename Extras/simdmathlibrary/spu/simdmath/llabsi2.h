/* llabsi2 - returns absolute value of input.
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

#ifndef ___SIMD_MATH_LLABSI2_H___
#define ___SIMD_MATH_LLABSI2_H___

#include <simdmath.h>
#include <spu_intrinsics.h>

static inline vector signed long long
_llabsi2 (vector signed long long in)
{
  vec_uint4 sign = (vec_uint4)spu_rlmaska((vec_int4)in, -31);
  sign = spu_shuffle(sign, sign, ((vec_uchar16){ 0,0,0,0,0,0,0,0, 8,8,8,8,8,8,8,8}));

  vec_uint4 add_1 = ((vec_uint4){0,1,0,1});
  vec_uint4 res = spu_nor((vec_uint4)in, (vec_uint4)in);
  res = spu_addx( res, add_1, spu_slqwbyte(spu_genc(res, add_1), 4));
  res = spu_sel( (vec_uint4)in, res, sign);

  return ((vec_llong2)(res));
}

#endif
