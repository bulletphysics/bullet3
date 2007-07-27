/* nextafterd2 - find next representable floating-point value towards 2nd param.
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

#ifndef ___SIMD_MATH_NEXTAFTERD2_H___
#define ___SIMD_MATH_NEXTAFTERD2_H___

#include <simdmath.h>
#include <spu_intrinsics.h>

static inline vector double
_nextafterd2 (vector double xx, vector double yy)
{
  vec_uint4 abs_x, abs_y, sign_x, abs_dif;
  vec_uint4 is_sub, is_zerox, is_zeroy;
  vec_uint4 is_equal, is_infy, is_nany;
  vec_uint4 res0, res1, res;
  vec_uint4 vec_zero = ((vec_uint4){0,0,0,0});
  vec_uint4 vec_one  = ((vec_uint4){0,1,0,1});
  vec_uint4 vec_m1   = ((vec_uint4){0x80000000,1,0x80000000,1});
  vec_uint4 msk_exp  = ((vec_uint4){0x7FF00000,0,0x7FF00000,0});
  vec_uint4 msk_abs  = ((vec_uint4){0x7FFFFFFF,-1,0x7FFFFFFF,-1});
  vec_uchar16 msk_all_eq = ((vec_uchar16){4,5,6,7,0,1,2,3,12,13,14,15,8,9,10,11});

  // mask sign bit
  abs_x = spu_and( (vec_uint4)xx, msk_abs);
  abs_y = spu_and( (vec_uint4)yy, msk_abs);

  is_zerox = spu_cmpeq( abs_x, vec_zero);
  is_zerox = spu_and( is_zerox, spu_shuffle(is_zerox,is_zerox,msk_all_eq));

  // -0 exception
  sign_x = spu_and((vec_uint4)xx, ((vec_uint4){0x80000000,0,0x80000000,0}));
  sign_x = spu_sel(sign_x, vec_zero, is_zerox);

  // if same sign |y| < |x| -> decrease 
  abs_dif = spu_subx(abs_y, abs_x, spu_rlqwbyte(spu_genb(abs_y, abs_x), 4));
  is_sub = spu_xor((vec_uint4)yy, sign_x);	// not same sign -> decrease
  is_sub = spu_or(is_sub, abs_dif);
  is_sub = spu_rlmaska(is_sub, -31);
  is_sub = spu_shuffle(is_sub,is_sub,((vec_uchar16){0,0,0,0,0,0,0,0,8,8,8,8,8,8,8,8}));

  res0 = spu_addx( abs_x, vec_one, spu_rlqwbyte(spu_genc(abs_x,vec_one),4)); // calc increase
  res1 = spu_subx( abs_x, vec_one, spu_rlqwbyte(spu_genb(abs_x,vec_one),4)); // calc decrease
  res  = spu_sel( res0, res1, is_sub);	// select increase or decrease
  res  = spu_or( res, sign_x);			// set sign

  // check exception
  // 0 -> -1
  res = spu_sel(res, vec_m1, spu_and(is_zerox, is_sub));

  // check equal (include 0,-0)
  is_zeroy = spu_cmpeq( abs_y, vec_zero);
  is_zeroy = spu_and( is_zeroy, spu_shuffle(is_zeroy,is_zeroy,msk_all_eq));
  is_equal = spu_cmpeq((vec_uint4)xx, (vec_uint4)yy);
  is_equal = spu_and(is_equal, spu_shuffle(is_equal,is_equal,msk_all_eq));
  is_equal = spu_or(is_equal, spu_and(is_zeroy, is_zerox));
  res = spu_sel(res, (vec_uint4)yy, is_equal);

  // check nan
  is_infy = spu_cmpeq( abs_y, msk_exp);
  is_infy = spu_and( is_infy, spu_shuffle(is_infy,is_infy,msk_all_eq));
  is_nany = spu_and( abs_y, msk_exp);
  is_nany = spu_cmpeq( is_nany, msk_exp);
  is_nany = spu_and( is_nany, spu_shuffle(is_nany,is_nany,msk_all_eq));
  is_nany = spu_sel( is_nany, vec_zero, is_infy);
  res = spu_sel(res, (vec_uint4)yy, is_nany);

  return (vec_double2)res;
}

#endif
