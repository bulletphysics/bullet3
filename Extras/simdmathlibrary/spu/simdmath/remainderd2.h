/* A vector double is returned that contains the remainder xi REM yi,
        for the corresponding elements of vector double x and vector double y.
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

#ifndef ___SIMD_MATH_REMAINDERD2_H___
#define ___SIMD_MATH_REMAINDERD2_H___

#include <simdmath.h>
#include <spu_intrinsics.h>

#include <simdmath/_remainder.h>
#include <simdmath/fmodd2.h>


static inline vector double
_remainderd2(vector double x, vector double yy)
{
  vec_uchar16 splat_hi   = ((vec_uchar16){ 0,1,2,3,0,1,2,3, 8,9,10,11, 8,9,10,11});
  vec_uint4 y_hi;
  vec_uint4 abs_x, abs_yy, abs_2x, abs_2y;
  vec_uint4 bias;
  vec_uint4 nan_out, overflow;
  vec_uint4 result;
  vec_uint4 half_smax = spu_splats((unsigned int)0x7FEFFFFF);
  vec_uint4 sign_mask = (vec_uint4)(spu_splats(0x8000000000000000ULL));
  vec_uint4 exp_mask  = (vec_uint4)(spu_splats(0x7FF0000000000000ULL));
  vec_uint4 val_nan   = (vec_uint4)(spu_splats(0x7FF8000000000000ULL));
  vec_uint4 vec_zero = spu_splats((unsigned int)0);
  vec_uint4 is_zeroy;

  // cut sign
  abs_x = spu_andc((vec_uint4)x, sign_mask);
  abs_yy = spu_andc((vec_uint4)yy, sign_mask);
  y_hi = spu_shuffle(abs_yy, abs_yy, splat_hi);


  // check nan out
  is_zeroy = spu_cmpeq(abs_yy, vec_zero);
  is_zeroy = spu_and(is_zeroy, spu_rlqwbyte(is_zeroy, 4));
  nan_out = __vec_gt64_half(abs_yy, exp_mask);  // y > 7FF00000
  nan_out = spu_or(nan_out, spu_cmpgt(abs_x, half_smax)); // x >= 7FF0000000000000
  nan_out = spu_or(nan_out, is_zeroy);                    // y = 0
  nan_out = spu_shuffle(nan_out, nan_out, splat_hi);


  // make y x2
  abs_2y = __rem_twice_d(abs_yy); // 2 x y

  result = (vec_uint4)_fmodd2((vec_double2)abs_x, (vec_double2)abs_2y);

  //  abs_x = spu_sel(spu_andc(result, sign_mask), abs_x, spu_cmpgt(y_hi, spu_splats((unsigned int)0x7FBFFFFF)));
  abs_x = spu_sel(result, abs_x, spu_cmpgt(y_hi, spu_splats((unsigned int)0x7FEFFFFF)));

  /* if (2*x > y)
   *     x -= y
   *     if (2*x >= y) x -= y
   */
  overflow = spu_cmpgt(y_hi, spu_splats((unsigned int)0x7FEFFFFF));
  // make x2
  abs_2x = __rem_twice_d(abs_x);  // 2 x x

  bias = __vec_gt64(abs_2x, abs_yy);  // abs_2x > abs_yy
  bias = spu_andc(bias, overflow);

  abs_x = spu_sel(abs_x, __rem_sub_d(abs_x, abs_yy), bias);


  overflow = spu_or(overflow, spu_shuffle(spu_rlmaska(abs_x, -31), vec_zero, splat_hi)); // minous

  // make x2
  abs_2x = __rem_twice_d(spu_andc(abs_x, sign_mask));  // 2 x x  unsupport minous 
  bias = spu_andc(bias, spu_rlmaska(__rem_sub_d(abs_2x, abs_yy), -31));
  bias = spu_andc(spu_shuffle(bias, bias, splat_hi), overflow);
  abs_x = spu_sel(abs_x, __rem_sub_d(abs_x, abs_yy), bias);

  /* select final answer 
   */
  result = spu_xor(abs_x, spu_and((vec_uint4)x, sign_mask)); // set sign
  result = spu_sel(result, val_nan, nan_out); // if nan

  return ((vec_double2)result);
}

#endif
