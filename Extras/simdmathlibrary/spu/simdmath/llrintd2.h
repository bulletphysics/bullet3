/* llrintd2 - rounds two doubles in to two nearest 64bit integer.
              consistent with the current rounding mode.
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

#ifndef ___SIMD_MATH_LLRINTD2_H___
#define ___SIMD_MATH_LLRINTD2_H___

#include <simdmath.h>
#include <spu_intrinsics.h>

// 
// Handles no exception
// over flow will return unspecified data

static inline vector signed long long
_llrintd2 (vector double in)
{
  int shift0, shift1;
  vec_uchar16 splat_msb = ((vec_uchar16){0,0,0,0,0,0,0,0, 8,8,8,8,8,8,8,8});
  vec_int4 exp;
  vec_uint4 mant, mant0, mant1, sign, mask, borrow;
  vec_uint4 implied_one = ((vec_uint4){ 0, 0, 0x00100000, 0});
  vec_uint4 exp_mask    = ((vec_uint4){-1,-1, 0xFFF00000, 0});
  vec_double2 bias;

  vec_uint4 vec_zero = ((vec_uint4){0,0,0,0});
  // check denormalized
  vec_uint4 exp_in = spu_and( (vec_uint4)in, 0x7FF00000 );
  vec_uint4 is_denorm = spu_cmpeq( exp_in, 0 );
  vec_uint4 ofs = spu_and( ((vec_uint4){0x00100000,0,0x00100000,0}), is_denorm);

  // check zero
  vec_uint4 abs_x = spu_and((vec_uint4)in, ((vec_uint4){0x7FFFFFFF,-1,0x7FFFFFFF,-1}));
  vec_uint4 is_zerox = spu_cmpeq( abs_x, vec_zero);
  is_zerox = spu_and( is_zerox, spu_shuffle(is_zerox,is_zerox, ((vec_uchar16){4,5,6,7,0,1,2,3,12,13,14,15,8,9,10,11})));
  ofs = spu_sel( ofs, vec_zero, is_zerox);

  vec_double2 xx = (vec_double2)spu_or( (vec_uint4)in, ofs );

  /* Round the input according to the current rounding mode.
   */
  vec_uint4 is_large = spu_cmpgt( exp_in, 0x43200000 );
  is_large = spu_shuffle(is_large,is_large,((vec_uchar16){0,0,0,0,0,0,0,0,8,8,8,8,8,8,8,8}));
  bias = spu_sel((vec_double2)((vec_ullong2){0x4330000000000000ULL,0x4330000000000000ULL}), ((vec_double2){0.0,0.0}), (vec_ullong2)is_large);
  bias = spu_sel(bias, xx, (vec_ullong2)spu_splats(0x8000000000000000ULL));

  //  bias = spu_sel((vec_double2)((vec_ullong2)spu_splats(0x4330000000000000ULL)), xx, 
  //		 (vec_ullong2)spu_splats(0x8000000000000000ULL));
  mant = (vec_uint4)(spu_sub(spu_add(xx, bias), bias));

  /* Determine how many bits to shift the mantissa to correctly
   * align it into long long element 0.
   */
  exp = spu_and(spu_rlmask((vec_int4)mant, -20), 0x7FF);
  exp = spu_add(exp, -1011);
  shift0 = spu_extract(exp, 0);
  shift1 = spu_extract(exp, 2);

  mask = spu_cmpgt(exp, 0);
  mask = spu_shuffle(mask, mask, splat_msb);

  /* Algn mantissa bits
   */
  mant0 = spu_sel(spu_rlmaskqwbyte(mant, -8), implied_one, exp_mask);
  mant1 = spu_sel(mant, implied_one, exp_mask);

  mant0 = spu_slqwbytebc(spu_slqw(mant0, shift0), shift0);
  mant1 = spu_slqwbytebc(spu_slqw(mant1, shift1), shift1);

  mant = spu_shuffle(mant0, mant1, ((vec_uchar16){0,1,2,3,4,5,6,7, 16,17,18,19,20,21,22,23}));
  mant = spu_and(mant, mask);

  /* Compute the two's complement of the mantissa if the 
   * input is negative.
   */
  sign = (vec_uint4)spu_rlmaska((vec_int4)xx, -31);
  sign = spu_shuffle(sign, sign, splat_msb);

  mant = spu_xor(mant, sign);
  borrow = spu_genb(mant, sign);
  borrow = spu_shuffle(borrow, borrow, ((vec_uchar16){ 
	4,5,6,7, 192,192,192,192,
	  12,13,14,15, 192,192,192,192}));
  mant = spu_subx(mant, sign, borrow);

  return ((vec_llong2)(mant));
}

#endif
