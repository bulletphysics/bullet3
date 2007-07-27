/* llroundd2 - rounds two doubles in to two nearest 64bit integer.
               0.5 will be rounded to far from 0
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

#ifndef ___SIMD_MATH_LLROUNDD2_H___
#define ___SIMD_MATH_LLROUNDD2_H___

#include <simdmath.h>
#include <spu_intrinsics.h>

// 
// Handles no exception
// over flow will return unspecified data

static inline vector signed long long
_llroundd2 (vector double in)
{
  int shift0, shift1;
  vec_uchar16 splat_msb = { 0,0,0,0,0,0,0,0, 8,8,8,8,8,8,8,8};
  vec_int4 exp;
  vec_uint4 mant, mant0, mant1, sign, mask, borrow, addend;
  vec_uint4 implied_one = { 0, 0, 0x00100000, 0};
  vec_uint4 exp_mask = { -1, -1,0xFFF00000, 0};

  /* Determine how many bits to shift the mantissa to correctly
   * align it into long long element 0.
   */
  exp = spu_and(spu_rlmask((vec_int4)in, -20), 0x7FF);
  exp = spu_add(exp, -1011);
  shift0 = spu_extract(exp, 0);
  shift1 = spu_extract(exp, 2);

  mask = spu_cmpgt(exp, 0);
  mask = spu_shuffle(mask, mask, splat_msb);

  /* Algn mantissa bits
   */
  mant0 = spu_sel(spu_rlmaskqwbyte((vec_uint4)in, -8), implied_one, exp_mask);
  mant1 = spu_sel((vec_uint4)in, implied_one, exp_mask);

  mant0 = spu_slqwbytebc(spu_slqw(mant0, shift0), shift0);
  mant1 = spu_slqwbytebc(spu_slqw(mant1, shift1), shift1);

  mant = spu_shuffle(mant0, mant1, ((vec_uchar16){0,1,2,3,4,5,6,7, 16,17,18,19,20,21,22,23}));
  mant = spu_and(mant, mask);

  /* Perform round by adding 1 if the fraction bits are 
   * greater than or equal to .5
   */
  addend = spu_shuffle(mant0, mant1, ((vec_uchar16){0x80,0x80,0x80,0x80,0x80,0x80,0x80,8, 0x80,0x80,0x80,0x80,0x80,0x80,0x80,24}));
  addend = spu_rlmask(addend, -7);
  //  addend = spu_and(spu_rlqw(mant, 1), ((vec_uint4){ 0,1,0,1}));
  mant = spu_addx(mant, addend, spu_rlqwbyte(spu_genc(mant, addend), 4));

  /* Compute the two's complement of the mantissa if the 
   * input is negative.
   */
  sign = (vec_uint4)spu_rlmaska((vec_int4)in, -31);
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
