/* llrintf4 - rounds four floats in to four nearest 64bit integer.
              On SPU the rounding mode for floats is always towards 0.
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

#ifndef ___SIMD_MATH_LLRINTF4_H___
#define ___SIMD_MATH_LLRINTF4_H___

#include <simdmath.h>
#include <spu_intrinsics.h>

// 
// Handles no exception
// over flow will return unspecified data

static inline llroundf4_t 
_llrintf4 (vector float in)
{
  llroundf4_t res;
  vec_int4 exp;
  vec_uint4 mant0, mant1, mant2, mant3;
  vec_uint4 mask, mask0, mask1;
  vec_uint4 sign, sign0, sign1;
  vec_uint4 borrow0, borrow1;
  vec_uint4 res0, res1;
  int shift0, shift1, shift2, shift3;

  /* Place mantissa bits (including implied most signficant
   * bit) into the most significant bits of element 3. Elements
   * 0, 1, and 2 are zeroed.
   */
  mant0 = spu_sel(spu_rlmaskqwbyte((vec_uint4)in,-11), ((vec_uint4){0, 0, 0, 0x80000000}), ((vec_uint4){-1, -1, -1, 0x800000FF}));
  mant1 = spu_sel(spu_rlmaskqwbyte((vec_uint4)in, -7), ((vec_uint4){0, 0, 0, 0x80000000}), ((vec_uint4){-1, -1, -1, 0x800000FF}));
  mant2 = spu_sel(spu_rlmaskqwbyte((vec_uint4)in, -3), ((vec_uint4){0, 0, 0, 0x80000000}), ((vec_uint4){-1, -1, -1, 0x800000FF}));
  mant3 = spu_sel(    spu_rlqwbyte((vec_uint4)in,  1), ((vec_uint4){0, 0, 0, 0x80000000}), ((vec_uint4){-1, -1, -1, 0x800000FF}));

  /* Determine how many bits to shift the mantissa to correctly
   * align it into long long element 0.
   */
  exp = spu_and(spu_rlmask((vec_int4)in, -23), 0xFF);
  exp = spu_add(exp, -94);
  shift0 = spu_extract(exp, 0);
  shift1 = spu_extract(exp, 1);
  shift2 = spu_extract(exp, 2);
  shift3 = spu_extract(exp, 3);

  /* Algn mantissa bits
   */
  mant0 = spu_slqwbytebc(spu_slqw(mant0, shift0), shift0);
  mant1 = spu_slqwbytebc(spu_slqw(mant1, shift1), shift1);
  mant2 = spu_slqwbytebc(spu_slqw(mant2, shift2), shift2);
  mant3 = spu_slqwbytebc(spu_slqw(mant3, shift3), shift3);

  mask  = spu_cmpgt(exp, 0);
  mask0 = spu_shuffle(mask, mask, ((vec_uchar16){0,0,0,0,0,0,0,0,  4, 4, 4, 4, 4, 4, 4, 4}));
  mask1 = spu_shuffle(mask, mask, ((vec_uchar16){8,8,8,8,8,8,8,8, 12,12,12,12,12,12,12,12}));

  res0 = spu_shuffle(mant0, mant1,((vec_uchar16){0,1,2,3,4,5,6,7, 16,17,18,19,20,21,22,23}));
  res1 = spu_shuffle(mant2, mant3,((vec_uchar16){0,1,2,3,4,5,6,7, 16,17,18,19,20,21,22,23}));
  res0 = spu_and(res0, mask0);
  res1 = spu_and(res1, mask1);

  /* Compute the two's complement of the mantissa if the 
   * input is negative.
   */
  sign = (vec_uint4)spu_rlmaska((vec_int4)in, -31);
  sign0 = spu_shuffle(sign, sign, ((vec_uchar16){0,0,0,0,0,0,0,0,  4, 4, 4, 4, 4, 4, 4, 4}));
  sign1 = spu_shuffle(sign, sign, ((vec_uchar16){8,8,8,8,8,8,8,8, 12,12,12,12,12,12,12,12}));

  res0 = spu_xor(res0, sign0);
  res1 = spu_xor(res1, sign1);
  borrow0 = spu_genb(res0, sign0);
  borrow1 = spu_genb(res1, sign1);
  borrow0 = spu_shuffle(borrow0, borrow0, ((vec_uchar16){4,5,6,7,0xc0,0xc0,0xc0,0xc0, 12,13,14,15,0xc0,0xc0,0xc0,0xc0}));
  borrow1 = spu_shuffle(borrow1, borrow1, ((vec_uchar16){4,5,6,7,0xc0,0xc0,0xc0,0xc0, 12,13,14,15,0xc0,0xc0,0xc0,0xc0}));
  res.vll[0] = (vec_llong2)spu_subx(res0, sign0, borrow0);
  res.vll[1] = (vec_llong2)spu_subx(res1, sign1, borrow1);

  return res;
}

#endif
