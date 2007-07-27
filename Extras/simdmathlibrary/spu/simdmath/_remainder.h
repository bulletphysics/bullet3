/* A vector double is returned that contains the internal routine regarding remainder.
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

#ifndef ___SIMD_MATH__REMAINDER_H___
#define ___SIMD_MATH__REMAINDER_H___

#include <simdmath/_vec_utils.h>

/*
 * double formated x2
 */
static inline vec_uint4
__rem_twice_d(vec_uint4 aa)
{
  vec_uint4 norm = spu_cmpgt(aa, (vec_uint4)(spu_splats(0x000FFFFFFFFFFFFFULL))); // exp > 0
  norm = spu_shuffle(norm, norm, ((vec_uchar16){ 0,1,2,3,0,1,2,3, 8,9,10,11, 8,9,10,11}));

  // if denorm or zero << 1 , if norm exp + 1
  return spu_sel(spu_slqw(aa, 1), spu_add(aa, (vec_uint4)(spu_splats(0x0010000000000000ULL))), norm); // x2
}

/*
 * subtraction function in limited confdition
 */
static inline vec_uint4
__rem_sub_d(vec_uint4 aa, vec_uint4 bb)
{
  // which is bigger input aa or bb
  vec_uint4 is_bigb = __vec_gt64(bb, aa);  // bb > aa

  // need denorm calc ?
  vec_uint4 norm_a, norm_b;
  norm_a = spu_cmpgt(aa, (vec_uint4)(spu_splats(0x000FFFFFFFFFFFFFULL)));
  norm_b = spu_cmpgt(bb, (vec_uint4)(spu_splats(0x000FFFFFFFFFFFFFULL)));
  norm_a = spu_and(norm_a, norm_b);
  norm_a = spu_shuffle(norm_a, norm_a,((vec_uchar16){ 0,1,2,3,0,1,2,3, 8,9,10,11, 8,9,10,11}));

  // calc (aa - bb) and (bb - aa)
  vec_uint4 res_a, res_b, res;
  vec_uint4 borrow_a, borrow_b;
  vec_uchar16 mask_b = ((vec_uchar16){4,5,6,7,192,192,192,192,12,13,14,15,192,192,192,192});
  borrow_a = spu_genb(aa, bb);
  borrow_b = spu_genb(bb, aa);
  borrow_a = spu_shuffle(borrow_a, borrow_a, mask_b);
  borrow_b = spu_shuffle(borrow_b, borrow_b, mask_b);
  res_a = spu_subx(aa, bb, borrow_a);
  res_b = spu_subx(bb, aa, borrow_b);
  res_b = spu_or(res_b, ((vec_uint4){0x80000000,0,0x80000000,0}));  // set sign

  res = spu_sel(res_a, res_b, is_bigb);  // select (aa - bb) or (bb - aa)
  // select normal calc or special
  res = spu_sel(res, (vec_uint4)spu_sub((vec_double2)aa, (vec_double2)bb), norm_a);

  return res;
}

#endif

