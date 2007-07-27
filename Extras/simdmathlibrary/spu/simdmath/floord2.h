/* floord2 - for each of two doule slots, round up to smallest integer not more than the value.
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

#ifndef ___SIMD_MATH_FLOORD2_H___
#define ___SIMD_MATH_FLOORD2_H___

#include <simdmath.h>
#include <spu_intrinsics.h>

static inline vector double
_floord2(vector double in)
{
  vec_uchar16 swap_words = ((vec_uchar16){4,5,6,7, 0,1,2,3, 12,13,14,15, 8,9,10,11});
  vec_uchar16 splat_hi = ((vec_uchar16){0,1,2,3,0,1,2,3, 8,9,10,11, 8,9,10,11});
  vec_uint4 one =  ((vec_uint4){0, 1, 0, 1});
  vec_int4 exp, shift;
  vec_uint4 mask, mask_1, frac_mask, addend, insert, pos, equal0, e_0, e_00, e_sign, exp_ge0;
  vec_ullong2 sign = spu_splats(0x8000000000000000ULL);
  vec_double2 in_hi, out;
  vec_double2 one_d = spu_splats((double)1.0);
  vec_uint4 zero = spu_splats((unsigned int)0x0);

  /* This function generates the following component
   * based upon the inputs.
   *
   *   mask = bits of the input that need to be replaced.
   *   insert = value of the bits that need to be replaced
   *   addend = value to be added to perform function.
   *
   * These are applied as follows:.
   *
   *   out = ((in & mask) | insert) + addend
   */

  in_hi = spu_shuffle(in, in, splat_hi);
  exp = spu_and(spu_rlmask((vec_int4)in_hi, -20), 0x7FF);
  shift = spu_sub(((vec_int4){1023, 1043, 1023, 1043}), exp);

  /* clamp shift to the range 0 to -31.
   */
  shift = spu_sel(spu_splats((int)-32), spu_andc(shift, (vec_int4)spu_cmpgt(shift, 0)), spu_cmpgt(shift, -32));
  frac_mask = spu_rlmask(((vec_uint4){0xFFFFF, -1, 0xFFFFF, -1}), shift);
  exp_ge0 = spu_cmpgt(exp, 0x3FE);
  mask = spu_orc(frac_mask, exp_ge0);

  /* addend = ((in & mask) && (in >= 0)) ? mask+1 : 0
   */
  mask_1 = spu_addx(mask, one, spu_rlqwbyte(spu_genc(mask, one), 4));
  pos = spu_cmpgt((vec_int4)in_hi, -1);
  //pos = spu_cmpgt((vec_int4)in_hi, 0x0); //it is also work
  equal0 = spu_cmpeq(spu_and((vec_uint4)in, mask), 0);
  addend = spu_andc(spu_andc(mask_1, pos), spu_and(equal0, spu_shuffle(equal0, equal0, swap_words)));

  /* insert
   */
  e_0 = spu_cmpeq(spu_andc((vec_uint4)in, (vec_uint4)sign), zero);
  e_00 = spu_and(e_0, spu_shuffle(e_0, e_0, swap_words));
  // e_sign = spu_sel((vec_uint4)one_d, zero, spu_cmpeq( spu_and((vec_uint4)in_hi, spu_splats((unsigned int)0x80000000)), zero));
  e_sign = spu_andc( (vec_uint4)one_d, spu_cmpeq( spu_and((vec_uint4)in_hi,spu_splats((unsigned int)0x80000000)), zero));
  insert =spu_andc(spu_andc(e_sign, e_00), exp_ge0);

  /* replace insert
   */
  in = spu_sel(in, (vec_double2)insert, spu_andc((vec_ullong2)mask, sign));

  /* in + addend
   */
  out = (vec_double2)spu_addx((vec_uint4)in, addend, spu_rlqwbyte(spu_genc((vec_uint4)in, addend), 4));

  return (out);
}

#endif
