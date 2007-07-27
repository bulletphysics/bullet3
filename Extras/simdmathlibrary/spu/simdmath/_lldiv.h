/* Common functions for lldivi2/lldivu2
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

#ifndef ___SIMD_MATH_LLDIV_H___
#define ___SIMD_MATH_LLDIV_H___

#include <spu_intrinsics.h>

static inline vector unsigned long long 
__ll_spu_cntlz(vector unsigned long long x)
{
  vec_uint4 cnt;

  cnt = spu_cntlz((vec_uint4)x);
  cnt = spu_add(cnt, spu_and(spu_cmpeq(cnt, 32), spu_rlqwbyte(cnt, 4)));
  cnt = spu_shuffle(cnt, cnt, ((vec_uchar16){0x80,0x80,0x80,0x80, 0,1,2,3, 0x80,0x80,0x80,0x80, 8,9,10,11}));

  return (vec_ullong2)cnt;
}

static inline vector unsigned long long 
__ll_spu_sl(vector unsigned long long x, vector unsigned long long count)
{
  vec_ullong2 mask = (vec_ullong2){0xffffffffffffffffull, 0ull};
  vec_ullong2 x_upper, x_lower;

  // shift upper word
  x_upper = spu_and(x, mask);
  x_upper = spu_slqwbytebc(x_upper, spu_extract((vec_uint4)count, 1));
  x_upper = spu_slqw(x_upper, spu_extract((vec_uint4)count, 1));
  
  // shift lower word
  x_lower = spu_slqwbytebc(x, spu_extract((vec_uint4)count, 3));
  x_lower = spu_slqw(x_lower, spu_extract((vec_uint4)count, 3));

  return spu_sel(x_lower, x_upper, mask);
}

static inline vector unsigned long long 
__ll_spu_rlmask(vector unsigned long long x, vector unsigned long long count)
{
  vec_ullong2 mask = (vec_ullong2){0xffffffffffffffffull, 0ull};
  vec_ullong2 x_upper, x_lower;
  vec_uint4 cnt_byte;

  cnt_byte = spu_add((vec_uint4)count, 7);

  // shift upper word
  x_upper = spu_rlmaskqwbytebc(x, spu_extract(cnt_byte, 1));
  x_upper = spu_rlmaskqw(x_upper, spu_extract((vec_uint4)count, 1));
  
  // shift lower word
  x_lower = spu_andc(x, mask);
  x_lower = spu_rlmaskqwbytebc(x_lower, spu_extract(cnt_byte, 3));
  x_lower = spu_rlmaskqw(x_lower, spu_extract((vec_uint4)count, 3));

  return spu_sel(x_lower, x_upper, mask);
}

static inline vector unsigned long long 
__ll_spu_cmpeq_zero(vector unsigned long long x)
{
  vec_uint4 cmp;

  cmp = spu_cmpeq((vec_uint4)x, 0);
  return (vec_ullong2)spu_and(cmp, spu_shuffle(cmp, cmp, ((vec_uchar16){4,5,6,7, 0,1,2,3, 12,13,14,15, 8,9,10,11})));
}

static inline vector unsigned long long 
__ll_spu_cmpgt(vector unsigned long long x, vector unsigned long long y)
{
  vec_uint4 gt;

  gt = spu_cmpgt((vec_uint4)x, (vec_uint4)y);
  gt = spu_sel(gt, spu_rlqwbyte(gt, 4), spu_cmpeq((vec_uint4)x, (vec_uint4)y));
  return (vec_ullong2)spu_shuffle(gt, gt, ((vec_uchar16){0,1,2,3, 0,1,2,3, 8,9,10,11, 8,9,10,11}));
}

static inline vector unsigned long long 
__ll_spu_sub(vector unsigned long long x, vector unsigned long long y)
{
  vec_uint4 borrow;

  borrow = spu_genb((vec_uint4)x, (vec_uint4)y);
  borrow = spu_shuffle(borrow, borrow, ((vec_uchar16){4,5,6,7, 0xc0,0xc0,0xc0,0xc0, 12,13,14,15, 0xc0,0xc0,0xc0,0xc0}));
  return (vec_ullong2)spu_subx((vec_uint4)x, (vec_uint4)y, borrow);
}

#endif // __LLDIV_H__

