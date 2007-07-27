/* fmaxd2 - for each of two double slots, compute maximum of x and y
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

#ifndef ___SIMD_MATH_FMAXD2_H___
#define ___SIMD_MATH_FMAXD2_H___

#include <simdmath.h>
#include <spu_intrinsics.h>
                          
/* Return the maximum numeric value of their arguments. If one argument
 * is a NaN, fmax returns the other value.  If both are NaNs, then a NaN
 * is returned.
 */

static inline vector double
_fmaxd2 (vector double x, vector double y)
{
  vec_ullong2 selector, denorm;
  vec_double2 x_offset, y_offset, diff;
  vec_uint4 nan_x, abs_x, gt, eq;
  vec_uint4 sign = (vec_uint4){0x80000000, 0, 0x80000000, 0};
  vec_uint4 infinity = (vec_uint4){0x7FF00000, 0, 0x7FF00000, 0};
  vec_uint4 exp0 = (vec_uint4){0x3FF00000, 0, 0x3FF00000, 0};

  /* If both x and y are denorm or zero, then set 0x3ff to exponent
   */
  denorm = (vec_ullong2)spu_cmpeq(spu_and((vec_uint4)spu_or(x, y), infinity), 0);
  x_offset = spu_sel(x, spu_or(x, (vec_double2)exp0), denorm);
  y_offset = spu_sel(y, spu_or(y, (vec_double2)exp0), denorm);

  /* If x is a NaN, then select y as max
   */
  abs_x = spu_andc((vec_uint4)x, sign);
  gt = spu_cmpgt(abs_x, infinity);
  eq = spu_cmpeq(abs_x, infinity);
  nan_x = spu_or(gt, spu_and(eq, spu_rlqwbyte(gt, 4)));

  diff = spu_sub(x_offset, y_offset);
  selector = (vec_ullong2)spu_orc(nan_x, spu_cmpgt((vec_int4)diff, -1));
  selector = spu_shuffle(selector, selector, ((vec_uchar16){0,1,2,3, 0,1,2,3, 8,9,10,11, 8,9,10,11}));

  return spu_sel(x, y, selector);
}

#endif
