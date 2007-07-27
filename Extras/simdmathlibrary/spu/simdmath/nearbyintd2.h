/* nearbyintd2 - Round the input to the nearest integer according to
      the current rounding mode without raising an inexact exception.
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

#ifndef ___SIMD_MATH_NEARBYINTD2_H___
#define ___SIMD_MATH_NEARBYINTD2_H___

#include <simdmath.h>
#include <spu_intrinsics.h>

static inline vector double 
_nearbyintd2(vector double in)
{
  vec_uint4 fpscr;
  vec_ullong2 sign = ((vec_ullong2){0x8000000000000000ULL,0x8000000000000000ULL});
  vec_double2 out, addend;
  vec_uint4 vec_zero = ((vec_uint4){0,0,0,0});

  fpscr = spu_mffpscr();

  // check denormalized
  vec_uint4 exp = spu_and( (vec_uint4)in, 0x7FF00000 );
  vec_uint4 is_denorm = spu_cmpeq( exp, 0 );
  vec_uint4 ofs = spu_and( ((vec_uint4){0x00100000,0,0x00100000,0}), is_denorm);

  // check zero
  vec_uint4 abs_x = spu_and((vec_uint4)in, ((vec_uint4){0x7FFFFFFF,-1,0x7FFFFFFF,-1}));
  vec_uint4 is_zerox = spu_cmpeq( abs_x, vec_zero);
  is_zerox = spu_and( is_zerox, spu_shuffle(is_zerox,is_zerox, ((vec_uchar16){4,5,6,7,0,1,2,3,12,13,14,15,8,9,10,11})));
  ofs = spu_sel( ofs, vec_zero, is_zerox);

  vec_double2 xx = (vec_double2)spu_or( (vec_uint4)in, ofs );

  /* Add 2^53 and then subtract 2^53 to affect a round to be performed by the
   * hardware. Also preserve the input sign so that negative inputs that 
   * round to zero generate a -0.0.
   */
  vec_uint4 is_large = spu_cmpgt( exp, 0x43200000 );
  is_large = spu_shuffle(is_large,is_large,((vec_uchar16){0,0,0,0,0,0,0,0,8,8,8,8,8,8,8,8}));
  addend = spu_sel((vec_double2)((vec_ullong2){0x4330000000000000ULL,0x4330000000000000ULL}), ((vec_double2){0.0,0.0}), (vec_ullong2)is_large);
  addend = spu_sel(addend, xx, sign);

  out = spu_sel(spu_sub(spu_add(xx, addend), addend), xx, sign);

  spu_mtfpscr(fpscr);

  return (out);
}

#endif
