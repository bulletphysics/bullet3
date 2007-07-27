/* lldivi2 - 
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

#ifndef ___SIMD_MATH_LLDIVI2_H___
#define ___SIMD_MATH_LLDIVI2_H___

#include <simdmath.h>
#include <spu_intrinsics.h>

#include <simdmath/_lldiv.h>
#include <simdmath/lldivu2.h>

static inline vector signed long long
__lldivi2_negatell2 (vector signed long long x)
{
  vector signed int zero = (vector signed int){0,0,0,0};
  vector signed int borrow;

  borrow = spu_genb(zero, (vec_int4)x);
  borrow = spu_shuffle(borrow, borrow, ((vec_uchar16){4,5,6,7, 0xc0,0xc0,0xc0,0xc0, 12,13,14,15, 0xc0,0xc0,0xc0,0xc0}));
  return (vec_llong2)spu_subx(zero, (vec_int4)x, borrow);
}

// lldivi2 - for each of two signed long long interger slots, compute quotient and remainder of 
// numer/denom and store in lldivi2_t struct.  Divide by zero produces quotient = 0, remainder = numerator.

static inline lldivi2_t
_lldivi2 (vector signed long long numer, vector signed long long denom)
{
  lldivi2_t res;
  lldivu2_t resAbs;
  vec_ullong2 numerAbs, denomAbs;
  vec_uint4 numerPos, denomPos, quotNeg;

  // Determine whether result needs sign change

  numerPos = spu_cmpgt((vec_int4)numer, -1);
  numerPos = spu_shuffle(numerPos, numerPos, ((vec_uchar16){0,0,0,0,0,0,0,0, 8,8,8,8,8,8,8,8}));
  denomPos = spu_cmpgt((vec_int4)denom, -1);
  denomPos = spu_shuffle(denomPos, denomPos, ((vec_uchar16){0,0,0,0,0,0,0,0, 8,8,8,8,8,8,8,8}));
  quotNeg = spu_xor( numerPos, denomPos );
    
  // Use absolute values of numerator, denominator

  numerAbs = (vec_ullong2)spu_sel(__lldivi2_negatell2(numer), numer, (vec_ullong2)numerPos);
  denomAbs = (vec_ullong2)spu_sel(__lldivi2_negatell2(denom), denom, (vec_ullong2)denomPos);

  // Get difference of leading zeros.

  resAbs = _lldivu2(numerAbs, denomAbs);
  res.quot = spu_sel((vec_llong2)resAbs.quot, __lldivi2_negatell2((vec_llong2)resAbs.quot),
		     (vec_ullong2)quotNeg);
  res.rem = spu_sel(__lldivi2_negatell2((vec_llong2)resAbs.rem), (vec_llong2)resAbs.rem,
		    (vec_ullong2)numerPos);

  return res;
}

#endif
