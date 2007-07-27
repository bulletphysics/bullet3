/* recipd2 - for each of two double slots, compute reciprocal.
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

#ifndef ___SIMD_MATH_RECIPD2_H___
#define ___SIMD_MATH_RECIPD2_H___

#include <simdmath.h>
#include <spu_intrinsics.h>

#include <simdmath/isinfd2.h>
#include <simdmath/is0denormd2.h>
#include <simdmath/isnand2.h>

// Handles exceptional values as follows:
// NaN -> NaN
// (+,-)Inf -> (+,-)0
// (+,-)0 -> (+,-)Inf
// Denormal inputs are treated as zero.

static inline vector double
_recipd2 (vector double x)
{
  vec_ullong2 expmask, signmask;
  vec_double2 one, man, exp, nexp, y1, y2, y3, zero, inf, result;
  vec_float4  onef, manf, y0f, y1f;

  expmask = spu_splats(0x7ff0000000000000ull);
  signmask = spu_splats(0x8000000000000000ull);
  onef = spu_splats(1.0f);
  one = spu_extend( onef );

  // Factor ( mantissa x 2^exponent ) into ( mantissa x 2 ) and ( 2^(exponent-1) ).
  // Invert exponent part with subtraction.

  exp = spu_and( x, (vec_double2)expmask );
  nexp = (vec_double2)spu_sub( (vec_uint4)expmask, (vec_uint4)exp );
  man = spu_sel( x, (vec_double2)spu_splats(0x40000000), expmask );

  // Compute mantissa part with single and double precision Newton-Raphson steps.
  // Then multiply with 2^(1-exponent).

  manf = spu_roundtf( man );
  y0f = spu_re( manf );
  y1f = spu_madd( spu_nmsub( manf, y0f, onef ), y0f, y0f );
  y1 = spu_extend( y1f );
  y2 = spu_madd( spu_nmsub( man, y1, one ), y1, y1 );
  y3 = spu_madd( spu_nmsub( man, y2, one ), y2, y2 );
  y3 = spu_mul( y3, nexp );

  // Choose iterated result or special value.

  zero = spu_and( x, (vec_double2)signmask );
  inf = spu_sel( (vec_double2)expmask, x, signmask );

  result = spu_sel( y3, zero, _isinfd2 ( x ) );
  result = spu_sel( result, inf, _is0denormd2 ( x ) );
  result = spu_sel( result, x, _isnand2( x ) );

  return result;
}

#endif
