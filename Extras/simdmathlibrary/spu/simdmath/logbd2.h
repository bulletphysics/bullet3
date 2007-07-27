/* logbd2 - for each element of vector x, return the exponent of normalized double x' as floating point value
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

#ifndef ___SIMD_MATH_LOGBD2_H___
#define ___SIMD_MATH_LOGBD2_H___

#include <simdmath.h>
#include <spu_intrinsics.h>
#include <math.h>

static inline vector double
_logbd2 (vector double x)
{
  vec_uchar16 even = (vec_uchar16)(vec_uint4){ 0x00010203, 0x00010203, 0x08090a0b, 0x08090a0b };
  vec_uchar16 odd = (vec_uchar16)(vec_uint4){ 0x04050607, 0x04050607, 0x0c0d0e0f, 0x0c0d0e0f };
  vec_uchar16 swapEvenOdd = (vec_uchar16)(vec_uint4){ 0x04050607, 0x00010203, 0x0c0d0e0f, 0x08090a0b };

  vec_ullong2 sign = spu_splats(0x8000000000000000ull);
  vec_ullong2 expn = spu_splats(0x7ff0000000000000ull);
  vec_ullong2 zero = spu_splats(0x0000000000000000ull);

  vec_ullong2 isnan, isinf, iszero;
  vec_double2 logb = (vec_double2)zero;
  vec_llong2 e1, e2;
  vec_uint4 cmpgt, cmpeq, cmpzr;
  vec_int4 lz, lz0, lz1;

  //NAN: x is NaN (all-ones exponent and non-zero mantissa)
  cmpgt = spu_cmpgt( (vec_uint4)spu_or( (vec_ullong2)x, sign ), (vec_uint4)spu_or(sign, expn) );
  cmpeq = spu_cmpeq( (vec_uint4)spu_or( (vec_ullong2)x, sign ), (vec_uint4)spu_or(sign, expn) );
  isnan = (vec_ullong2)spu_or( spu_shuffle( cmpgt, cmpgt, even ),
			       spu_and( spu_shuffle( cmpeq, cmpeq, even ), 
					spu_shuffle( cmpgt, cmpgt, odd ) ) );
  logb = spu_sel( logb, (vec_double2)spu_splats(0x7FF8000000000000ll), isnan );

  //INF: x is infinite (all-ones exponent and zero mantissa)
  isinf = (vec_ullong2)spu_and( cmpeq, spu_shuffle( cmpeq, cmpeq, swapEvenOdd ) );
  logb = spu_sel( logb, (vec_double2)spu_splats(__builtin_huge_val()), isinf );

  //HUGE_VAL: x is zero (zero exponent and zero mantissa)
  cmpzr = spu_cmpeq( (vec_uint4)spu_andc( (vec_ullong2)x, sign ), (vec_uint4)zero );
  iszero = (vec_ullong2)spu_and( cmpzr, spu_shuffle( cmpzr, cmpzr, swapEvenOdd ) );
  logb = spu_sel( logb, (vec_double2)spu_splats(-__builtin_huge_val()), iszero );

  //Integer Exponent: if x is normal or subnormal, return unbiased exponent of normalized double x
  e1 = (vec_llong2)spu_and( (vec_llong2)x, (vec_llong2)expn );
  e2 = (vec_llong2)spu_rlmask((vec_uint4)e1, -20);

  lz = (vec_int4)spu_cntlz( (vec_uint4)spu_andc( (vec_ullong2)x, sign) );
  lz0 = (vec_int4)spu_shuffle( lz, lz, even );
  lz0 = spu_sel( (vec_int4)zero, spu_sub( lz0, spu_splats((int)12) ), spu_cmpgt( lz0, (int)11 ) );
  lz1 = spu_sel( (vec_int4)zero, spu_shuffle( lz, lz, odd), spu_cmpeq( lz0, (int)20 ) );

  logb = spu_sel( logb, spu_extend( spu_convtf( spu_sub( spu_sub( (vec_int4)e2, spu_splats((int)1023) ), spu_add( lz0, lz1 ) ), 0 ) ), 
		  spu_nor( isnan, spu_or( isinf, iszero ) ) );

  return logb;
}

#endif
