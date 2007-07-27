/* frexpd2 - for each element of vector x, return the normalized fraction and store the exponent of x'
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

#ifndef ___SIMD_MATH_FREXPD2_H___
#define ___SIMD_MATH_FREXPD2_H___

#include <simdmath.h>
#include <spu_intrinsics.h>
#include <math.h>

#define __FREXPD_DBL_NAN 0x7FF8000000000000ull

static inline vector double
_frexpd2 (vector double x, vector signed long long *pexp)
{
  vec_uchar16 even = (vec_uchar16)(vec_uint4){ 0x00010203, 0x00010203, 0x08090a0b, 0x08090a0b };
  vec_uchar16 swapEvenOdd = (vec_uchar16)(vec_uint4){ 0x04050607, 0x00010203, 0x0c0d0e0f, 0x08090a0b };

  vec_ullong2 maskdw = (vec_ullong2){0xffffffffffffffffull, 0ull};

  vec_ullong2 sign = spu_splats(0x8000000000000000ull);
  vec_ullong2 expn = spu_splats(0x7ff0000000000000ull);
  vec_ullong2 zero = spu_splats(0x0000000000000000ull);

  vec_ullong2 isnan, isinf, iszero;
  vec_ullong2 e0, x0, x1;
  vec_uint4 cmpgt, cmpeq, cmpzr;
  vec_int4 lz, lz0, sh, ex;
  vec_double2 fr, frac = (vec_double2)zero;

  //NAN: x is NaN (all-ones exponent and non-zero mantissa)
  cmpgt = spu_cmpgt( (vec_uint4)spu_or( (vec_ullong2)x, sign ), (vec_uint4)spu_or(sign, expn) );
  cmpeq = spu_cmpeq( (vec_uint4)spu_or( (vec_ullong2)x, sign ), (vec_uint4)spu_or(sign, expn) );
  isnan = (vec_ullong2)spu_or( cmpgt, spu_and( cmpeq, spu_rlqwbyte( cmpgt, -4 ) ) );
  isnan = (vec_ullong2)spu_shuffle( isnan, isnan, even );
  frac = spu_sel( frac, (vec_double2)spu_splats(__FREXPD_DBL_NAN), isnan );

  //INF: x is infinite (all-ones exponent and zero mantissa)
  isinf = (vec_ullong2)spu_and( cmpeq, spu_shuffle( cmpeq, cmpeq, swapEvenOdd ) );
  frac = spu_sel( frac, x , isinf );

  //x is zero (zero exponent and zero mantissa)
  cmpzr = spu_cmpeq( (vec_uint4)spu_andc( (vec_ullong2)x, sign ), (vec_uint4)zero );
  iszero = (vec_ullong2)spu_and( cmpzr, spu_shuffle( cmpzr, cmpzr, swapEvenOdd ) );

  frac = spu_sel( frac, (vec_double2)zero , iszero );
  *pexp = spu_sel( *pexp, (vec_llong2)zero , iszero );

  //Integer Exponent: if x is normal or subnormal

  //...shift left to normalize fraction, zero shift if normal
  lz = (vec_int4)spu_cntlz( (vec_uint4)spu_andc( (vec_ullong2)x, sign) );
  lz0 = (vec_int4)spu_shuffle( lz, lz, even );
  sh = spu_sel( (vec_int4)zero, spu_sub( lz0, spu_splats((int)11) ), spu_cmpgt( lz0, (int)11 ) );
  sh = spu_sel( sh, spu_add( sh, lz ), spu_cmpeq( lz0, (int)32 ) );

  x0 = spu_slqw( spu_slqwbytebc( spu_and( (vec_ullong2)x, maskdw ), spu_extract(sh, 1) ), spu_extract(sh, 1) );
  x1 = spu_slqw( spu_slqwbytebc( (vec_ullong2)x, spu_extract(sh, 3) ), spu_extract(sh, 3) );
  fr = (vec_double2)spu_sel( x1, x0, maskdw );
  fr = spu_sel( fr, (vec_double2)spu_splats(0x3FE0000000000000ull), expn );
  fr = spu_sel( fr, x, sign );

  e0 = spu_rlmaskqw( spu_rlmaskqwbyte(spu_and( (vec_ullong2)x, expn ),-6), -4 );
  ex = spu_sel( spu_sub( (vec_int4)e0, spu_splats((int)1022) ), spu_sub( spu_splats((int)-1021), sh ), spu_cmpgt( sh, (int)0 ) );

  frac = spu_sel( frac, fr, spu_nor( isnan, spu_or( isinf, iszero ) ) );
  *pexp = spu_sel( *pexp, spu_extend( ex ), spu_nor( isnan, spu_or( isinf, iszero ) ) );

  return frac;
}

#endif
