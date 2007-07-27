/* fpclassifyd2 - for each element of vector x, return classification of x': FP_NAN, FP_INFINITE, FP_NORMAL, FP_SUBNORMAL, FP_ZERO
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

#ifndef ___SIMD_MATH_FPCLASSIFYD2_H___
#define ___SIMD_MATH_FPCLASSIFYD2_H___

#include <simdmath.h>
#include <spu_intrinsics.h>
#include <math.h>

static inline vector signed long long
_fpclassifyd2 (vector double x)
{
  vec_uchar16 even = (vec_uchar16)(vec_uint4){ 0x00010203, 0x00010203, 0x08090a0b, 0x08090a0b };
  vec_uchar16 odd = (vec_uchar16)(vec_uint4){ 0x04050607, 0x04050607, 0x0c0d0e0f, 0x0c0d0e0f };
  vec_uchar16 swapEvenOdd = (vec_uchar16)(vec_uint4){ 0x04050607, 0x00010203, 0x0c0d0e0f, 0x08090a0b };

  vec_ullong2 sign = spu_splats(0x8000000000000000ull);
  vec_ullong2 expn = spu_splats(0x7ff0000000000000ull);
  vec_ullong2 signexpn = spu_splats(0xfff0000000000000ull);
  vec_ullong2 zero = spu_splats(0x0000000000000000ull);

  vec_ullong2 mask;
  vec_llong2 classtype;
  vec_uint4 cmpgt, cmpeq;

  //FP_NORMAL: normal unless nan, infinity, zero, or denorm
  classtype = spu_splats((long long)FP_NORMAL);

  //FP_NAN: all-ones exponent and non-zero mantissa
  cmpgt = spu_cmpgt( (vec_uint4)spu_or( (vec_ullong2)x, sign ), (vec_uint4)signexpn );
  cmpeq = spu_cmpeq( (vec_uint4)spu_or( (vec_ullong2)x, sign ), (vec_uint4)signexpn );
  mask = (vec_ullong2)spu_or( spu_shuffle( cmpgt, cmpgt, even ),
			      spu_and( spu_shuffle( cmpeq, cmpeq, even ), 
				       spu_shuffle( cmpgt, cmpgt, odd ) ) );
  classtype = spu_sel( classtype, spu_splats((long long)FP_NAN), mask );

  //FP_INFINITE: all-ones exponent and zero mantissa
  mask = (vec_ullong2)spu_and( cmpeq, spu_shuffle( cmpeq, cmpeq, swapEvenOdd ) );
  classtype = spu_sel( classtype, spu_splats((long long)FP_INFINITE), mask );

  //FP_ZERO: zero exponent and zero mantissa
  cmpeq = spu_cmpeq( (vec_uint4)spu_andc( (vec_ullong2)x, sign ), (vec_uint4)zero );
  mask = (vec_ullong2)spu_and( cmpeq, spu_shuffle( cmpeq, cmpeq, swapEvenOdd ) );
  classtype = spu_sel( classtype, spu_splats((long long)FP_ZERO), mask );
   
  //FP_SUBNORMAL: zero exponent and non-zero mantissa
  cmpeq = spu_cmpeq( (vec_uint4)spu_and( (vec_ullong2)x, expn ), (vec_uint4)zero );
  cmpgt = spu_cmpgt( (vec_uint4)spu_andc( (vec_ullong2)x, signexpn ), (vec_uint4)zero );
  mask = (vec_ullong2)spu_and( spu_shuffle( cmpeq, cmpeq, even ), 
			       spu_or( cmpgt, spu_shuffle( cmpgt, cmpgt, swapEvenOdd ) ) );
  classtype = spu_sel( classtype, spu_splats((long long)FP_SUBNORMAL), mask );

  return classtype;
}

#endif
