/* lldivu2 - 
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

#ifndef ___SIMD_MATH_LLDIVU2_H___
#define ___SIMD_MATH_LLDIVU2_H___

#include <simdmath.h>
#include <spu_intrinsics.h>

#include <simdmath/_lldiv.h>

// lldivu2 - for each of two unsigned long long interger slots, compute quotient and remainder of 
// numer/denom and store in lldivu2_t struct.  Divide by zero produces quotient = 0, remainder = numerator.

static inline lldivu2_t
_lldivu2 (vector unsigned long long numer, vector unsigned long long denom)
{
  lldivu2_t res;
  vec_uint4 denomZeros, numerZeros;
  vec_int4 shift;
  vec_ullong2 denomShifted, oneShifted, denomLeft, oneLeft;
  vec_ullong2 quot, newQuot;
  vec_ullong2 newNum, skip, cont;
  int       anyCont;

  // Get difference of leading zeros.

  denomZeros = (vec_uint4)__ll_spu_cntlz( denom );
  numerZeros = (vec_uint4)__ll_spu_cntlz( numer );

  shift = (vec_int4)spu_sub( denomZeros, numerZeros );

  // Shift denom to align leading one with numerator's

  denomShifted = __ll_spu_sl( denom, (vec_ullong2)shift );
  oneShifted = __ll_spu_sl( spu_splats(1ull), (vec_ullong2)shift );
  oneShifted = spu_sel( oneShifted, spu_splats(0ull), __ll_spu_cmpeq_zero( denom ) );

  // Shift left all leading zeros.

  denomLeft = __ll_spu_sl( denom, (vec_ullong2)denomZeros );
  oneLeft = __ll_spu_sl( spu_splats(1ull), (vec_ullong2)denomZeros );

  quot = spu_splats(0ull);

  do
    {
      cont = __ll_spu_cmpgt( oneShifted, spu_splats(0ull) );
      anyCont = spu_extract( spu_gather((vec_uint4)cont ), 0 );

      newQuot = spu_or( quot, oneShifted );

      // Subtract shifted denominator from remaining numerator 
      // when denominator is not greater.

      skip = __ll_spu_cmpgt( denomShifted, numer );
      newNum = __ll_spu_sub( numer, denomShifted );

      // If denominator is greater, next shift is one more, otherwise
      // next shift is number of leading zeros of remaining numerator.

      numerZeros = (vec_uint4)spu_sel( __ll_spu_cntlz( newNum ), (vec_ullong2)numerZeros, skip );
      shift = (vec_int4)spu_sub( (vec_uint4)skip, numerZeros );

      oneShifted = __ll_spu_rlmask( oneLeft, (vec_ullong2)shift );
      denomShifted = __ll_spu_rlmask( denomLeft, (vec_ullong2)shift );

      quot = spu_sel( newQuot, quot, skip );
      numer = spu_sel( newNum, numer, spu_orc(skip,cont) );
    } 
  while ( anyCont );

  res.quot = quot;
  res.rem = numer;
  return res;
}

#endif
