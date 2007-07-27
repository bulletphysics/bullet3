/* divu4 - 
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

#ifndef ___SIMD_MATH_DIVU4_H___
#define ___SIMD_MATH_DIVU4_H___

#include <simdmath.h>
#include <spu_intrinsics.h>

// divu4 - for each of four unsigned integer slots, compute quotient and remainder of numer/denom
// and store in divu4_t struct.  Divide by zero produces quotient = 0, remainder = numerator.

static inline divu4_t
_divu4 (vector unsigned int numer, vector unsigned int denom)
{
  divu4_t res;
  vec_int4 shift;
  vec_uint4 quot, newQuot;
  vec_uint4 denomZeros, numerZeros, denomLeft, oneLeft, denomShifted, oneShifted;
  vec_uint4 newNum, skip, cont;
  int       anyCont;

  // Get difference of leading zeros.
  // Any possible negative value will be interpreted as a shift > 31

  denomZeros = spu_cntlz( denom );
  numerZeros = spu_cntlz( numer );

  shift = (vec_int4)spu_sub( denomZeros, numerZeros );

  // Shift denom to align leading one with numerator's

  denomShifted = spu_sl( denom, (vec_uint4)shift );
  oneShifted = spu_sl( spu_splats(1U), (vec_uint4)shift );
  oneShifted = spu_sel( oneShifted, spu_splats(0U), spu_cmpeq( denom, 0 ) );

  // Shift left all leading zeros.

  denomLeft = spu_sl( denom, denomZeros );
  oneLeft = spu_sl( spu_splats(1U), denomZeros );

  quot = spu_splats(0U);

  do
    {
      cont = spu_cmpgt( oneShifted, 0U );
      anyCont = spu_extract( spu_gather( cont ), 0 );

      newQuot = spu_or( quot, oneShifted );

      // Subtract shifted denominator from remaining numerator 
      // when denominator is not greater.

      skip = spu_cmpgt( denomShifted, numer );
      newNum = spu_sub( numer, denomShifted );

      // If denominator is greater, next shift is one more, otherwise
      // next shift is number of leading zeros of remaining numerator.

      numerZeros = spu_sel( spu_cntlz( newNum ), numerZeros, skip );
      shift = (vec_int4)spu_sub( skip, numerZeros );

      oneShifted = spu_rlmask( oneLeft, shift );
      denomShifted = spu_rlmask( denomLeft, shift );

      quot = spu_sel( newQuot, quot, skip );
      numer = spu_sel( newNum, numer, spu_orc(skip,cont) );
    } 
  while ( anyCont );

  res.quot = quot;
  res.rem = numer;
  return res;
}

#endif
