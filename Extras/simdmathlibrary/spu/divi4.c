/* divi4 - 
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

#include <simdmath.h>
#include <spu_intrinsics.h>

// divi4 - for each of four integer slots, compute quotient and remainder of numer/denom
// and store in divi4_t struct.  Divide by zero produces quotient = 0, remainder = numerator.

divi4_t divi4 (vector signed int numer, vector signed int denom)
{
   divi4_t res;
   vec_int4 quot, newQuot, shift;
   vec_uint4 numerPos, denomPos, quotNeg;
   vec_uint4 numerAbs, denomAbs;
   vec_uint4 denomZeros, numerZeros, denomLeft, oneLeft, denomShifted, oneShifted;
   vec_uint4 newNum, skip, cont;
   int       anyCont;

   // Determine whether result needs sign change

   numerPos = spu_cmpgt( numer, -1 );
   denomPos = spu_cmpgt( denom, -1 );
   quotNeg = spu_xor( numerPos, denomPos );
    
   // Use absolute values of numerator, denominator

   numerAbs = (vec_uint4)spu_sel( spu_sub( 0, numer ), numer, numerPos );
   denomAbs = (vec_uint4)spu_sel( spu_sub( 0, denom ), denom, denomPos );

   // Get difference of leading zeros.
   // Any possible negative value will be interpreted as a shift > 31

   denomZeros = spu_cntlz( denomAbs );
   numerZeros = spu_cntlz( numerAbs );

   shift = (vec_int4)spu_sub( denomZeros, numerZeros );

   // Shift denom to align leading one with numerator's

   denomShifted = spu_sl( denomAbs, (vec_uint4)shift );
   oneShifted = spu_sl( (vec_uint4)spu_splats(1), (vec_uint4)shift );
   oneShifted = spu_sel( oneShifted, (vec_uint4)spu_splats(0), spu_cmpeq( denom, 0 ) );

   // Shift left all leading zeros.

   denomLeft = spu_sl( denomAbs, denomZeros );
   oneLeft = spu_sl( (vec_uint4)spu_splats(1), denomZeros );

   quot = spu_splats(0);

   do
   {
      cont = spu_cmpgt( oneShifted, 0U );
      anyCont = spu_extract( spu_gather( cont ), 0 );

      newQuot = spu_or( quot, (vec_int4)oneShifted );

      // Subtract shifted denominator from remaining numerator 
      // when denominator is not greater.

      skip = spu_cmpgt( denomShifted, numerAbs );
      newNum = spu_sub( numerAbs, denomShifted );

      // If denominator is greater, next shift is one more, otherwise
      // next shift is number of leading zeros of remaining numerator.

      numerZeros = spu_sel( spu_cntlz( newNum ), numerZeros, skip );
      shift = (vec_int4)spu_sub( skip, numerZeros );

      oneShifted = spu_rlmask( oneLeft, shift );
      denomShifted = spu_rlmask( denomLeft, shift );

      quot = spu_sel( newQuot, quot, skip );
      numerAbs = spu_sel( newNum, numerAbs, spu_orc(skip,cont) );
   } 
   while ( anyCont );

   res.quot = spu_sel( quot, spu_sub( 0, quot ), quotNeg );
   res.rem = spu_sel( spu_sub( 0, (vec_int4)numerAbs ), (vec_int4)numerAbs, numerPos );
   return res;
}

