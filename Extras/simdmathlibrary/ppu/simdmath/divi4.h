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

#ifndef ___SIMD_MATH_DIVI4_H___
#define ___SIMD_MATH_DIVI4_H___

#include <simdmath.h>
#include <altivec.h>

#include <simdmath/_vec_utils.h>

// divi4 - for each of four integer slots, compute quotient and remainder of numer/denom
// and store in divi4_t struct.  Divide by zero produces quotient = 0, remainder = numerator.

static inline divi4_t
_divi4 (vector signed int numer, vector signed int denom )
{
  vector signed int minusone = __vec_splatsi4(-1);
  vector unsigned int zero = __vec_splatsu4(0);
  vector unsigned int one = __vec_splatsu4(1);
  vector unsigned int k158 = __vec_splatsu4(158);
  vector unsigned int k23 = __vec_splatsu4(23);
   
  divi4_t res;
  vector unsigned int numerPos, denomPos, quotNeg;
  vector unsigned int numerAbs, denomAbs;
  vector unsigned int denomZeros, numerZeros, shift, denomShifted, oneShifted;
  vector unsigned int quot, newQuot, skip, newNum, cont;
  int       anyCont;
       
  // determine whether result needs sign change
 
  numerPos = (vector unsigned int)vec_cmpgt( numer, minusone );
  denomPos = (vector unsigned int)vec_cmpgt( denom, minusone );
  quotNeg = vec_xor( numerPos, denomPos );
   
  // use absolute values of numerator, denominator
 
  numerAbs = (vector unsigned int)vec_sel( vec_sub( (vector signed int)zero, numer ), numer, numerPos );
  denomAbs = (vector unsigned int)vec_sel( vec_sub( (vector signed int)zero, denom ), denom, denomPos );
 
  // get difference of leading zeros to align denom with numer

  denomZeros = vec_sub( k158, vec_sr( (vector unsigned int)vec_ctf( denomAbs, 0 ), k23 ) );
  numerZeros = vec_sub( k158, vec_sr( (vector unsigned int)vec_ctf( numerAbs, 0 ), k23 ) );
      
  shift = vec_sub( denomZeros, numerZeros );
  denomShifted = vec_sl( denomAbs, shift );
  oneShifted = vec_sl( one, shift );
  oneShifted = vec_sel( oneShifted, zero, vec_or( vec_cmpeq( denomAbs, zero ), 
						  vec_cmpgt( denomAbs, numerAbs ) ) );
   
  // long division

  quot = zero;
   
  do
    {
      cont = (vector unsigned int)vec_cmpgt( oneShifted, zero );
      anyCont = vec_any_gt( oneShifted, zero );
      skip = (vector unsigned int)vec_cmpgt( denomShifted, numerAbs );
      
      newQuot = vec_or( quot, oneShifted );
      newNum = vec_sub( numerAbs, denomShifted );
      
      oneShifted = vec_sr( oneShifted, one );
      denomShifted = vec_sr( denomShifted, one );
      
      quot = vec_sel( newQuot, quot, skip );
      numerAbs = vec_sel( numerAbs, newNum, vec_andc( cont, skip ) );
    }
  while ( anyCont );

  res.quot = (vector signed int)vec_sel( quot, vec_sub( zero, quot ), quotNeg );
  res.rem = (vector signed int)vec_sel( (vector unsigned int)vec_sub( (vector signed int)zero, (vector signed int)numerAbs ), numerAbs, numerPos );
  return res;
}

#endif
