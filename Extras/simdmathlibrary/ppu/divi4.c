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
#include <altivec.h>

#include "common-types.h"



// divi4 - for each of four integer slots, compute quotient and remainder of numer/denom
// and store in divi4_t struct.  Divide by zero produces quotient = 0, remainder = numerator.

divi4_t
divi4 (vec_int4 numer, vec_int4 denom )
{
   vec_int4 minusone = vec_splatsi4(-1);
   vec_uint4 zero = vec_splatsu4(0);
   vec_uint4 one = vec_splatsu4(1);
   vec_uint4 k158 = vec_splatsu4(158);
   vec_uint4 k23 = vec_splatsu4(23);
   
   divi4_t res;
   vec_uint4 numerPos, denomPos, quotNeg;
   vec_uint4 numerAbs, denomAbs;
   vec_uint4 denomZeros, numerZeros, shift, denomShifted, oneShifted;
   vec_uint4 quot, newQuot, skip, newNum, cont;
   int       anyCont;
       
   // determine whether result needs sign change
 
   numerPos = (vec_uint4)vec_cmpgt( numer, minusone );
   denomPos = (vec_uint4)vec_cmpgt( denom, minusone );
   quotNeg = vec_xor( numerPos, denomPos );
   
   // use absolute values of numerator, denominator
 
   numerAbs = (vec_uint4)vec_sel( vec_sub( (vec_int4)zero, numer ), numer, numerPos );
   denomAbs = (vec_uint4)vec_sel( vec_sub( (vec_int4)zero, denom ), denom, denomPos );
 
   // get difference of leading zeros to align denom with numer

   denomZeros = vec_sub( k158, vec_sr( (vec_uint4)vec_ctf( denomAbs, 0 ), k23 ) );
   numerZeros = vec_sub( k158, vec_sr( (vec_uint4)vec_ctf( numerAbs, 0 ), k23 ) );
      
   shift = vec_sub( denomZeros, numerZeros );
   denomShifted = vec_sl( denomAbs, shift );
   oneShifted = vec_sl( one, shift );
   oneShifted = vec_sel( oneShifted, zero, vec_or( vec_cmpeq( denomAbs, zero ), 
                                                   vec_cmpgt( denomAbs, numerAbs ) ) );
   
   // long division

   quot = zero;
   
   do
   {
      cont = (vec_uint4)vec_cmpgt( oneShifted, zero );
      anyCont = vec_any_gt( oneShifted, zero );
      skip = (vec_uint4)vec_cmpgt( denomShifted, numerAbs );
      
      newQuot = vec_or( quot, oneShifted );
      newNum = vec_sub( numerAbs, denomShifted );
      
      oneShifted = vec_sr( oneShifted, one );
      denomShifted = vec_sr( denomShifted, one );
      
      quot = vec_sel( newQuot, quot, skip );
      numerAbs = vec_sel( numerAbs, newNum, vec_andc( cont, skip ) );
   }
   while ( anyCont );

   res.quot = (vec_int4)vec_sel( quot, vec_sub( zero, quot ), quotNeg );
   res.rem = (vec_int4)vec_sel( (vec_uint4)vec_sub( (vec_int4)zero, (vec_int4)numerAbs ), numerAbs, numerPos );
   return res;
}

