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
#include <spu_intrinsics.h>

#include <simdmath/divu4.h>

// divi4 - for each of four integer slots, compute quotient and remainder of numer/denom
// and store in divi4_t struct.  Divide by zero produces quotient = 0, remainder = numerator.

static inline divi4_t
_divi4 (vector signed int numer, vector signed int denom)
{
  divu4_t resAbs;
  divi4_t res;
  vec_uint4 numerPos, denomPos, quotNeg;
  vec_uint4 numerAbs, denomAbs;

  // Determine whether result needs sign change

  numerPos = spu_cmpgt( numer, -1 );
  denomPos = spu_cmpgt( denom, -1 );
  quotNeg = spu_xor( numerPos, denomPos );
    
  // Use absolute values of numerator, denominator

  numerAbs = (vec_uint4)spu_sel( spu_sub( 0, numer ), numer, numerPos );
  denomAbs = (vec_uint4)spu_sel( spu_sub( 0, denom ), denom, denomPos );

  resAbs = _divu4(numerAbs, denomAbs);

  res.quot = spu_sel( (vec_int4)resAbs.quot, spu_sub( 0, (vec_int4)resAbs.quot ), quotNeg );
  res.rem = spu_sel( spu_sub( 0, (vec_int4)resAbs.rem ), (vec_int4)resAbs.rem, numerPos );
  return res;
}

#endif
