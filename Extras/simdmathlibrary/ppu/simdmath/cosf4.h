/* cosf4 - 
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

#ifndef ___SIMD_MATH_COSF4_H___
#define ___SIMD_MATH_COSF4_H___

#include <simdmath.h>
#include <altivec.h>

#include <simdmath/_sincos.h>
#include <simdmath/_vec_utils.h>


//
//     Computes the cosine of each of the four slots
//     by using a polynomial approximation.
//
static inline vector float
_cosf4 (vector float x)
{
  vector float xl,xl2,xl3,res;
  vector signed int   q;

  // Range reduction using : xl = angle * TwoOverPi;
  //  
  xl = vec_madd(x, __vec_splatsf4(0.63661977236f), __vec_splatsf4(0.0f));

  // Find the quadrant the angle falls in
  // using:  q = (int) (ceil(abs(xl))*sign(xl))
  //
  xl = vec_add(xl, vec_sel(__vec_splatsf4(0.5f), xl, __vec_splatsu4(0x80000000)));
  q = vec_cts(xl, 0);

     
  // Compute an offset based on the quadrant that the angle falls in
  // 
  vector signed int offset = vec_add(__vec_splatsi4(1), vec_and(q, __vec_splatsi4(0x3)));

  // Remainder in range [-pi/4..pi/4]
  //
  vector float qf = vec_ctf(q,0);
  vector float p1 = vec_nmsub(qf, __vec_splatsf4(__SINCOSF_KC1), x);
  xl  = vec_nmsub(qf, __vec_splatsf4(__SINCOSF_KC2), p1);
    
  // Compute x^2 and x^3
  //
  xl2 = vec_madd(xl, xl, __vec_splatsf4(0.0f));
  xl3 = vec_madd(xl2, xl, __vec_splatsf4(0.0f));
    

  // Compute both the sin and cos of the angles
  // using a polynomial expression:
  //   cx = 1.0f + xl2 * ((C0 * xl2 + C1) * xl2 + C2), and
  //   sx = xl + xl3 * ((S0 * xl2 + S1) * xl2 + S2)
  //
  vector float ct1 = vec_madd(__vec_splatsf4(__SINCOSF_CC0), xl2, __vec_splatsf4(__SINCOSF_CC1));
  vector float st1 = vec_madd(__vec_splatsf4(__SINCOSF_SC0), xl2, __vec_splatsf4(__SINCOSF_SC1));

  vector float ct2 = vec_madd(ct1, xl2, __vec_splatsf4(__SINCOSF_CC2));
  vector float st2 = vec_madd(st1, xl2, __vec_splatsf4(__SINCOSF_SC2));
    
  vector float cx = vec_madd(ct2, xl2, __vec_splatsf4(1.0f));
  vector float sx = vec_madd(st2, xl3, xl);

  // Use the cosine when the offset is odd and the sin
  // when the offset is even
  //
  vector unsigned int mask1 =
    (vector unsigned int)vec_cmpeq(vec_and(offset, __vec_splatsi4(0x1)), __vec_splatsi4(0));
  res = vec_sel(cx, sx, mask1);

  // Flip the sign of the result when (offset mod 4) = 1 or 2
  //
  vector unsigned int mask2 =
    (vector unsigned int)vec_cmpeq(vec_and(offset, __vec_splatsi4(0x2)), __vec_splatsi4(0));
  res = vec_sel((vector float)vec_xor(__vec_splatsu4(0x80000000U), (vector unsigned int)res), res, mask2);

  return res;
}

#endif
