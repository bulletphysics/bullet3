/* tanf4 - 
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

#ifndef ___SIMD_MATH_TANF4_H___
#define ___SIMD_MATH_TANF4_H___

#include <simdmath.h>
#include <altivec.h>

#include <simdmath/_sincos.h>
#include <simdmath/divf4.h>

//
// Computes the tangent of all four slots of x by using a polynomia approximation. 
//
static inline vector float
_tanf4 (vector float x)
{
  vector float xl,xl2,xl3,res;
  vector signed int q;

  // Range reduction using : xl = angle * TwoOverPi;
  //  
  xl = vec_madd(x, __vec_splatsf4(0.63661977236f),__vec_splatsf4(0.0f));

  // Find the quadrant the angle falls in
  // using:  q = (int) (ceil(abs(x))*sign(x))
  //
  xl = vec_add(xl,vec_sel(__vec_splatsf4(0.5f),xl,__vec_splatsu4(0x80000000)));
  q = vec_cts(xl,0);

     
  // Remainder in range [-pi/4..pi/4]
  //
  vector float qf = vec_ctf(q,0);
  vector float p1 = vec_nmsub(qf,__vec_splatsf4(__SINCOSF_KC1),x);
  xl  = vec_nmsub(qf,__vec_splatsf4(__SINCOSF_KC2),p1);
    
  // Compute x^2 and x^3
  //
  xl2 = vec_madd(xl,xl,__vec_splatsf4(0.0f));
  xl3 = vec_madd(xl2,xl,__vec_splatsf4(0.0f));
 
  // Compute both the sin and cos of the angles
  // using a polynomial expression:
  //   cx = 1.0f + x2 * (C0 * x2 + C1), and
  //   sx = xl + x3 * S0
  //
  vector float ct2 = vec_madd(__vec_splatsf4( 0.0097099364f),xl2,__vec_splatsf4(-0.4291161787f));
    
  vector float cx = vec_madd(ct2,xl2,__vec_splatsf4(1.0f));
  vector float sx = vec_madd(__vec_splatsf4(-0.0957822992f),xl3,xl);

    
  // Compute both cx/sx and sx/cx
  //
  vector float cxosx = _divf4(cx,sx);
  vector float sxocx = _divf4(sx,cx);

  vector float ncxosx = (vector float)vec_xor(__vec_splatsu4(0x80000000),(vector unsigned int)cxosx);

  // For odd numbered quadrants return -cx/sx , otherwise return
  // sx/cx
  //
  vector unsigned int mask =
    (vector unsigned int)vec_cmpeq(vec_and(q,__vec_splatsi4(0x1)),__vec_splatsi4(0));
  res = vec_sel(ncxosx,sxocx,mask);

  return res;
}

#endif
