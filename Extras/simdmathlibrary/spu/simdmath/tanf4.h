/* tanf4 - for each of four float slots, compute the tangent by using a polynomial approximation.
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
#include <spu_intrinsics.h>

#include <simdmath/_sincos.h>
#include <simdmath/divf4.h>

static inline vector float
_tanf4 (vector float x)
{
  vector float xl,x2,x3,res;
  vector signed int q;

  // Range reduction using : xl = angle * TwoOverPi;
  //  
  xl = spu_mul(x, spu_splats(0.63661977236f));

  // Find the quadrant the angle falls in
  // using:  q = (int) (ceil(abs(x))*sign(x))
  //
  xl = spu_add(xl,spu_sel(spu_splats(0.5f),xl,spu_splats(0x80000000)));
  q = spu_convts(xl,0);

     
  // Remainder in range [-pi/4..pi/4]
  //
  vec_float4 qf = spu_convtf(q,0);
  vec_float4 p1 = spu_nmsub(qf,spu_splats(__SINCOSF_KC1),x);
  xl  = spu_nmsub(qf,spu_splats(__SINCOSF_KC2),p1);
    
  // Compute x^2 and x^3
  //
  x2 = spu_mul(xl,xl);
  x3 = spu_mul(x2,xl);
    

  // Compute both the sin and cos of the angles
  // using a polynomial expression:
  //   cx = 1.0f + x2 * (C0 * x2 + C1), and
  //   sx = xl + x3 * S0
  //
  vec_float4 ct2 = spu_madd(spu_splats( 0.0097099364f),x2,spu_splats(-0.4291161787f));
    
  vec_float4 cx = spu_madd(ct2,x2,spu_splats(1.0f));
  vec_float4 sx = spu_madd(spu_splats(-0.0957822992f),x3,xl);

    
  // Compute both cx/sx and sx/cx
  //
  vec_float4 cxosx = _divf4(cx,sx);
  vec_float4 sxocx = _divf4(sx,cx);

  vec_float4 ncxosx = (vec_float4)spu_xor(spu_splats(0x80000000),(vec_uint4)cxosx);

  // For odd numbered quadrants return -cx/sx , otherwise return
  // sx/cx
  //
  vec_uint4 mask = spu_cmpeq(spu_and(q,(int)0x1),spu_splats((int)0));
  res = spu_sel(ncxosx,sxocx,mask);

  return res;
}

#endif
