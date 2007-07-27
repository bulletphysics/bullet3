/* sincosf4
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

#ifndef ___SIMD_MATH_SINCOSF4_H___
#define ___SIMD_MATH_SINCOSF4_H___

#include <simdmath.h>
#include <spu_intrinsics.h>

#include <simdmath/_sincos.h>

//
//     Computes both the sine and cosine of the all four slots of x
//     by using a polynomial approximation.
//
static inline void
_sincosf4 (vector float x, vector float *s, vector float *c)
{
  vec_float4 xl,xl2,xl3;
  vec_int4   q;
  vec_int4   offsetSin, offsetCos;
  vec_float4 ts, tc;

  // Range reduction using : xl = angle * TwoOverPi;
  //  
  xl = spu_mul(x, spu_splats(0.63661977236f));

  // Find the quadrant the angle falls in
  // using:  q = (int) (ceil(abs(xl))*sign(xl))
  //
  xl = spu_add(xl,spu_sel(spu_splats(0.5f),xl,spu_splats(0x80000000)));
  q = spu_convts(xl,0);

     
  // Compute the offset based on the quadrant that the angle falls in.
  // Add 1 to the offset for the cosine. 
  //
  offsetSin = spu_and(q,spu_splats((int)0x3));
  offsetCos = spu_add(spu_splats(1),offsetSin);

  // Remainder in range [-pi/4..pi/4]
  //
  vec_float4 qf = spu_convtf(q,0);
  vec_float4 p1 = spu_nmsub(qf,spu_splats(__SINCOSF_KC1),x);
  xl  = spu_nmsub(qf,spu_splats(__SINCOSF_KC2),p1);
    
  // Compute x^2 and x^3
  //
  xl2 = spu_mul(xl,xl);
  xl3 = spu_mul(xl2,xl);
    

  // Compute both the sin and cos of the angles
  // using a polynomial expression:
  //   cx = 1.0f + xl2 * ((C0 * xl2 + C1) * xl2 + C2), and
  //   sx = xl + xl3 * ((S0 * xl2 + S1) * xl2 + S2)
  //
  vec_float4 ct1 = spu_madd(spu_splats(__SINCOSF_CC0),xl2,spu_splats(__SINCOSF_CC1));
  vec_float4 st1 = spu_madd(spu_splats(__SINCOSF_SC0),xl2,spu_splats(__SINCOSF_SC1));

  vec_float4 ct2 = spu_madd(ct1,xl2,spu_splats(__SINCOSF_CC2));
  vec_float4 st2 = spu_madd(st1,xl2,spu_splats(__SINCOSF_SC2));
    
  vec_float4 cx = spu_madd(ct2,xl2,spu_splats(1.0f));
  vec_float4 sx = spu_madd(st2,xl3,xl);

  // Use the cosine when the offset is odd and the sin
  // when the offset is even
  //
  vec_uint4 sinMask = spu_cmpeq(spu_and(offsetSin,(int)0x1),spu_splats((int)0));
  vec_uint4 cosMask = spu_cmpeq(spu_and(offsetCos,(int)0x1),spu_splats((int)0));
  ts = spu_sel(cx,sx,sinMask);
  tc = spu_sel(cx,sx,cosMask);

  // Flip the sign of the result when (offset mod 4) = 1 or 2
  //
  sinMask = spu_cmpeq(spu_and(offsetSin,(int)0x2),spu_splats((int)0));
  cosMask = spu_cmpeq(spu_and(offsetCos,(int)0x2),spu_splats((int)0));
    
  ts = spu_sel((vec_float4)spu_xor(spu_splats(0x80000000),(vec_uint4)ts),ts,sinMask);
  tc = spu_sel((vec_float4)spu_xor(spu_splats(0x80000000),(vec_uint4)tc),tc,cosMask);

  *s = ts;
  *c = tc;
}

#endif
