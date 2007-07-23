/* sincosf4 -
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

#include "sincos_c.h"
#include "common-types.h"

//
//     Computes both the sine and cosine of the all four slots of x
//     by using a polynomial approximation.
//
void sincosf4 (vector float x, vector float *s, vector float *c)
{
    vec_float4 xl,xl2,xl3;
    vec_int4   q;
    vec_int4   offsetSin, offsetCos;

    // Range reduction using : xl = angle * TwoOverPi;
    //  
    xl = vec_madd(x, vec_splatsf4(0.63661977236f),vec_splatsf4(0.0f));

    // Find the quadrant the angle falls in
    // using:  q = (int) (ceil(abs(xl))*sign(xl))
    //
    xl = vec_add(xl,vec_sel(vec_splatsf4(0.5f),xl,vec_splatsu4(0x80000000)));
    q = vec_cts(xl,0);

     
    // Compute the offset based on the quadrant that the angle falls in.
    // Add 1 to the offset for the cosine. 
    //
    offsetSin = vec_and(q,vec_splatsi4((int)0x3));
    offsetCos = vec_add(vec_splatsi4(1),offsetSin);

    // Remainder in range [-pi/4..pi/4]
    //
    vec_float4 qf = vec_ctf(q,0);
    vec_float4 p1 = vec_nmsub(qf,vec_splatsf4(_SINCOS_KC1),x);
    xl  = vec_nmsub(qf,vec_splatsf4(_SINCOS_KC2),p1);
    
    // Compute x^2 and x^3
    //
    xl2 = vec_madd(xl,xl,vec_splatsf4(0.0f));
    xl3 = vec_madd(xl2,xl,vec_splatsf4(0.0f));
    

    // Compute both the sin and cos of the angles
    // using a polynomial expression:
    //   cx = 1.0f + xl2 * ((C0 * xl2 + C1) * xl2 + C2), and
    //   sx = xl + xl3 * ((S0 * xl2 + S1) * xl2 + S2)
    //
    vec_float4 ct1 = vec_madd(vec_splatsf4(_SINCOS_CC0),xl2,vec_splatsf4(_SINCOS_CC1));
    vec_float4 st1 = vec_madd(vec_splatsf4(_SINCOS_SC0),xl2,vec_splatsf4(_SINCOS_SC1));

    vec_float4 ct2 = vec_madd(ct1,xl2,vec_splatsf4(_SINCOS_CC2));
    vec_float4 st2 = vec_madd(st1,xl2,vec_splatsf4(_SINCOS_SC2));
    
    vec_float4 cx = vec_madd(ct2,xl2,vec_splatsf4(1.0f));
    vec_float4 sx = vec_madd(st2,xl3,xl);

    // Use the cosine when the offset is odd and the sin
    // when the offset is even
    //
    vec_uint4 sinMask = (vec_uint4)vec_cmpeq(vec_and(offsetSin,vec_splatsi4(0x1)),vec_splatsi4(0));
    vec_uint4 cosMask = (vec_uint4)vec_cmpeq(vec_and(offsetCos,vec_splatsi4(0x1)),vec_splatsi4(0));    
    *s = vec_sel(cx,sx,sinMask);
    *c = vec_sel(cx,sx,cosMask);

    // Flip the sign of the result when (offset mod 4) = 1 or 2
    //
    sinMask = (vec_uint4)vec_cmpeq(vec_and(offsetSin,vec_splatsi4(0x2)),vec_splatsi4(0));
    cosMask = (vec_uint4)vec_cmpeq(vec_and(offsetCos,vec_splatsi4(0x2)),vec_splatsi4(0));
    
    *s = vec_sel((vec_float4)vec_xor(vec_splatsu4(0x80000000),(vec_uint4)*s),*s,sinMask);
    *c = vec_sel((vec_float4)vec_xor(vec_splatsu4(0x80000000),(vec_uint4)*c),*c,cosMask);
    
}

