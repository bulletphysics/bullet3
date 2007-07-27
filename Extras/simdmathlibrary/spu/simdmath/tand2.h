/* tand2
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

#ifndef ___SIMD_MATH_TAND2_H___
#define ___SIMD_MATH_TAND2_H___

#include <simdmath.h>
#include <spu_intrinsics.h>

#include <simdmath/_sincos.h>
#include <simdmath/divd2.h>
#include <simdmath/isinfd2.h>
#include <simdmath/is0denormd2.h>

#define  __TAND_CC0 -0.00020844956382258822
#define  __TAND_CC1  0.02334489464693293380
#define  __TAND_CC2 -0.46161689768996201755
#define  __TAND_SC0 -0.00000748373924372997
#define  __TAND_SC1  0.00280592875035233052
#define  __TAND_SC2 -0.12828356435663158978

/*
 * Computes the tangent of the given angles by first reducing the
 * range to [-pi/4..pi/4] and performing the appropriate sin/cos ratio
 */
static inline vector double
_tand2 (vector double x)
{
  vec_double2 xl,x2,x3,res;
  vec_double2 nan = (vec_double2)spu_splats(0x7ff8000000000000ull);
  vec_uchar16 copyEven = (vec_uchar16)(vec_uint4){ 0x00010203, 0x00010203, 0x08090a0b, 0x08090a0b };

  // Range reduction using : xl = angle * TwoOverPi;
  //  
  xl = spu_mul(x, spu_splats(0.63661977236758134307553505349005744));


  // Find the quadrant the angle falls in
  // using:  q = (int) (ceil(abs(xl))*sign(xl))
  //
  xl = spu_add(xl,spu_sel(spu_splats(0.5),xl,spu_splats(0x8000000000000000ull)));
  vec_float4 xf = spu_roundtf(xl);
  vec_int4 q = spu_convts(xf,0);
  q = spu_shuffle(q,q,copyEven);

     
  // Remainder in range [-pi/4..pi/4]
  //
  vec_float4 qf = spu_convtf(q,0);
  vec_double2 qd = spu_extend(qf);
  vec_double2 p1 = spu_nmsub(qd,spu_splats(__SINCOSD_KC1),x);
  xl = spu_nmsub(qd,spu_splats(__SINCOSD_KC2),p1);
    
  // Compute x^2 and x^3
  //
  x2 = spu_mul(xl,xl);
  x3 = spu_mul(x2,xl);
    

  // Compute both the sin and cos of the angles
  // using a polynomial expression:
  //   cx = 1.0f + x2 * ((C0 * x2 + C1) * x2 + C2), and
  //   sx = x + x3 * ((S0 * x2 + S1) * x2 + S2)
  //
  vec_double2 ct1 = spu_madd(spu_splats(__TAND_CC0),x2,spu_splats(__TAND_CC1));
  vec_double2 st1 = spu_madd(spu_splats(__TAND_SC0),x2,spu_splats(__TAND_SC1));

  vec_double2 ct2 = spu_madd(ct1,x2,spu_splats(__TAND_CC2));
  vec_double2 st2 = spu_madd(st1,x2,spu_splats(__TAND_SC2));
    
  vec_double2 cx = spu_madd(ct2,x2,spu_splats(1.0));
  vec_double2 sx = spu_madd(st2,x3,xl);

    
  // Compute both cx/sx and sx/cx
  //
  vec_double2 cxosx = _divd2(cx,sx);
  vec_double2 sxocx = _divd2(sx,cx);

  vec_double2 ncxosx = (vec_double2)spu_xor(spu_splats(0x8000000000000000ull),(vec_ullong2)cxosx);

  // For odd numbered quadrants return -cx/sx , otherwise return
  // sx/cx
  //
  vec_ullong2 mask = (vec_ullong2)spu_cmpeq(spu_and(q,(int)0x1),spu_splats((int)0));
  res = spu_sel(ncxosx,sxocx,mask);

  // If input = +/-Inf return NAN
  //
  res = spu_sel(res,nan,_isinfd2 (x));

  // If input =0 or denorm return input
  //
  res = spu_sel(res,x, _is0denormd2 (x));

  return res;
}

#endif
