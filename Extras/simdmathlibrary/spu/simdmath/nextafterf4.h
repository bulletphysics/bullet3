/* nextafterf4  - for each of four float slots,
                  return the the next representable value after x in the direction fo y,
                  if x is euqal to y, the result is y.
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

#ifndef ___SIMD_MATH_NEXTAFTERF4_H___
#define ___SIMD_MATH_NEXTAFTERF4_H___

#include <simdmath.h>
#include <spu_intrinsics.h>


static inline vector float
_nextafterf4(vector float x, vector float y)
{
  vec_float4 x_not_dec, lala_inc, lala_dec;
  vec_uint4 abs_inc_number, abs_dec_number;

  vec_uint4 A, B;

  //abs_inc, abs_dec  
  abs_inc_number = spu_sel(spu_splats((unsigned int)0x800000), spu_add((vec_uint4)x, spu_splats((unsigned int)0x1)), spu_cmpabsgt(x, spu_splats(0.0f)));
  abs_dec_number = (vec_uint4)spu_add((vec_float4)spu_sub((vec_uint4)x, spu_splats((unsigned int)0x1)), spu_splats(0.0f));

  //x<= y 
  A=  spu_andc(abs_inc_number, spu_splats((unsigned int)0x80000000));
  // in < 0
  B= abs_dec_number;

  lala_inc = spu_sel((vec_float4)A, (vec_float4)B, spu_cmpgt(spu_splats(0.0f), x));

  // in <=0,   abs_inc ( if in==0, set result's sign to -)
  //A= spu_or(spu_splats((unsigned int)0x80000000), spu_andc(abs_inc_number, spu_splats((unsigned int)0x80000000)));
  A= spu_or(abs_inc_number, spu_splats((unsigned int)0x80000000));
  // in > 0
  B = abs_dec_number;
  lala_dec = spu_sel((vec_float4)A, (vec_float4)B, spu_cmpgt(x, spu_splats(0.0f)));


  x_not_dec = spu_sel(y, lala_inc, spu_cmpgt(y, x));

  // (x <= y) ||  (x > y)
  return spu_sel(x_not_dec, lala_dec, spu_cmpgt(x, y));
}

#endif
