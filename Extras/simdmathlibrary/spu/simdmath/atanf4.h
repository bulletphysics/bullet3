/* atanf4 - 
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

#ifndef ___SIMD_MATH_ATANF4_H___
#define ___SIMD_MATH_ATANF4_H___

#include <simdmath.h>
#include <spu_intrinsics.h>

#include <simdmath/recipf4.h>

//
// Computes the inverse tangent of all four slots of x. 
//
static inline vector float
_atanf4 (vector float x)
{
  vec_float4 bias;
  vec_float4 x2, x3, x4, x8, x9;
  vec_float4 hi, lo;
  vec_float4 result;
  vec_float4 inv_x;
  vec_uint4 sign;
  vec_uint4 select;
    
  sign = spu_sl(spu_rlmask((vec_uint4)x, -31), 31);
  inv_x = _recipf4(x);
  inv_x = (vec_float4)spu_xor((vec_uint4)inv_x, spu_splats(0x80000000u));
    
  select = (vec_uint4)spu_cmpabsgt(x, spu_splats(1.0f));
  bias = (vec_float4)spu_or(sign, (vec_uint4)(spu_splats(1.57079632679489661923f)));
  bias = (vec_float4)spu_and((vec_uint4)bias, select);
    
  x = spu_sel(x, inv_x, select);
    
  bias = spu_add(bias, x);
  x2 = spu_mul(x, x);
  x3 = spu_mul(x2, x);
  x4 = spu_mul(x2, x2);
  x8 = spu_mul(x4, x4);
  x9 = spu_mul(x8, x);
  hi = spu_madd(spu_splats(0.0028662257f), x2, spu_splats(-0.0161657367f));
  hi = spu_madd(hi, x2, spu_splats(0.0429096138f));
  hi = spu_madd(hi, x2, spu_splats(-0.0752896400f));
  hi = spu_madd(hi, x2, spu_splats(0.1065626393f));
  lo = spu_madd(spu_splats(-0.1420889944f), x2, spu_splats(0.1999355085f));
  lo = spu_madd(lo, x2, spu_splats(-0.3333314528f));
  lo = spu_madd(lo, x3, bias);
    
  result = spu_madd(hi, x9, lo);
    
  return result;
}

#endif
