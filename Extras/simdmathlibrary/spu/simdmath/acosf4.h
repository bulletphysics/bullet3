/* acosf4 - 
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

#ifndef ___SIMD_MATH_ACOSF4_H___
#define ___SIMD_MATH_ACOSF4_H___

#include <simdmath.h>
#include <spu_intrinsics.h>

#include <simdmath/sqrtf4.h>

//
// Computes the inverse cosine of all four slots of x
//
static inline vector float
_acosf4 (vector float x)
{
  vec_float4 result, xabs;
  vec_float4 t1;
  vec_float4 xabs2, xabs4;
  vec_float4 hi, lo;
  vec_float4 neg, pos;
  vec_uint4 select;
    
  xabs   = (vec_float4)(spu_rlmask(spu_sl((vec_uint4)(x), 1), -1));
  select = (vec_uint4)(spu_rlmaska((vector signed int)(x), -31));
    
  t1 = _sqrtf4(spu_sub( spu_splats(1.0f), xabs));
    
  /* Instruction counts can be reduced if the polynomial was
   * computed entirely from nested (dependent) fma's. However, 
   * to reduce the number of pipeline stalls, the polygon is evaluated 
   * in two halves (hi amd lo). 
   */
  xabs2 = spu_mul(xabs,  xabs);
  xabs4 = spu_mul(xabs2, xabs2);
  hi = spu_madd(spu_splats(-0.0012624911f), xabs, spu_splats(0.0066700901f));
  hi = spu_madd(hi, xabs, spu_splats(-0.0170881256f));
  hi = spu_madd(hi, xabs, spu_splats( 0.0308918810f));
  lo = spu_madd(spu_splats(-0.0501743046f), xabs, spu_splats(0.0889789874f));
  lo = spu_madd(lo, xabs, spu_splats(-0.2145988016f));
  lo = spu_madd(lo, xabs, spu_splats( 1.5707963050f));
    
  result = spu_madd(hi, xabs4, lo);
    
  /* Adjust the result if x is negactive.
   */
  neg = spu_nmsub(t1, result, spu_splats(3.1415926535898f));
  pos = spu_mul(t1, result);
    
  result = spu_sel(pos, neg, select);
    
  return result;
}

#endif
