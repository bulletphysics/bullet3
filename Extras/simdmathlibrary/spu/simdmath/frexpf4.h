/* frexpf4 - for each element of vector x, return the normalized fraction and store the exponent of x'
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

#ifndef ___SIMD_MATH_FREXPF4_H___
#define ___SIMD_MATH_FREXPF4_H___

#include <simdmath.h>
#include <spu_intrinsics.h>

static inline vector float
_frexpf4 (vector float x, vector signed int *pexp)
{
  vec_int4 zeros = spu_splats((int)0);
  vec_uint4 zeromask = spu_cmpeq(x, (vec_float4)zeros);
 
  vec_uint4 expmask = spu_splats(0x7F800000U);
  vec_int4 e1 = spu_and((vec_int4)x, (vec_int4)expmask);
  vec_int4 e2 = spu_sub(spu_rlmask(e1,-23), spu_splats((int)126));
  *pexp = spu_sel(e2, zeros, zeromask);
 
  vec_float4 m2 = spu_sel(x, (vec_float4)spu_splats((int)0x3F000000), expmask);

  return spu_sel(m2, (vec_float4)zeros, zeromask);
}

#endif
