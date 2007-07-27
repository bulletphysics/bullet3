/* log1pf4 - 
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

#ifndef ___SIMD_MATH_LOG1PF4_H___
#define ___SIMD_MATH_LOG1PF4_H___

#include <simdmath.h>
#include <spu_intrinsics.h>

#include <simdmath/logf4.h>
#include <simdmath/divf4.h>

static inline vector float
_log1pf4 (vector float x)
{
  vec_uint4 nearzeromask = spu_and(spu_cmpgt(x, spu_splats(-0.5f)),
				   spu_cmpgt(spu_splats(0.5f), x));
  vec_float4 x2 = spu_mul(x,x);
  vec_float4 d0, d1, n0, n1;

  d0 = spu_madd(x , spu_splats(1.5934420741f), spu_splats(0.8952856868f));
  d1 = spu_madd(x , spu_splats(0.1198195734f), spu_splats(0.8377145063f));
  d1 = spu_madd(x2, d1, d0);
  
  n0 = spu_madd(x , spu_splats(1.1457993413f), spu_splats(0.8952856678f));
  n1 = spu_madd(x , spu_splats(0.0082862580f), spu_splats(0.3394238808f));
  n1 = spu_madd(x2, n1, n0);
 
  return spu_sel(_logf4(spu_add(x, spu_splats(1.0f))),
                 spu_mul(x, _divf4(n1, d1)),
                 nearzeromask);
}

#endif
