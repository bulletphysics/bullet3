/* expm1f4 - 
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

#ifndef ___SIMD_MATH_EXPMLF4_H___
#define ___SIMD_MATH_EXPMLF4_H___

#include <simdmath.h>
#include <spu_intrinsics.h>

#include <simdmath/expf4.h>
#include <simdmath/divf4.h>

#define __EXPM1F_ln1by2 -0.6931471805599f
#define __EXPM1F_ln3by2  0.4054651081082f

static inline vector float
_expm1f4 (vector float x)
{
  vec_uint4 nearzeromask = spu_and(spu_cmpgt(x, spu_splats(__EXPM1F_ln1by2)),
				   spu_cmpgt(spu_splats(__EXPM1F_ln3by2), x));
  vec_float4 x2 = spu_mul(x,x);
  vec_float4 d0, d1, n0, n1;
  
  d0 = spu_madd(x , spu_splats(-0.3203561199f), spu_splats(0.9483177697f));
  d1 = spu_madd(x2, spu_splats(0.0326527809f), d0);
  
  n0 = spu_madd(x , spu_splats(0.1538026623f), spu_splats(0.9483177732f));
  n1 = spu_madd(x , spu_splats(0.0024490478f), spu_splats(0.0305274668f));
  n1 = spu_madd(x2, n1, n0);
 
  return spu_sel(spu_sub(_expf4(x), spu_splats(1.0f)),
                 spu_mul(x, _divf4(n1, d1)),
                 nearzeromask);
}

#endif
