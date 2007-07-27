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

#ifndef ___SIMD_MATH_EXPF4_H___
#define ___SIMD_MATH_EXPF4_H___

#include <simdmath.h>
#include <spu_intrinsics.h>

#include <simdmath/divf4.h>
#include <simdmath/ldexpf4.h>

#define __EXPF_C1     -0.6931470632553101f
#define __EXPF_C2     -1.1730463525082e-7f

#define __EXPF_INVLN2  1.4426950408889634f

static inline vector float
_expf4 (vector float x)
{
  vec_uint4 xnegmask = spu_cmpgt(spu_splats(0.0f), x);
  vec_float4  goffset  = spu_sel(spu_splats(0.5f),spu_splats(-0.5f),xnegmask);
  vec_float4 g  = spu_mul(x, spu_splats(__EXPF_INVLN2));  
  vec_int4 xexp = spu_convts(spu_add(g, goffset),0);
  
  g = spu_convtf(xexp, 0);
  g = spu_madd(g, spu_splats(__EXPF_C2), spu_madd(g, spu_splats(__EXPF_C1), x));
  vec_float4 z  = spu_mul(g, g);
  vec_float4 a = spu_mul(z, spu_splats(0.0999748594f));
  vec_float4 b = spu_mul(g, 
			 spu_madd(z, 
				  spu_splats(0.0083208258f), 
				  spu_splats(0.4999999992f)
				  )
			 );
  
  vec_float4 foo  = _divf4(spu_add(spu_splats(1.0f), spu_add(a, b)),
			   spu_add(spu_splats(1.0f), spu_sub(a, b)));

  return _ldexpf4(foo, xexp);
  
}

#endif
