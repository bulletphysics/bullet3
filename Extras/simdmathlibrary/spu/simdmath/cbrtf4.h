/* cbrtf4 - 
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

#ifndef ___SIMD_MATH_CBRTF4_H___
#define ___SIMD_MATH_CBRTF4_H___

#include <simdmath.h>
#include <spu_intrinsics.h>

#include <simdmath/frexpf4.h>
#include <simdmath/ldexpf4.h>
#include <simdmath/divf4.h>

static inline vec_int4
__cbrtf4_calc_quot(vec_int4 n)
{
  vec_int4 quot;
  vec_uint4 negxexpmask = spu_cmpgt(spu_splats(0), n);
  n = spu_sel(n, spu_add(n,2), negxexpmask);

  quot = spu_add(spu_rlmaska(n,-2), spu_rlmaska(n,-4));
  quot = spu_add(quot, spu_rlmaska(quot, -4));
  quot = spu_add(quot, spu_rlmaska(quot, -8));
  quot = spu_add(quot, spu_rlmaska(quot,-16));
  vec_int4 r = spu_sub(spu_sub(n,quot), spu_sl(quot,1));
  quot = spu_add(quot, spu_rlmaska(spu_add(spu_add(r,5), spu_sl (r,2)), -4));
  return quot;
}

#define __CBRTF_cbrt2 1.2599210498948731648             // 2^(1/3)
#define __CBRTF_sqr_cbrt2 1.5874010519681994748         // 2^(2/3)

static inline vector float
_cbrtf4 (vector float x)
{
  vec_float4 zeros = spu_splats(0.0f);
  vec_uint4 zeromask = spu_cmpeq(x, zeros);
  vec_int4 xexp;
  vec_float4 sgnmask = (vec_float4)spu_splats(0x7FFFFFFF);
  vec_uint4 negmask = spu_cmpgt(spu_splats(0.0f), x);
  x = spu_and(x, sgnmask);

  x = _frexpf4(x, &xexp);
  vec_float4 p = spu_madd(
			  spu_madd(x, spu_splats(-0.191502161678719066f), spu_splats(0.697570460207922770f)),
			  x,
			  spu_splats(0.492659620528969547f)
			  );
  vec_float4 p3 = spu_mul(p, spu_mul(p, p));
  vec_int4 quot = __cbrtf4_calc_quot(xexp);
  vec_int4 modval = spu_sub(spu_sub(xexp,quot), spu_sl(quot,1)); // mod = xexp - 3*quotient
  vec_float4 factor = spu_splats((float)(1.0/__CBRTF_sqr_cbrt2));
  factor = spu_sel(factor, spu_splats((float)(1.0/__CBRTF_cbrt2)), spu_cmpeq(modval,-1));
  factor = spu_sel(factor, spu_splats((float)(      1.0)), spu_cmpeq(modval, 0));
  factor = spu_sel(factor, spu_splats((float)(    __CBRTF_cbrt2)), spu_cmpeq(modval, 1));
  factor = spu_sel(factor, spu_splats((float)(__CBRTF_sqr_cbrt2)), spu_cmpeq(modval, 2));

  vec_float4 pre  = spu_mul(p, factor);
  vec_float4 numr = spu_madd(x , spu_splats(2.0f), p3);
  vec_float4 denr = spu_madd(p3, spu_splats(2.0f), x );
  vec_float4 res = spu_mul(pre, _divf4(numr, denr));
  res = _ldexpf4(res, quot);

  return spu_sel(spu_sel(res, spu_orc(res,sgnmask), negmask),
		 zeros,
		 zeromask);
}

#endif
