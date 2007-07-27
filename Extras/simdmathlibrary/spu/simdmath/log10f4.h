/* log10f4 - 
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

#ifndef ___SIMD_MATH_LOG10F4_H___
#define ___SIMD_MATH_LOG10F4_H___

#include <simdmath.h>
#include <spu_intrinsics.h>

#include <simdmath/divf4.h>

#define __LOG10F_loga2msb 0.3010299205780f 
#define __LOG10F_loga2lsb 7.5085978266e-8f
#define __LOG10F_logaemsb 0.4342944622040f
#define __LOG10F_logaelsb 1.9699272335e-8f
#define __LOG10F_logae    0.4342944819033f

#define __LOG10F_c0 0.2988439998f 
#define __LOG10F_c1 0.3997655209f
#define __LOG10F_c2 0.6666679125f

static inline vector float
_log10f4 (vector float x)
{
  vec_int4 zeros = spu_splats((int)0);
  vec_float4 ones = spu_splats(1.0f);
  vec_uint4 zeromask = spu_cmpeq(x, (vec_float4)zeros);

  vec_uint4 expmask = spu_splats(0x7F800000U);
  vec_int4 xexp = spu_add( spu_rlmask(spu_and((vec_int4)x, (vec_int4)expmask), -23), -126 );
  x = spu_sel(x, (vec_float4)spu_splats((int)0x3F000000), expmask);

  vec_uint4  mask = spu_cmpgt(spu_splats(0.7071067811865f), x);
  x    = spu_sel(x   , spu_add(x, x)                   , mask);
  xexp = spu_sel(xexp, spu_sub(xexp,spu_splats((int)1)), mask);
  
  vec_float4 x1 = spu_sub(x , ones);
  vec_float4 z  = _divf4  (x1, spu_add(x, ones));
  vec_float4 w  = spu_mul(z , z);
  vec_float4 polyw;
  polyw = spu_madd(spu_splats(__LOG10F_c0), w, spu_splats(__LOG10F_c1));
  polyw = spu_madd(polyw                  , w, spu_splats(__LOG10F_c2));
  
  vec_float4 yneg = spu_mul(z, spu_msub(polyw, w, x1));
  vec_float4 wnew = spu_convtf(xexp,0);
  
  vec_float4 zz1 = spu_madd(spu_splats(__LOG10F_logaemsb), x1, 
			    spu_mul(spu_splats(__LOG10F_loga2msb),wnew));
  vec_float4 zz2 = spu_madd(spu_splats(__LOG10F_logaelsb), x1,
			    spu_madd(spu_splats(__LOG10F_loga2lsb), wnew, 
				     spu_mul(spu_splats(__LOG10F_logae), yneg))
			    );
  
  return spu_sel(spu_add(zz1,zz2), (vec_float4)zeromask, zeromask);
}

#endif
