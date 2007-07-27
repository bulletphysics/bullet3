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
#include <altivec.h>

#include <simdmath/frexpf4.h>
#include <simdmath/ldexpf4.h>
#include <simdmath/divf4.h>

static inline vector signed int
__cbrtf4_calc_quot(vector signed int n)
{
  vector signed int quot;
  vector unsigned int negxexpmask = (vector unsigned int)vec_cmpgt(__vec_splatsi4(0), n);
  n = vec_sel(n, vec_add(n, __vec_splatsi4(2)), negxexpmask);

  quot = vec_add(vec_sra(n, __vec_splatsu4(2)), vec_sra(n, __vec_splatsu4(4)));
  quot = vec_add(quot, vec_sra(quot, __vec_splatsu4(4)));
  quot = vec_add(quot, vec_sra(quot, __vec_splatsu4(8)));
  quot = vec_add(quot, vec_sra(quot, __vec_splatsu4(16)));
  vector signed int r = vec_sub(vec_sub(n,quot), vec_sl(quot, __vec_splatsu4(1)));
  quot = vec_add(quot,
		 vec_sra(vec_add(vec_add(r, __vec_splatsi4(5)),
				 vec_sl (r, __vec_splatsu4(2))),
			 __vec_splatsu4(4)));

  return quot;
}

#define __CBRTF_cbrt2 1.2599210498948731648             // 2^(1/3)
#define __CBRTF_sqr_cbrt2 1.5874010519681994748         // 2^(2/3)

static inline vector float
_cbrtf4 (vector float x)
{
  vector float zeros = __vec_splatsf4(0.0f);
  vector signed int xexp;
  vector float sgnmask = (vector float)__vec_splatsi4(0x80000000);
  vector unsigned int negmask = (vector unsigned int)vec_cmpgt(zeros, x);
  x = vec_andc(x, sgnmask);

  x = _frexpf4(x, &xexp);
  vector float p =
    vec_madd(vec_madd(x, __vec_splatsf4(-0.191502161678719066f), __vec_splatsf4(0.697570460207922770f)),
	     x,
	     __vec_splatsf4(0.492659620528969547f));
  vector float p3 = vec_madd(p, vec_madd(p, p, zeros), zeros);

  vector signed int quot = __cbrtf4_calc_quot(xexp);
  // mod = xexp - 3*quotient
  vector signed int modval = vec_sub(vec_sub(xexp,quot), vec_sl(quot, __vec_splatsu4(1)));
  vector float factor =  __vec_splatsf4(1.0/__CBRTF_sqr_cbrt2);
  factor = vec_sel(factor, __vec_splatsf4(1.0/__CBRTF_cbrt2), vec_cmpeq(modval, __vec_splatsi4(-1)));
  factor = vec_sel(factor, __vec_splatsf4(               1.0), vec_cmpeq(modval, __vec_splatsi4( 0)));
  factor = vec_sel(factor, __vec_splatsf4(    __CBRTF_cbrt2), vec_cmpeq(modval, __vec_splatsi4( 1)));
  factor = vec_sel(factor, __vec_splatsf4(__CBRTF_sqr_cbrt2), vec_cmpeq(modval, __vec_splatsi4( 2)));

  vector float pre  = vec_madd(p, factor, zeros);
  vector float numr = vec_madd(x , __vec_splatsf4(2.0f), p3);
  vector float denr = vec_madd(p3, __vec_splatsf4(2.0f), x );
  vector float res = vec_madd(pre, _divf4(numr, denr), zeros);
  res = _ldexpf4(res, quot);

  return vec_sel(res, vec_or(res,sgnmask), negmask);
}

#endif
