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

#include <simdmath.h>
#include <altivec.h>

#include "common-types.h"

#define __calcQuot(xexp) n = xexp;							  \
  vec_uint4 negxexpmask = (vec_uint4)vec_cmpgt( ((vec_int4){0, 0, 0, 0}) , n);		  \
  n = vec_sel(n, vec_add(n, ((vec_int4){2, 2, 2, 2}) ), negxexpmask);			  \
  											  \
  quot = vec_add(vec_sra(n, ((vec_uint4){2, 2, 2, 2}) ), vec_sra(n, ((vec_uint4){4, 4, 4, 4}) )); \
  quot = vec_add(quot, vec_sra(quot, ((vec_uint4){4, 4, 4, 4}) ));		 		  \
  quot = vec_add(quot, vec_sra(quot, ((vec_uint4){8, 8, 8, 8}) ));				  \
  quot = vec_add(quot, vec_sra(quot, ((vec_uint4){16, 16, 16, 16}) ));			  \
  vec_int4 r = vec_sub(vec_sub(n,quot), vec_sl(quot, ((vec_uint4){1, 1, 1, 1}) ));		  \
  quot = vec_add(									  \
		 quot,									  \
		 vec_sra(								  \
			 vec_add(							  \
				 vec_add(r,((vec_int4){5, 5, 5, 5})),			  \
				 vec_sl (r,((vec_uint4){2, 2, 2, 2}))			  \
				 ),							  \
			 ((vec_uint4){4, 4, 4, 4})					  \
			 )								  \
		 );									  \

#define _CBRTF_H_cbrt2 1.2599210498948731648             // 2^(1/3)
#define _CBRTF_H_sqr_cbrt2 1.5874010519681994748         // 2^(2/3)

vector float
cbrtf4 (vector float x)
{
  vec_float4 zeros = (vec_float4){0.0f, 0.0f, 0.0f, 0.0f};
  vec_int4 xexp, n;
  vec_float4 sgnmask = (vec_float4)(vec_splatsi4(0x80000000));
  vec_uint4 negmask = (vec_uint4)vec_cmpgt(zeros, x);
  x = vec_andc(x, sgnmask);

  x = frexpf4(x, &xexp);
  vec_float4 p = vec_madd(
			  vec_madd(x, vec_splatsf4(-0.191502161678719066f), vec_splatsf4(0.697570460207922770f)),
			  x,
			  vec_splatsf4(0.492659620528969547f)
			  );
  vec_float4 p3 = vec_madd(p, vec_madd(p, p, zeros), zeros);

  vec_int4 quot;
  __calcQuot(xexp);
  vec_int4 modval = vec_sub(vec_sub(xexp,quot), vec_sl(quot,vec_splatsu4(1))); // mod = xexp - 3*quotient
  vec_float4 factor =  vec_splatsf4(1.0/_CBRTF_H_sqr_cbrt2);
  factor = vec_sel(factor, vec_splatsf4(1.0/_CBRTF_H_cbrt2), vec_cmpeq(modval,vec_splatsi4(-1)));
  factor = vec_sel(factor, vec_splatsf4(               1.0), vec_cmpeq(modval,vec_splatsi4( 0)));
  factor = vec_sel(factor, vec_splatsf4(    _CBRTF_H_cbrt2), vec_cmpeq(modval,vec_splatsi4( 1)));
  factor = vec_sel(factor, vec_splatsf4(_CBRTF_H_sqr_cbrt2), vec_cmpeq(modval,vec_splatsi4( 2)));

  vec_float4 pre  = vec_madd(p, factor, zeros);
  vec_float4 numr = vec_madd(x , vec_splatsf4(2.0f), p3);
  vec_float4 denr = vec_madd(p3, vec_splatsf4(2.0f), x );
  vec_float4 res = vec_madd(pre, divf4(numr, denr), zeros);
  res = ldexpf4(res, quot);

  return vec_sel(res, vec_or(res,sgnmask), negmask);
}

/*
_FUNC_DEF(vec_float4, cbrtf4, (vec_float4 x))
{
  vec_uint4 neg  = (vec_uint4)vec_cmpgt((vec_float4)(0.0f), x);
  vec_float4  sbit = (vec_float4)(vec_float4)((int)0x80000000);
  vec_float4 absx = vec_andc(x, sbit);
  vec_float4 res = exp2f4(vec_mul((vec_float4)(0.3333333333333f), log2f4(absx)));
  res = vec_sel(res, vec_or(sbit, res), neg);
  return res;
}
*/
