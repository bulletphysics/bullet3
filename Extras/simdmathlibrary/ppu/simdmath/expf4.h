/* expf4 -
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
#include <altivec.h>

#include <simdmath/divf4.h>
#include <simdmath/ldexpf4.h>

#define __EXPF_C1     -0.6931470632553101f
#define __EXPF_C2     -1.1730463525082e-7f

#define __EXPF_INVLN2  1.4426950408889634f

static inline vector float
_expf4 (vector float x)
{
  vector float zeros = __vec_splatsf4(0.0f);
  vector unsigned int xnegmask = (vector unsigned int)vec_cmpgt(zeros, x);
  vector float goffset  = vec_sel(__vec_splatsf4( 0.5f),__vec_splatsf4(-0.5f),xnegmask);
  vector float g  = vec_madd(x, __vec_splatsf4(__EXPF_INVLN2), zeros);  
  vector signed int xexp = vec_cts(vec_add(g, goffset),0);
  
  g = vec_ctf(xexp, 0);
  g = vec_madd(g, __vec_splatsf4(__EXPF_C2), vec_madd(g, __vec_splatsf4(__EXPF_C1), x));
  vector float z  = vec_madd(g, g, zeros);
  vector float a = vec_madd(z, __vec_splatsf4(0.0999748594f), zeros);
  vector float b = vec_madd(g, 
			    vec_madd(z, 
				     __vec_splatsf4(0.0083208258f), 
				     __vec_splatsf4(0.4999999992f)),
			    zeros);
  
  vector float foo  = _divf4(vec_add(__vec_splatsf4(1.0f), vec_add(a, b)),
			     vec_add(__vec_splatsf4(1.0f), vec_sub(a, b)));
  
  return _ldexpf4(foo, xexp);
}

#endif
