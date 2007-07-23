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

#include <simdmath.h>
#include <altivec.h>

#include "common-types.h"

#define _EXPM1F_H_ln1by2 ((float)-0.6931471805599f)
#define _EXPM1F_H_ln3by2 ((float) 0.4054651081082f)

vector float
expm1f4 (vector float x)
{
  vec_float4 zeros = vec_splatsf4(0.0f);
  vec_uint4 nearzeromask = (vec_uint4)vec_and(vec_cmpgt(x, vec_splatsf4(_EXPM1F_H_ln1by2)),
					      vec_cmpgt(vec_splatsf4(_EXPM1F_H_ln3by2), x));
  vec_float4 x2 = vec_madd(x,x,zeros);
  vec_float4 d0, d1, n0, n1;

  d0 = vec_madd(x , vec_splatsf4(-0.3203561199f), vec_splatsf4(0.9483177697f));
  d1 = vec_madd(x2, vec_splatsf4( 0.0326527809f), d0);
  
  n0 = vec_madd(x , vec_splatsf4(0.1538026623f), vec_splatsf4(0.9483177732f));
  n1 = vec_madd(x , vec_splatsf4(0.0024490478f), vec_splatsf4(0.0305274668f));
  n1 = vec_madd(x2, n1, n0);
 
  return vec_sel(vec_sub(expf4(x), vec_splatsf4(1.0f)),
                 vec_madd(x, divf4(n1, d1), zeros),
                 nearzeromask);
}
