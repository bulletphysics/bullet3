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


#include <simdmath.h>
#include <altivec.h>

#include "common-types.h"

#define _LOG10F_H_loga2msb  ((float) 0.3010299205780f) 
#define _LOG10F_H_loga2lsb  ((float) 7.5085978266e-8f)
#define _LOG10F_H_logaemsb  ((float) 0.4342944622040f)
#define _LOG10F_H_logaelsb  ((float) 1.9699272335e-8f)
#define _LOG10F_H_neglogae ((float)-0.4342944819033f)

#define _LOG10F_H_c0 ((float)(-0.2988439998f)) 
#define _LOG10F_H_c1 ((float)(-0.3997655209f))
#define _LOG10F_H_c2 ((float)(-0.6666679125f))

vector float
log10f4 (vector float x)
{
  vec_int4 zeros = vec_splatsi4(0);
  vec_float4 ones = vec_splatsf4(1.0f);
  //vec_uchar16 zeromask = (vec_uchar16)vec_cmpeq(x, (vec_float4)zeros);

  vec_int4 expmask = vec_splatsi4(0x7F800000);
  vec_int4 xexp = vec_add( vec_sr(vec_and((vec_int4)x, expmask), vec_splatsu4(23)), vec_splatsi4(-126) );
  x = vec_sel(x, (vec_float4)(vec_splatsi4(0x3F000000)), (vec_uint4)expmask);

  vec_uint4  mask = (vec_uint4)vec_cmpgt( vec_splatsf4((float)0.7071067811865f), x);
  x    = vec_sel(x   , vec_add(x, x)               , mask);
  xexp = vec_sel(xexp, vec_sub(xexp, vec_splatsi4(1)), mask);
  
  vec_float4 x1 = vec_sub(x , ones);
  vec_float4 z  = divf4  (x1, vec_add(x, ones));
  vec_float4 w  = vec_madd(z , z, (vec_float4)zeros);
  vec_float4 polywneg;
  polywneg = vec_madd(vec_splatsf4(_LOG10F_H_c0), w, vec_splatsf4(_LOG10F_H_c1));
  polywneg = vec_madd(polywneg                  , w, vec_splatsf4(_LOG10F_H_c2));
  
  vec_float4 y = vec_madd(z, vec_madd(polywneg, w, x1), (vec_float4)zeros);
  vec_float4 wnew = vec_ctf(xexp,0);
  
  vec_float4 zz1 = vec_madd(vec_splatsf4(_LOG10F_H_logaemsb), x1, 
			    vec_madd(vec_splatsf4(_LOG10F_H_loga2msb),wnew,(vec_float4)zeros));
  vec_float4 zz2 = vec_madd(vec_splatsf4(_LOG10F_H_logaelsb), x1,
			    vec_madd(vec_splatsf4(_LOG10F_H_loga2lsb), wnew, 
				     vec_madd(vec_splatsf4(_LOG10F_H_neglogae),y,(vec_float4)zeros))
			    );
  
  //return vec_sel(vec_add(zz1,zz2), (vec_float4)zeromask, zeromask);
  return vec_add(zz1, zz2);
}


