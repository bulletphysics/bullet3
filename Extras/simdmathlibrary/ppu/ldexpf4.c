/* ldexpf4 - 
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

vector float
ldexpf4 (vector float x, vector signed int exp)
{
  vec_int4 zeros = vec_splatsi4(0);

  vec_int4 expmask = vec_splatsi4(0x7F800000);
  vec_int4 e1 = vec_and((vec_int4)x, expmask);
  vec_int4 e2 = vec_sr(e1,vec_splatsu4(23));

  vec_uint4 maxmask = (vec_uint4)vec_cmpgt(exp, vec_splatsi4(255));
  vec_uint4 minmask = (vec_uint4)vec_cmpgt(vec_splatsi4(-255), exp);
  minmask = vec_or (minmask, (vec_uint4)vec_cmpeq(x, (vec_float4)zeros));

  vec_int4 esum = vec_add(e2, exp);

  maxmask = vec_or (maxmask, (vec_uint4)vec_cmpgt(esum, vec_splatsi4(255)));
  maxmask = vec_and(maxmask, vec_splatsu4(0x7FFFFFFF));
  minmask = vec_or (minmask, (vec_uint4)vec_cmpgt(zeros, esum));

  x = vec_sel(x, (vec_float4)vec_sl(esum,vec_splatsu4(23)), (vec_uint4)expmask);
  x = vec_sel(x, (vec_float4)zeros, minmask);
  x = vec_sel(x, (vec_float4)maxmask, maxmask);
  return x;
}
