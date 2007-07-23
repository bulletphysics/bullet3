/* exp2f4
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
#include <math.h>

#include "common-types.h"

vector float
powf4 (vector float x, vector float y)
{
  vec_int4 zeros = (vec_int4){0,0,0,0};
  vec_uint4 zeromask = (vec_uint4)vec_cmpeq((vec_float4)zeros, x);

  vec_uint4 negmask  = (vec_uint4)vec_cmpgt((vec_float4)zeros, x);
  
  vec_float4 sbit = (vec_float4)(vec_splatsi4(0x80000000));
  vec_float4 absx = vec_andc(x, sbit);
  vec_float4 absy = vec_andc(y, sbit);
  vec_uint4 oddy = vec_and(vec_ctu(absy, 0), vec_splatsu4(0x00000001));
  negmask = vec_and(negmask, (vec_uint4)vec_cmpgt(oddy, (vec_uint4)zeros));

  vec_float4 res = exp2f4(vec_madd(y, log2f4(absx), (vec_float4)zeros));
  res = vec_sel(res, vec_or(sbit, res), negmask);


  return vec_sel(res, (vec_float4)zeros, zeromask);
}

/*
{
  vec_int4 zeros = vec_splats(0);
  vec_int4 ones  = (vec_int4)vec_splats((char)0xFF);
  vec_uint4 zeromask = (vec_uint4)vec_cmpeq((vec_float4)zeros, x);
  vec_uint4 onemask  = (vec_uint4)vec_cmpeq((vec_float4)ones , y);
  vec_uint4 negmask  = (vec_uint4)vec_cmpgt(vec_splats(0.0f),  x);
  vec_float4 sbit = (vec_float4)(vec_int4)(0x80000000);
  vec_float4 absx = vec_andc(x, sbit);
  vec_float4 absy = vec_andc(y, sbit);
  vec_uint4  oddy = vec_and(vec_convtu(absy, 0), (vec_uint4)vec_splats(0x00000001));
  negmask         = vec_and(negmask, (vec_uint4)vec_cmpgt(oddy, (vec_uint4)zeros));

  

}

*/
