/* frexpf4 -
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

#ifndef ___SIMD_MATH_FREXPF4_H___
#define ___SIMD_MATH_FREXPF4_H___

#include <simdmath.h>
#include <altivec.h>

#include <simdmath/_vec_utils.h>

static inline vector float
_frexpf4 (vector float x, vector signed int *exp)
{
  vector signed int zeros = __vec_splatsi4(0);
  vector unsigned int zeromask = (vector unsigned int)vec_cmpeq(x, (vector float)zeros);
 
  vector signed int expmask = __vec_splatsi4(0x7F800000);
  vector signed int e1 = vec_and ( (vector signed int)x, expmask);
  vector signed int e2 = vec_sub(vec_sr(e1, __vec_splatsu4(23) ), __vec_splatsi4(126) );
  *exp = vec_sel(e2, zeros, zeromask);
 
  vector float m2 = vec_sel(x, (vector float)(__vec_splatsi4(0x3F000000)), (vector unsigned int)expmask);

  return vec_sel(m2, (vector float)zeros, zeromask);
}

#endif
