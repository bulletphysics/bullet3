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

#ifndef ___SIMD_MATH_POWF4_H___
#define ___SIMD_MATH_POWF4_H___

#include <simdmath.h>
#include <altivec.h>
#include <math.h>

#include <simdmath/exp2f4.h>
#include <simdmath/log2f4.h>

static inline vector float
_powf4 (vector float x, vector float y)
{
  vector signed int zeros = __vec_splatsi4(0);
  vector unsigned int zeromask = (vector unsigned int)vec_cmpeq((vector float)zeros, x);

  vector unsigned int negmask  = (vector unsigned int)vec_cmpgt((vector float)zeros, x);
  
  vector float sbit = (vector float)(__vec_splatsi4(0x80000000));
  vector float absx = vec_andc(x, sbit);
  vector float absy = vec_andc(y, sbit);
  vector unsigned int oddy = vec_and(vec_ctu(absy, 0), __vec_splatsu4(0x00000001));
  negmask = vec_and(negmask, (vector unsigned int)vec_cmpgt(oddy, (vector unsigned int)zeros));

  vector float res = _exp2f4(vec_madd(y, _log2f4(absx), (vector float)zeros));
  res = vec_sel(res, vec_or(sbit, res), negmask);


  return vec_sel(res, (vector float)zeros, zeromask);
}

#endif
