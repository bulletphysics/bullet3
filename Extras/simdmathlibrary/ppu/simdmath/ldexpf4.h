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

#ifndef ___SIMD_MATH_LDEXPF4_H___
#define ___SIMD_MATH_LDEXPF4_H___

#include <simdmath.h>
#include <altivec.h>

#include <simdmath/_vec_utils.h>

static inline vector float
_ldexpf4 (vector float x, vector signed int exp)
{
  vector signed int zeros = __vec_splatsi4(0);

  vector signed int expmask = __vec_splatsi4(0x7F800000);
  vector signed int e1 = vec_and((vector signed int)x, expmask);
  vector signed int e2 = vec_sr(e1,__vec_splatsu4(23));

  vector unsigned int maxmask = (vector unsigned int)vec_cmpgt(exp, __vec_splatsi4(255));
  vector unsigned int minmask = (vector unsigned int)vec_cmpgt(__vec_splatsi4(-255), exp);
  minmask = vec_or (minmask, (vector unsigned int)vec_cmpeq(x, (vector float)zeros));

  vector signed int esum = vec_add(e2, exp);

  maxmask = vec_or (maxmask, (vector unsigned int)vec_cmpgt(esum, __vec_splatsi4(255)));
  maxmask = vec_and(maxmask, __vec_splatsu4(0x7FFFFFFF));
  minmask = vec_or (minmask, (vector unsigned int)vec_cmpgt(zeros, esum));

  x = vec_sel(x, (vector float)vec_sl(esum,__vec_splatsu4(23)), (vector unsigned int)expmask);
  x = vec_sel(x, (vector float)zeros, minmask);
  x = vec_sel(x, (vector float)maxmask, maxmask);
  return x;
}

#endif
