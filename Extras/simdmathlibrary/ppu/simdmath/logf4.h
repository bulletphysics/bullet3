/* logf4 - 
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

#ifndef ___SIMD_MATH_LOGF4_H___
#define ___SIMD_MATH_LOGF4_H___

#include <simdmath.h>
#include <altivec.h>

#include <simdmath/divf4.h>


#define __LOGF_ln2msb     0.6931470632553f
#define __LOGF_negln2lsb -1.1730463525e-7f

#define __LOGF_c0 -0.2988439998f
#define __LOGF_c1 -0.3997655209f
#define __LOGF_c2 -0.6666679125f

static inline vector float
_logf4 (vector float x)
{
  vector signed int zeros = __vec_splatsi4(0);
  vector float ones = __vec_splatsf4(1.0f);

  vector signed int expmask = __vec_splatsi4(0x7F800000);
  vector signed int xexp =
    vec_add( vec_sr(vec_and((vector signed int)x, expmask), __vec_splatsu4(23)), __vec_splatsi4(-126) );
  x = vec_sel(x, (vector float)(__vec_splatsi4(0x3F000000)), (vector unsigned int)expmask);


  vector unsigned int  mask = (vector unsigned int)vec_cmpgt(__vec_splatsf4(0.7071067811865f), x);
  x    = vec_sel(x   , vec_add(x, x)                   , mask);
  xexp = vec_sel(xexp, vec_sub(xexp,__vec_splatsi4(1)), mask);
  
  vector float x1 = vec_sub(x , ones);
  vector float z  = _divf4  (x1, vec_add(x, ones));
  vector float w  = vec_madd(z , z, (vector float)zeros);
  vector float polywneg;
  polywneg = vec_madd(__vec_splatsf4(__LOGF_c0), w, __vec_splatsf4(__LOGF_c1));
  polywneg = vec_madd(polywneg                 , w, __vec_splatsf4(__LOGF_c2));

  vector float y    = vec_madd(z, vec_madd(polywneg, w, x1), (vector float)zeros);
  vector float wnew = vec_ctf(xexp,0);
  vector float zz1     = vec_madd(__vec_splatsf4(__LOGF_ln2msb)   , wnew, x1);
  vector float zz2neg  = vec_madd(__vec_splatsf4(__LOGF_negln2lsb), wnew, y );

  return vec_sub(zz1,zz2neg);
}

#endif
