/* atanf4 - 
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

#ifndef ___SIMD_MATH_ATANF4_H___
#define ___SIMD_MATH_ATANF4_H___

#include <simdmath.h>
#include <altivec.h>

#include <simdmath/recipf4.h>

//
// Computes the inverse tangent of all four slots of x. 
//
static inline vector float
_atanf4 (vector float x)
{
  vector float bias;
  vector float x2, x3, x4, x8, x9;
  vector float hi, lo;
  vector float result;
  vector float inv_x;
  vector unsigned int sign;
  vector unsigned int select;
  vector float xabs;
  vector float vzero = __vec_splatsf4(0.0f);
    
  sign = vec_and((vector unsigned int)x, __vec_splatsu4(0x80000000));
  xabs = (vector float)vec_andc((vector unsigned int)x, __vec_splatsu4(0x80000000));
  inv_x = _recipf4(x);
  inv_x = (vector float)vec_xor((vector unsigned int)inv_x, __vec_splatsu4(0x80000000));
  select = (vector unsigned int)vec_cmpgt(xabs, __vec_splatsf4(1.0f));
  bias = (vector float)vec_or(sign, (vector unsigned int)(__vec_splatsf4(1.57079632679489661923f)));
  bias = (vector float)vec_and((vector unsigned int)bias, select);
    
  x = vec_sel(x, inv_x, select);
    
  /* Instruction counts can be reduced if the polynomial was
   * computed entirely from nested (dependent) fma's. However, 
   * to reduce the number of pipeline stalls, the polygon is evaluated 
   * in two halves(hi and lo).
   */
  bias = vec_add(bias, x);
  x2 = vec_madd(x,  x,  vzero);
  x3 = vec_madd(x2, x,  vzero);
  x4 = vec_madd(x2, x2, vzero);
  x8 = vec_madd(x4, x4, vzero);
  x9 = vec_madd(x8, x,  vzero);
  hi = vec_madd(__vec_splatsf4(0.0028662257), x2, __vec_splatsf4(-0.0161657367));
  hi = vec_madd(hi, x2, __vec_splatsf4(0.0429096138));
  hi = vec_madd(hi, x2, __vec_splatsf4(-0.0752896400));
  hi = vec_madd(hi, x2, __vec_splatsf4(0.1065626393));
  lo = vec_madd(__vec_splatsf4(-0.1420889944), x2, __vec_splatsf4(0.1999355085));
  lo = vec_madd(lo, x2, __vec_splatsf4(-0.3333314528));
  lo = vec_madd(lo, x3, bias);
    
  result = vec_madd(hi, x9, lo);    
  return result;
}

#endif
