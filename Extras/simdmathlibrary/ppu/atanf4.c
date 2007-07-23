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

#include <simdmath.h>
#include <altivec.h>

#include "common-types.h"

//
// Computes the inverse tangent of all four slots of x. 
//
vector float
atanf4 (vector float x)
{
    vec_float4 bias;
    vec_float4 x2, x3, x4, x8, x9;
    vec_float4 hi, lo;
    vec_float4 result;
    vec_float4 inv_x;
    vec_uint4 sign;
    vec_uint4 select;
    vec_float4 xabs;
    vec_float4 vzero = (vec_float4){0.0, 0.0, 0.0, 0.0};
    
    sign = vec_and((vec_uint4)x, vec_splatsu4(0x80000000));
    xabs = (vec_float4)vec_andc((vec_uint4)x, vec_splatsu4(0x80000000));
    inv_x = recipf4(x);
    inv_x = (vec_float4)vec_xor((vec_uint4)inv_x, vec_splatsu4(0x80000000));
    select = (vec_uint4)vec_cmpgt(xabs, ((vec_float4){1.0, 1.0, 1.0, 1.0}) );
    bias = (vec_float4)vec_or(sign, (vec_uint4)(vec_splatsf4(1.57079632679489661923f)));
    bias = (vec_float4)vec_and((vec_uint4)bias, select);
    
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
    hi = vec_madd(vec_splatsf4(0.0028662257), x2, vec_splatsf4(-0.0161657367));
    hi = vec_madd(hi, x2, vec_splatsf4(0.0429096138));
    hi = vec_madd(hi, x2, vec_splatsf4(-0.0752896400));
    hi = vec_madd(hi, x2, vec_splatsf4(0.1065626393));
    lo = vec_madd(vec_splatsf4(-0.1420889944), x2, vec_splatsf4(0.1999355085));
    lo = vec_madd(lo, x2, vec_splatsf4(-0.3333314528));
    lo = vec_madd(lo, x3, bias);
    
    result = vec_madd(hi, x9, lo);    
    return result;
}

