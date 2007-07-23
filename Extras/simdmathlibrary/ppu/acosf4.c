/* acosf4 - 
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
// Computes the inverse cosine of all four slots of x. 
//
vector float
acosf4 (vector float x)
{
    vec_float4 result, xabs;
    vec_float4 t1;
    vec_float4 xabs2, xabs4;
    vec_float4 hi, lo;
    vec_float4 neg, pos;
    vec_uint4 select;
    
    xabs = vec_abs(x);
    select = (vec_uint4)(vec_sra((vec_int4)(x), ((vec_uint4){31, 31, 31, 31}) ));
    
    t1 = sqrtf4(vec_sub( ((vec_float4){1.0, 1.0, 1.0, 1.0}) , xabs));
    
    /* Instruction counts can be reduced if the polynomial was
     * computed entirely from nested (dependent) fma's. However, 
     * to reduce the number of pipeline stalls, the polygon is evaluated 
     * in two halves (hi amd lo). 
     */
    xabs2 = vec_madd(xabs,  xabs, ((vec_float4){0.0f, 0.0f, 0.0f, 0.0f}) );
    xabs4 = vec_madd(xabs2, xabs2, ((vec_float4){0.0f, 0.0f, 0.0f, 0.0f}) );
    hi = vec_madd( ((vec_float4){-0.0012624911, -0.0012624911, -0.0012624911, -0.0012624911}) , xabs, ((vec_float4){0.0066700901, 0.0066700901, 0.0066700901, 0.0066700901}) );
    hi = vec_madd(hi, xabs, vec_splatsf4(-0.0170881256));
    hi = vec_madd(hi, xabs, vec_splatsf4( 0.0308918810));
    lo = vec_madd(vec_splatsf4(-0.0501743046), xabs, vec_splatsf4(0.0889789874));
    lo = vec_madd(lo, xabs, vec_splatsf4(-0.2145988016));
    lo = vec_madd(lo, xabs, vec_splatsf4( 1.5707963050));
    
    result = vec_madd(hi, xabs4, lo);
    
    /* Adjust the result if x is negactive.
     */
    neg = vec_nmsub(t1, result, vec_splatsf4(3.1415926535898f));
    pos = vec_madd(t1, result, ((vec_float4){0.0f, 0.0f, 0.0f, 0.0f}) );
    
    result = vec_sel(pos, neg, select);
    
    return result;
}


