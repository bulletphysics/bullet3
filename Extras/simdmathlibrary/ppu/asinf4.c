/* asinf4 - Computes the inverse sine of all four slots of x
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

vector float asinf4 (vector float x)
{
    // positive = (x > 0)
    //
    vec_uint4 positive = (vec_uint4)vec_cmpgt(x,vec_splatsf4(0.0f));

    // x = absf(x)
    //
    x = vec_abs(x);

    // gtHalf = (|x| > 0.5)
    //
    vec_uint4 gtHalf = (vec_uint4)vec_cmpgt(x,vec_splatsf4(0.5f));    


    // if (x > 0.5)
    //    g = 0.5 - 0.5*x
    //    x = -2 * sqrtf(g)
    // else
    //    g = x * x
    //
    vec_float4 g = vec_sel(vec_madd(x,x,vec_splatsf4(0.0f)),vec_madd(vec_splatsf4(-0.5f),x,vec_splatsf4(0.5f)),gtHalf);
    
    x = vec_sel(x,vec_madd(vec_splatsf4(-2.0f),sqrtf4(g),vec_splatsf4(0.0f)),gtHalf);

    // Compute the polynomials and take their ratio
    //  denom = (1.0f*g + -0.554846723e+1f)*g + 5.603603363f
    //  num = x * g * (-0.504400557f * g + 0.933933258f)
    //
    vec_float4 denom = vec_add(g,vec_splatsf4(-5.54846723f));
    vec_float4 num = vec_madd(vec_splatsf4(-0.504400557f),g,vec_splatsf4(0.933933258f));
    denom = vec_madd(denom,g,vec_splatsf4(5.603603363f));
    num = vec_madd(vec_madd(x,g,vec_splatsf4(0.0f)),num,vec_splatsf4(0.0f));

    
    // x = x + num / denom
    //
    x = vec_add(x,divf4(num,denom));

    // if (x > 0.5)
    //    x = x + M_PI_2
    //
    x = vec_sel(x,vec_add(x,vec_splatsf4(1.57079632679489661923f)),gtHalf);

    
    // if (!positive) x = -x
    //
    x = vec_sel((vec_float4)vec_xor(vec_splatsi4(0x80000000),(vec_int4)x),x,positive);

    return x;
}

