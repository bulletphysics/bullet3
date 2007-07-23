/* Testcase for copysignf4
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

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "common-test.h"
#include "testutils.h"
#include "simdmath.h"

int main()
{
   TEST_SET_START("20040917114054EJL", "EJL", "copysignf");
   
   float x0m = hide_float(1989.0f);
   float x0s = hide_float(-319875.0f);
   float x0c = hide_float(-1989.0f);
   float x1m = hide_float(9013.0f);
   float x1s = hide_float(185.0f);
   float x1c = hide_float(9013.0f);
   
   vec_float4 x0m_v = vec_splat_float(x0m);
   vec_float4 x0s_v = vec_splat_float(x0s);
   vec_float4 x0c_v = vec_splat_float(x0c);

   vec_float4 x1m_v = vec_splat_float(x1m);
   vec_float4 x1s_v = vec_splat_float(x1s);
   vec_float4 x1c_v = vec_splat_float(x1c);
   
   vec_float4 res_v;

   TEST_START("copysignf4");
   res_v = copysignf4( x0m_v, x0s_v );
   TEST_CHECK("20040917114058EJL", allequal_float4( res_v, x0c_v ), 0);
   res_v = copysignf4( x1m_v, x1s_v );
   TEST_CHECK("20040917114100EJL", allequal_float4( res_v, x1c_v ), 0);
   
   TEST_SET_DONE();
   
   TEST_EXIT();
}
