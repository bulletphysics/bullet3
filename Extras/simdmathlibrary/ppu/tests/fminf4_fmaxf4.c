/* Testcase for fminf4 and fmaxf4
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

#include <stdlib.h>
#include <math.h>
#include "common-test.h"
#include "testutils.h"
#include "simdmath.h"

int main()
{
   TEST_SET_START("20040928184342EJL","EJL", "fminf4_fmaxf4");
   
   float x0min = hide_float(1760.135f);
   float x0max = hide_float(19355.03f);

   float x1min = hide_float(-12351.9f);
   float x1max = hide_float(-139.035f);

   float x2min = hide_float(-1.0);
   float x2max = hide_float(0.0);

   vec_float4 x0min_v = vec_splat_float(x0min);
   vec_float4 x0max_v = vec_splat_float(x0max);

   vec_float4 x1min_v = vec_splat_float(x1min); 
   vec_float4 x1max_v = vec_splat_float(x1max); 

   vec_float4 x2min_v = vec_splat_float(x2min); 
   vec_float4 x2max_v = vec_splat_float(x2max); 

   vec_float4 res_v;

   TEST_START("fminf4");
   res_v = fminf4(x0min_v, x0max_v);
   TEST_CHECK("20040928184345EJL", allequal_float4( res_v, x0min_v ), 0);
   res_v = fminf4(x0max_v, x0min_v);      
   TEST_CHECK("20040928184349EJL", allequal_float4( res_v, x0min_v ), 0);
   res_v = fminf4(x1min_v, x1max_v);
   TEST_CHECK("20040928184351EJL", allequal_float4( res_v, x1min_v ), 0);
   res_v = fminf4(x1max_v, x1min_v);      
   TEST_CHECK("20040928184353EJL", allequal_float4( res_v, x1min_v ), 0);
   res_v = fminf4(x2min_v, x2max_v);
   TEST_CHECK("20040928184354EJL", allequal_float4( res_v, x2min_v ), 0);
   res_v = fminf4(x2max_v, x2min_v);      
   TEST_CHECK("20040928184356EJL", allequal_float4( res_v, x2min_v ), 0);
   
   TEST_START("fmaxf4");
   res_v = fmaxf4(x0min_v, x0max_v);
   TEST_CHECK("20040928184411EJL", allequal_float4( res_v, x0max_v ), 0);
   res_v = fmaxf4(x0max_v, x0min_v);      
   TEST_CHECK("20040928184413EJL", allequal_float4( res_v, x0max_v ), 0);
   res_v = fmaxf4(x1min_v, x1max_v);
   TEST_CHECK("20040928184415EJL", allequal_float4( res_v, x1max_v ), 0);
   res_v = fmaxf4(x1max_v, x1min_v);      
   TEST_CHECK("20040928184416EJL", allequal_float4( res_v, x1max_v ), 0);
   res_v = fmaxf4(x2min_v, x2max_v);
   TEST_CHECK("20040928184417EJL", allequal_float4( res_v, x2max_v ), 0);
   res_v = fmaxf4(x2max_v, x2min_v);      
   TEST_CHECK("20040928184419EJL", allequal_float4( res_v, x2max_v ), 0);
   
   TEST_SET_DONE();
   
   TEST_EXIT();
}
