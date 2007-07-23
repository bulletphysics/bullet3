/* Testcase for sqrtf4
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
   TEST_SET_START("20040928182549EJL","EJL", "sqrtf4");

   unsigned int i3 =   0x742c4455;
   unsigned int i3r =  0x59d20034;
   unsigned int i4 =   0x75e7753f; 
   unsigned int i4r =  0x5aac1fb5;
   unsigned int i5 =   0x4baa9e3c; 
   unsigned int i5r =  0x4593c7d8;
   unsigned int i6 =   0x39344296; 
   unsigned int i6r =  0x3c56d14c; 
   unsigned int i7 =   0x68a586b0; 
   unsigned int i7r =  0x54118f09;

   float x0 = hide_float(0.0f);
   float x0r = hide_float(0.0f);   

   float x3 = hide_float(make_float(i3));
   float x3r = hide_float(make_float(i3r));   
   float x4 = hide_float(make_float(i4));     
   float x4r = hide_float(make_float(i4r));   
   float x5 = hide_float(make_float(i5));     
   float x5r = hide_float(make_float(i5r));   
   float x6 = hide_float(make_float(i6));     
   float x6r = hide_float(make_float(i6r));   
   float x7 = hide_float(make_float(i7));
   float x7r = hide_float(make_float(i7r));
   
   vec_float4 x0_v = vec_splat_float(x0);
   vec_float4 x0r_v = vec_splat_float(x0r);

   vec_float4 x3_v = vec_splat_float(x3);
   vec_float4 x3r_v = vec_splat_float(x3r);
   vec_float4 x4_v = vec_splat_float(x4);
   vec_float4 x4r_v = vec_splat_float(x4r);
   vec_float4 x5_v = vec_splat_float(x5);
   vec_float4 x5r_v = vec_splat_float(x5r);
   vec_float4 x6_v = vec_splat_float(x6);
   vec_float4 x6r_v = vec_splat_float(x6r);
   vec_float4 x7_v = vec_splat_float(x7);
   vec_float4 x7r_v = vec_splat_float(x7r);
   
   vec_float4 res_v;

   TEST_START("sqrtf4");
   res_v = sqrtf4(x0_v);
   TEST_CHECK("20040928182551EJL", allequal_float4( res_v, x0r_v ), 0);
   res_v = sqrtf4(x3_v);
   TEST_CHECK("20040928182552EJL", allequal_ulps_float4( res_v, x3r_v, 2 ), 0);
   res_v = sqrtf4(x4_v);
   TEST_CHECK("20040928182554EJL", allequal_ulps_float4( res_v, x4r_v, 2 ), 0);
   res_v = sqrtf4(x5_v);
   TEST_CHECK("20040928182556EJL", allequal_ulps_float4( res_v, x5r_v, 2 ), 0);
   res_v = sqrtf4(x6_v);
   TEST_CHECK("20040928182557EJL", allequal_ulps_float4( res_v, x6r_v, 2 ), 0);
   res_v = sqrtf4(x7_v);
   TEST_CHECK("20040928182559EJL", allequal_ulps_float4( res_v, x7r_v, 2 ), 0);
   
   TEST_SET_DONE();
   
   TEST_EXIT();
}
