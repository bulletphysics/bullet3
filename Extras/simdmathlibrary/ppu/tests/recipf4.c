/* Testcase for recipf4
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
   TEST_SET_START("20040920142553EJL","EJL", "recipf4");

   unsigned int i1 = 0xff000000;   // -2^127
   unsigned int i2 = 0xfe7fffff;   // -2^126 - 1 ulp
   unsigned int i2r = 0x80800001;  
   unsigned int i3 =   0x75013340; // random values
   unsigned int i3r =  0x09fd9f35;
   unsigned int i4 =   0x75e7753f; 
   unsigned int i4r =  0x090d9277;
   unsigned int i5 =   0x4c7fed5a; 
   unsigned int i5r =  0x32800954;
   unsigned int i6 =   0x3a0731f0; 
   unsigned int i6r =  0x44f2602e; 
   unsigned int i7 =   0x69784a07; 
   unsigned int i7r =  0x1583f9a3;

   float x1 = hide_float(make_float(i1));
   float x1r = hide_float(0.0f);
   float x2 = hide_float(make_float(i2));
   float x2r = hide_float(make_float(i2r));   
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
   
   vec_float4 x1_v = vec_splat_float(x1);
   vec_float4 x1r_v = vec_splat_float(x1r);
   vec_float4 x2_v = vec_splat_float(x2);
   vec_float4 x2r_v = vec_splat_float(x2r);
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

   TEST_START("recipf4");
   res_v = recipf4(x1_v);
   TEST_CHECK("20040920142600EJL", allequal_float4( res_v, x1r_v), 0);
   res_v = recipf4(x2_v);
   TEST_CHECK("20040920142602EJL", allequal_ulps_float4( res_v, x2r_v, 2 ), 0);
   res_v = recipf4(x3_v);
   TEST_CHECK("20040920142604EJL", allequal_ulps_float4( res_v, x3r_v, 2 ), 0);
   res_v = recipf4(x4_v);
   TEST_CHECK("20040920142606EJL", allequal_ulps_float4( res_v, x4r_v, 2 ), 0);
   res_v = recipf4(x5_v);
   TEST_CHECK("20040920142608EJL", allequal_ulps_float4( res_v, x5r_v, 2 ), 0);
   res_v = recipf4(x6_v);
   TEST_CHECK("20040920142609EJL", allequal_ulps_float4( res_v, x6r_v, 2 ), 0);
   res_v = recipf4(x7_v);
   TEST_CHECK("20040920142611EJL", allequal_ulps_float4( res_v, x7r_v, 2 ), 0);
   
   TEST_SET_DONE();
   
   TEST_EXIT();
}
