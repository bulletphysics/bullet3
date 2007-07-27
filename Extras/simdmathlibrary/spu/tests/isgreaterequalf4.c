/* Test isgreaterequalf4 for SPU
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
#include <float.h>
#include "simdmath.h"
#include "common-test.h"
#include "testutils.h"

int main()
{
   TEST_SET_START("20060817000000AAN","AAN", "isgreaterequalf4");
   
   float x0 = hide_float(-0.0f);
   float y0 = hide_float( 0.0f);
   unsigned int r0 = 0xffffffff;

   float x1 = hide_float(FLT_MAX);  //+Smax
   float y1 = hide_float(-FLT_MAX);  //-Smax
   unsigned int r1 = 0xffffffff;

   float x2 = hide_float(-0.0000000013152f);
   float y2 = hide_float(-234245.85323441f);
   unsigned int r2 = 0xffffffff;

   float x3 = hide_float(-168.97345223013f);
   float y3 = hide_float(-168.97345223013f);
   unsigned int r3 = 0xffffffff;

   float x4 = hide_float(-83532.96153153f);
   float y4 = hide_float(-FLT_MIN);  //-Smin
   unsigned int r4 = 0x00000000;

   float x5 = hide_float(-321.01234567f);
   float y5 = hide_float(876543.12345f);
   unsigned int r5 = 0x00000000;

   float x6 = hide_float(FLT_MIN);  // Smin
   float y6 = hide_float(0.0031529324f);
   unsigned int r6 = 0x00000000;

   float x7 = hide_float(5172.2845321f);
   float y7 = hide_float(5172.2845321f);
   unsigned int r7 = 0xffffffff;

   float x8 = hide_float(264.345643345f);
   float y8 = hide_float(2353705.31415f);
   unsigned int r8 = 0x00000000;

   float x9 = hide_float(FLT_MAX);  // Smax
   float y9 = hide_float(9.43574552184f);
   unsigned int r9 = 0xffffffff;

   vec_float4 x0_v = spu_splats(x0);
   vec_float4 y0_v = spu_splats(y0);
   vec_uint4  r0_v = spu_splats(r0);

   vec_float4 x1_v = spu_splats(x1);
   vec_float4 y1_v = spu_splats(y1);
   vec_uint4  r1_v = spu_splats(r1);

   vec_float4 x2_v = spu_splats(x2);
   vec_float4 y2_v = spu_splats(y2);
   vec_uint4  r2_v = spu_splats(r2);

   vec_float4 x3_v = spu_splats(x3);
   vec_float4 y3_v = spu_splats(y3);
   vec_uint4  r3_v = spu_splats(r3);

   vec_float4 x4_v = spu_splats(x4);
   vec_float4 y4_v = spu_splats(y4);
   vec_uint4  r4_v = spu_splats(r4);

   vec_float4 x5_v = spu_splats(x5);
   vec_float4 y5_v = spu_splats(y5);
   vec_uint4  r5_v = spu_splats(r5);

   vec_float4 x6_v = spu_splats(x6);
   vec_float4 y6_v = spu_splats(y6);
   vec_uint4  r6_v = spu_splats(r6);

   vec_float4 x7_v = spu_splats(x7);
   vec_float4 y7_v = spu_splats(y7);
   vec_uint4  r7_v = spu_splats(r7);

   vec_float4 x8_v = spu_splats(x8);
   vec_float4 y8_v = spu_splats(y8);
   vec_uint4  r8_v = spu_splats(r8);

   vec_float4 x9_v = spu_splats(x9);
   vec_float4 y9_v = spu_splats(y9);
   vec_uint4  r9_v = spu_splats(r9);
   
   vec_uint4 res_v;

   TEST_START("isgreaterequalf4");

   res_v = (vec_uint4)isgreaterequalf4(x0_v, y0_v);
   TEST_CHECK("20060817000000AAN", allequal_uint4( res_v, r0_v ), 0);
   res_v = (vec_uint4)isgreaterequalf4(x1_v, y1_v);
   TEST_CHECK("20060817000001AAN", allequal_uint4( res_v, r1_v ), 0);
   res_v = (vec_uint4)isgreaterequalf4(x2_v, y2_v);
   TEST_CHECK("20060817000002AAN", allequal_uint4( res_v, r2_v ), 0);
   res_v = (vec_uint4)isgreaterequalf4(x3_v, y3_v);
   TEST_CHECK("20060817000003AAN", allequal_uint4( res_v, r3_v ), 0);
   res_v = (vec_uint4)isgreaterequalf4(x4_v, y4_v);
   TEST_CHECK("20060817000004AAN", allequal_uint4( res_v, r4_v ), 0);
   res_v = (vec_uint4)isgreaterequalf4(x5_v, y5_v);
   TEST_CHECK("20060817000005AAN", allequal_uint4( res_v, r5_v ), 0);
   res_v = (vec_uint4)isgreaterequalf4(x6_v, y6_v);
   TEST_CHECK("20060817000006AAN", allequal_uint4( res_v, r6_v ), 0);
   res_v = (vec_uint4)isgreaterequalf4(x7_v, y7_v);
   TEST_CHECK("20060817000007AAN", allequal_uint4( res_v, r7_v ), 0);
   res_v = (vec_uint4)isgreaterequalf4(x8_v, y8_v);
   TEST_CHECK("20060817000008AAN", allequal_uint4( res_v, r8_v ), 0);
   res_v = (vec_uint4)isgreaterequalf4(x9_v, y9_v);
   TEST_CHECK("20060817000009AAN", allequal_uint4( res_v, r9_v ), 0);
   
   TEST_SET_DONE();
   
   TEST_EXIT();
}
