/* Test hypotf4 for SPU
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
#include "simdmath.h"
#include "common-test.h"
#include "testutils.h"

int main()
{
   TEST_SET_START("20060831000000AAN","AAN", "hypotf4");

   // Does not test precesion, which depends on sqrtf4 implementation
   // Uses Pythagorean triplets to test result validity

   //-Norm, -0
   float x0 = hide_float(-168.97345223013f);
   float y0 = hide_float(-0.0);
   float r0 = hide_float( 168.97345223013f);

   //-Unf, -Norm
   float x1 = hide_float(-1.0e-999);
   float y1 = hide_float(-83532.96153153f);
   float r1 = hide_float( 83532.96153153f);

   //-Unf, 0
   float x2 = hide_float(-1.0e-999);
   float y2 = hide_float( 0.0);
   float r2 = hide_float( 0.0);

    //+Unf, +Norm
   float x3 = hide_float(1.0e-999);
   float y3 = hide_float(0.0031529324f);
   float r3 = hide_float(0.0031529324f);

   //+Norm, +Norm
   float x4 = hide_float(5.5e12);
   float y4 = hide_float(4.8e12);
   float r4 = hide_float(7.3e12);

   //+Norm, -Denorm
   float x5 = hide_float(12.0e12);
   float y5 = hide_float(make_float(0x803AAAAA));
   float r5 = hide_float(12.0e12);

   //-Norm, +Norm
   float x6 = hide_float(-0.0000000008);
   float y6 = hide_float( 0.0000000015);
   float r6 = hide_float( 0.0000000017);

   //+Norm, -Norm
   float x7 = hide_float(  7.0e-6);
   float y7 = hide_float(-24.0e-6);
   float r7 = hide_float( 25.0e-6);

   //+Norm, +Norm
   float x8 = hide_float(0.0055f);
   float y8 = hide_float(0.0048f);
   float r8 = hide_float(0.0073f);

   //+Denorm, +Norm
   float x9 = hide_float(make_float(0x007AAAAA));
   float y9 = hide_float(2.8f);
   float r9 = hide_float(2.8f);

   //-Norm, +Norm
   float x10 = hide_float(-8000.0);
   float y10 = hide_float(39.0e2);
   float r10 = hide_float(8900.0);

   //+Norm, +Norm
   float x11 = hide_float(6.5e16);
   float y11 = hide_float(7.2e16);
   float r11 = hide_float(9.7e16);

   //-Norm, -Norm
   float x12 = hide_float(-0.0035);
   float y12 = hide_float(-12e-4);
   float r12 = hide_float(3700e-6);

   //+Norm, +Norm
   float x13 = hide_float(456548.6027761f);
   float y13 = hide_float(106165.2293520f);
   float r13 = hide_float(468729.8610289f);

   vec_float4 x0_v = spu_splats(x0);
   vec_float4 y0_v = spu_splats(y0);
   vec_float4 r0_v = spu_splats(r0);

   vec_float4 x1_v = spu_splats(x1);
   vec_float4 y1_v = spu_splats(y1);
   vec_float4 r1_v = spu_splats(r1);

   vec_float4 x2_v = spu_splats(x2);
   vec_float4 y2_v = spu_splats(y2);
   vec_float4 r2_v = spu_splats(r2);

   vec_float4 x3_v = spu_splats(x3);
   vec_float4 y3_v = spu_splats(y3);
   vec_float4 r3_v = spu_splats(r3);

   vec_float4 x4_v = spu_splats(x4);
   vec_float4 y4_v = spu_splats(y4);
   vec_float4 r4_v = spu_splats(r4);

   vec_float4 x5_v = spu_splats(x5);
   vec_float4 y5_v = spu_splats(y5);
   vec_float4 r5_v = spu_splats(r5);

   vec_float4 x6_v = spu_splats(x6);
   vec_float4 y6_v = spu_splats(y6);
   vec_float4 r6_v = spu_splats(r6);

   vec_float4 x7_v = spu_splats(x7);
   vec_float4 y7_v = spu_splats(y7);
   vec_float4 r7_v = spu_splats(r7);

   vec_float4 x8_v = spu_splats(x8);
   vec_float4 y8_v = spu_splats(y8);
   vec_float4 r8_v = spu_splats(r8);

   vec_float4 x9_v = spu_splats(x9);
   vec_float4 y9_v = spu_splats(y9);
   vec_float4 r9_v = spu_splats(r9);

   vec_float4 x10_v = spu_splats(x10);
   vec_float4 y10_v = spu_splats(y10);
   vec_float4 r10_v = spu_splats(r10);

   vec_float4 x11_v = spu_splats(x11);
   vec_float4 y11_v = spu_splats(y11);
   vec_float4 r11_v = spu_splats(r11);

   vec_float4 x12_v = spu_splats(x12);
   vec_float4 y12_v = spu_splats(y12);
   vec_float4 r12_v = spu_splats(r12);

   vec_float4 x13_v = spu_splats(x13);
   vec_float4 y13_v = spu_splats(y13);
   vec_float4 r13_v = spu_splats(r13);

   vec_float4 res_v;
   int tolerance = 0x00000001;
   
   TEST_START("hypotf4");

   res_v = (vec_float4)hypotf4(x0_v, y0_v);
   TEST_CHECK("20060831000000AAN", allequal_ulps_float4( res_v, r0_v, tolerance ), 0);
   res_v = (vec_float4)hypotf4(x1_v, y1_v);
   TEST_CHECK("20060831000001AAN", allequal_ulps_float4( res_v, r1_v, tolerance ), 0);
   res_v = (vec_float4)hypotf4(x2_v, y2_v);
   TEST_CHECK("20060831000002AAN", allequal_ulps_float4( res_v, r2_v, tolerance ), 0);
   res_v = (vec_float4)hypotf4(x3_v, y3_v);
   TEST_CHECK("20060831000003AAN", allequal_ulps_float4( res_v, r3_v, tolerance ), 0);
   res_v = (vec_float4)hypotf4(x4_v, y4_v);
   TEST_CHECK("20060831000004AAN", allequal_ulps_float4( res_v, r4_v, tolerance ), 0);
   res_v = (vec_float4)hypotf4(x5_v, y5_v);
   TEST_CHECK("20060831000005AAN", allequal_ulps_float4( res_v, r5_v, tolerance ), 0);
   res_v = (vec_float4)hypotf4(x6_v, y6_v);
   TEST_CHECK("20060831000006AAN", allequal_ulps_float4( res_v, r6_v, tolerance ), 0);
   res_v = (vec_float4)hypotf4(x7_v, y7_v);
   TEST_CHECK("20060831000007AAN", allequal_ulps_float4( res_v, r7_v, tolerance ), 0);
   res_v = (vec_float4)hypotf4(x8_v, y8_v);
   TEST_CHECK("20060831000008AAN", allequal_ulps_float4( res_v, r8_v, tolerance ), 0);
   res_v = (vec_float4)hypotf4(x9_v, y9_v);
   TEST_CHECK("20060831000009AAN", allequal_ulps_float4( res_v, r9_v, tolerance ), 0);
   res_v = (vec_float4)hypotf4(x10_v, y10_v);
   TEST_CHECK("20060831000010AAN", allequal_ulps_float4( res_v, r10_v, tolerance ), 0);
   res_v = (vec_float4)hypotf4(x11_v, y11_v);
   TEST_CHECK("20060831000011AAN", allequal_ulps_float4( res_v, r11_v, tolerance ), 0);
   res_v = (vec_float4)hypotf4(x12_v, y12_v);
   TEST_CHECK("20060831000012AAN", allequal_ulps_float4( res_v, r12_v, tolerance ), 0);
   res_v = (vec_float4)hypotf4(x13_v, y13_v);
   TEST_CHECK("20060831000013AAN", allequal_ulps_float4( res_v, r13_v, tolerance ), 0);
   
   TEST_SET_DONE();
   
   TEST_EXIT();
}
