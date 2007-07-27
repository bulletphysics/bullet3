/* Test ilogbf4 for SPU
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
#include <limits.h>
#include <math.h>
#include <float.h>
#include "simdmath.h"
#include "common-test.h"
#include "testutils.h"

int main()
{
   TEST_SET_START("20060904000000AAN","AAN", "ilogbf4");

   // -Norm (IEEE-754: -Nan)
   float x0 = hide_float(make_float(0xFFC00000));
   int r0 = 128;
   
   // -Norm (IEEE-754: -Inf)
   float x1 = hide_float(-HUGE_VALF);
   int r1 = 128;
   
   // -Smax
   float x2 = hide_float(-FLT_MAX);
   int r2 = 128;

   // -Norm
   float x3 = hide_float(-824842.58421394);
   int r3 = 19;
   
   // -Smin
   float x4 = hide_float(make_float(0x80800000));
   int r4 = -126;
   
   // -Denorm
   float x5 = hide_float(make_float(0x807AAAAA));
   int r5 = (int)FP_ILOGB0;
   
   // -Unf
   float x6 = hide_float(-1.0e-999);
   int r6 = (int)FP_ILOGB0;
   
   // -0
   float x7 = hide_float(-0.0);
   int r7 = (int)FP_ILOGB0;

   // 0
   float x8 = hide_float( 0.0);
   int r8 = (int)FP_ILOGB0;
   
   // +Unf
   float x9 = hide_float( 1.0e-999);
   int r9 = (int)FP_ILOGB0;

   // +Denorm
   float x10 = hide_float(make_float(0x007AAAAA));
   int r10 = (int)FP_ILOGB0;
   
   // +Smin
   float x11 = hide_float(make_float(0x00800000));
   int r11 = -126;
   
   // +Norm
   float x12 = hide_float(3.14152634);
   int r12 = 1;

   // +Norm
   float x13 = hide_float(7.0673903e37);
   int r13 = 125;

   // +Norm
   float x14 = hide_float(2.4673e14);
   int r14 = 47;

   // +Norm
   float x15 = hide_float(7.235672e-25);
   int r15 = -81;

   // +Norm
   float x16 = hide_float(9.452854e17);
   int r16 = 59;

   // +Norm
   float x17 = hide_float(3.045784e-18);
   int r17 = -59;

   // -Norm
   float x18 = hide_float(-6.459273e7);
   int r18 = 25;

   // -Norm
   float x19 = hide_float(-2.493472e-9);
   int r19 = -29;

   // -Norm
   float x20 = hide_float(-1.4824543e28);
   int r20 = 93;

   // -Norm
   float x21 = hide_float(-5.53856231e-27);
   int r21 = -88;

   // -Norm
   float x22 = hide_float(-1.000001);
   int r22 = 0;

   // +Smax
   float x23 = hide_float(FLT_MAX);
   int r23 = 128;
   
   //+Norm (IEEE-754: +Inf)
   float x24 = hide_float( HUGE_VALF);
   int r24 = 128;
   
   //+Norm (IEEE-754: +Nan)
   float x25 = hide_float(make_float(0x7FC00000));
   int r25 = 128;

   // Compound
   vec_float4 x26_v = (vec_float4) { -2.561286432e10, FLT_MAX, -1.0e-999, 7.235672e-25 };
   vec_int4 r26_v = (vec_int4) { 34, 128, FP_ILOGB0, -81 };

   // Compound
   vec_float4 x27_v = (vec_float4) { 345.27533, 7.0673903e37, -0.0, -2.40e-310 };
   vec_int4 r27_v = (vec_int4) { 8, 125, FP_ILOGB0, FP_ILOGB0 };

   // Compound
   vec_float4 x28_v = (vec_float4) { make_float(0x7FC00000), -824842.58421394, -0.0, -3678342.8765343 };
   vec_int4 r28_v = (vec_int4) { 128, 19, FP_ILOGB0, 21 };

   // Compound
   vec_float4 x29_v = (vec_float4) { HUGE_VALF, 1.0e-99, -5.53856231e-27, make_float(0xFFC00000) };
   vec_int4 r29_v = (vec_int4) { 128, FP_ILOGB0, -88, 128 };

   // Compound
   vec_float4 x30_v = (vec_float4) { 1.2e-57, -1.2e-19, 3.045784e-18, -HUGE_VALF } ;
   vec_int4 r30_v = (vec_int4) { FP_ILOGB0, -63, -59, 128 };

   vec_float4 x0_v = spu_splats(x0);
   vec_int4 r0_v = spu_splats(r0);

   vec_float4 x1_v = spu_splats(x1);
   vec_int4 r1_v = spu_splats(r1);

   vec_float4 x2_v = spu_splats(x2);
   vec_int4 r2_v = spu_splats(r2);

   vec_float4 x3_v = spu_splats(x3);
   vec_int4 r3_v = spu_splats(r3);

   vec_float4 x4_v = spu_splats(x4);
   vec_int4 r4_v = spu_splats(r4);

   vec_float4 x5_v = spu_splats(x5);
   vec_int4 r5_v = spu_splats(r5);

   vec_float4 x6_v = spu_splats(x6);
   vec_int4 r6_v = spu_splats(r6);

   vec_float4 x7_v = spu_splats(x7);
   vec_int4 r7_v = spu_splats(r7);

   vec_float4 x8_v = spu_splats(x8);
   vec_int4 r8_v = spu_splats(r8);

   vec_float4 x9_v = spu_splats(x9);
   vec_int4 r9_v = spu_splats(r9);

   vec_float4 x10_v = spu_splats(x10);
   vec_int4 r10_v = spu_splats(r10);

   vec_float4 x11_v = spu_splats(x11);
   vec_int4 r11_v = spu_splats(r11);

   vec_float4 x12_v = spu_splats(x12);
   vec_int4 r12_v = spu_splats(r12);

   vec_float4 x13_v = spu_splats(x13);
   vec_int4 r13_v = spu_splats(r13);

   vec_float4 x14_v = spu_splats(x14);
   vec_int4 r14_v = spu_splats(r14);

   vec_float4 x15_v = spu_splats(x15);
   vec_int4 r15_v = spu_splats(r15);

   vec_float4 x16_v = spu_splats(x16);
   vec_int4 r16_v = spu_splats(r16);

   vec_float4 x17_v = spu_splats(x17);
   vec_int4 r17_v = spu_splats(r17);

   vec_float4 x18_v = spu_splats(x18);
   vec_int4 r18_v = spu_splats(r18);

   vec_float4 x19_v = spu_splats(x19);
   vec_int4 r19_v = spu_splats(r19);

   vec_float4 x20_v = spu_splats(x20);
   vec_int4 r20_v = spu_splats(r20);

   vec_float4 x21_v = spu_splats(x21);
   vec_int4 r21_v = spu_splats(r21);

   vec_float4 x22_v = spu_splats(x22);
   vec_int4 r22_v = spu_splats(r22);

   vec_float4 x23_v = spu_splats(x23);
   vec_int4 r23_v = spu_splats(r23);

   vec_float4 x24_v = spu_splats(x24);
   vec_int4 r24_v = spu_splats(r24);

   vec_float4 x25_v = spu_splats(x25);
   vec_int4 r25_v = spu_splats(r25);
   
   vec_int4 res_v;

   TEST_START("ilogbf4");

   res_v = (vec_int4)ilogbf4(x0_v);
   TEST_CHECK("20060904000000AAN", allequal_int4( res_v, r0_v ), 0);
   res_v = (vec_int4)ilogbf4(x1_v);
   TEST_CHECK("20060904000001AAN", allequal_int4( res_v, r1_v ), 0);
   res_v = (vec_int4)ilogbf4(x2_v);
   TEST_CHECK("20060904000002AAN", allequal_int4( res_v, r2_v ), 0);
   res_v = (vec_int4)ilogbf4(x3_v);
   TEST_CHECK("20060904000003AAN", allequal_int4( res_v, r3_v ), 0);
   res_v = (vec_int4)ilogbf4(x4_v);
   TEST_CHECK("20060904000004AAN", allequal_int4( res_v, r4_v ), 0);
   res_v = (vec_int4)ilogbf4(x5_v);
   TEST_CHECK("20060904000005AAN", allequal_int4( res_v, r5_v ), 0);
   res_v = (vec_int4)ilogbf4(x6_v);
   TEST_CHECK("20060904000006AAN", allequal_int4( res_v, r6_v ), 0);
   res_v = (vec_int4)ilogbf4(x7_v);
   TEST_CHECK("20060904000007AAN", allequal_int4( res_v, r7_v ), 0);
   res_v = (vec_int4)ilogbf4(x8_v);
   TEST_CHECK("20060904000008AAN", allequal_int4( res_v, r8_v ), 0);
   res_v = (vec_int4)ilogbf4(x9_v);
   TEST_CHECK("20060904000009AAN", allequal_int4( res_v, r9_v ), 0);
   res_v = (vec_int4)ilogbf4(x10_v);
   TEST_CHECK("20060904000010AAN", allequal_int4( res_v, r10_v ), 0);
   res_v = (vec_int4)ilogbf4(x11_v);
   TEST_CHECK("20060904000011AAN", allequal_int4( res_v, r11_v ), 0);
   res_v = (vec_int4)ilogbf4(x12_v);
   TEST_CHECK("20060904000012AAN", allequal_int4( res_v, r12_v ), 0);
   res_v = (vec_int4)ilogbf4(x13_v);
   TEST_CHECK("20060904000013AAN", allequal_int4( res_v, r13_v ), 0);
   res_v = (vec_int4)ilogbf4(x14_v);
   TEST_CHECK("20060904000014AAN", allequal_int4( res_v, r14_v ), 0);
   res_v = (vec_int4)ilogbf4(x15_v);
   TEST_CHECK("20060904000015AAN", allequal_int4( res_v, r15_v ), 0);
   res_v = (vec_int4)ilogbf4(x16_v);
   TEST_CHECK("20060904000016AAN", allequal_int4( res_v, r16_v ), 0);
   res_v = (vec_int4)ilogbf4(x17_v);
   TEST_CHECK("20060904000017AAN", allequal_int4( res_v, r17_v ), 0);
   res_v = (vec_int4)ilogbf4(x18_v);
   TEST_CHECK("20060904000018AAN", allequal_int4( res_v, r18_v ), 0);
   res_v = (vec_int4)ilogbf4(x19_v);
   TEST_CHECK("20060904000019AAN", allequal_int4( res_v, r19_v ), 0);
   res_v = (vec_int4)ilogbf4(x20_v);
   TEST_CHECK("20060904000020AAN", allequal_int4( res_v, r20_v ), 0);
   res_v = (vec_int4)ilogbf4(x21_v);
   TEST_CHECK("20060904000021AAN", allequal_int4( res_v, r21_v ), 0);
   res_v = (vec_int4)ilogbf4(x22_v);
   TEST_CHECK("20060904000022AAN", allequal_int4( res_v, r22_v ), 0);
   res_v = (vec_int4)ilogbf4(x23_v);
   TEST_CHECK("20060904000023AAN", allequal_int4( res_v, r23_v ), 0);
   res_v = (vec_int4)ilogbf4(x24_v);
   TEST_CHECK("20060904000024AAN", allequal_int4( res_v, r24_v ), 0);
   res_v = (vec_int4)ilogbf4(x25_v);
   TEST_CHECK("20060904000025AAN", allequal_int4( res_v, r25_v ), 0);
   res_v = (vec_int4)ilogbf4(x26_v);
   TEST_CHECK("20060904000026AAN", allequal_int4( res_v, r26_v ), 0);
   res_v = (vec_int4)ilogbf4(x27_v);
   TEST_CHECK("20060904000027AAN", allequal_int4( res_v, r27_v ), 0);
   res_v = (vec_int4)ilogbf4(x28_v);
   TEST_CHECK("20060904000028AAN", allequal_int4( res_v, r28_v ), 0);
   res_v = (vec_int4)ilogbf4(x29_v);
   TEST_CHECK("20060904000029AAN", allequal_int4( res_v, r29_v ), 0);
   res_v = (vec_int4)ilogbf4(x30_v);
   TEST_CHECK("20060904000030AAN", allequal_int4( res_v, r30_v ), 0);
   
   TEST_SET_DONE();
   
   TEST_EXIT();
}
