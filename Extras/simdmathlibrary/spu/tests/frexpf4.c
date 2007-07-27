/* Test frexpf4 for SPU
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
   TEST_SET_START("20060907000000AAN","AAN", "frexpf4");

   // -Norm (IEEE-754: -Nan)
   float x0 = hide_float(make_float(0xFFC00000));
   float r0 = hide_float(make_float(0xBF400000));
   int e0 = 129;
   
   // -Norm (IEEE-754: -Inf)
   float x1 = hide_float(-HUGE_VALF);
   float r1 = hide_float(make_float(0xBF7FFFFF));
   int e1 = 129;
   
   // -Smax
   float x2 = hide_float(make_float(0xFFFFFFFF));
   float r2 = hide_float(make_float(0xBF7FFFFF));
   int e2 = 129;

   // -Norm
   float x3 = hide_float(-824842.58421394);
   float r3 = hide_float(make_float(0xBF4960A9));
   int e3 = 20;
   
   // -Smin
   float x4 = hide_float(make_float(0x80800000));
   float r4 = hide_float(make_float(0xBF000000));
   int e4 = -125;
   
   // -Denorm
   float x5 = hide_float(make_float(0x807AAAAA));
   float r5 = 0.0;
   int e5 = 0;
   
   // -Unf
   float x6 = hide_float(-1.0e-999);
   float r6 = 0.0;
   int e6 = 0;
   
   // -0
   float x7 = hide_float(-0.0);
   float r7 = 0.0;
   int e7 = 0;

   // 0
   float x8 = hide_float( 0.0);
   float r8 = 0.0;
   int e8 = 0;
   
   // +Unf
   float x9 = hide_float( 1.0e-999);
   float r9 = 0.0;
   int e9 = 0;

   // +Denorm
   float x10 = hide_float(make_float(0x007AAAAA));
   float r10 = 0.0;
   int e10 = 0;
   
   // +Smin
   float x11 = hide_float(make_float(0x00800000));
   float r11 = hide_float(make_float(0x3F000000));
   int e11 = -125;
   
   // +Norm
   float x12 = hide_float(3.14152634);
   float r12 = hide_float(make_float(0x3F490EC4));
   int e12 = 2;

   // +Norm
   float x13 = hide_float(7.0673903e37);
   float r13 = hide_float(make_float(0x3F54AD32));
   int e13 = 126;

   // +Norm
   float x14 = hide_float(2.4673e14);
   float r14 = hide_float(make_float(0x3F60664E));
   int e14 = 48;

   // +Norm
   float x15 = hide_float(7.235672e-25);
   float r15 = hide_float(make_float(0x3F5FEEE6));
   int e15 = -80;

   // +Norm
   float x16 = hide_float(9.452854e17);
   float r16 = hide_float(make_float(0x3F51E541));
   int e16 = 60;

   // +Norm
   float x17 = hide_float(3.045784e-18);
   float r17 = hide_float(make_float(0x3F60BD3C));
   int e17 = -58;

   // -Norm
   float x18 = hide_float(-6.459273e7);
   float r18 = hide_float(make_float(0xBF7666D6));
   int e18 = 26;

   // -Norm
   float x19 = hide_float(-2.493472e-9);
   float r19 = hide_float(make_float(0xBF2B59A0));
   int e19 = -28;

   // -Norm
   float x20 = hide_float(-1.4824543e28);
   float r20 = hide_float(make_float(0xBF3F9A4C));
   int e20 = 94;

   // -Norm
   float x21 = hide_float(-5.53856231e-27);
   float r21 = hide_float(make_float(0xBF5B67B2));
   int e21 = -87;

   // -Norm
   float x22 = hide_float(-1.000001);
   float r22 = hide_float(make_float(0xBF000008));
   int e22 = 1;

   // +Smax
   float x23 = hide_float(make_float(0x7FFFFFFF));
   float r23 = hide_float(make_float(0x3F7FFFFF));
   int e23 = 129;
   
   //+Norm (IEEE-754: +Inf)
   float x24 = hide_float(HUGE_VALF);
   float r24 = hide_float(make_float(0x3F7FFFFF));
   int e24 = 129;
   
   //+Norm (IEEE-754: +Nan)
   float x25 = hide_float(make_float(0x7FC00000));
   float r25 = hide_float(make_float(0x3F400000));
   int e25 = 129;

   // Compound
   vec_float4 x26_v = (vec_float4) { -2.561286432e10, make_float(0x7FFFFFFF), -1.0e-999, 7.235672e-25 };
   vec_float4 r26_v = (vec_float4) { make_float(0xBF3ED4A9), make_float(0x3F7FFFFF), 0.0, make_float(0x3F5FEEE6) };
   vec_int4 e26_v = (vec_int4) { 35, 129, 0, -80 };

   // Compound
   vec_float4 x27_v = (vec_float4) { 345.27533f, 7.0673903e37, -0.0f, -2.40e-310 };
   vec_float4 r27_v = (vec_float4) { make_float(0x3F2CA33E), make_float(0x3F54AD32), 0.0, 0.0};
   vec_int4 e27_v = (vec_int4) { 9, 126, 0, 0 };

   // Compound
   vec_float4 x28_v = (vec_float4) { make_float(0x7FC00000), -824842.58421394f, -0.0f, -3678342.8765343f };
   vec_float4 r28_v = (vec_float4) { make_float(0x3F400000), make_float(0xBF4960A9), 0.0, make_float(0xBF60821B) };
   vec_int4 e28_v = (vec_int4) { 129, 20, 0, 22 };

   // Compound
   vec_float4 x29_v = (vec_float4) { HUGE_VALF, 1.0e-99, -5.53856231e-27, make_float(0xFFC00000) };
   vec_float4 r29_v = (vec_float4) { make_float(0x3F7FFFFF), 0.0, make_float(0xBF5B67B2), make_float(0xBF400000) };
   vec_int4 e29_v = (vec_int4) { 129, 0, -87, 129 };

   // Compound
   vec_float4 x30_v = (vec_float4) { 1.2e-57, -1.2e-19, 3.045784e-18, -HUGE_VALF } ;
   vec_float4 r30_v = (vec_float4) { 0.0, make_float(0xBF0DABC6 ), make_float(0x3F60BD3C), make_float(0xBF7FFFFF) };
   vec_int4 e30_v = (vec_int4) { 0, -62, -58, 129 };

   vec_float4 x0_v = spu_splats(x0);
   vec_float4 r0_v = spu_splats(r0);
   vec_int4   e0_v = spu_splats(e0);

   vec_float4 x1_v = spu_splats(x1);
   vec_float4 r1_v = spu_splats(r1);
   vec_int4   e1_v = spu_splats(e1);

   vec_float4 x2_v = spu_splats(x2);
   vec_float4 r2_v = spu_splats(r2);
   vec_int4   e2_v = spu_splats(e2);

   vec_float4 x3_v = spu_splats(x3);
   vec_float4 r3_v = spu_splats(r3);
   vec_int4   e3_v = spu_splats(e3);

   vec_float4 x4_v = spu_splats(x4);
   vec_float4 r4_v = spu_splats(r4);
   vec_int4   e4_v = spu_splats(e4);

   vec_float4 x5_v = spu_splats(x5);
   vec_float4 r5_v = spu_splats(r5);
   vec_int4   e5_v = spu_splats(e5);

   vec_float4 x6_v = spu_splats(x6);
   vec_float4 r6_v = spu_splats(r6);
   vec_int4   e6_v = spu_splats(e6);

   vec_float4 x7_v = spu_splats(x7);
   vec_float4 r7_v = spu_splats(r7);
   vec_int4   e7_v = spu_splats(e7);

   vec_float4 x8_v = spu_splats(x8);
   vec_float4 r8_v = spu_splats(r8);
   vec_int4   e8_v = spu_splats(e8);

   vec_float4 x9_v = spu_splats(x9);
   vec_float4 r9_v = spu_splats(r9);
   vec_int4   e9_v = spu_splats(e9);

   vec_float4 x10_v = spu_splats(x10);
   vec_float4 r10_v = spu_splats(r10);
   vec_int4   e10_v = spu_splats(e10);

   vec_float4 x11_v = spu_splats(x11);
   vec_float4 r11_v = spu_splats(r11);
   vec_int4   e11_v = spu_splats(e11);

   vec_float4 x12_v = spu_splats(x12);
   vec_float4 r12_v = spu_splats(r12);
   vec_int4 e12_v = spu_splats(e12);

   vec_float4 x13_v = spu_splats(x13);
   vec_float4 r13_v = spu_splats(r13);
   vec_int4   e13_v = spu_splats(e13);

   vec_float4 x14_v = spu_splats(x14);
   vec_float4 r14_v = spu_splats(r14);
   vec_int4   e14_v = spu_splats(e14);

   vec_float4 x15_v = spu_splats(x15);
   vec_float4 r15_v = spu_splats(r15);
   vec_int4   e15_v = spu_splats(e15);

   vec_float4 x16_v = spu_splats(x16);
   vec_float4 r16_v = spu_splats(r16);
   vec_int4   e16_v = spu_splats(e16);

   vec_float4 x17_v = spu_splats(x17);
   vec_float4 r17_v = spu_splats(r17);
   vec_int4   e17_v = spu_splats(e17);

   vec_float4 x18_v = spu_splats(x18);
   vec_float4 r18_v = spu_splats(r18);
   vec_int4   e18_v = spu_splats(e18);

   vec_float4 x19_v = spu_splats(x19);
   vec_float4 r19_v = spu_splats(r19);
   vec_int4   e19_v = spu_splats(e19);

   vec_float4 x20_v = spu_splats(x20);
   vec_float4 r20_v = spu_splats(r20);
   vec_int4   e20_v = spu_splats(e20);

   vec_float4 x21_v = spu_splats(x21);
   vec_float4 r21_v = spu_splats(r21);
   vec_int4   e21_v = spu_splats(e21);

   vec_float4 x22_v = spu_splats(x22);
   vec_float4 r22_v = spu_splats(r22);
   vec_int4   e22_v = spu_splats(e22);

   vec_float4 x23_v = spu_splats(x23);
   vec_float4 r23_v = spu_splats(r23);
   vec_int4   e23_v = spu_splats(e23);

   vec_float4 x24_v = spu_splats(x24);
   vec_float4 r24_v = spu_splats(r24);
   vec_int4   e24_v = spu_splats(e24);

   vec_float4 x25_v = spu_splats(x25);
   vec_float4 r25_v = spu_splats(r25);
   vec_int4   e25_v = spu_splats(e25);
   
   vec_float4 res_v;
   vec_int4 exp_v;

   TEST_START("frexpf4");

   res_v = (vec_float4)frexpf4(x0_v, &exp_v);
   TEST_CHECK("20060907000000AAN", allequal_ulps_float4( res_v, r0_v, 0 ), 0);
   TEST_CHECK("20060907000000AAN", allequal_int4( exp_v, e0_v ), 0);
   res_v = (vec_float4)frexpf4(x1_v, &exp_v);
   TEST_CHECK("20060907000001AAN", allequal_ulps_float4( res_v, r1_v, 0 ), 0);
   TEST_CHECK("20060907000001AAN", allequal_int4( exp_v, e1_v ), 0);
   res_v = (vec_float4)frexpf4(x2_v, &exp_v);
   TEST_CHECK("20060907000002AAN", allequal_ulps_float4( res_v, r2_v, 0 ), 0);
   TEST_CHECK("20060907000002AAN", allequal_int4( exp_v, e2_v ), 0);
   res_v = (vec_float4)frexpf4(x3_v, &exp_v);
   TEST_CHECK("20060907000003AAN", allequal_float4( res_v, r3_v ), 0);
   TEST_CHECK("20060907000003AAN", allequal_int4( exp_v, e3_v ), 0);
   res_v = (vec_float4)frexpf4(x4_v, &exp_v);
   TEST_CHECK("20060907000004AAN", allequal_float4( res_v, r4_v ), 0);
   TEST_CHECK("20060907000004AAN", allequal_int4( exp_v, e4_v ), 0);
   res_v = (vec_float4)frexpf4(x5_v, &exp_v);
   TEST_CHECK("20060907000005AAN", allequal_float4( res_v, r5_v ), 0);
   TEST_CHECK("20060907000005AAN", allequal_int4( exp_v, e5_v ), 0);
   res_v = (vec_float4)frexpf4(x6_v, &exp_v);
   TEST_CHECK("20060907000006AAN", allequal_float4( res_v, r6_v ), 0);
   TEST_CHECK("20060907000006AAN", allequal_int4( exp_v, e6_v ), 0);
   res_v = (vec_float4)frexpf4(x7_v, &exp_v);
   TEST_CHECK("20060907000007AAN", allequal_float4( res_v, r7_v ), 0);
   TEST_CHECK("20060907000007AAN", allequal_int4( exp_v, e7_v ), 0);
   res_v = (vec_float4)frexpf4(x8_v, &exp_v);
   TEST_CHECK("20060907000008AAN", allequal_float4( res_v, r8_v ), 0);
   TEST_CHECK("20060907000008AAN", allequal_int4( exp_v, e8_v ), 0);
   res_v = (vec_float4)frexpf4(x9_v, &exp_v);
   TEST_CHECK("20060907000009AAN", allequal_float4( res_v, r9_v ), 0);
   TEST_CHECK("20060907000009AAN", allequal_int4( exp_v, e9_v ), 0);
   res_v = (vec_float4)frexpf4(x10_v, &exp_v);
   TEST_CHECK("20060907000010AAN", allequal_float4( res_v, r10_v ), 0);
   TEST_CHECK("20060907000010AAN", allequal_int4( exp_v, e10_v ), 0);
   res_v = (vec_float4)frexpf4(x11_v, &exp_v);
   TEST_CHECK("20060907000011AAN", allequal_float4( res_v, r11_v ), 0);
   TEST_CHECK("20060907000011AAN", allequal_int4( exp_v, e11_v ), 0);
   res_v = (vec_float4)frexpf4(x12_v, &exp_v);
   TEST_CHECK("20060907000012AAN", allequal_float4( res_v, r12_v ), 0);
   TEST_CHECK("20060907000012AAN", allequal_int4( exp_v, e12_v ), 0);
   res_v = (vec_float4)frexpf4(x13_v, &exp_v);
   TEST_CHECK("20060907000013AAN", allequal_float4( res_v, r13_v ), 0);
   TEST_CHECK("20060907000013AAN", allequal_int4( exp_v, e13_v ), 0);
   res_v = (vec_float4)frexpf4(x14_v, &exp_v);
   TEST_CHECK("20060907000014AAN", allequal_float4( res_v, r14_v ), 0);
   TEST_CHECK("20060907000014AAN", allequal_int4( exp_v, e14_v ), 0);
   res_v = (vec_float4)frexpf4(x15_v, &exp_v);
   TEST_CHECK("20060907000015AAN", allequal_float4( res_v, r15_v ), 0);
   TEST_CHECK("20060907000015AAN", allequal_int4( exp_v, e15_v ), 0);
   res_v = (vec_float4)frexpf4(x16_v, &exp_v);
   TEST_CHECK("20060907000016AAN", allequal_float4( res_v, r16_v ), 0);
   TEST_CHECK("20060907000016AAN", allequal_int4( exp_v, e16_v ), 0);
   res_v = (vec_float4)frexpf4(x17_v, &exp_v);
   TEST_CHECK("20060907000017AAN", allequal_float4( res_v, r17_v ), 0);
   TEST_CHECK("20060907000017AAN", allequal_int4( exp_v, e17_v ), 0);
   res_v = (vec_float4)frexpf4(x18_v, &exp_v);
   TEST_CHECK("20060907000018AAN", allequal_float4( res_v, r18_v ), 0);
   TEST_CHECK("20060907000018AAN", allequal_int4( exp_v, e18_v ), 0);
   res_v = (vec_float4)frexpf4(x19_v, &exp_v);
   TEST_CHECK("20060907000019AAN", allequal_float4( res_v, r19_v ), 0);
   TEST_CHECK("20060907000019AAN", allequal_int4( exp_v, e19_v ), 0);
   res_v = (vec_float4)frexpf4(x20_v, &exp_v);
   TEST_CHECK("20060907000020AAN", allequal_float4( res_v, r20_v ), 0);
   TEST_CHECK("20060907000020AAN", allequal_int4( exp_v, e20_v ), 0);
   res_v = (vec_float4)frexpf4(x21_v, &exp_v);
   TEST_CHECK("20060907000021AAN", allequal_float4( res_v, r21_v ), 0);
   TEST_CHECK("20060907000021AAN", allequal_int4( exp_v, e21_v ), 0);
   res_v = (vec_float4)frexpf4(x22_v, &exp_v);
   TEST_CHECK("20060907000022AAN", allequal_float4( res_v, r22_v ), 0);
   TEST_CHECK("20060907000022AAN", allequal_int4( exp_v, e22_v ), 0);
   res_v = (vec_float4)frexpf4(x23_v, &exp_v);
   TEST_CHECK("20060907000023AAN", allequal_float4( res_v, r23_v ), 0);
   TEST_CHECK("20060907000023AAN", allequal_int4( exp_v, e23_v ), 0);
   res_v = (vec_float4)frexpf4(x24_v, &exp_v);
   TEST_CHECK("20060907000024AAN", allequal_float4( res_v, r24_v ), 0);
   TEST_CHECK("20060907000024AAN", allequal_int4( exp_v, e24_v ), 0);
   res_v = (vec_float4)frexpf4(x25_v, &exp_v);
   TEST_CHECK("20060907000025AAN", allequal_float4( res_v, r25_v ), 0);
   TEST_CHECK("20060907000025AAN", allequal_int4( exp_v, e25_v ), 0);
   res_v = (vec_float4)frexpf4(x26_v, &exp_v);
   TEST_CHECK("20060907000026AAN", allequal_float4( res_v, r26_v ), 0);
   TEST_CHECK("20060907000026AAN", allequal_int4( exp_v, e26_v ), 0);
   res_v = (vec_float4)frexpf4(x27_v, &exp_v);
   TEST_CHECK("20060907000027AAN", allequal_float4( res_v, r27_v ), 0);
   TEST_CHECK("20060907000027AAN", allequal_int4( exp_v, e27_v ), 0);
   res_v = (vec_float4)frexpf4(x28_v, &exp_v);
   TEST_CHECK("20060907000028AAN", allequal_float4( res_v, r28_v ), 0);
   TEST_CHECK("20060907000028AAN", allequal_int4( exp_v, e28_v ), 0);
   res_v = (vec_float4)frexpf4(x29_v, &exp_v);
   TEST_CHECK("20060907000029AAN", allequal_float4( res_v, r29_v ), 0);
   TEST_CHECK("20060907000029AAN", allequal_int4( exp_v, e29_v ), 0);
   res_v = (vec_float4)frexpf4(x30_v, &exp_v);
   TEST_CHECK("20060907000030AAN", allequal_float4( res_v, r30_v ), 0);
   TEST_CHECK("20060907000030AAN", allequal_int4( exp_v, e30_v ), 0);
   
   TEST_SET_DONE();
   
   TEST_EXIT();
}
