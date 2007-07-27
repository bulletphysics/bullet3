/* Test frexpd2 for SPU
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
   TEST_SET_START("20060907000000AAN","AAN", "frexpd2");

   vec_double2 res_v;
   vec_llong2 exp_v;

   // -Nan
   double x0 = hide_double(-nan(""));
   double r0 = hide_double( nan(""));
   //long long e0 = 0;
   
   // -Inf
   double x1 = hide_double(-HUGE_VAL);
   double r1 = x1;
   //long long e1 = 0;
   
   // -Dmax
   double x2 = hide_double(-DBL_MAX);
   double r2 = hide_double(make_double(0xBFEFFFFFFFFFFFFFull));
   long long e2 = 1024;

   // -Norm
   double x3 = hide_double(-824842.58421394);
   double r3 = hide_double(make_double(0xBFE92C152B1E16ECull));
   long long e3 = 20;
   
   // -Dmin
   double x4 = hide_double(make_double(0x8000000000000001ull));
   double r4 = hide_double(make_double(0xBFE0000000000000ull));
   long long e4 = -1073;
   
   // -Denorm
   double x5 = hide_double(-2.40e-310);
   double r5 = hide_double(make_double(0xBFE6170DBAADCD80ull));
   long long e5 = -1028;
   
   // -Unf
   double x6 = hide_double(-1.0e-999);
   double r6 = hide_double(make_double(0x0000000000000000ull));
   long long e6 = 0;
   
   // -0
   double x7 = hide_double(-0.0);
   double r7 = hide_double(make_double(0x0000000000000000ull));
   long long e7 = 0;

   // 0
   double x8 = hide_double( 0.0);
   double r8 = hide_double(make_double(0x0000000000000000ull));
   long long e8 = 0;
   
   // +Unf
   double x9 = hide_double( 1.0e-999);
   double r9 = hide_double(make_double(0x0000000000000000ull));
   long long e9 = 0;

   // +Denorm
   double x10 = hide_double( 2.40e-310);
   double r10 = hide_double(make_double(0x3FE6170DBAADCD80ull));
   long long e10 = -1028;
   
   // +Dmin
   double x11 = hide_double(make_double(0x0000000000000001ull));
   double r11 = hide_double(make_double(0x3FE0000000000000ull));
   long long e11 = -1073;
   
   // +Norm
   double x12 = hide_double(3.14152634);
   double r12 = hide_double(make_double(0x3FE921D88FCE94A3ull));
   long long e12 = 2;

   // +Norm
   double x13 = hide_double(7.0673903e149);
   double r13 = hide_double(make_double(0x3FEBA2C056BA0DB2ull));
   long long e13 = 498;

   // +Norm
   double x14 = hide_double(2.4673e304);
   double r14 = hide_double(make_double(0x3FE1FD485BDF688Eull));
   long long e14 = 1012;

   // +Norm
   double x15 = hide_double(7.235672e-25);
   double r15 = hide_double(make_double(0x3FEBFDDCCA6FF682ull));
   long long e15 = -80;

   // +Denorm
   double x16 = hide_double(9.452854e-312);
   double r16 = hide_double(make_double(0x3FEBD784FE999000ull));
   long long e16 = -1033;

   // +Demorm
   double x17 = hide_double(3.045784e-320);
   double r17 = hide_double(make_double(0x3FE8150000000000ull));
   long long e17 = -1061;

   // -Norm
   double x18 = hide_double(-6.459273e7);
   double r18 = hide_double(make_double(0xBFEECCDAD0000000ull));
   long long e18 = 26;

   // -Norm
   double x19 = hide_double(-2.493472e-99);
   double r19 = hide_double(make_double(0xBFE5D0BDA52F448Cull));
   long long e19 = -327;

   // -Norm
   double x20 = hide_double(-1.4824543e128);
   double r20 = hide_double(make_double(0xBFEB5FFBEAE7B3E1ull));
   long long e20 = 426;

   // -Denorm
   double x21 = hide_double(-5.53856231e-315);
   double r21 = hide_double(make_double(0xBFE0B457A5000000ull));
   long long e21 = -1043;

   // -Demorm
   double x22 = hide_double(-2.5684367e-312);
   double r22 = hide_double(make_double(0xBFEE427A82514000ull));
   long long e22 = -1035;

   // +Dmax
   double x23 = hide_double(DBL_MAX);
   double r23 = hide_double(make_double(0x3FEFFFFFFFFFFFFFull));
   long long e23 = 1024;
   
   // +Inf
   double x24 = hide_double(HUGE_VAL);
   double r24 = x24;
   //long long e24 = 0;
   
   //+Nan
   double x25 = hide_double( nan(""));
   double r25 = hide_double( nan(""));
   //long long e25 = 0;

   // Compound
   vec_double2 x26_v = (vec_double2) { -2.561286432e-317, -1.0e-999 };
   vec_double2 r26_v = (vec_double2) { hide_double(make_double(0xBFE3C69940000000ull)), 0.0 };
   vec_llong2  e26_v = (vec_llong2)  { -1051, 0 };

   // Compound
   vec_double2 x27_v = (vec_double2) { 345.27533, -8.673e-310 };
   vec_double2 r27_v = (vec_double2) { hide_double(make_double(0x3FE59467C06E19B9ull)), hide_double(make_double(0xBFE3F4FCCDB156C0ull)) };
   vec_llong2  e27_v = (vec_llong2)  { 9, -1026 };

   // Compound
   vec_llong2 keep28_v = exp_v;
   vec_double2 x28_v = (vec_double2) { nan(""), -3678342.8765343 };
   vec_double2 r28_v = (vec_double2) { nan(""), hide_double(make_double(0xBFEC1043703246A4ull)) };
   vec_llong2  e28_v = (vec_llong2)  { spu_extract(exp_v, 0), 22 };

   // Compound
   vec_llong2 keep29_v = exp_v;
   vec_double2 x29_v = (vec_double2) { HUGE_VAL, -nan("") };
   vec_double2 r29_v = (vec_double2) { HUGE_VAL, nan("") };
   vec_llong2  e29_v = (vec_llong2)  { spu_extract(exp_v, 0), spu_extract(exp_v, 1) };

   // Compound
   vec_llong2 keep30_v = exp_v;
   vec_double2 x30_v = (vec_double2) { -1.2e-99, -HUGE_VAL } ;
   vec_double2 r30_v = (vec_double2) { hide_double(make_double(0xBFE4FF632B6A83E4ull)), -HUGE_VAL };
   vec_llong2  e30_v = (vec_llong2)  { -328, spu_extract(exp_v, 1) };

   vec_llong2 keep0_v = exp_v;
   vec_double2 x0_v = spu_splats(x0);
   vec_double2 r0_v = spu_splats(r0);
   vec_llong2  e0_v = exp_v;

   vec_llong2 keep1_v = exp_v;
   vec_double2 x1_v = spu_splats(x1);
   vec_double2 r1_v = spu_splats(r1);
   vec_llong2  e1_v = exp_v;

   vec_double2 x2_v = spu_splats(x2);
   vec_double2 r2_v = spu_splats(r2);
   vec_llong2  e2_v = spu_splats(e2);

   vec_double2 x3_v = spu_splats(x3);
   vec_double2 r3_v = spu_splats(r3);
   vec_llong2  e3_v = spu_splats(e3);

   vec_double2 x4_v = spu_splats(x4);
   vec_double2 r4_v = spu_splats(r4);
   vec_llong2  e4_v = spu_splats(e4);

   vec_double2 x5_v = spu_splats(x5);
   vec_double2 r5_v = spu_splats(r5);
   vec_llong2  e5_v = spu_splats(e5);

   vec_double2 x6_v = spu_splats(x6);
   vec_double2 r6_v = spu_splats(r6);
   vec_llong2  e6_v = spu_splats(e6);

   vec_double2 x7_v = spu_splats(x7);
   vec_double2 r7_v = spu_splats(r7);
   vec_llong2  e7_v = spu_splats(e7);

   vec_double2 x8_v = spu_splats(x8);
   vec_double2 r8_v = spu_splats(r8);
   vec_llong2  e8_v = spu_splats(e8);

   vec_double2 x9_v = spu_splats(x9);
   vec_double2 r9_v = spu_splats(r9);
   vec_llong2  e9_v = spu_splats(e9);

   vec_double2 x10_v = spu_splats(x10);
   vec_double2 r10_v = spu_splats(r10);
   vec_llong2  e10_v = spu_splats(e10);

   vec_double2 x11_v = spu_splats(x11);
   vec_double2 r11_v = spu_splats(r11);
   vec_llong2  e11_v = spu_splats(e11);

   vec_double2 x12_v = spu_splats(x12);
   vec_double2 r12_v = spu_splats(r12);
   vec_llong2  e12_v = spu_splats(e12);

   vec_double2 x13_v = spu_splats(x13);
   vec_double2 r13_v = spu_splats(r13);
   vec_llong2  e13_v = spu_splats(e13);

   vec_double2 x14_v = spu_splats(x14);
   vec_double2 r14_v = spu_splats(r14);
   vec_llong2  e14_v = spu_splats(e14);

   vec_double2 x15_v = spu_splats(x15);
   vec_double2 r15_v = spu_splats(r15);
   vec_llong2  e15_v = spu_splats(e15);

   vec_double2 x16_v = spu_splats(x16);
   vec_double2 r16_v = spu_splats(r16);
   vec_llong2  e16_v = spu_splats(e16);

   vec_double2 x17_v = spu_splats(x17);
   vec_double2 r17_v = spu_splats(r17);
   vec_llong2  e17_v = spu_splats(e17);

   vec_double2 x18_v = spu_splats(x18);
   vec_double2 r18_v = spu_splats(r18);
   vec_llong2  e18_v = spu_splats(e18);

   vec_double2 x19_v = spu_splats(x19);
   vec_double2 r19_v = spu_splats(r19);
   vec_llong2  e19_v = spu_splats(e19);

   vec_double2 x20_v = spu_splats(x20);
   vec_double2 r20_v = spu_splats(r20);
   vec_llong2  e20_v = spu_splats(e20);

   vec_double2 x21_v = spu_splats(x21);
   vec_double2 r21_v = spu_splats(r21);
   vec_llong2  e21_v = spu_splats(e21);

   vec_double2 x22_v = spu_splats(x22);
   vec_double2 r22_v = spu_splats(r22);
   vec_llong2  e22_v = spu_splats(e22);

   vec_double2 x23_v = spu_splats(x23);
   vec_double2 r23_v = spu_splats(r23);
   vec_llong2  e23_v = spu_splats(e23);

   vec_llong2 keep24_v = exp_v;
   vec_double2 x24_v = spu_splats(x24);
   vec_double2 r24_v = spu_splats(r24);
   vec_llong2  e24_v = exp_v;

   vec_llong2 keep25_v = exp_v;
   vec_double2 x25_v = spu_splats(x25);
   vec_double2 r25_v = spu_splats(r25);
   vec_llong2  e25_v = exp_v;
   
   TEST_START("frexpd2");

   exp_v = keep0_v;
   res_v = (vec_double2)frexpd2(x0_v, &exp_v);
   TEST_CHECK("20060907000000AAN", allnan_double2( res_v ), 0);  (void)r0_v;
   TEST_CHECK("20060907000000AAN", allequal_llong2( exp_v, e0_v ), 0);
   exp_v = keep1_v;
   res_v = (vec_double2)frexpd2(x1_v, &exp_v);
   TEST_CHECK("20060907000001AAN", allequal_double2( res_v, r1_v ), 0);
   TEST_CHECK("20060907000001AAN", allequal_llong2( exp_v, e1_v ), 0);
   res_v = (vec_double2)frexpd2(x2_v, &exp_v);
   TEST_CHECK("20060907000002AAN", allequal_double2( res_v, r2_v ), 0);
   TEST_CHECK("20060907000002AAN", allequal_llong2( exp_v, e2_v ), 0);
   res_v = (vec_double2)frexpd2(x3_v, &exp_v);
   TEST_CHECK("20060907000003AAN", allequal_double2( res_v, r3_v ), 0);
   TEST_CHECK("20060907000003AAN", allequal_llong2( exp_v, e3_v ), 0);
   res_v = (vec_double2)frexpd2(x4_v, &exp_v);
   TEST_CHECK("20060907000004AAN", allequal_double2( res_v, r4_v ), 0);
   TEST_CHECK("20060907000004AAN", allequal_llong2( exp_v, e4_v ), 0);
   res_v = (vec_double2)frexpd2(x5_v, &exp_v);
   TEST_CHECK("20060907000005AAN", allequal_double2( res_v, r5_v ), 0);
   TEST_CHECK("20060907000005AAN", allequal_llong2( exp_v, e5_v ), 0);
   res_v = (vec_double2)frexpd2(x6_v, &exp_v);
   TEST_CHECK("20060907000006AAN", allequal_double2( res_v, r6_v ), 0);
   TEST_CHECK("20060907000006AAN", allequal_llong2( exp_v, e6_v ), 0);
   res_v = (vec_double2)frexpd2(x7_v, &exp_v);
   TEST_CHECK("20060907000007AAN", allequal_double2( res_v, r7_v ), 0);
   TEST_CHECK("20060907000007AAN", allequal_llong2( exp_v, e7_v ), 0);
   res_v = (vec_double2)frexpd2(x8_v, &exp_v);
   TEST_CHECK("20060907000008AAN", allequal_double2( res_v, r8_v ), 0);
   TEST_CHECK("20060907000008AAN", allequal_llong2( exp_v, e8_v ), 0);
   res_v = (vec_double2)frexpd2(x9_v, &exp_v);
   TEST_CHECK("20060907000009AAN", allequal_double2( res_v, r9_v ), 0);
   TEST_CHECK("20060907000009AAN", allequal_llong2( exp_v, e9_v ), 0);
   res_v = (vec_double2)frexpd2(x10_v, &exp_v);
   TEST_CHECK("20060907000010AAN", allequal_double2( res_v, r10_v ), 0);
   TEST_CHECK("20060907000010AAN", allequal_llong2( exp_v, e10_v ), 0);
   res_v = (vec_double2)frexpd2(x11_v, &exp_v);
   TEST_CHECK("20060907000011AAN", allequal_double2( res_v, r11_v ), 0);
   TEST_CHECK("20060907000011AAN", allequal_llong2( exp_v, e11_v ), 0);
   res_v = (vec_double2)frexpd2(x12_v, &exp_v);
   TEST_CHECK("20060907000012AAN", allequal_double2( res_v, r12_v ), 0);
   TEST_CHECK("20060907000012AAN", allequal_llong2( exp_v, e12_v ), 0);
   res_v = (vec_double2)frexpd2(x13_v, &exp_v);
   TEST_CHECK("20060907000013AAN", allequal_double2( res_v, r13_v ), 0);
   TEST_CHECK("20060907000013AAN", allequal_llong2( exp_v, e13_v ), 0);
   res_v = (vec_double2)frexpd2(x14_v, &exp_v);
   TEST_CHECK("20060907000014AAN", allequal_double2( res_v, r14_v ), 0);
   TEST_CHECK("20060907000014AAN", allequal_llong2( exp_v, e14_v ), 0);
   res_v = (vec_double2)frexpd2(x15_v, &exp_v);
   TEST_CHECK("20060907000015AAN", allequal_double2( res_v, r15_v ), 0);
   TEST_CHECK("20060907000015AAN", allequal_llong2( exp_v, e15_v ), 0);
   res_v = (vec_double2)frexpd2(x16_v, &exp_v);
   TEST_CHECK("20060907000016AAN", allequal_double2( res_v, r16_v ), 0);
   TEST_CHECK("20060907000016AAN", allequal_llong2( exp_v, e16_v ), 0);
   res_v = (vec_double2)frexpd2(x17_v, &exp_v);
   TEST_CHECK("20060907000017AAN", allequal_double2( res_v, r17_v ), 0);
   TEST_CHECK("20060907000017AAN", allequal_llong2( exp_v, e17_v ), 0);
   res_v = (vec_double2)frexpd2(x18_v, &exp_v);
   TEST_CHECK("20060907000018AAN", allequal_double2( res_v, r18_v ), 0);
   TEST_CHECK("20060907000018AAN", allequal_llong2( exp_v, e18_v ), 0);
   res_v = (vec_double2)frexpd2(x19_v, &exp_v);
   TEST_CHECK("20060907000019AAN", allequal_double2( res_v, r19_v ), 0);
   TEST_CHECK("20060907000019AAN", allequal_llong2( exp_v, e19_v ), 0);
   res_v = (vec_double2)frexpd2(x20_v, &exp_v);
   TEST_CHECK("20060907000020AAN", allequal_double2( res_v, r20_v ), 0);
   TEST_CHECK("20060907000020AAN", allequal_llong2( exp_v, e20_v ), 0);
   res_v = (vec_double2)frexpd2(x21_v, &exp_v);
   TEST_CHECK("20060907000021AAN", allequal_double2( res_v, r21_v ), 0);
   TEST_CHECK("20060907000021AAN", allequal_llong2( exp_v, e21_v ), 0);
   res_v = (vec_double2)frexpd2(x22_v, &exp_v);
   TEST_CHECK("20060907000022AAN", allequal_double2( res_v, r22_v ), 0);
   TEST_CHECK("20060907000022AAN", allequal_llong2( exp_v, e22_v ), 0);
   res_v = (vec_double2)frexpd2(x23_v, &exp_v);
   TEST_CHECK("20060907000023AAN", allequal_double2( res_v, r23_v ), 0);
   TEST_CHECK("20060907000023AAN", allequal_llong2( exp_v, e23_v ), 0);
   exp_v = keep24_v;
   res_v = (vec_double2)frexpd2(x24_v, &exp_v);
   TEST_CHECK("20060907000024AAN", allequal_double2( res_v, r24_v ), 0);
   TEST_CHECK("20060907000024AAN", allequal_llong2( exp_v, e24_v ), 0);
   exp_v = keep25_v;
   res_v = (vec_double2)frexpd2(x25_v, &exp_v);
   TEST_CHECK("20060907000025AAN", allnan_double2( res_v ), 0);   (void)r25_v;
   TEST_CHECK("20060907000025AAN", allequal_llong2( exp_v, e25_v ), 0);
   res_v = (vec_double2)frexpd2(x26_v, &exp_v);
   TEST_CHECK("20060907000026AAN", allequal_double2( res_v, r26_v ), 0);
   TEST_CHECK("20060907000026AAN", allequal_llong2( exp_v, e26_v ), 0);
   res_v = (vec_double2)frexpd2(x27_v, &exp_v);
   TEST_CHECK("20060907000027AAN", allequal_double2( res_v, r27_v ), 0);
   TEST_CHECK("20060907000027AAN", allequal_llong2( exp_v, e27_v ), 0);
   exp_v = keep28_v;
   res_v = (vec_double2)frexpd2(x28_v, &exp_v);
   TEST_CHECK("20060907000028AAN", allequal_ulps_double2( res_v, r28_v, 0 ), 0);
   TEST_CHECK("20060907000028AAN", allequal_llong2( exp_v, e28_v ), 0);
   exp_v = keep29_v;
   res_v = (vec_double2)frexpd2(x29_v, &exp_v);
   TEST_CHECK("20060907000029AAN", allequal_ulps_double2( res_v, r29_v, 0 ), 0);
   TEST_CHECK("20060907000029AAN", allequal_llong2( exp_v, e29_v ), 0);
   exp_v = keep30_v;
   res_v = (vec_double2)frexpd2(x30_v, &exp_v);
   TEST_CHECK("20060907000030AAN", allequal_ulps_double2( res_v, r30_v, 0 ), 0);
   TEST_CHECK("20060907000000AAN", allequal_llong2( exp_v, e30_v ), 0);
   
   TEST_SET_DONE();
   
   TEST_EXIT();
}
