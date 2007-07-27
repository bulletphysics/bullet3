/* Test hypotd2 for SPU
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
   TEST_SET_START("20060831000000AAN","AAN", "hypotd2");

   // Does not test precesion, which depends on sqrtd2 implementation
   // Uses Pythagorean triplets to test result validity

   //-QNaN, Norm
   double x0 = hide_double(-nan(""));
   double y0 = hide_double(1.0);
   double r0 = hide_double( nan(""));

   //-QNaN, -Norm
   double x1 = hide_double(-nan(""));
   double y1 = hide_double(-21345853556.492);
   double r1 = hide_double( nan(""));

   //-Inf, -QNaN
   double x2 = hide_double(-HUGE_VAL);
   double y2 = hide_double(make_double(0xFFFFFFFFFFFFFFFFull));
   double r2 = hide_double( nan(""));

   //-Norm, -SNaN
   double x3 = hide_double(-67418234.34256245);
   double y3 = hide_double(make_double(0xFFF7FFFFFFFFFFFFull));
   double r3 = hide_double( nan(""));

   //-Norm, -Denorm
   double x4 = hide_double(-4.0e120);
   double y4 = hide_double(-3.0e-320);
   double r4 = hide_double( 4.0e120);

   //-Norm, -Inf
   double x5 = hide_double(-168.97345223013);
   double y5 = hide_double(-HUGE_VAL);
   double r5 = hide_double(HUGE_VAL);

   //+Inf, -Inf
   double x6 = hide_double(HUGE_VAL);
   double y6 = hide_double(-HUGE_VAL);
   double r6 = hide_double(HUGE_VAL);

   //-Norm, -0
   double x7 = hide_double(-168.97345223013);
   double y7 = hide_double(-0.0);
   double r7 = hide_double( 168.97345223013);

   //-Unf, -Norm
   double x8 = hide_double(-1.0e-999);
   double y8 = hide_double(-83532.96153153);
   double r8 = hide_double( 83532.96153153);

   //-Unf, 0
   double x9 = hide_double(-1.0e-999);
   double y9 = hide_double(0.0);
   double r9 = hide_double(0.0);

   //QNaN, 0
   double x10 = hide_double(make_double(0x7FFFFFFFFFFFFFFFull));
   double y10 = hide_double( 0.0);
   double r10 = hide_double( nan(""));

   //+Unf, +QNaN
   double x11 = hide_double( 1.0e-999);
   double y11 = hide_double( nan(""));
   double r11 = hide_double( nan(""));

   //+Unf, +Norm
   double x12 = hide_double( 1e-999);
   double y12 = hide_double(0.0031529324);
   double r12 = hide_double(0.0031529324);

   //+Norm, +Norm
   double x13 = hide_double(55000.0e116);
   double y13 = hide_double(480.0e118);
   double r13 = hide_double(7.3e120);

   //+Norm, -Denorm
   double x14 = hide_double(120.0e120);
   double y14 = hide_double(-5.0e-321);
   double r14 = hide_double(120.0e120/*1.3e101*/);

   //-Norm, +Demorm
   double x15 = hide_double(-0.0000000008);
   double y15 = hide_double(1.5e-320);
   double r15 = hide_double( 0.8e-9);

   //+Norm, -Norm
   double x16 = hide_double( 7.0e-6);
   double y16 = hide_double(-24.0e-6);
   double r16 = hide_double(25.0e-6);

   //+Norm, +Norm
   double x17 = hide_double(0.0055);
   double y17 = hide_double(0.0048);
   double r17 = hide_double(0.0073);

   //+Denorm, +Norm
   double x18 = hide_double(4.5e-320);
   double y18 = hide_double(2.8);
   double r18 = hide_double(2.8);

   //-Norm, +Norm
   double x19 = hide_double(-8000.0);
   double y19 = hide_double(39.0e2);
   double r19 = hide_double(8900.0);

   //+Norm, +Norm
   double x20 = hide_double(6.5e128);
   double y20 = hide_double(7.2e128);
   double r20 = hide_double(9.7e128);

   //-Norm, -Norm
   double x21 = hide_double(-0.0035);
   double y21 = hide_double(-1.2e-3);
   double r21 = hide_double( 0.0037);

   //+Norm, +Norm
   double x22 = hide_double(456548.6027761);
   double y22 = hide_double(106165.2293520);
   double r22 = hide_double(468729.8610289);

   //+Inf, +Ovf
   double x23 = hide_double(HUGE_VAL);
   double y23 = hide_double(1.0e999);
   double r23 = hide_double(HUGE_VAL);

   //+Norm, +QNaN
   double x24 = hide_double(264.345643345);
   double y24 = hide_double(make_double(0x7FFAAAAAAAAAAAAAull));
   double r24 = hide_double( nan(""));

   //+Inf, +QNaN
   double x25 = hide_double(HUGE_VAL);
   double y25 = hide_double(nan(""));
   double r25 = hide_double(nan(""));

   vec_double2 x0_v = spu_splats(x0);
   vec_double2 y0_v = spu_splats(y0);
   vec_double2 r0_v = spu_splats(r0);

   vec_double2 x1_v = spu_splats(x1);
   vec_double2 y1_v = spu_splats(y1);
   vec_double2 r1_v = spu_splats(r1);

   vec_double2 x2_v = spu_splats(x2);
   vec_double2 y2_v = spu_splats(y2);
   vec_double2 r2_v = spu_splats(r2);

   vec_double2 x3_v = spu_splats(x3);
   vec_double2 y3_v = spu_splats(y3);
   vec_double2 r3_v = spu_splats(r3);

   vec_double2 x4_v = spu_splats(x4);
   vec_double2 y4_v = spu_splats(y4);
   vec_double2 r4_v = spu_splats(r4);

   vec_double2 x5_v = spu_splats(x5);
   vec_double2 y5_v = spu_splats(y5);
   vec_double2 r5_v = spu_splats(r5);

   vec_double2 x6_v = spu_splats(x6);
   vec_double2 y6_v = spu_splats(y6);
   vec_double2 r6_v = spu_splats(r6);

   vec_double2 x7_v = spu_splats(x7);
   vec_double2 y7_v = spu_splats(y7);
   vec_double2 r7_v = spu_splats(r7);

   vec_double2 x8_v = spu_splats(x8);
   vec_double2 y8_v = spu_splats(y8);
   vec_double2 r8_v = spu_splats(r8);

   vec_double2 x9_v = spu_splats(x9);
   vec_double2 y9_v = spu_splats(y9);
   vec_double2 r9_v = spu_splats(r9);

   vec_double2 x10_v = spu_splats(x10);
   vec_double2 y10_v = spu_splats(y10);
   vec_double2 r10_v = spu_splats(r10);

   vec_double2 x11_v = spu_splats(x11);
   vec_double2 y11_v = spu_splats(y11);
   vec_double2 r11_v = spu_splats(r11);

   vec_double2 x12_v = spu_splats(x12);
   vec_double2 y12_v = spu_splats(y12);
   vec_double2 r12_v = spu_splats(r12);

   vec_double2 x13_v = spu_splats(x13);
   vec_double2 y13_v = spu_splats(y13);
   vec_double2 r13_v = spu_splats(r13);

   vec_double2 x14_v = spu_splats(x14);
   vec_double2 y14_v = spu_splats(y14);
   vec_double2 r14_v = spu_splats(r14);

   vec_double2 x15_v = spu_splats(x15);
   vec_double2 y15_v = spu_splats(y15);
   vec_double2 r15_v = spu_splats(r15);

   vec_double2 x16_v = spu_splats(x16);
   vec_double2 y16_v = spu_splats(y16);
   vec_double2 r16_v = spu_splats(r16);

   vec_double2 x17_v = spu_splats(x17);
   vec_double2 y17_v = spu_splats(y17);
   vec_double2 r17_v = spu_splats(r17);

   vec_double2 x18_v = spu_splats(x18);
   vec_double2 y18_v = spu_splats(y18);
   vec_double2 r18_v = spu_splats(r18);

   vec_double2 x19_v = spu_splats(x19);
   vec_double2 y19_v = spu_splats(y19);
   vec_double2 r19_v = spu_splats(r19);

   vec_double2 x20_v = spu_splats(x20);
   vec_double2 y20_v = spu_splats(y20);
   vec_double2 r20_v = spu_splats(r20);

   vec_double2 x21_v = spu_splats(x21);
   vec_double2 y21_v = spu_splats(y21);
   vec_double2 r21_v = spu_splats(r21);

   vec_double2 x22_v = spu_splats(x22);
   vec_double2 y22_v = spu_splats(y22);
   vec_double2 r22_v = spu_splats(r22);

   vec_double2 x23_v = spu_splats(x23);
   vec_double2 y23_v = spu_splats(y23);
   vec_double2 r23_v = spu_splats(r23);

   vec_double2 x24_v = spu_splats(x24);
   vec_double2 y24_v = spu_splats(y24);
   vec_double2 r24_v = spu_splats(r24);

   vec_double2 x25_v = spu_splats(x25);
   vec_double2 y25_v = spu_splats(y25);
   vec_double2 r25_v = spu_splats(r25);
     
   vec_double2 res_v;
   int tolerance = (int)0x0000000000000001ull;

   TEST_START("hypotd2");

   res_v = (vec_double2)hypotd2(x0_v, y0_v);
   TEST_CHECK("20060831000000AAN", allnan_double2( res_v ), 0);  (void)r0_v;
   res_v = (vec_double2)hypotd2(x1_v, y1_v);
   TEST_CHECK("20060831000001AAN", allnan_double2( res_v ), 0);  (void)r1_v;
   res_v = (vec_double2)hypotd2(x2_v, y2_v);
   TEST_CHECK("20060831000002AAN", allnan_double2( res_v ), 0);  (void)r2_v;
   res_v = (vec_double2)hypotd2(x3_v, y3_v);
   TEST_CHECK("20060831000003AAN", allnan_double2( res_v ), 0);  (void)r3_v;
   res_v = (vec_double2)hypotd2(x4_v, y4_v);
   TEST_CHECK("20060831000004AAN", allequal_ulps_double2( res_v, r4_v, tolerance ), 0);
   res_v = (vec_double2)hypotd2(x5_v, y5_v);
   TEST_CHECK("20060831000005AAN", allequal_ulps_double2( res_v, r5_v, 0 ), 0);
   res_v = (vec_double2)hypotd2(x6_v, y6_v);
   TEST_CHECK("20060831000006AAN", allequal_ulps_double2( res_v, r6_v, 0 ), 0);
   res_v = (vec_double2)hypotd2(x7_v, y7_v);
   TEST_CHECK("20060831000007AAN", allequal_ulps_double2( res_v, r7_v, tolerance ), 0);
   res_v = (vec_double2)hypotd2(x8_v, y8_v);
   TEST_CHECK("20060831000008AAN", allequal_ulps_double2( res_v, r8_v, tolerance ), 0);
   res_v = (vec_double2)hypotd2(x9_v, y9_v);
   TEST_CHECK("20060831000009AAN", allequal_ulps_double2( res_v, r9_v, tolerance ), 0);
   res_v = (vec_double2)hypotd2(x10_v, y10_v);
   TEST_CHECK("20060831000010AAN", allnan_double2( res_v ), 0);  (void)r10_v;
   res_v = (vec_double2)hypotd2(x11_v, y11_v);
   TEST_CHECK("20060831000011AAN", allnan_double2( res_v ), 0);  (void)r11_v;
   res_v = (vec_double2)hypotd2(x12_v, y12_v);
   TEST_CHECK("20060831000012AAN", allequal_ulps_double2( res_v, r12_v, tolerance ), 0);
   res_v = (vec_double2)hypotd2(x13_v, y13_v);
   TEST_CHECK("20060831000013AAN", allequal_ulps_double2( res_v, r13_v, tolerance ), 0);
   res_v = (vec_double2)hypotd2(x14_v, y14_v);
   TEST_CHECK("20060831000014AAN", allequal_ulps_double2( res_v, r14_v, tolerance ), 0);
   res_v = (vec_double2)hypotd2(x15_v, y15_v);
   TEST_CHECK("20060831000015AAN", allequal_ulps_double2( res_v, r15_v, tolerance ), 0);
   res_v = (vec_double2)hypotd2(x16_v, y16_v);
   TEST_CHECK("20060831000016AAN", allequal_ulps_double2( res_v, r16_v, tolerance ), 0);
   res_v = (vec_double2)hypotd2(x17_v, y17_v);
   TEST_CHECK("20060831000017AAN", allequal_ulps_double2( res_v, r17_v, tolerance ), 0);
   res_v = (vec_double2)hypotd2(x18_v, y18_v);
   TEST_CHECK("20060831000018AAN", allequal_ulps_double2( res_v, r18_v, tolerance ), 0);
   res_v = (vec_double2)hypotd2(x19_v, y19_v);
   TEST_CHECK("20060831000019AAN", allequal_ulps_double2( res_v, r19_v, tolerance ), 0);
   res_v = (vec_double2)hypotd2(x20_v, y20_v);
   TEST_CHECK("20060831000020AAN", allequal_ulps_double2( res_v, r20_v, tolerance ), 0);
   res_v = (vec_double2)hypotd2(x21_v, y21_v);
   TEST_CHECK("20060831000021AAN", allequal_ulps_double2( res_v, r21_v, tolerance ), 0);
   res_v = (vec_double2)hypotd2(x22_v, y22_v);
   TEST_CHECK("20060831000022AAN", allequal_ulps_double2( res_v, r22_v, tolerance ), 0);
   res_v = (vec_double2)hypotd2(x23_v, y23_v);
   TEST_CHECK("20060831000023AAN", allequal_ulps_double2( res_v, r23_v, 0 ), 0);
   res_v = (vec_double2)hypotd2(x24_v, y24_v);
   TEST_CHECK("20060831000024AAN", allnan_double2( res_v ), 0);  (void)r24_v;
   res_v = (vec_double2)hypotd2(x25_v, y25_v);
   TEST_CHECK("20060831000025AAN", allnan_double2( res_v ), 0);  (void)r25_v;
   
   TEST_SET_DONE();
   
   TEST_EXIT();
}
