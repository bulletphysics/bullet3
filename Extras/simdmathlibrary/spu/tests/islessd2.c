/* Test islessd2 for SPU
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
   TEST_SET_START("20060825000000AAN","AAN", "islessd2");

   //-QNaN: NG
   double x0 = hide_double(-nan(""));
   double y0 = hide_double(1.0);
   unsigned long long r0 = 0x0000000000000000ull;

   //+Inf > -Inf
   double x1 = hide_double( HUGE_VAL);
   double y1 = hide_double(-HUGE_VAL);
   unsigned long long r1 = 0x0000000000000000ull;

   //-Inf < -Dmax
   double x2 = hide_double(-HUGE_VAL);
   double y2 = hide_double(-DBL_MAX);
   unsigned long long r2 = 0xffffffffffffffffull;

   //-Norm > -Inf
   double x3 = hide_double(-67418234.34256245);
   double y3 = hide_double(-HUGE_VAL);
   unsigned long long r3 = 0x0000000000000000ull;

   //-Norm < -Denorm
   double x4 = hide_double(-273453.3234458053);
   double y4 = hide_double(-3.0e-321);
   unsigned long long r4 = 0xffffffffffffffffull;

   //-Norm = -Norm
   double x5 = hide_double(-168.97345223013);
   double y5 = hide_double(-168.97345223013);
   unsigned long long r5 = 0x0000000000000000ull;

   //-Norm > -Norm
   double x6 = hide_double(-168.97345223013);
   double y6 = hide_double(-21345853556.492);
   unsigned long long r6 = 0x0000000000000000ull;

   //-Norm < -0
   double x7 = hide_double(-168.97345223013);
   double y7 = hide_double(-0.0);
   unsigned long long r7 = 0xffffffffffffffffull;

   //-Unf > -Norm
   double x8 = hide_double(-1.0e-999);
   double y8 = hide_double(-83532.96153153);
   unsigned long long r8 = 0x0000000000000000ull;

   //-Unf = 0
   double x9 = hide_double(-1.0e-999);
   double y9 = hide_double(0.0);
   unsigned long long r9 = 0x0000000000000000ull;

   //-0 = 0
   double x10 = hide_double(-0.0);
   double y10 = hide_double( 0.0);
   unsigned long long r10 = 0x0000000000000000ull;

   //+Unf = 0
   double x11 = hide_double( 1.0e-999);
   double y11 = hide_double( 0.0);
   unsigned long long r11 = 0x0000000000000000ull;

   //+Unf < +Norm
   double x12 = hide_double( 1e-999);
   double y12 = hide_double(0.0031529324);
   unsigned long long r12 = 0xffffffffffffffffull;

   //+Norm > +Denorm
   double x13 = hide_double(5172.2845321);
   double y13 = hide_double(3.0e-321);
   unsigned long long r13 = 0x0000000000000000ull;

   //+Norm = +Norm
   double x14 = hide_double(5172.2845321);
   double y14 = hide_double(5172.2845321);
   unsigned long long r14 = 0x0000000000000000ull;

   //+Norm < +Norm
   double x15 = hide_double(264.345643345);
   double y15 = hide_double(2353705.31415);
   unsigned long long r15 = 0xffffffffffffffffull;

   //+Norm > -Norm
   double x16 = hide_double( 926.605118542);
   double y16 = hide_double(-9.43574552184);
   unsigned long long r16 = 0x0000000000000000ull;

   //+Norm < +Dmax
   double x17 = hide_double( 926.605118542);
   double y17 = hide_double(DBL_MAX);
   unsigned long long r17 = 0xffffffffffffffffull;
   
   //+Inf > +Dmax
   double x18 = hide_double(HUGE_VAL);
   double y18 = hide_double(DBL_MAX);
   unsigned long long r18 = 0x0000000000000000ull;

   //+QNaN: NG
   double x19 = hide_double(nan(""));
   double y19 = hide_double(3.14);
   unsigned long long r19 = 0x0000000000000000ull;

   vec_double2 x0_v = spu_splats(x0);
   vec_double2 y0_v = spu_splats(y0);
   vec_ullong2 r0_v = spu_splats(r0);

   vec_double2 x1_v = spu_splats(x1);
   vec_double2 y1_v = spu_splats(y1);
   vec_ullong2 r1_v = spu_splats(r1);

   vec_double2 x2_v = spu_splats(x2);
   vec_double2 y2_v = spu_splats(y2);
   vec_ullong2 r2_v = spu_splats(r2);

   vec_double2 x3_v = spu_splats(x3);
   vec_double2 y3_v = spu_splats(y3);
   vec_ullong2 r3_v = spu_splats(r3);

   vec_double2 x4_v = spu_splats(x4);
   vec_double2 y4_v = spu_splats(y4);
   vec_ullong2 r4_v = spu_splats(r4);

   vec_double2 x5_v = spu_splats(x5);
   vec_double2 y5_v = spu_splats(y5);
   vec_ullong2 r5_v = spu_splats(r5);

   vec_double2 x6_v = spu_splats(x6);
   vec_double2 y6_v = spu_splats(y6);
   vec_ullong2 r6_v = spu_splats(r6);

   vec_double2 x7_v = spu_splats(x7);
   vec_double2 y7_v = spu_splats(y7);
   vec_ullong2 r7_v = spu_splats(r7);

   vec_double2 x8_v = spu_splats(x8);
   vec_double2 y8_v = spu_splats(y8);
   vec_ullong2 r8_v = spu_splats(r8);

   vec_double2 x9_v = spu_splats(x9);
   vec_double2 y9_v = spu_splats(y9);
   vec_ullong2 r9_v = spu_splats(r9);

   vec_double2 x10_v = spu_splats(x10);
   vec_double2 y10_v = spu_splats(y10);
   vec_ullong2 r10_v = spu_splats(r10);

   vec_double2 x11_v = spu_splats(x11);
   vec_double2 y11_v = spu_splats(y11);
   vec_ullong2 r11_v = spu_splats(r11);

   vec_double2 x12_v = spu_splats(x12);
   vec_double2 y12_v = spu_splats(y12);
   vec_ullong2 r12_v = spu_splats(r12);

   vec_double2 x13_v = spu_splats(x13);
   vec_double2 y13_v = spu_splats(y13);
   vec_ullong2 r13_v = spu_splats(r13);

   vec_double2 x14_v = spu_splats(x14);
   vec_double2 y14_v = spu_splats(y14);
   vec_ullong2 r14_v = spu_splats(r14);

   vec_double2 x15_v = spu_splats(x15);
   vec_double2 y15_v = spu_splats(y15);
   vec_ullong2 r15_v = spu_splats(r15);

   vec_double2 x16_v = spu_splats(x16);
   vec_double2 y16_v = spu_splats(y16);
   vec_ullong2 r16_v = spu_splats(r16);

   vec_double2 x17_v = spu_splats(x17);
   vec_double2 y17_v = spu_splats(y17);
   vec_ullong2 r17_v = spu_splats(r17);

   vec_double2 x18_v = spu_splats(x18);
   vec_double2 y18_v = spu_splats(y18);
   vec_ullong2 r18_v = spu_splats(r18);

   vec_double2 x19_v = spu_splats(x19);
   vec_double2 y19_v = spu_splats(y19);
   vec_ullong2 r19_v = spu_splats(r19);
     
   vec_ullong2 res_v;

   TEST_START("islessd2");

   res_v = (vec_ullong2)islessd2(x0_v, y0_v);
   TEST_CHECK("20060825000000AAN", allequal_ullong2( res_v, r0_v ), 0);
   res_v = (vec_ullong2)islessd2(x1_v, y1_v);
   TEST_CHECK("20060825000001AAN", allequal_ullong2( res_v, r1_v ), 0);
   res_v = (vec_ullong2)islessd2(x2_v, y2_v);
   TEST_CHECK("20060825000002AAN", allequal_ullong2( res_v, r2_v ), 0);
   res_v = (vec_ullong2)islessd2(x3_v, y3_v);
   TEST_CHECK("20060825000003AAN", allequal_ullong2( res_v, r3_v ), 0);
   res_v = (vec_ullong2)islessd2(x4_v, y4_v);
   TEST_CHECK("20060825000004AAN", allequal_ullong2( res_v, r4_v ), 0);
   res_v = (vec_ullong2)islessd2(x5_v, y5_v);
   TEST_CHECK("20060825000005AAN", allequal_ullong2( res_v, r5_v ), 0);
   res_v = (vec_ullong2)islessd2(x6_v, y6_v);
   TEST_CHECK("20060825000006AAN", allequal_ullong2( res_v, r6_v ), 0);
   res_v = (vec_ullong2)islessd2(x7_v, y7_v);
   TEST_CHECK("20060825000007AAN", allequal_ullong2( res_v, r7_v ), 0);
   res_v = (vec_ullong2)islessd2(x8_v, y8_v);
   TEST_CHECK("20060825000008AAN", allequal_ullong2( res_v, r8_v ), 0);
   res_v = (vec_ullong2)islessd2(x9_v, y9_v);
   TEST_CHECK("20060825000009AAN", allequal_ullong2( res_v, r9_v ), 0);
   res_v = (vec_ullong2)islessd2(x10_v, y10_v);
   TEST_CHECK("20060825000000AAN", allequal_ullong2( res_v, r10_v ), 0);
   res_v = (vec_ullong2)islessd2(x11_v, y11_v);
   TEST_CHECK("20060825000001AAN", allequal_ullong2( res_v, r11_v ), 0);
   res_v = (vec_ullong2)islessd2(x12_v, y12_v);
   TEST_CHECK("20060825000002AAN", allequal_ullong2( res_v, r12_v ), 0);
   res_v = (vec_ullong2)islessd2(x13_v, y13_v);
   TEST_CHECK("20060825000003AAN", allequal_ullong2( res_v, r13_v ), 0);
   res_v = (vec_ullong2)islessd2(x14_v, y14_v);
   TEST_CHECK("20060825000004AAN", allequal_ullong2( res_v, r14_v ), 0);
   res_v = (vec_ullong2)islessd2(x15_v, y15_v);
   TEST_CHECK("20060825000005AAN", allequal_ullong2( res_v, r15_v ), 0);
   res_v = (vec_ullong2)islessd2(x16_v, y16_v);
   TEST_CHECK("20060825000006AAN", allequal_ullong2( res_v, r16_v ), 0);
   res_v = (vec_ullong2)islessd2(x17_v, y17_v);
   TEST_CHECK("20060825000007AAN", allequal_ullong2( res_v, r17_v ), 0);
   res_v = (vec_ullong2)islessd2(x18_v, y18_v);
   TEST_CHECK("20060825000008AAN", allequal_ullong2( res_v, r18_v ), 0);
   res_v = (vec_ullong2)islessd2(x19_v, y19_v);
   TEST_CHECK("20060825000009AAN", allequal_ullong2( res_v, r19_v ), 0);
   
   TEST_SET_DONE();
   
   TEST_EXIT();
}
