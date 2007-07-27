/* Test isnand2 for SPU
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
   TEST_SET_START("20060830000000AAN","AAN", "isnand2");

   // -QNan
   double x0 = hide_double(-nan(""));
   unsigned long long r0 = 0xffffffffffffffffull;
   
   // -Inf
   double x1 = hide_double(-HUGE_VAL);
   unsigned long long r1 = 0x0000000000000000ull;
   
   // -Dmax
   double x2 = hide_double(-DBL_MAX);
   unsigned long long r2 = 0x0000000000000000ull;

   // -QNaN
   double x3 = hide_double(make_double(0xFFFFFFFFFFFFFFFFull));
   unsigned long long r3 = 0xffffffffffffffffull;
   
   // -SNaN
   double x4 = hide_double(make_double(0xFFF7FFFFFFFFFFFFull));
   unsigned long long r4 = 0xffffffffffffffffull;
   
   // -Denorm
   double x5 = hide_double(-2.40e-310);
   unsigned long long r5 = 0x0000000000000000ull;
   
   // -Unf
   double x6 = hide_double(-1.0e-999);
   unsigned long long r6 = 0x0000000000000000ull;
   
   // -0
   double x7 = hide_double(-0.0);
   unsigned long long r7 = 0x0000000000000000ull;

   // 0
   double x8 = hide_double( 0.0);
   unsigned long long r8 = 0x0000000000000000ull;
   
   // +Inf
   double x9 = hide_double( 1.0e999);
   unsigned long long r9 = 0x0000000000000000ull;

   // +QNaN
   double x10 = hide_double(make_double(0x7FFFFFFFFFFFFFFFull));
   unsigned long long r10 = 0xffffffffffffffffull;
   
   // +Dmin
   double x11 = hide_double( DBL_MIN);
   unsigned long long r11 = 0x0000000000000000ull;
   
   // +Norm
   double x12 = hide_double(3.14152634);
   unsigned long long r12 = 0x0000000000000000ull;
   
   // +SNaN
   double x13 = hide_double(make_double(0x7FF3333333333333ull));
   unsigned long long r13 = 0xffffffffffffffffull;
   
   // +Inf
   double x14 = hide_double(HUGE_VAL);
   unsigned long long r14 = 0x0000000000000000ull;
   
   //+Nan
   double x15 = hide_double( nan(""));
   unsigned long long r15 = 0xffffffffffffffffull;

   // Compound
   vec_double2 x16_v = (vec_double2) {make_double(0xFFF7000000000000ull), -1.0e-999 };
   vec_ullong2 r16_v = (vec_ullong2) {0xffffffffffffffffull, 0x0000000000000000ull};

   // Compound
   vec_double2 x17_v = (vec_double2) { 345.27533, -2.40e-310 };
   vec_ullong2 r17_v = (vec_ullong2) {0x0000000000000000ull, 0x0000000000000000ull};

   // Compound
   vec_double2 x18_v = (vec_double2) { nan(""), -3678342.8765343 };
   vec_ullong2 r18_v = (vec_ullong2) {0xffffffffffffffffull, 0x0000000000000000ull};

   // Compound
   vec_double2 x19_v = (vec_double2) { HUGE_VAL, -nan("") };
   vec_ullong2 r19_v = (vec_ullong2) {0x0000000000000000ull, 0xffffffffffffffffull};

   // Compound
   vec_double2 x20_v = (vec_double2) { make_double(0x7FF8000000000000ull), -HUGE_VAL} ;
   vec_ullong2 r20_v = (vec_ullong2) {0xffffffffffffffffull, 0x0000000000000000ull};

   vec_double2 x0_v = spu_splats(x0);
   vec_ullong2  r0_v = spu_splats(r0);

   vec_double2 x1_v = spu_splats(x1);
   vec_ullong2  r1_v = spu_splats(r1);

   vec_double2 x2_v = spu_splats(x2);
   vec_ullong2  r2_v = spu_splats(r2);

   vec_double2 x3_v = spu_splats(x3);
   vec_ullong2  r3_v = spu_splats(r3);

   vec_double2 x4_v = spu_splats(x4);
   vec_ullong2  r4_v = spu_splats(r4);

   vec_double2 x5_v = spu_splats(x5);
   vec_ullong2  r5_v = spu_splats(r5);

   vec_double2 x6_v = spu_splats(x6);
   vec_ullong2  r6_v = spu_splats(r6);

   vec_double2 x7_v = spu_splats(x7);
   vec_ullong2  r7_v = spu_splats(r7);

   vec_double2 x8_v = spu_splats(x8);
   vec_ullong2  r8_v = spu_splats(r8);

   vec_double2 x9_v = spu_splats(x9);
   vec_ullong2  r9_v = spu_splats(r9);

   vec_double2 x10_v = spu_splats(x10);
   vec_ullong2  r10_v = spu_splats(r10);

   vec_double2 x11_v = spu_splats(x11);
   vec_ullong2  r11_v = spu_splats(r11);

   vec_double2 x12_v = spu_splats(x12);
   vec_ullong2  r12_v = spu_splats(r12);

   vec_double2 x13_v = spu_splats(x13);
   vec_ullong2  r13_v = spu_splats(r13);

   vec_double2 x14_v = spu_splats(x14);
   vec_ullong2  r14_v = spu_splats(r14);

   vec_double2 x15_v = spu_splats(x15);
   vec_ullong2  r15_v = spu_splats(r15);
   
   vec_ullong2 res_v;

   TEST_START("isnand2");

   res_v = (vec_ullong2)isnand2(x0_v);
   TEST_CHECK("20060830000000AAN", allequal_ullong2( res_v, r0_v ), 0);
   res_v = (vec_ullong2)isnand2(x1_v);
   TEST_CHECK("20060830000001AAN", allequal_ullong2( res_v, r1_v ), 0);
   res_v = (vec_ullong2)isnand2(x2_v);
   TEST_CHECK("20060830000002AAN", allequal_ullong2( res_v, r2_v ), 0);
   res_v = (vec_ullong2)isnand2(x3_v);
   TEST_CHECK("20060830000003AAN", allequal_ullong2( res_v, r3_v ), 0);
   res_v = (vec_ullong2)isnand2(x4_v);
   TEST_CHECK("20060830000004AAN", allequal_ullong2( res_v, r4_v ), 0);
   res_v = (vec_ullong2)isnand2(x5_v);
   TEST_CHECK("20060830000005AAN", allequal_ullong2( res_v, r5_v ), 0);
   res_v = (vec_ullong2)isnand2(x6_v);
   TEST_CHECK("20060830000006AAN", allequal_ullong2( res_v, r6_v ), 0);
   res_v = (vec_ullong2)isnand2(x7_v);
   TEST_CHECK("20060830000007AAN", allequal_ullong2( res_v, r7_v ), 0);
   res_v = (vec_ullong2)isnand2(x8_v);
   TEST_CHECK("20060830000008AAN", allequal_ullong2( res_v, r8_v ), 0);
   res_v = (vec_ullong2)isnand2(x9_v);
   TEST_CHECK("20060830000009AAN", allequal_ullong2( res_v, r9_v ), 0);
   res_v = (vec_ullong2)isnand2(x10_v);
   TEST_CHECK("20060830000010AAN", allequal_ullong2( res_v, r10_v ), 0);
   res_v = (vec_ullong2)isnand2(x11_v);
   TEST_CHECK("20060830000011AAN", allequal_ullong2( res_v, r11_v ), 0);
   res_v = (vec_ullong2)isnand2(x12_v);
   TEST_CHECK("20060830000012AAN", allequal_ullong2( res_v, r12_v ), 0);
   res_v = (vec_ullong2)isnand2(x13_v);
   TEST_CHECK("20060830000013AAN", allequal_ullong2( res_v, r13_v ), 0);
   res_v = (vec_ullong2)isnand2(x14_v);
   TEST_CHECK("20060830000014AAN", allequal_ullong2( res_v, r14_v ), 0);
   res_v = (vec_ullong2)isnand2(x15_v);
   TEST_CHECK("20060830000015AAN", allequal_ullong2( res_v, r15_v ), 0);
   res_v = (vec_ullong2)isnand2(x16_v);
   TEST_CHECK("20060830000016AAN", allequal_ullong2( res_v, r16_v ), 0);
   res_v = (vec_ullong2)isnand2(x17_v);
   TEST_CHECK("20060830000017AAN", allequal_ullong2( res_v, r17_v ), 0);
   res_v = (vec_ullong2)isnand2(x18_v);
   TEST_CHECK("20060830000018AAN", allequal_ullong2( res_v, r18_v ), 0);
   res_v = (vec_ullong2)isnand2(x19_v);
   TEST_CHECK("20060830000019AAN", allequal_ullong2( res_v, r19_v ), 0);
   res_v = (vec_ullong2)isnand2(x20_v);
   TEST_CHECK("20060830000020AAN", allequal_ullong2( res_v, r20_v ), 0);
   
   TEST_SET_DONE();
   
   TEST_EXIT();
}
