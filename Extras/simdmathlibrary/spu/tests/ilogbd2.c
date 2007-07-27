/* Test ilogbd2 for SPU
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
   TEST_SET_START("20060904000000AAN","AAN", "ilogbd2");

   // -Nan
   double x0 = hide_double(-nan(""));
   long long r0 = (long long)FP_ILOGBNAN;
   
   // -Inf
   double x1 = hide_double(-HUGE_VAL);
   long long r1 = (long long)FP_ILOGB0;
   
   // -Dmax
   double x2 = hide_double(-DBL_MAX);
   long long r2 = 1023ll;

   // -Norm
   double x3 = hide_double(-824842.58421394);
   long long r3 = 19ll;
   
   // -Dmin
   double x4 = hide_double(-DBL_MIN);
   long long r4 = -1022ll;
   
   // -Denorm
   double x5 = hide_double(-2.40e-310);
   long long r5 = -1029ll;
   
   // -Unf
   double x6 = hide_double(-1.0e-999);
   long long r6 = (long long)FP_ILOGB0;
   
   // -0
   double x7 = hide_double(-0.0);
   long long r7 = (long long)FP_ILOGB0;

   // 0
   double x8 = hide_double( 0.0);
   long long r8 = (long long)FP_ILOGB0;
   
   // +Unf
   double x9 = hide_double( 1.0e-999);
   long long r9 = (long long)FP_ILOGB0;

   // +Denorm
   double x10 = hide_double( 2.40e-310);
   long long r10 = -1029ll;
   
   // +Dmin
   double x11 = hide_double( DBL_MIN);
   long long r11 = -1022ll;
   
   // +Norm
   double x12 = hide_double(3.14152634);
   long long r12 = 1ll;

   // +Norm
   double x13 = hide_double(7.0673903e149);
   long long r13 = 497ll;

   // +Norm
   double x14 = hide_double(2.4673e304);
   long long r14 = 1011ll;

   // +Norm
   double x15 = hide_double(7.235672e-25);
   long long r15 = -81ll;

   // +Denorm
   double x16 = hide_double(9.452854e-312);
   long long r16 = -1034ll;

   // +Demorm
   double x17 = hide_double(3.045784e-320);
   long long r17 = -1062ll;

   // -Norm
   double x18 = hide_double(-6.459273e7);
   long long r18 = 25ll;

   // -Norm
   double x19 = hide_double(-2.493472e-99);
   long long r19 = -328ll;

   // -Norm
   double x20 = hide_double(-1.4824543e128);
   long long r20 = 425ll;

   // -Denorm
   double x21 = hide_double(-5.53856231e-315);
   long long r21 = -1044ll;

   // -Demorm
   double x22 = hide_double(-2.5684367e-312);
   long long r22 = -1036ll;

   // +Dmax
   double x23 = hide_double(DBL_MAX);
   long long r23 = 1023ll;
   
   // +Inf
   double x24 = hide_double(HUGE_VAL);
   long long r24 = (long long)FP_ILOGB0;
   
   //+Nan
   double x25 = hide_double( nan(""));
   long long r25 = (long long)FP_ILOGBNAN;

   // Compound
   vec_double2 x26_v = (vec_double2) { -2.561286432e-317, -1.0e-999 };
   vec_llong2 r26_v = (vec_llong2) { -1052ll, FP_ILOGB0 };

   // Compound
   vec_double2 x27_v = (vec_double2) { 345.27533, -8.673e-310 };
   vec_llong2 r27_v = (vec_llong2) { 8ll, -1027ll };

   // Compound
   vec_double2 x28_v = (vec_double2) { nan(""), -3678342.8765343 };
   vec_llong2 r28_v = (vec_llong2) { FP_ILOGBNAN, 21ll };

   // Compound
   vec_double2 x29_v = (vec_double2) { HUGE_VAL, -nan("") };
   vec_llong2 r29_v = (vec_llong2) { FP_ILOGB0, FP_ILOGBNAN };

   // Compound
   vec_double2 x30_v = (vec_double2) { -1.2e-99, -HUGE_VAL } ;
   vec_llong2 r30_v = (vec_llong2) { -329ll, FP_ILOGB0 };

   vec_double2 x0_v = spu_splats(x0);
   vec_llong2 r0_v = spu_splats(r0);

   vec_double2 x1_v = spu_splats(x1);
   vec_llong2 r1_v = spu_splats(r1);

   vec_double2 x2_v = spu_splats(x2);
   vec_llong2 r2_v = spu_splats(r2);

   vec_double2 x3_v = spu_splats(x3);
   vec_llong2 r3_v = spu_splats(r3);

   vec_double2 x4_v = spu_splats(x4);
   vec_llong2 r4_v = spu_splats(r4);

   vec_double2 x5_v = spu_splats(x5);
   vec_llong2 r5_v = spu_splats(r5);

   vec_double2 x6_v = spu_splats(x6);
   vec_llong2 r6_v = spu_splats(r6);

   vec_double2 x7_v = spu_splats(x7);
   vec_llong2 r7_v = spu_splats(r7);

   vec_double2 x8_v = spu_splats(x8);
   vec_llong2 r8_v = spu_splats(r8);

   vec_double2 x9_v = spu_splats(x9);
   vec_llong2 r9_v = spu_splats(r9);

   vec_double2 x10_v = spu_splats(x10);
   vec_llong2 r10_v = spu_splats(r10);

   vec_double2 x11_v = spu_splats(x11);
   vec_llong2 r11_v = spu_splats(r11);

   vec_double2 x12_v = spu_splats(x12);
   vec_llong2 r12_v = spu_splats(r12);

   vec_double2 x13_v = spu_splats(x13);
   vec_llong2 r13_v = spu_splats(r13);

   vec_double2 x14_v = spu_splats(x14);
   vec_llong2 r14_v = spu_splats(r14);

   vec_double2 x15_v = spu_splats(x15);
   vec_llong2 r15_v = spu_splats(r15);

   vec_double2 x16_v = spu_splats(x16);
   vec_llong2 r16_v = spu_splats(r16);

   vec_double2 x17_v = spu_splats(x17);
   vec_llong2 r17_v = spu_splats(r17);

   vec_double2 x18_v = spu_splats(x18);
   vec_llong2 r18_v = spu_splats(r18);

   vec_double2 x19_v = spu_splats(x19);
   vec_llong2 r19_v = spu_splats(r19);

   vec_double2 x20_v = spu_splats(x20);
   vec_llong2 r20_v = spu_splats(r20);

   vec_double2 x21_v = spu_splats(x21);
   vec_llong2 r21_v = spu_splats(r21);

   vec_double2 x22_v = spu_splats(x22);
   vec_llong2 r22_v = spu_splats(r22);

   vec_double2 x23_v = spu_splats(x23);
   vec_llong2 r23_v = spu_splats(r23);

   vec_double2 x24_v = spu_splats(x24);
   vec_llong2 r24_v = spu_splats(r24);

   vec_double2 x25_v = spu_splats(x25);
   vec_llong2 r25_v = spu_splats(r25);
   
   vec_llong2 res_v;

   TEST_START("ilogbd2");

   res_v = (vec_llong2)ilogbd2(x0_v);
   TEST_CHECK("20060904000000AAN", allequal_llong2( res_v, r0_v ), 0);
   res_v = (vec_llong2)ilogbd2(x1_v);
   TEST_CHECK("20060904000001AAN", allequal_llong2( res_v, r1_v ), 0);
   res_v = (vec_llong2)ilogbd2(x2_v);
   TEST_CHECK("20060904000002AAN", allequal_llong2( res_v, r2_v ), 0);
   res_v = (vec_llong2)ilogbd2(x3_v);
   TEST_CHECK("20060904000003AAN", allequal_llong2( res_v, r3_v ), 0);
   res_v = (vec_llong2)ilogbd2(x4_v);
   TEST_CHECK("20060904000004AAN", allequal_llong2( res_v, r4_v ), 0);
   res_v = (vec_llong2)ilogbd2(x5_v);
   TEST_CHECK("20060904000005AAN", allequal_llong2( res_v, r5_v ), 0);
   res_v = (vec_llong2)ilogbd2(x6_v);
   TEST_CHECK("20060904000006AAN", allequal_llong2( res_v, r6_v ), 0);
   res_v = (vec_llong2)ilogbd2(x7_v);
   TEST_CHECK("20060904000007AAN", allequal_llong2( res_v, r7_v ), 0);
   res_v = (vec_llong2)ilogbd2(x8_v);
   TEST_CHECK("20060904000008AAN", allequal_llong2( res_v, r8_v ), 0);
   res_v = (vec_llong2)ilogbd2(x9_v);
   TEST_CHECK("20060904000009AAN", allequal_llong2( res_v, r9_v ), 0);
   res_v = (vec_llong2)ilogbd2(x10_v);
   TEST_CHECK("20060904000010AAN", allequal_llong2( res_v, r10_v ), 0);
   res_v = (vec_llong2)ilogbd2(x11_v);
   TEST_CHECK("20060904000011AAN", allequal_llong2( res_v, r11_v ), 0);
   res_v = (vec_llong2)ilogbd2(x12_v);
   TEST_CHECK("20060904000012AAN", allequal_llong2( res_v, r12_v ), 0);
   res_v = (vec_llong2)ilogbd2(x13_v);
   TEST_CHECK("20060904000013AAN", allequal_llong2( res_v, r13_v ), 0);
   res_v = (vec_llong2)ilogbd2(x14_v);
   TEST_CHECK("20060904000014AAN", allequal_llong2( res_v, r14_v ), 0);
   res_v = (vec_llong2)ilogbd2(x15_v);
   TEST_CHECK("20060904000015AAN", allequal_llong2( res_v, r15_v ), 0);
   res_v = (vec_llong2)ilogbd2(x16_v);
   TEST_CHECK("20060904000016AAN", allequal_llong2( res_v, r16_v ), 0);
   res_v = (vec_llong2)ilogbd2(x17_v);
   TEST_CHECK("20060904000017AAN", allequal_llong2( res_v, r17_v ), 0);
   res_v = (vec_llong2)ilogbd2(x18_v);
   TEST_CHECK("20060904000018AAN", allequal_llong2( res_v, r18_v ), 0);
   res_v = (vec_llong2)ilogbd2(x19_v);
   TEST_CHECK("20060904000019AAN", allequal_llong2( res_v, r19_v ), 0);
   res_v = (vec_llong2)ilogbd2(x20_v);
   TEST_CHECK("20060904000020AAN", allequal_llong2( res_v, r20_v ), 0);
   res_v = (vec_llong2)ilogbd2(x21_v);
   TEST_CHECK("20060904000021AAN", allequal_llong2( res_v, r21_v ), 0);
   res_v = (vec_llong2)ilogbd2(x22_v);
   TEST_CHECK("20060904000022AAN", allequal_llong2( res_v, r22_v ), 0);
   res_v = (vec_llong2)ilogbd2(x23_v);
   TEST_CHECK("20060904000023AAN", allequal_llong2( res_v, r23_v ), 0);
   res_v = (vec_llong2)ilogbd2(x24_v);
   TEST_CHECK("20060904000024AAN", allequal_llong2( res_v, r24_v ), 0);
   res_v = (vec_llong2)ilogbd2(x25_v);
   TEST_CHECK("20060904000025AAN", allequal_llong2( res_v, r25_v ), 0);
   res_v = (vec_llong2)ilogbd2(x26_v);
   TEST_CHECK("20060904000026AAN", allequal_llong2( res_v, r26_v ), 0);
   res_v = (vec_llong2)ilogbd2(x27_v);
   TEST_CHECK("20060904000027AAN", allequal_llong2( res_v, r27_v ), 0);
   res_v = (vec_llong2)ilogbd2(x28_v);
   TEST_CHECK("20060904000028AAN", allequal_llong2( res_v, r28_v ), 0);
   res_v = (vec_llong2)ilogbd2(x29_v);
   TEST_CHECK("20060904000029AAN", allequal_llong2( res_v, r29_v ), 0);
   res_v = (vec_llong2)ilogbd2(x30_v);
   TEST_CHECK("20060904000030AAN", allequal_llong2( res_v, r30_v ), 0);
   
   TEST_SET_DONE();
   
   TEST_EXIT();
}
