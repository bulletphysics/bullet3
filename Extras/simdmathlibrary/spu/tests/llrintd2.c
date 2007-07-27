/* Test llrintd2 for SPU
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
/**
 *
 *@@ llrintd2 - rounds two doubles in to two nearest 64bit integer.
 *
 *@brief
 * boundary test for llrintd2. 
 * consistent with the current rounding mode.
 *
 *@pre
 *
 *@criteria
 * Run this program and check no error will be occurred.
 *
 *@note
 *
 **/


#include <stdio.h>
#include <stdlib.h>
#include <math.h>
//#include <fenv.h>
#include <float.h>

#include "simdmath.h"
#include "common-test.h"
#include "testutils.h"

int main()
{
	typedef union {
		unsigned long long int ll;
		double x;
	} SrcType64;
	SrcType64 tmp64;
	TEST_SET_START("20060921101000NM","NM", "llrintd2");

//   unsigned long long i11 = 0x7FFFFFFFFFFFFDFFull; //limit
//   unsigned long long i12 = 0xFFFFFFFFFFFFFDFFull; //limit

//   double x0  = hide_double(-HUGE_VAL);           // -Inf
//   double x1  = hide_double(HUGE_VAL);            // Inf
   double x2  = hide_double(0.0);                  // +0
   double x3  = hide_double(-0.0);                 // -0
//   double x4  = hide_double(nan(""));            // NaN -> NaN
   double x5  = hide_double( 0.5);
   double x6  = hide_double(-0.5);
   double x7  = hide_double( 0.4999999999999999);  // 0
   double x8  = hide_double(-0.4999999999999999);  // 0
   double x9  = hide_double( 999999999999999.5);   // 1000000000000000
   double x10 = hide_double(-999999999999999.5);   //-1000000000000000
//   double x11 = hide_double(make_double(i11));     // 9223372036854774784
//   double x12 = hide_double(make_double(i12));     //-9223372036854774784
   double x11 = hide_double(9223372036854774784ll);     // 9223372036854774784
   double x12 = hide_double(-9223372036854774784ll);     //-9223372036854774784
   double x13 = DBL_MIN;
   double x14 = (0.0 - DBL_MIN);

   double x15 = hide_double( 0.5000000000000001);
   double x16 = hide_double(-0.000001);

   tmp64.ll = 0x3FEFFFFFFFFFFFFFULL;
   double x17 = tmp64.x;
   tmp64.ll = 0xC32FFFFFFFFFFFFFULL;
   double x18 = tmp64.x;

	vec_double2 x0_v = ((vec_double2){x2, x3});    //+0,-0
	vec_llong2  r0_v = ((vec_llong2){0,0});        // 0, 0
	vec_double2 x1_v = ((vec_double2){x5, x8});    //+0.5,-0.4999999999999999
	vec_llong2  r10_v = ((vec_llong2){0,0});
	vec_llong2  r11_v = ((vec_llong2){0,0});
	vec_llong2  r12_v = ((vec_llong2){1,0});
	vec_llong2  r13_v = ((vec_llong2){0,-1});
	vec_double2 x2_v = ((vec_double2){x7, x6});    //+0.4999999999999999, -0.5
	vec_llong2  r20_v = ((vec_llong2){0,0});
	vec_llong2  r21_v = ((vec_llong2){0,0});
	vec_llong2  r22_v = ((vec_llong2){1,0});
	vec_llong2  r23_v = ((vec_llong2){0,-1});
	vec_double2 x3_v = ((vec_double2){x10, x11});  //-999999999999999.5, 9223372036854774784
	vec_llong2  r30_v = ((vec_llong2){-1000000000000000ll,9223372036854774784ll});
	vec_llong2  r31_v = ((vec_llong2){-999999999999999ll, 9223372036854774784ll});
	vec_llong2  r32_v = ((vec_llong2){-999999999999999ll, 9223372036854774784ll});
	vec_llong2  r33_v = ((vec_llong2){-1000000000000000ll,9223372036854774784ll});
	vec_double2 x4_v = ((vec_double2){x12, x9});  //-9223372036854774784, 999999999999999.5
	vec_llong2  r40_v = ((vec_llong2){-9223372036854774784ll,1000000000000000ll});
	vec_llong2  r41_v = ((vec_llong2){-9223372036854774784ll,999999999999999ll});
	vec_llong2  r42_v = ((vec_llong2){-9223372036854774784ll,1000000000000000ll});
	vec_llong2  r43_v = ((vec_llong2){-9223372036854774784ll,999999999999999ll});
	vec_double2 x5_v = ((vec_double2){x13, x14});
	vec_llong2  r50_v = ((vec_llong2){0,0});
	vec_llong2  r51_v = ((vec_llong2){0,0});
	vec_llong2  r52_v = ((vec_llong2){1,0});
	vec_llong2  r53_v = ((vec_llong2){0,-1});
	vec_double2 x6_v = ((vec_double2){x15, x16});
	vec_llong2  r60_v = ((vec_llong2){1,0});
	vec_llong2  r61_v = ((vec_llong2){0,0});
	vec_llong2  r62_v = ((vec_llong2){1,0});
	vec_llong2  r63_v = ((vec_llong2){0,-1});

	vec_double2 x7_v = ((vec_double2){x17, x18});
	vec_llong2  r70_v = ((vec_llong2){1,-4503599627370496LL});
	vec_llong2  r71_v = ((vec_llong2){0,-4503599627370495LL});
	vec_llong2  r72_v = ((vec_llong2){1,-4503599627370495LL});
	vec_llong2  r73_v = ((vec_llong2){0,-4503599627370496LL});
	
	vec_llong2  res_v;

   TEST_START("llrintd2");

   spu_mtfpscr(((vec_uint4){0x0000,0,0,0}));     //change FP mode
   res_v = llrintd2 (x0_v);
   TEST_CHECK("20060921101001NM", allequal_llong2( res_v, r0_v ), 0);
   res_v = llrintd2 (x1_v);
   TEST_CHECK("20060921101002NM", allequal_llong2( res_v, r10_v ), 0);
   res_v = llrintd2 (x2_v);
   TEST_CHECK("20060921101003NM", allequal_llong2( res_v, r20_v ), 0);
   res_v = llrintd2 (x3_v);
   TEST_CHECK("20060921101004NM", allequal_llong2( res_v, r30_v ), 0);
   res_v = llrintd2 (x4_v);
   TEST_CHECK("20060921101005NM", allequal_llong2( res_v, r40_v ), 0);
   res_v = llrintd2 (x5_v);
   TEST_CHECK("20060921101006NM", allequal_llong2( res_v, r50_v ), 0);
   res_v = llrintd2 (x6_v);
   TEST_CHECK("20060921101007NM", allequal_llong2( res_v, r60_v ), 0);
   res_v = llrintd2 (x7_v);
   TEST_CHECK("20060921101017NM", allequal_llong2( res_v, r70_v ), 0);

   spu_mtfpscr(((vec_uint4){0x0500,0,0,0}));     //change FP mode
   res_v = llrintd2 (x0_v);
   TEST_CHECK("20060921101008NM", allequal_llong2( res_v, r0_v ), 0);
   res_v = llrintd2 (x1_v);
   TEST_CHECK("20060921101009NM", allequal_llong2( res_v, r11_v ), 0);
   res_v = llrintd2 (x2_v);
   TEST_CHECK("20060921101010NM", allequal_llong2( res_v, r21_v ), 0);
   res_v = llrintd2 (x3_v);
   TEST_CHECK("20060921101011NM", allequal_llong2( res_v, r31_v ), 0);
   res_v = llrintd2 (x4_v);
   TEST_CHECK("20060921101012NM", allequal_llong2( res_v, r41_v ), 0);
   res_v = llrintd2 (x5_v);
   TEST_CHECK("20060921101013NM", allequal_llong2( res_v, r51_v ), 0);
   res_v = llrintd2 (x6_v);
   TEST_CHECK("20060921101014NM", allequal_llong2( res_v, r61_v ), 0);
   res_v = llrintd2 (x7_v);
   TEST_CHECK("20060921101027NM", allequal_llong2( res_v, r71_v ), 0);

   spu_mtfpscr(((vec_uint4){0x0a00,0,0,0}));     //change FP mode
   res_v = llrintd2 (x0_v);
   TEST_CHECK("20060921101015NM", allequal_llong2( res_v, r0_v ), 0);
   res_v = llrintd2 (x1_v);
   TEST_CHECK("20060921101016NM", allequal_llong2( res_v, r12_v ), 0);
   res_v = llrintd2 (x2_v);
   TEST_CHECK("20060921101017NM", allequal_llong2( res_v, r22_v ), 0);
   res_v = llrintd2 (x3_v);
   TEST_CHECK("20060921101018NM", allequal_llong2( res_v, r32_v ), 0);
   res_v = llrintd2 (x4_v);
   TEST_CHECK("20060921101019NM", allequal_llong2( res_v, r42_v ), 0);
   res_v = llrintd2 (x5_v);
   TEST_CHECK("20060921101020NM", allequal_llong2( res_v, r52_v ), 0);
   res_v = llrintd2 (x6_v);
   TEST_CHECK("20060921101021NM", allequal_llong2( res_v, r62_v ), 0);
   res_v = llrintd2 (x7_v);
   TEST_CHECK("20060921101037NM", allequal_llong2( res_v, r72_v ), 0);

   spu_mtfpscr(((vec_uint4){0x0f00,0,0,0}));     //change FP mode
   res_v = llrintd2 (x0_v);
   TEST_CHECK("20060921101022NM", allequal_llong2( res_v, r0_v ), 0);
   res_v = llrintd2 (x1_v);
   TEST_CHECK("20060921101023NM", allequal_llong2( res_v, r13_v ), 0);
   res_v = llrintd2 (x2_v);
   TEST_CHECK("20060921101024NM", allequal_llong2( res_v, r23_v ), 0);
   res_v = llrintd2 (x3_v);
   TEST_CHECK("20060921101025NM", allequal_llong2( res_v, r33_v ), 0);
   res_v = llrintd2 (x4_v);
   TEST_CHECK("20060921101026NM", allequal_llong2( res_v, r43_v ), 0);
   res_v = llrintd2 (x5_v);
   TEST_CHECK("20060921101027NM", allequal_llong2( res_v, r53_v ), 0);
   res_v = llrintd2 (x6_v);
   TEST_CHECK("20060921101028NM", allequal_llong2( res_v, r63_v ), 0);
   res_v = llrintd2 (x7_v);
   TEST_CHECK("20060921101047NM", allequal_llong2( res_v, r73_v ), 0);


   TEST_SET_DONE();

   TEST_EXIT();

}
