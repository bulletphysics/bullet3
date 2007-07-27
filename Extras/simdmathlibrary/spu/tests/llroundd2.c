/* Test llroundd2 for SPU
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
 *@@ llroundd2 - rounds two doubles in to two nearest 64bit integer.
 *
 *@brief
 * boundary test for llroundd2. 0.5 will be rounded to far from 0.
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
#include <float.h>
#include "simdmath.h"
#include "common-test.h"
#include "testutils.h"


int main()
{
	TEST_SET_START("20060916101000NM","NM", "llroundd2");

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

	vec_double2 x0_v = ((vec_double2){x2, x3});    //+0,-0
	vec_llong2  r0_v = ((vec_llong2){0,0});        // 0, 0
	vec_double2 x1_v = ((vec_double2){x5, x8});    //+0.5,-0.4999999999999999
	vec_llong2  r1_v = ((vec_llong2){1,0});        //   1, 0
	vec_double2 x2_v = ((vec_double2){x7, x6});    //+0.4999999999999999, -0.5
	vec_llong2  r2_v = ((vec_llong2){0,-1});       //                  0, -1
	vec_double2 x3_v = ((vec_double2){x10, x11});  //-999999999999999.5, 9223372036854774784
	vec_llong2  r3_v = ((vec_llong2){-1000000000000000ll,9223372036854774784ll});
	vec_double2 x4_v = ((vec_double2){x12, x9});  //-9223372036854774784, 999999999999999.5
	vec_llong2  r4_v = ((vec_llong2){-9223372036854774784ll,1000000000000000ll});
	vec_double2 x5_v = ((vec_double2){x13, x14});
	vec_llong2  r5_v = ((vec_llong2){0,0});
	
	vec_llong2  res_v;

   TEST_START("llroundd2");
   res_v = llroundd2 (x0_v);
   TEST_CHECK("20060916101001NM", allequal_llong2( res_v, r0_v ), 0);
   res_v = llroundd2 (x1_v);
   TEST_CHECK("20060916101002NM", allequal_llong2( res_v, r1_v ), 0);
   res_v = llroundd2 (x2_v);
   TEST_CHECK("20060916101003NM", allequal_llong2( res_v, r2_v ), 0);
   res_v = llroundd2 (x3_v);
   TEST_CHECK("20060916101004NM", allequal_llong2( res_v, r3_v ), 0);
   res_v = llroundd2 (x4_v);
   TEST_CHECK("20060916101005NM", allequal_llong2( res_v, r4_v ), 0);
   res_v = llroundd2 (x5_v);
   TEST_CHECK("20060916101006NM", allequal_llong2( res_v, r5_v ), 0);

   TEST_SET_DONE();

   TEST_EXIT();

}
