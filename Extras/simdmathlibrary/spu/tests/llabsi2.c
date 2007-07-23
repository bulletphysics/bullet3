/* Test llabsi2 for SPU
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
 *@@ llabsi2 - returns absolute value of input.
 *
 *@brief
 * boundary test for llabsi2.
 *
 *@pre
 *
 *@criteria
 * Run this program and check no error will be occurred.
 *
 *@note
 * 
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
	TEST_SET_START("20060831134500NM","NM", "llabsi2");

	vec_llong2 x0_v = ((vec_llong2){ 0, 0});
	vec_llong2 r0_v = ((vec_llong2){ 0, 0});

	vec_llong2 x1_v = ((vec_llong2){-1, 1});
	vec_llong2 r1_v = ((vec_llong2){ 1, 1});

	vec_llong2 x2_v = ((vec_llong2){ 1,-1});
	vec_llong2 r2_v = ((vec_llong2){ 1, 1});
	// 0x7FFFFFFFFFFFFFFF
	vec_llong2 x3_v = ((vec_llong2){ 9223372036854775807LL,-9223372036854775807LL});
	vec_llong2 r3_v = ((vec_llong2){ 9223372036854775807LL, 9223372036854775807LL});
	// 0x8000000000000000
	vec_llong2 x4_v = ((vec_llong2){0x8000000000000000LL,0x8000000000000000LL});
	vec_llong2 r4_v = ((vec_llong2){0x8000000000000000LL,0x8000000000000000LL});

	vec_llong2  res_v;

   TEST_START("llabsi2");
   res_v = llabsi2 (x0_v);
   TEST_CHECK("20060831134501NM", allequal_llong2( res_v, r0_v ), 0);
   res_v = llabsi2 (x1_v);
   TEST_CHECK("20060831134502NM", allequal_llong2( res_v, r1_v ), 0);
   res_v = llabsi2 (x2_v);
   TEST_CHECK("20060831134503NM", allequal_llong2( res_v, r2_v ), 0);
   res_v = llabsi2 (x3_v);
   TEST_CHECK("20060831134504NM", allequal_llong2( res_v, r3_v ), 0);
   res_v = llabsi2 (x4_v);
   TEST_CHECK("20060831134505NM", allequal_llong2( res_v, r4_v ), 0);

   TEST_SET_DONE();

   TEST_EXIT();

}
