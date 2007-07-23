/* Test roundd2 for SPU
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
 *@@ ceild2for each of two doule slots, 
 * round up to smallest integer not less than the value.
 *
 *@brief
 * boundary test for ceild2.
 * 
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

typedef union {
	struct {
		double xxx[2];
		double ans[2];
	} dbl;
	struct {
		unsigned long long xxx[2];
		unsigned long long ans[2];
	} ull;
} TestVec_Roundd2;

int main()
{
	TestVec_Roundd2 test_a[] = {
		{
			ull:{
			// 0 -> 0 , -0 -> -0
			{0x0000000000000000ULL,0x8000000000000000ULL},
			{0x0000000000000000ULL,0x8000000000000000ULL}
			}
		},{
			ull:{
			// -Inf -> -Inf , Inf -> Inf
			{0xFFF0000000000000ULL,0x7FF0000000000000ULL},
			{0xFFF0000000000000ULL,0x7FF0000000000000ULL}
			}
		},{
			ull:{
			// MAX -> MAX , MIN -> MIN
			{0x7FEFFFFFFFFFFFFFULL,0xFFEFFFFFFFFFFFFFULL},
			{0x7FEFFFFFFFFFFFFFULL,0xFFEFFFFFFFFFFFFFULL}
			}
		},{
			ull:{
			// Denormalize -> 0
			{0x0000000000000001ULL,0x8000000000000010ULL},
			{0x3ff0000000000000ULL,0x8000000000000000ULL}
			}
		},{
			ull:{
			// Denormalize -> 0
			{0x800FFFFFFFFFFFFFULL,0x000FFFFFFFFFFFFFULL},
			{0x8000000000000000ULL,0x3ff0000000000000ULL}
			}
		},{
			ull:{
			// border
			{0x4320000000000001ULL,0xC320000000000001ULL},
			{0x4320000000000002ULL,0xC320000000000000ULL}
			}
		},{
			dbl:{
			{1.0, -1.0},
			{1.0, -1.0}
			}
		},{
			dbl:{
			{0.5, -0.5},
			{1.0, -0.0}
			}
		},{
			dbl:{
			{-2.75, 3.2},
			{-2.0, 4.0}
			}
		},{
			ull:{
			// Nan
			{0xFFF0000000000001ULL,0x7FF0000000000001ULL},
			{0xFFF0000000000001ULL,0x7FF0000000000001ULL}
			}
		},{
			ull:{
			{0ULL,0ULL},
			{0ULL,0ULL}
			}
		}
	};
	int ii, test_ctr = 1;
	char msg[80];
	vec_double2 res_v;
	//vec_double2  input;

	TEST_SET_START("51651906100000NM","CED", "ceild2");

   TEST_START("ceild2");

	for (ii=0; ; ii++) {
		if ( (test_a[ii].ull.xxx[0] == 0) && (test_a[ii].ull.xxx[1] == 0) ) break;
		//input = *((vec_double2 *)&test_a[ii].dbl.xxx[0]);
		res_v = ceild2 (*((vec_double2 *)&test_a[ii].dbl.xxx[0]) );
		sprintf(msg,"5165190610%04dCED", test_ctr++);
		TEST_CHECK(msg, allequal_llong2( (vec_llong2)res_v, (vec_llong2)*((vec_double2 *)&test_a[ii].dbl.ans[0])), 0);
	}

   TEST_SET_DONE();

   TEST_EXIT();

}
