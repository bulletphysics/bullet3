/* Test nearbyintd2 for SPU
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
 *@@ nearbyintd2 - Round the input to the nearest integer according to
 *    the current rounding mode without raising an inexact exception.
 *
 *@brief
 * boundary test for nearbyintd2. 
 * 
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

typedef struct {
	unsigned long long int  xxx[2];
	unsigned long long int  ans0[2];
	unsigned long long int  ans1[2];
	unsigned long long int  ans2[2];
	unsigned long long int  ans3[2];
} TestVec64_NerI;

int main()
{
	TestVec64_NerI test_a[] = {
		{
			// zero
			{0x0000000000000000ULL,0x8000000000000000ULL},
			{0x0000000000000000ULL,0x8000000000000000ULL},
			{0x0000000000000000ULL,0x8000000000000000ULL},
			{0x0000000000000000ULL,0x8000000000000000ULL},
			{0x0000000000000000ULL,0x8000000000000000ULL}
		},{
			// border
			{0xc330000000000000ULL,0x4330000000000000ULL},
			{0xc330000000000000ULL,0x4330000000000000ULL},
			{0xc330000000000000ULL,0x4330000000000000ULL},
			{0xc330000000000000ULL,0x4330000000000000ULL},
			{0xc330000000000000ULL,0x4330000000000000ULL}
		},{
			// MIN , MAX
			{0xFFEFFFFFFFFFFFFFULL,0x7FEFFFFFFFFFFFFFULL},
			{0xFFEFFFFFFFFFFFFFULL,0x7FEFFFFFFFFFFFFFULL},
			{0xFFEFFFFFFFFFFFFFULL,0x7FEFFFFFFFFFFFFFULL},
			{0xFFEFFFFFFFFFFFFFULL,0x7FEFFFFFFFFFFFFFULL},
			{0xFFEFFFFFFFFFFFFFULL,0x7FEFFFFFFFFFFFFFULL}
		},{
			// Inf , -Inf
			{0x7FF0000000000000ULL,0xFFF0000000000000ULL},
			{0x7FF0000000000000ULL,0xFFF0000000000000ULL},
			{0x7FF0000000000000ULL,0xFFF0000000000000ULL},
			{0x7FF0000000000000ULL,0xFFF0000000000000ULL},
			{0x7FF0000000000000ULL,0xFFF0000000000000ULL}
		},{
			// denotmalized 
			{0x8000000000000001ULL,0x0000000000000001ULL},
			{0x8000000000000000ULL,0x0000000000000000ULL},
			{0x8000000000000000ULL,0x0000000000000000ULL},
			{0x8000000000000000ULL,0x3ff0000000000000ULL},
			{0xbff0000000000000ULL,0x0000000000000000ULL}
		},{
			// denotmalized 
			{0x0008000000000000ULL,0x8008000000000000ULL},
			{0x0000000000000000ULL,0x8000000000000000ULL},
			{0x0000000000000000ULL,0x8000000000000000ULL},
			{0x3ff0000000000000ULL,0x8000000000000000ULL},
			{0x0000000000000000ULL,0xbff0000000000000ULL}
		},{
			// 1.0
			{0x3ff0000000000000ULL,0xbff0000000000000ULL},
			{0x3ff0000000000000ULL,0xbff0000000000000ULL},
			{0x3ff0000000000000ULL,0xbff0000000000000ULL},
			{0x3ff0000000000000ULL,0xbff0000000000000ULL},
			{0x3ff0000000000000ULL,0xbff0000000000000ULL}
		},{
			// 1.5
			{0x3ff8000000000000ULL,0xbff8000000000000ULL},
			{0x4000000000000000ULL,0xc000000000000000ULL},
			{0x3ff0000000000000ULL,0xbff0000000000000ULL},
			{0x4000000000000000ULL,0xbff0000000000000ULL},
			{0x3ff0000000000000ULL,0xc000000000000000ULL}
		},{
			// 2.5
			{0x4004000000000000ULL,0xc004000000000000ULL},
			{0x4000000000000000ULL,0xc000000000000000ULL},
			{0x4000000000000000ULL,0xc000000000000000ULL},
			{0x4008000000000000ULL,0xc000000000000000ULL},
			{0x4000000000000000ULL,0xc008000000000000ULL}
		},{
			{0ULL,0ULL},
			{0ULL,0ULL},
			{0ULL,0ULL},
			{0ULL,0ULL},
			{0ULL,0ULL}
			
		}
	};
	int ii, test_ctr = 1;
	char msg[80];
	vec_double2 res_v;

	TEST_SET_START("20060829200000NM","NM", "nearbyintd2");

   TEST_START("nearbyintd2");

	for (ii=0; ; ii++) {
		if ( (test_a[ii].xxx[0] == 0) && (test_a[ii].xxx[1] == 0) ) break;

		// set Floating point round mode
		spu_mtfpscr(((vec_uint4){0x0000,0,0,0}));
		res_v = nearbyintd2 (*((vec_double2 *)&test_a[ii].xxx[0]));
		sprintf(msg,"2006082920%04dNM", test_ctr++);
		TEST_CHECK(msg, allequal_llong2( (vec_llong2)res_v, *((vec_llong2 *)&test_a[ii].ans0[0])), 0);

		spu_mtfpscr(((vec_uint4){0x0500,0,0,0}));
		res_v = nearbyintd2 (*((vec_double2 *)&test_a[ii].xxx[0]));
		sprintf(msg,"2006082920%04dNM", test_ctr++);
		TEST_CHECK(msg, allequal_llong2( (vec_llong2)res_v, *((vec_llong2 *)&test_a[ii].ans1[0])), 0);

		spu_mtfpscr(((vec_uint4){0x0a00,0,0,0}));
		res_v = nearbyintd2 (*((vec_double2 *)&test_a[ii].xxx[0]));
		sprintf(msg,"2006082920%04dNM", test_ctr++);
		TEST_CHECK(msg, allequal_llong2( (vec_llong2)res_v, *((vec_llong2 *)&test_a[ii].ans2[0])), 0);

		spu_mtfpscr(((vec_uint4){0x0f00,0,0,0}));
		res_v = nearbyintd2 (*((vec_double2 *)&test_a[ii].xxx[0]));
		sprintf(msg,"2006082920%04dNM", test_ctr++);
		TEST_CHECK(msg, allequal_llong2( (vec_llong2)res_v, *((vec_llong2 *)&test_a[ii].ans3[0])), 0);
	}

   TEST_SET_DONE();

   TEST_EXIT();

}
