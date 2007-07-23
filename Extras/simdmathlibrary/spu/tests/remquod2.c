/* Test remquod2 for SPU
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

/*
 * note: cannot calc too far numbers correctry
 *       ex. x=0xFFE0000000000000,y=0x0008000000000000
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "simdmath.h"
#include "common-test.h"
#include "testutils.h"

typedef struct {
	unsigned long long int  xxx[2];
	unsigned long long int  yyy[2];
	unsigned long long int  quo[2];
	unsigned long long int  a_res[2];
	unsigned long long int  a_quo[2];
} TestVec64_RemqD;

int main()
{
	TestVec64_RemqD test_a[] = {
		{
			// normal 2.5/1.5 29/3
			{0x4004000000000000ULL,0x403d000000000000ULL},
			{0x3ff8000000000000ULL,0x4008000000000000ULL},
			{0x0000000000000000ULL,0x0000000000000000ULL},
			{0xbfe0000000000000ULL,0xbff0000000000000ULL},
			{0x0000000000000002ULL,0x0000000000000002ULL}
		},{
			// normal 
			{0x09d0000000000006ULL,0x1000000000000000ULL},
			{0x8010000000000005ULL,0x0010000000000007ULL},
			{0x0000000000000000ULL,0x0000000000000000ULL},
			{0x800000000000007dULL,0x80037ffffffff1a5ULL},
			{0xFFFFFFFFFFFFFFFFULL,0x0000000000000003ULL}
		},{
			// denorm
			{0x0000000000000001ULL,0x800ffffffffffff3ULL},
			{0x8000000000000001ULL,0x8000000000000005ULL},
			{0x0000000000000000ULL,0x0000000000000000ULL},
			{0x0000000000000000ULL,0x0000000000000002ULL},
			{0xFFFFFFFFFFFFFFFFULL,0x0000000000000001ULL}
		},{
			// divide by inf
			{0xFFEFFFFFFFFFFFFFULL,0x0001000000000000ULL},
			{0x7FF0000000000000ULL,0x7FF0000000000000ULL},
			{0x0000000000000000ULL,0x0000000000000000ULL},
			{0xFFEFFFFFFFFFFFFFULL,0x0001000000000000ULL},
			{0x0000000000000000ULL,0x0000000000000000ULL}
		},{
			{0ULL,0ULL},
			{0ULL,0ULL},
			{0ULL,0ULL},
			{0ULL,0ULL},
			{0ULL,0ULL}
			
		}
	};
	TestVec64_RemqD test_b[] = {
		{
			// divide by zero -> nan
			{0x0000000000000000ULL,0x8000000000000000ULL},
			{0x0000000000000000ULL,0x8000000000000000ULL},
			{0x0000000000000000ULL,0x0000000000000000ULL},
			{0x7ff8000000000000ULL,0x7ff8000000000000ULL},
			{0x0000000000000000ULL,0x0000000000000000ULL}
		},{
			// Inf , -Inf
			{0x7FF0000000000000ULL,0xFFF0000000000000ULL},
			{0x7FF0000000000000ULL,0xFFF0000000000000ULL},
			{0x0000000000000000ULL,0x0000000000000000ULL},
			{0x7ff8000000000000ULL,0x7ff8000000000000ULL},
			{0x0000000000000000ULL,0x0000000000000000ULL}
		},{
			// border
			{0xFFE0000000000000ULL,0x7FEFFFFFFFFFFFFFULL},
			{0x0008000000000000ULL,0x0010000000000000ULL},
			{0x0000000000000000ULL,0x0000000000000000ULL},
			{0x8000000000000000ULL,0x0000000000000000ULL},
			{0x0000000000000000ULL,0x0000000000000000ULL}
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

	TEST_SET_START("20060919210000NM","NM", "remquod2");

   TEST_START("remquod2");

	for (ii=0; ; ii++) {
		if ( (test_a[ii].xxx[0] == 0) && (test_a[ii].xxx[1] == 0) ) break;

		// set Floating point round mode
		res_v = remquod2 (*((vec_double2 *)&test_a[ii].xxx[0]), *((vec_double2 *)&test_a[ii].yyy[0]), ((vec_llong2 *)&test_a[ii].quo[0]));
		sprintf(msg,"2006091921%04dNM", test_ctr++);
		TEST_CHECK(msg, allequal_llong2( (vec_llong2)res_v, *((vec_llong2 *)&test_a[ii].a_res[0])), 0);
		sprintf(msg,"2006091922%04dNM", test_ctr++);
		TEST_CHECK(msg, allequal_llong2( *((vec_llong2 *)&test_a[ii].quo[0]), *((vec_llong2 *)&test_a[ii].a_quo[0])), 0);
	}
	for (ii=0; ; ii++) {
		if ( (test_b[ii].xxx[0] == 0) && (test_b[ii].xxx[1] == 0) ) break;

		// set Floating point round mode
		res_v = remquod2 (*((vec_double2 *)&test_b[ii].xxx[0]), *((vec_double2 *)&test_b[ii].yyy[0]), ((vec_llong2 *)&test_b[ii].quo[0]));
		sprintf(msg,"2006091923%04dNM", test_ctr++);
		TEST_CHECK(msg, allequal_llong2( (vec_llong2)res_v, *((vec_llong2 *)&test_b[ii].a_res[0])), 0);
	}

   TEST_SET_DONE();
   
   TEST_EXIT();
}
