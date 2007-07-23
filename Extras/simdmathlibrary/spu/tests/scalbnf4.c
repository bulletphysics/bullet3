/* Test nextafterf4 for SPU
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

typedef union {
	struct {
		float xxx[4];
		int   exp[4];
		float ans[4];
	} flt;
	struct {
		unsigned int xxx[4];
		unsigned int exp[4];
		unsigned int ans[4];
	} ui;
} TestVec_ScalF4;


int main()
{
	TestVec_ScalF4 test_a[] = {
		{
			ui:{
			// 0 -> 0 , -0 -> -0
			{0x00000000,0x80000000,0x80000000,0x00000000},
			{0x000000FF,0x00000001,0xFFFFFFFF,0xFFFFFF00},
			{0x00000000,0x80000000,0x80000000,0x00000000}
			}
		},{
			ui:{
			// Inf
			{0xFF800000,0x7F800000,0x7F800000,0xFF800000},
			{0x000000FF,0x00000001,0xFFFFFFFF,0xFFFFFF00},
			{0xFFFFFFFF,0x7FFFFFFF,0x7F000000,0x80000000}
			}
		},{
			ui:{
			// MAX MIN
			{0x7F7FFFFF,0xFF7FFFFF,0x7F7FFFFF,0xFF7FFFFF},
			{0x00000001,0x00000001,0xFFFFFFFF,0xFFFFFF00},
			{0x7FFFFFFF,0xFFFFFFFF,0x7EFFFFFF,0x00000000}
			}
		},{
			flt:{
			{-1.0, 1.0, -1.0, 1.0},
			{   1,   1,   -1,  -1},
			{-2.0, 2.0, -0.5, 0.5}
			}
		},{
			ui:{
			// 
			{0x80ffffff,0x80ffffff,0x00ffffff,0x00ffffff},
			{0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF,0xFFFFFFFF},
			{0x80000000,0x80000000,0x00000000,0x00000000}
			}
		},{
			ui:{
			{0,0,0,0},
			{0,0,0,0},
			{0,0,0,0}
			}
		}
	};
	int ii, test_ctr = 1;
	char msg[80];
	vec_float4 res_v;

	TEST_SET_START("20060907150000NM","NM", "scalbnf4");

   TEST_START("scalbnf4");

	for (ii=0; ; ii++) {
		if ( (test_a[ii].ui.xxx[0] == 0) && (test_a[ii].ui.xxx[1] == 0) ) break;

		res_v = scalbnf4 (*((vec_float4 *)&test_a[ii].flt.xxx[0]), *((vec_int4 *)&test_a[ii].flt.exp[0]) );
		sprintf(msg,"2006090715%04dNM", test_ctr++);
		TEST_CHECK(msg, allequal_float4( res_v, *((vec_float4 *)&test_a[ii].flt.ans[0])), 0);
	}

   TEST_SET_DONE();

   TEST_EXIT();
}
