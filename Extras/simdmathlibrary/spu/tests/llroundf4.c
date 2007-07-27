/* Test llroundf4 for SPU
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
 *@@ llroundf4 - rounds four floats in to four nearest 64bit integer.
 *
 *@brief
 * boundary test for llroundf4. 0.5 will be rounded to far from 0.
 *
 *@pre
 *
 *@criteria
 * Run this program and check no error will be occurred.
 *
 *@note
 * when comparing with return of ppu scalar math library 
 * answer of 0x??7fffff and 0x??ffffff was something strange
 *
 **/



#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include "simdmath.h"
#include "common-test.h"
#include "testutils.h"

static inline llroundf4_t set_llroundf4_t(long long int in0, long long int in1, long long int in2, long long int in3)
{
	llroundf4_t res;

	res.vll[0] = ((vec_llong2){in0,in1});
	res.vll[1] = ((vec_llong2){in2,in3});

	return res;
}
int main()
{
	TEST_SET_START("20060917101000NM","NM", "llroundf4");

//   unsigned long i11 = 0x5efffffful; // 9223371487098961920
//   unsigned long i12 = 0xdefffffful; //-9223371487098961920
//   unsigned long i11 = 0x49fffffful; //2097151.875000
//   unsigned long i12 = 0x4a7ffffful; //4194303.750000

//   float x0  = hide_float(-FLT_MAX);           // -Inf
//   float x1  = hide_float(FLT_MAX);            // Inf
   float x2  = hide_float(0.0);                  // +0
   float x3  = hide_float(-0.0);                 // -0
//   float x4  = hide_float(NANF);            // NaN -> NaN
   float x5  = hide_float( 0.5);
   float x6  = hide_float(-0.5);
   float x7  = hide_float(-0.499999);
   float x8  = hide_float( 0.499999);
   float x9  = hide_float(-999999.5);
   float x10 = hide_float( 999999.5);
//   float x11 = hide_float(make_float(i11));
//   float x12 = hide_float(make_float(i12));
   float x11 = hide_float( 9223371487098961920.);
   float x12 = hide_float(-9223371487098961920.);
   float x13 = (0.0 - FLT_MIN);
   float x14 = FLT_MIN;
   float x15 = hide_float(-2097151.875000);
   float x16 = hide_float(-4194303.750000);
   float x17 = hide_float( 4194303.750000);
   float x18 = hide_float( 2097151.875000);

	vec_float4  x0_v = ((vec_float4){ x2, x3, x5, x6});
	llroundf4_t r0_v = set_llroundf4_t(0,  0,  1, -1);

	vec_float4  x1_v = ((vec_float4){ x7, x8,       x9,     x10});
	llroundf4_t r1_v = set_llroundf4_t(0,  0, -1000000, 1000000);

	vec_float4  x2_v = ((vec_float4){                    x11,                    x12, x13, x14});
	llroundf4_t r2_v = set_llroundf4_t(9223371487098961920ll, -9223371487098961920ll,    0,  0);

	vec_float4  x3_v = ((vec_float4){       x15,      x16,     x17,     x18});
	llroundf4_t r3_v = set_llroundf4_t(-2097152, -4194304, 4194304, 2097152);

	llroundf4_t  res_v;

   TEST_START("llroundf4");
   res_v = llroundf4 (x0_v);
   TEST_CHECK("20060916101001NM", allequal_llroundf4( res_v, r0_v ), 0);
   res_v = llroundf4 (x1_v);
   TEST_CHECK("20060916101002NM", allequal_llroundf4( res_v, r1_v ), 0);
   res_v = llroundf4 (x2_v);
   TEST_CHECK("20060916101003NM", allequal_llroundf4( res_v, r2_v ), 0);
   res_v = llroundf4 (x3_v);
   TEST_CHECK("20060916101004NM", allequal_llroundf4( res_v, r3_v ), 0);

   TEST_SET_DONE();

   TEST_EXIT();

}
