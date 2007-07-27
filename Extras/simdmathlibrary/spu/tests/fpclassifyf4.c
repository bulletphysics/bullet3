/* Test fpclassifyf4 for SPU
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
#include "simdmath.h"
#include "common-test.h"
#include "testutils.h"

int main()
{
   TEST_SET_START("20060828000000AAN","AAN", "fpclassifyf4");

   // -Nan
   float x0 = hide_float(-NANF);
   int r0 = FP_NORMAL;
   
   // -Inf
   float x1 = hide_float(-HUGE_VALF);
   int r1 = FP_NORMAL;
   
   // -Smax
   float x2 = hide_float(make_float(0xFFFFFFFF));
   int r2 = FP_NORMAL;

   // -Norm
   float x3 = hide_float(-824842.58421394f);
   int r3 = FP_NORMAL;
   
   // -Smin
   float x4 = hide_float(make_float(0x80800000));
   int r4 = FP_NORMAL;
   
   // -Denorm
   float x5 = hide_float(make_float(0x803AAAAA));
   int r5 = FP_SUBNORMAL;
   
   // -Unf
   float x6 = hide_float(-1.0e-999);
   int r6 = FP_ZERO;
   
   // -0
   float x7 = hide_float(-0.0f);
   int r7 = FP_ZERO;

   // 0
   float x8 = hide_float( 0.0f);
   int r8 = FP_ZERO;
   
   // +Unf
   float x9 = hide_float( 1.0e-999);
   int r9 = FP_ZERO;

   // +Denorm
   float x10 = hide_float(make_float(0x003AAAAA));
   int r10 = FP_SUBNORMAL;
   
   // +Smin
   float x11 = hide_float(make_float(0x00800000));
   int r11 = FP_NORMAL;
   
   // +Norm
   float x12 = hide_float(3.14152634f);
   int r12 = FP_NORMAL;
   
   // +Smax
   float x13 = hide_float(make_float(0x7FFFFFFF));
   int r13 = FP_NORMAL;
   
   // +Inf
   float x14 = hide_float(HUGE_VALF);
   int r14 = FP_NORMAL;
   
   //+Nan
   float x15 = hide_float(NANF);
   int r15 = FP_NORMAL;
   
   // Compound
   vec_float4 x16_v = (vec_float4) {make_float(0x003AAAAA), -1.0e-999, 345.27533, make_float(0x803AAAAA)};
   vec_int4 r16_v = (vec_int4) {FP_SUBNORMAL, FP_ZERO, FP_NORMAL, FP_SUBNORMAL};

   vec_float4 x0_v = spu_splats(x0);
   vec_int4  r0_v = spu_splats(r0);

   vec_float4 x1_v = spu_splats(x1);
   vec_int4  r1_v = spu_splats(r1);

   vec_float4 x2_v = spu_splats(x2);
   vec_int4  r2_v = spu_splats(r2);

   vec_float4 x3_v = spu_splats(x3);
   vec_int4  r3_v = spu_splats(r3);

   vec_float4 x4_v = spu_splats(x4);
   vec_int4  r4_v = spu_splats(r4);

   vec_float4 x5_v = spu_splats(x5);
   vec_int4  r5_v = spu_splats(r5);

   vec_float4 x6_v = spu_splats(x6);
   vec_int4  r6_v = spu_splats(r6);

   vec_float4 x7_v = spu_splats(x7);
   vec_int4  r7_v = spu_splats(r7);

   vec_float4 x8_v = spu_splats(x8);
   vec_int4  r8_v = spu_splats(r8);

   vec_float4 x9_v = spu_splats(x9);
   vec_int4  r9_v = spu_splats(r9);

   vec_float4 x10_v = spu_splats(x10);
   vec_int4  r10_v = spu_splats(r10);

   vec_float4 x11_v = spu_splats(x11);
   vec_int4  r11_v = spu_splats(r11);

   vec_float4 x12_v = spu_splats(x12);
   vec_int4  r12_v = spu_splats(r12);

   vec_float4 x13_v = spu_splats(x13);
   vec_int4  r13_v = spu_splats(r13);

   vec_float4 x14_v = spu_splats(x14);
   vec_int4  r14_v = spu_splats(r14);

   vec_float4 x15_v = spu_splats(x15);
   vec_int4  r15_v = spu_splats(r15);
   
   vec_int4 res_v;

   TEST_START("fpclassifyf4");

   res_v = (vec_int4)fpclassifyf4(x0_v);
   TEST_CHECK("20060828000000AAN", allequal_int4( res_v, r0_v ), 0);
   res_v = (vec_int4)fpclassifyf4(x1_v);
   TEST_CHECK("20060828000001AAN", allequal_int4( res_v, r1_v ), 0);
   res_v = (vec_int4)fpclassifyf4(x2_v);
   TEST_CHECK("20060828000002AAN", allequal_int4( res_v, r2_v ), 0);
   res_v = (vec_int4)fpclassifyf4(x3_v);
   TEST_CHECK("20060828000003AAN", allequal_int4( res_v, r3_v ), 0);
   res_v = (vec_int4)fpclassifyf4(x4_v);
   TEST_CHECK("20060828000004AAN", allequal_int4( res_v, r4_v ), 0);
   res_v = (vec_int4)fpclassifyf4(x5_v);
   TEST_CHECK("20060828000005AAN", allequal_int4( res_v, r5_v ), 0);
   res_v = (vec_int4)fpclassifyf4(x6_v);
   TEST_CHECK("20060828000006AAN", allequal_int4( res_v, r6_v ), 0);
   res_v = (vec_int4)fpclassifyf4(x7_v);
   TEST_CHECK("20060828000007AAN", allequal_int4( res_v, r7_v ), 0);
   res_v = (vec_int4)fpclassifyf4(x8_v);
   TEST_CHECK("20060828000008AAN", allequal_int4( res_v, r8_v ), 0);
   res_v = (vec_int4)fpclassifyf4(x9_v);
   TEST_CHECK("20060828000009AAN", allequal_int4( res_v, r9_v ), 0);
   res_v = (vec_int4)fpclassifyf4(x10_v);
   TEST_CHECK("20060828000010AAN", allequal_int4( res_v, r10_v ), 0);
   res_v = (vec_int4)fpclassifyf4(x11_v);
   TEST_CHECK("20060828000011AAN", allequal_int4( res_v, r11_v ), 0);
   res_v = (vec_int4)fpclassifyf4(x12_v);
   TEST_CHECK("20060828000012AAN", allequal_int4( res_v, r12_v ), 0);
   res_v = (vec_int4)fpclassifyf4(x13_v);
   TEST_CHECK("20060828000013AAN", allequal_int4( res_v, r13_v ), 0);
   res_v = (vec_int4)fpclassifyf4(x14_v);
   TEST_CHECK("20060828000014AAN", allequal_int4( res_v, r14_v ), 0);
   res_v = (vec_int4)fpclassifyf4(x15_v);
   TEST_CHECK("20060828000015AAN", allequal_int4( res_v, r15_v ), 0);
   res_v = (vec_int4)fpclassifyf4(x16_v);
   TEST_CHECK("20060828000016AAN", allequal_int4( res_v, r16_v ), 0);
   
   TEST_SET_DONE();
   
   TEST_EXIT();
}
