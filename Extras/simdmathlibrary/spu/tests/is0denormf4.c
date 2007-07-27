/* Test is0denormf4 for SPU
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
   TEST_SET_START("20060830000000AAN","AAN", "is0denormf4");

   // -Nan
   float x0 = hide_float(-NANF);
   unsigned int r0 = 0x00000000;
   
   // -Inf
   float x1 = hide_float(-HUGE_VALF);
   unsigned int r1 = 0x00000000;
   
   // -Smax
   float x2 = hide_float(make_float(0xffffffff));
   unsigned int r2 = 0x00000000;

   // -Norm
   float x3 = hide_float(-824842.58421394f);
   unsigned int r3 = 0x00000000;
   
   // -Smin
   float x4 = hide_float(make_float(0x80800000));
   unsigned int r4 = 0x00000000;
   
   // -Denorm
   float x5 = hide_float(make_float(0x803aaaaa));
   unsigned int r5 = 0xffffffff;
   
   // -Unf
   float x6 = hide_float(-1.0e-999);
   unsigned int r6 = 0xffffffff;
   
   // -0
   float x7 = hide_float(-0.0f);
   unsigned int r7 = 0xffffffff;

   // 0
   float x8 = hide_float( 0.0f);
   unsigned int r8 = 0xffffffff;
   
   // +Unf
   float x9 = hide_float( 1.0e-999);
   unsigned int r9 = 0xffffffff;

   // +Denorm
   float x10 = hide_float(make_float(0x003aaaaa));
   unsigned int r10 = 0xffffffff;
   
   // +Smin
   float x11 = hide_float(make_float(0x00800000));
   unsigned int r11 = 0x00000000;
   
   // +Norm
   float x12 = hide_float(3.14152634f);
   unsigned int r12 = 0x00000000;
   
   // +Smax
   float x13 = hide_float(make_float(0x7fffffff));
   unsigned int r13 = 0x00000000;
   
   // +Inf
   float x14 = hide_float( HUGE_VALF);
   unsigned int r14 = 0x00000000;
   
   //+Nan
   float x15 = hide_float(NANF);
   unsigned int r15 = 0x00000000;
   
   // Compound
   vec_float4 x16_v = (vec_float4) {make_float(0x003AAAAA), -1.0e-999, 345.27533, make_float(0x803AAAAA)};
   vec_uint4 r16_v = (vec_uint4) {0xffffffff, 0xffffffff, 0x00000000, 0xffffffff};

   vec_float4 x0_v = spu_splats(x0);
   vec_uint4  r0_v = spu_splats(r0);

   vec_float4 x1_v = spu_splats(x1);
   vec_uint4  r1_v = spu_splats(r1);

   vec_float4 x2_v = spu_splats(x2);
   vec_uint4  r2_v = spu_splats(r2);

   vec_float4 x3_v = spu_splats(x3);
   vec_uint4  r3_v = spu_splats(r3);

   vec_float4 x4_v = spu_splats(x4);
   vec_uint4  r4_v = spu_splats(r4);

   vec_float4 x5_v = spu_splats(x5);
   vec_uint4  r5_v = spu_splats(r5);

   vec_float4 x6_v = spu_splats(x6);
   vec_uint4  r6_v = spu_splats(r6);

   vec_float4 x7_v = spu_splats(x7);
   vec_uint4  r7_v = spu_splats(r7);

   vec_float4 x8_v = spu_splats(x8);
   vec_uint4  r8_v = spu_splats(r8);

   vec_float4 x9_v = spu_splats(x9);
   vec_uint4  r9_v = spu_splats(r9);

   vec_float4 x10_v = spu_splats(x10);
   vec_uint4  r10_v = spu_splats(r10);

   vec_float4 x11_v = spu_splats(x11);
   vec_uint4  r11_v = spu_splats(r11);

   vec_float4 x12_v = spu_splats(x12);
   vec_uint4  r12_v = spu_splats(r12);

   vec_float4 x13_v = spu_splats(x13);
   vec_uint4  r13_v = spu_splats(r13);

   vec_float4 x14_v = spu_splats(x14);
   vec_uint4  r14_v = spu_splats(r14);

   vec_float4 x15_v = spu_splats(x15);
   vec_uint4  r15_v = spu_splats(r15);
   
   vec_uint4 res_v;

   TEST_START("is0denormf4");

   res_v = (vec_uint4)is0denormf4(x0_v);
   TEST_CHECK("20060830000000AAN", allequal_uint4( res_v, r0_v ), 0);
   res_v = (vec_uint4)is0denormf4(x1_v);
   TEST_CHECK("20060830000001AAN", allequal_uint4( res_v, r1_v ), 0);
   res_v = (vec_uint4)is0denormf4(x2_v);
   TEST_CHECK("20060830000002AAN", allequal_uint4( res_v, r2_v ), 0);
   res_v = (vec_uint4)is0denormf4(x3_v);
   TEST_CHECK("20060830000003AAN", allequal_uint4( res_v, r3_v ), 0);
   res_v = (vec_uint4)is0denormf4(x4_v);
   TEST_CHECK("20060830000004AAN", allequal_uint4( res_v, r4_v ), 0);
   res_v = (vec_uint4)is0denormf4(x5_v);
   TEST_CHECK("20060830000005AAN", allequal_uint4( res_v, r5_v ), 0);
   res_v = (vec_uint4)is0denormf4(x6_v);
   TEST_CHECK("20060830000006AAN", allequal_uint4( res_v, r6_v ), 0);
   res_v = (vec_uint4)is0denormf4(x7_v);
   TEST_CHECK("20060830000007AAN", allequal_uint4( res_v, r7_v ), 0);
   res_v = (vec_uint4)is0denormf4(x8_v);
   TEST_CHECK("20060830000008AAN", allequal_uint4( res_v, r8_v ), 0);
   res_v = (vec_uint4)is0denormf4(x9_v);
   TEST_CHECK("20060830000009AAN", allequal_uint4( res_v, r9_v ), 0);
   res_v = (vec_uint4)is0denormf4(x10_v);
   TEST_CHECK("20060830000010AAN", allequal_uint4( res_v, r10_v ), 0);
   res_v = (vec_uint4)is0denormf4(x11_v);
   TEST_CHECK("20060830000011AAN", allequal_uint4( res_v, r11_v ), 0);
   res_v = (vec_uint4)is0denormf4(x12_v);
   TEST_CHECK("20060830000012AAN", allequal_uint4( res_v, r12_v ), 0);
   res_v = (vec_uint4)is0denormf4(x13_v);
   TEST_CHECK("20060830000013AAN", allequal_uint4( res_v, r13_v ), 0);
   res_v = (vec_uint4)is0denormf4(x14_v);
   TEST_CHECK("20060830000014AAN", allequal_uint4( res_v, r14_v ), 0);
   res_v = (vec_uint4)is0denormf4(x15_v);
   TEST_CHECK("20060830000015AAN", allequal_uint4( res_v, r15_v ), 0);
   res_v = (vec_uint4)is0denormf4(x16_v);
   TEST_CHECK("20060830000016AAN", allequal_uint4( res_v, r16_v ), 0);
   
   TEST_SET_DONE();
   
   TEST_EXIT();
}
