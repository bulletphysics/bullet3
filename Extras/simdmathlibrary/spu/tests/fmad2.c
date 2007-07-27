/* Test fmad2 for SPU
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
 *@@ fmad2 - multiply and add (double).
 *
 *@brief
 * boundary test for fmad2. 
 *
 *@pre
 *
 *@criteria
 * if input parameters are denorm, it may not work correctly.
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
   TEST_SET_START("20060828114000MH","MH", "fmad2");

//   double denorm_min = hide_double(make_double(0x0000000000000001ull));
   double denorm_max = hide_double(make_double(0x000fffffffffffffull));
//   double norm_min   = hide_double(make_double(0x0010000000000000ull));
   double norm_max   = hide_double(make_double(0x7fefffffffffffffull));
   
   double x0 = hide_double(1760.135);
   double y0 = hide_double(19355.03);
   double z0 = hide_double(-12351.9);
   double a0 = hide_double(34055113.82905);

   double x1 = hide_double(-139.035);
   double y1 = hide_double(0.0);
   double z1 = hide_double(-1.0);

   double x2 = hide_double(nan(""));
   double y2 = hide_double(-1.0);
   double z2 = hide_double(-0.0);

   double x3 = hide_double(1.0);
   double y3 = hide_double(HUGE_VAL);
   double z3 = hide_double(-1.0);

   double x4 = norm_max;
   double y4 = norm_max;
   double z4 = hide_double(0.0);

   double x5 = hide_double(100.0);
   double y5 = denorm_max;
   double z5 = hide_double(0.0);
   double a5 = hide_double(make_double(0x0078fffffffffffeull));

   vec_double2 x0_v = spu_splats(x0); 
   vec_double2 y0_v = spu_splats(y0); 
   vec_double2 z0_v = spu_splats(z0); 
   vec_double2 x1_v = spu_splats(x1); 
   vec_double2 y1_v = spu_splats(y1); 
   vec_double2 z1_v = spu_splats(z1); 
   vec_double2 x2_v = spu_splats(x2); 
   vec_double2 y2_v = spu_splats(y2); 
   vec_double2 z2_v = spu_splats(z2); 
   vec_double2 x3_v = spu_splats(x3); 
   vec_double2 y3_v = spu_splats(y3); 
   vec_double2 z3_v = spu_splats(z3); 
   vec_double2 x4_v = spu_splats(x4); 
   vec_double2 y4_v = spu_splats(y4); 
   vec_double2 z4_v = spu_splats(z4); 
   vec_double2 x5_v = spu_splats(x5); 
   vec_double2 y5_v = spu_splats(y5); 
   vec_double2 z5_v = spu_splats(z5); 

   vec_double2 a0_v = spu_splats(a0); 
   vec_double2 a1_v = spu_splats(z1); 
   vec_double2 a5_v = spu_splats(a5); 

   vec_double2 res_v;

   TEST_START("fmad2");
   res_v = fmad2(x0_v, y0_v, z0_v);
   TEST_CHECK("20060828114001MH", allequal_ulps_double2( res_v, a0_v, 1 ), 0);
   res_v = fmad2(y0_v, x0_v, z0_v);
   TEST_CHECK("20060828114002MH", allequal_ulps_double2( res_v, a0_v, 1 ), 0);
   res_v = fmad2(x1_v, y1_v, z1_v);
   TEST_CHECK("20060828114003MH", allequal_ulps_double2( res_v, a1_v, 1 ), 0);
   res_v = fmad2(y1_v, x1_v, z1_v);
   TEST_CHECK("20060828114004MH", allequal_ulps_double2( res_v, a1_v, 1 ), 0);
   res_v = fmad2(x2_v, y2_v, z2_v);
   TEST_CHECK("20060828114005MH", allnan_double2( res_v ), 0);
   res_v = fmad2(y2_v, x2_v, z2_v);
   TEST_CHECK("20060828114006MH", allnan_double2( res_v ), 0);
   res_v = fmad2(x3_v, y3_v, z3_v);
   TEST_CHECK("20060828114007MH", allposinf_double2( res_v ), 0);
   res_v = fmad2(y3_v, x3_v, z3_v);
   TEST_CHECK("20060828114008MH", allposinf_double2( res_v ), 0);
   res_v = fmad2(x4_v, y4_v, z4_v);
   TEST_CHECK("20060828114009MH", allposinf_double2( res_v ), 0);
   res_v = fmad2(y4_v, x4_v, z4_v);
   TEST_CHECK("20060828114010MH", allposinf_double2( res_v ), 0);
   res_v = fmad2(x5_v, y5_v, z5_v);
   TEST_CHECK("20060828114011MH", allequal_ulps_double2( res_v, a5_v, 1 ), 0);
   res_v = fmad2(y5_v, x5_v, z5_v);
   TEST_CHECK("20060828114012MH", allequal_ulps_double2( res_v, a5_v, 1 ), 0);
//printf("res:%.10le, a5:%.10le\n", spu_extract(res_v, 0), spu_extract(a5_v, 0));

   TEST_SET_DONE();
   
   TEST_EXIT();
}
