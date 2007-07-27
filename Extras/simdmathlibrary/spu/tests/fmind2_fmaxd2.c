/* Test fmind2 and fmaxd2 for SPU
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
 *@@ fmind2_fmaxd2 - find minimum/maximum value.
 *
 *@brief
 * boundary test for fmind2/fmaxd2. 
 *
 *@pre
 *
 *@criteria
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
   TEST_SET_START("20060824103000MH","MH", "fmind2_fmaxd2");
   
   double denorm_min = hide_double(make_double(0x0000000000000001ull));
   double denorm_max = hide_double(make_double(0x000fffffffffffffull));
   double norm_min   = hide_double(make_double(0x0010000000000000ull));
   double norm_max   = hide_double(make_double(0x7fefffffffffffffull));

   double x0min = hide_double(1760.135);
   double x0max = hide_double(19355.03);

   double x1min = hide_double(-12351.9);
   double x1max = hide_double(-139.035);

   double x2min = hide_double(-1.0);
   double x2max = hide_double(0.0);

   double x3min = hide_double(nan(""));
   double x3max = hide_double(-1.0);

   double x4min = hide_double(-0.0);
   double x4max = hide_double(0.0);

   double x5min = denorm_min;
   double x5max = hide_double(1.0e-323);

   double x6min = norm_max;
   double x6max = hide_double(HUGE_VAL);

   double x7min = hide_double(-HUGE_VAL);
   double x7max = hide_double(19355.03);

   double x8min = hide_double(-HUGE_VAL);
   double x8max = hide_double(HUGE_VAL);

   double x9min = denorm_max;
   double x9max = norm_min;

   vec_double2 x0min_v = spu_splats(x0min);
   vec_double2 x0max_v = spu_splats(x0max);

   vec_double2 x1min_v = spu_splats(x1min); 
   vec_double2 x1max_v = spu_splats(x1max); 

   vec_double2 x2min_v = spu_splats(x2min); 
   vec_double2 x2max_v = spu_splats(x2max); 

   vec_double2 x3min_v = spu_splats(x3min); 
   vec_double2 x3max_v = spu_splats(x3max); 

   vec_double2 x4min_v = spu_splats(x4min); 
   vec_double2 x4max_v = spu_splats(x4max); 

   vec_double2 x5min_v = spu_splats(x5min); 
   vec_double2 x5max_v = spu_splats(x5max); 

   vec_double2 x6min_v = spu_splats(x6min); 
   vec_double2 x6max_v = spu_splats(x6max); 

   vec_double2 x7min_v = spu_splats(x7min); 
   vec_double2 x7max_v = spu_splats(x7max); 

   vec_double2 x8min_v = spu_splats(x8min); 
   vec_double2 x8max_v = spu_splats(x8max); 

   vec_double2 x9min_v = spu_splats(x9min); 
   vec_double2 x9max_v = spu_splats(x9max); 

   vec_double2 x51min_v = (vec_double2){x5min, x1min};
   vec_double2 x51max_v = (vec_double2){x5max, x1max};

   vec_double2 res_v;

   TEST_START("fmind2");
   res_v = fmind2(x0min_v, x0max_v);
   TEST_CHECK("20060824103001MH", allequal_double2( res_v, x0min_v ), 0);
   res_v = fmind2(x0max_v, x0min_v);      
   TEST_CHECK("20060824103002MH", allequal_double2( res_v, x0min_v ), 0);
   res_v = fmind2(x1min_v, x1max_v);
   TEST_CHECK("20060824103003MH", allequal_double2( res_v, x1min_v ), 0);
   res_v = fmind2(x1max_v, x1min_v);      
   TEST_CHECK("20060824103004MH", allequal_double2( res_v, x1min_v ), 0);
   res_v = fmind2(x2min_v, x2max_v);
   TEST_CHECK("20060824103005MH", allequal_double2( res_v, x2min_v ), 0);
   res_v = fmind2(x2max_v, x2min_v);      
   TEST_CHECK("20060824103006MH", allequal_double2( res_v, x2min_v ), 0);
   res_v = fmind2(x3min_v, x3max_v);
   TEST_CHECK("20060824103007MH", allequal_double2( res_v, x3max_v ), 0);
   res_v = fmind2(x3max_v, x3min_v);
   TEST_CHECK("20060824103008MH", allequal_double2( res_v, x3max_v ), 0);
   res_v = fmind2(x4min_v, x4max_v);
   TEST_CHECK("20060824103009MH", allequal_double2( res_v, x4min_v ), 0);
   res_v = fmind2(x4max_v, x4min_v);
   TEST_CHECK("20060824103010MH", allequal_double2( res_v, x4min_v ), 0);
   res_v = fmind2(x5min_v, x5max_v);
   TEST_CHECK("20060824103011MH", allequal_double2( res_v, x5min_v ), 0);
   res_v = fmind2(x5max_v, x5min_v);
   TEST_CHECK("20060824103012MH", allequal_double2( res_v, x5min_v ), 0);
   res_v = fmind2(x6min_v, x6max_v);
   TEST_CHECK("20060824103013MH", allequal_double2( res_v, x6min_v ), 0);
   res_v = fmind2(x6max_v, x6min_v);
   TEST_CHECK("20060824103014MH", allequal_double2( res_v, x6min_v ), 0);
   res_v = fmind2(x7min_v, x7max_v);
   TEST_CHECK("20060824103015MH", allequal_double2( res_v, x7min_v ), 0);
   res_v = fmind2(x7max_v, x7min_v);
   TEST_CHECK("20060824103016MH", allequal_double2( res_v, x7min_v ), 0);
   res_v = fmind2(x8min_v, x8max_v);
   TEST_CHECK("20060824103017MH", allequal_double2( res_v, x8min_v ), 0);
   res_v = fmind2(x8max_v, x8min_v);
   TEST_CHECK("20060824103018MH", allequal_double2( res_v, x8min_v ), 0);
   res_v = fmind2(x9min_v, x9max_v);
   TEST_CHECK("20060824103019MH", allequal_double2( res_v, x9min_v ), 0);
   res_v = fmind2(x9max_v, x9min_v);
   TEST_CHECK("20060824103020MH", allequal_double2( res_v, x9min_v ), 0);
   res_v = fmind2(x51min_v, x51max_v);
   TEST_CHECK("20060824103021MH", allequal_double2( res_v, x51min_v ), 0);
   res_v = fmind2(x51max_v, x51min_v);
   TEST_CHECK("20060824103022MH", allequal_double2( res_v, x51min_v ), 0);
   
   TEST_START("fmaxd2");
   res_v = fmaxd2(x0min_v, x0max_v);
   TEST_CHECK("20060824103101MH", allequal_double2( res_v, x0max_v ), 0);
   res_v = fmaxd2(x0max_v, x0min_v);      
   TEST_CHECK("20060824103102MH", allequal_double2( res_v, x0max_v ), 0);
   res_v = fmaxd2(x1min_v, x1max_v);
   TEST_CHECK("20060824103103MH", allequal_double2( res_v, x1max_v ), 0);
   res_v = fmaxd2(x1max_v, x1min_v);      
   TEST_CHECK("20060824103104MH", allequal_double2( res_v, x1max_v ), 0);
   res_v = fmaxd2(x2min_v, x2max_v);
   TEST_CHECK("20060824103105MH", allequal_double2( res_v, x2max_v ), 0);
   res_v = fmaxd2(x2max_v, x2min_v);      
   TEST_CHECK("20060824103106MH", allequal_double2( res_v, x2max_v ), 0);
   res_v = fmaxd2(x3min_v, x3max_v);
   TEST_CHECK("20060824103107MH", allequal_double2( res_v, x3max_v ), 0);
   res_v = fmaxd2(x3max_v, x3min_v);
   TEST_CHECK("20060824103108MH", allequal_double2( res_v, x3max_v ), 0);
   res_v = fmaxd2(x4min_v, x4max_v);
   TEST_CHECK("20060824103109MH", allequal_double2( res_v, x4max_v ), 0);
   res_v = fmaxd2(x4max_v, x4min_v);
   TEST_CHECK("20060824103110MH", allequal_double2( res_v, x4max_v ), 0);
   res_v = fmaxd2(x5min_v, x5max_v);
   TEST_CHECK("20060824103111MH", allequal_double2( res_v, x5max_v ), 0);
   res_v = fmaxd2(x5max_v, x5min_v);
   TEST_CHECK("20060824103112MH", allequal_double2( res_v, x5max_v ), 0);
   res_v = fmaxd2(x6min_v, x6max_v);
   TEST_CHECK("20060824103113MH", allequal_double2( res_v, x6max_v ), 0);
   res_v = fmaxd2(x6max_v, x6min_v);
   TEST_CHECK("20060824103114MH", allequal_double2( res_v, x6max_v ), 0);
   res_v = fmaxd2(x7min_v, x7max_v);
   TEST_CHECK("20060824103115MH", allequal_double2( res_v, x7max_v ), 0);
   res_v = fmaxd2(x7max_v, x7min_v);
   TEST_CHECK("20060824103116MH", allequal_double2( res_v, x7max_v ), 0);
   res_v = fmaxd2(x8min_v, x8max_v);
   TEST_CHECK("20060824103117MH", allequal_double2( res_v, x8max_v ), 0);
   res_v = fmaxd2(x8max_v, x8min_v);
   TEST_CHECK("20060824103118MH", allequal_double2( res_v, x8max_v ), 0);
   res_v = fmaxd2(x9min_v, x9max_v);
   TEST_CHECK("20060824103119MH", allequal_double2( res_v, x9max_v ), 0);
   res_v = fmaxd2(x9max_v, x9min_v);
   TEST_CHECK("20060824103120MH", allequal_double2( res_v, x9max_v ), 0);
   res_v = fmaxd2(x51min_v, x51max_v);
   TEST_CHECK("20060824103121MH", allequal_double2( res_v, x51max_v ), 0);
   res_v = fmaxd2(x51max_v, x51min_v);
   TEST_CHECK("20060824103122MH", allequal_double2( res_v, x51max_v ), 0);
   
   TEST_SET_DONE();
   
   TEST_EXIT();
}
