/* Test fdimd2 for SPU
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
 *@@ fdimd2 - compute positive difference.
 *
 *@brief
 * boundary test for fdimd2. 
 *
 *@pre
 *
 *@criteria
 * when both of two values are denormalized, it may not work correctly.
 *
 *@note
 * source of fdimd2.c was modified from IBM SDK1.1 math library
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
   TEST_SET_START("20060824151500MH","MH", "fdimd2");
   
   double x0min = hide_double(1760.135);
   double x0max = hide_double(19355.03);
   double x0dim = hide_double(19355.03 - 1760.135);

   double x1min = hide_double(-12351.9);
   double x1max = hide_double(-139.035);
   double x1dim = hide_double((-139.035) - (-12351.9));

   double x2min = hide_double(-1.0);
   double x2max = hide_double(0.0);
   double x2dim = hide_double(1.0);

   double x3min = hide_double(nan(""));
   double x3max = hide_double(-1.0);

   double x4min = hide_double(-0.0);
   double x4max = hide_double(0.0);

   double x5min = hide_double(5.0e-324);
   double x5max = hide_double(1.0e-323);
   double x5dim = hide_double(1.0e-323) - hide_double(5.0e-324);

   double x6min = hide_double(DBL_MAX);
   double x6max = hide_double(HUGE_VAL);

   double x7min = hide_double(-HUGE_VAL);
   double x7max = hide_double(19355.03);

   double x8min = hide_double(-HUGE_VAL);
   double x8max = hide_double(HUGE_VAL);

   vec_double2 x0min_v = spu_splats(x0min);
   vec_double2 x0max_v = spu_splats(x0max);
   vec_double2 x0dim_v = spu_splats(x0dim);

   vec_double2 x1min_v = spu_splats(x1min); 
   vec_double2 x1max_v = spu_splats(x1max); 
   vec_double2 x1dim_v = spu_splats(x1dim); 

   vec_double2 x2min_v = spu_splats(x2min); 
   vec_double2 x2max_v = spu_splats(x2max); 
   vec_double2 x2dim_v = spu_splats(x2dim); 

   vec_double2 x3min_v = spu_splats(x3min); 
   vec_double2 x3max_v = spu_splats(x3max); 

   vec_double2 x4min_v = spu_splats(x4min); 
   vec_double2 x4max_v = spu_splats(x4max); 

   vec_double2 x5min_v = spu_splats(x5min); 
   vec_double2 x5max_v = spu_splats(x5max); 
   vec_double2 x5dim_v = spu_splats(x5dim); 

   vec_double2 x6min_v = spu_splats(x6min); 
   vec_double2 x6max_v = spu_splats(x6max); 

   vec_double2 x7min_v = spu_splats(x7min); 
   vec_double2 x7max_v = spu_splats(x7max); 

   vec_double2 x8min_v = spu_splats(x8min); 
   vec_double2 x8max_v = spu_splats(x8max); 

   vec_double2 x9minmax_v = (vec_double2){x0min, x1max};
   vec_double2 x9maxmin_v = (vec_double2){x0max, x1min};
   vec_double2 x9dim1_v = (vec_double2){x0dim, 0.0};
   vec_double2 x9dim2_v = (vec_double2){0.0, x1dim};

   vec_double2 res_v;

   TEST_START("fdimd2");
   res_v = fdimd2(x0min_v, x0max_v);
   TEST_CHECK("20060824151501MH", allposzero_double2( res_v ), 0);
   res_v = fdimd2(x0max_v, x0min_v);      
   TEST_CHECK("20060824151502MH", allequal_double2( res_v, x0dim_v ), 0);
   res_v = fdimd2(x1min_v, x1max_v);
   TEST_CHECK("20060824151503MH", allposzero_double2( res_v ), 0);
   res_v = fdimd2(x1max_v, x1min_v);      
   TEST_CHECK("20060824151504MH", allequal_double2( res_v, x1dim_v ), 0);
   res_v = fdimd2(x2min_v, x2max_v);
   TEST_CHECK("20060824151505MH", allposzero_double2( res_v ), 0);
   res_v = fdimd2(x2max_v, x2min_v);      
   TEST_CHECK("20060824151506MH", allequal_double2( res_v, x2dim_v ), 0);
   res_v = fdimd2(x3min_v, x3max_v);
   TEST_CHECK("20060824151507MH", allnan_double2( res_v ), 0);
   res_v = fdimd2(x3max_v, x3min_v);
   TEST_CHECK("20060824151508MH", allnan_double2( res_v ), 0);
   res_v = fdimd2(x4min_v, x4max_v);
   TEST_CHECK("20060824151509MH", allposzero_double2( res_v ), 0);
   res_v = fdimd2(x4max_v, x4min_v);
   TEST_CHECK("20060824151510MH", allposzero_double2( res_v ), 0);
   res_v = fdimd2(x5min_v, x5max_v);
   TEST_CHECK("20060824151511MH", allposzero_double2( res_v ), 0);
   res_v = fdimd2(x5max_v, x5min_v);
   TEST_CHECK("20060824151512MH", allequal_double2( res_v, x5dim_v ), 0);
   res_v = fdimd2(x6min_v, x6max_v);
   TEST_CHECK("20060824151513MH", allposzero_double2( res_v ), 0);
   res_v = fdimd2(x6max_v, x6min_v);
   TEST_CHECK("20060824151514MH", allposinf_double2( res_v ), 0);
   res_v = fdimd2(x7min_v, x7max_v);
   TEST_CHECK("20060824151515MH", allposzero_double2( res_v ), 0);
   res_v = fdimd2(x7max_v, x7min_v);
   TEST_CHECK("20060824151516MH", allposinf_double2( res_v ), 0);
   res_v = fdimd2(x8min_v, x8max_v);
   TEST_CHECK("20060824151517MH", allposzero_double2( res_v ), 0);
   res_v = fdimd2(x8max_v, x8min_v);
   TEST_CHECK("20060824151518MH", allposinf_double2( res_v ), 0);
   res_v = fdimd2(x9minmax_v, x9maxmin_v);
   TEST_CHECK("20060824151519MH", allequal_double2( res_v, x9dim2_v ), 0);
   res_v = fdimd2(x9maxmin_v, x9minmax_v);
   TEST_CHECK("20060824151520MH", allequal_double2( res_v, x9dim1_v ), 0);
   
   TEST_SET_DONE();
   
   TEST_EXIT();
}
