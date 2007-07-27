/* Test sqrtd2 for SPU
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
   TEST_SET_START("20040928181417EJL","EJL", "sqrtd2");
   
   unsigned long long i6 =   0x7464fff515d76f87ull;
   unsigned long long i6r =  0x5a29ec408d8da268ull;
   unsigned long long i7 =   0x7606a4533cf5605eull;
   unsigned long long i7r =  0x5afaead3b0ed2de5ull;
   unsigned long long i8 =   0x4beae58c6f48733eull;
   unsigned long long i8r =  0x45ed566b9c6464d6ull;
   unsigned long long i9 =   0x3999ed5c8316b00bull;
   unsigned long long i9r =  0x3cc45e14871d1b1full;
   unsigned long long i10 =  0x68f7885c4b84b793ull;
   unsigned long long i10r = 0x54736778e6778980ull;
   unsigned long long i11 =  0x1aabc083c5c26227ull;
   unsigned long long i11r = 0x2d4dcce790f64a35ull;

   double x0 = hide_double(-HUGE_VAL);           // -Inf -> NaN
   double x1 = hide_double(HUGE_VAL);            // Inf -> Inf
   double x2 = hide_double(0.0);                // +0  -> +0
   double x3 = hide_double(-0.0);               // -0  -> -0
   double x4 = hide_double(nan(""));            // NaN -> NaN
   double x5 = hide_double(4.0);
   double x5r = hide_double(2.0);
   double x6 = hide_double(make_double(i6));
   double x6r = hide_double(make_double(i6r));
   double x7 = hide_double(make_double(i7));
   double x7r = hide_double(make_double(i7r));
   double x8 = hide_double(make_double(i8));
   double x8r = hide_double(make_double(i8r));
   double x9 = hide_double(make_double(i9));
   double x9r = hide_double(make_double(i9r));   
   double x10 = hide_double(make_double(i10));     
   double x10r = hide_double(make_double(i10r));   
   double x11 = hide_double(make_double(i11));     
   double x11r = hide_double(make_double(i11r));   
   
   vec_double2 x0_v = spu_splats(x0);
   vec_double2 x1_v = spu_splats(x1);
   vec_double2 x2_v = spu_splats(x2);
   vec_double2 x3_v = spu_splats(x3);
   vec_double2 x4_v = spu_splats(x4);
   vec_double2 x5_v = spu_splats(x5);
   vec_double2 x5r_v = spu_splats(x5r);
   vec_double2 x6_v = spu_splats(x6);
   vec_double2 x6r_v = spu_splats(x6r);
   vec_double2 x7_v = spu_splats(x7);
   vec_double2 x7r_v = spu_splats(x7r);
   vec_double2 x8_v = spu_splats(x8);
   vec_double2 x8r_v = spu_splats(x8r);
   vec_double2 x9_v = spu_splats(x9);
   vec_double2 x9r_v = spu_splats(x9r);
   vec_double2 x10_v = spu_splats(x10);
   vec_double2 x10r_v = spu_splats(x10r);
   vec_double2 x11_v = spu_splats(x11);
   vec_double2 x11r_v = spu_splats(x11r);
   
   vec_double2 res_v;

   TEST_START("sqrtd2");
   res_v = sqrtd2(x0_v);
   TEST_CHECK("20040928181422EJL", allnan_double2( res_v ), 0);
   res_v = sqrtd2(x1_v);
   TEST_CHECK("20040928181424EJL", allposinf_double2( res_v ), 0);
   res_v = sqrtd2(x2_v);
   TEST_CHECK("20040928181426EJL", allposzero_double2( res_v ), 0);
   res_v = sqrtd2(x3_v);
   TEST_CHECK("20040928181430EJL", allnegzero_double2( res_v ), 0);
   res_v = sqrtd2(x4_v);
   TEST_CHECK("20040928181432EJL", allnan_double2( res_v ), 0);
   res_v = sqrtd2(x5_v);
   TEST_CHECK("20040928181434EJL", allequal_double2( res_v, x5r_v ), 0);
   res_v = sqrtd2(x6_v);
   TEST_CHECK("20040928181436EJL", allequal_ulps_double2( res_v, x6r_v, 1 ), 0);
   res_v = sqrtd2(x7_v);
   TEST_CHECK("20040928181438EJL", allequal_ulps_double2( res_v, x7r_v, 1 ), 0);
   res_v = sqrtd2(x8_v);
   TEST_CHECK("20040928181440EJL", allequal_ulps_double2( res_v, x8r_v, 1 ), 0);
   res_v = sqrtd2(x9_v);
   TEST_CHECK("20040928181442EJL", allequal_ulps_double2( res_v, x9r_v, 1 ), 0);
   res_v = sqrtd2(x10_v);
   TEST_CHECK("20040928181444EJL", allequal_ulps_double2( res_v, x10r_v, 1 ), 0);
   res_v = sqrtd2(x11_v);
   TEST_CHECK("20040928181446EJL", allequal_ulps_double2( res_v, x11r_v, 1 ), 0);
      
   TEST_SET_DONE();

   TEST_EXIT();
}
