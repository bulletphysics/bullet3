/* Test divd2 for SPU
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
   TEST_SET_START("20040927182952EJL","EJL", "divd2");
   
   unsigned long long i3n = 0x747f7fefa0c3274aull;
   unsigned long long i3d = 0x7606a4533cf5605eull;
   unsigned long long i3r = 0x3e66426af0ec01b0ull;
   unsigned long long i4n = 0x4c042c295376566eull;
   unsigned long long i4d = 0x39b3720562510408ull;
   unsigned long long i4r = 0x52409928d3244077ull;
   unsigned long long i5n = 0x6911a64538a389aeull;
   unsigned long long i5d = 0x1ac4d062d451c99dull;
   unsigned long long i5r = 0x7ff0000000000000ull;
   unsigned long long i6n = 0x5b112f9d39e7de27ull;
   unsigned long long i6d = 0x5659f8dbe4993d7cull;
   unsigned long long i6r = 0x44a52cb9b9d2b2cdull;
   unsigned long long i7n = 0x7410065c772e25daull;
   unsigned long long i7d = 0x6a576b936e5f1034ull;
   unsigned long long i7r = 0x49a5e53936c1b556ull;
   unsigned long long i8n = 0x3605d9b2916be0f5ull;
   unsigned long long i8d = 0x61f25e39867b0a9eull;
   unsigned long long i8r = 0x1403088aa08482f2ull;

   double x0n = hide_double(-HUGE_VAL);          // -Inf/ Inf == NaN
   double x0d = hide_double(HUGE_VAL);           

   double x1n = hide_double(0.0);               // 0 / 0 == NaN
   double x1d = hide_double(-0.0);              

   double x2n = hide_double(nan(""));           // NaN / 2 == NaN
   double x2d = hide_double(2.0);               

   double x3n = hide_double(make_double(i3n));
   double x3d = hide_double(make_double(i3d));
   double x3r = hide_double(make_double(i3r));

   double x4n = hide_double(make_double(i4n));
   double x4d = hide_double(make_double(i4d));
   double x4r = hide_double(make_double(i4r));

   double x5n = hide_double(make_double(i5n));
   double x5d = hide_double(make_double(i5d));
   double x5r = hide_double(make_double(i5r));

   double x6n = hide_double(make_double(i6n));
   double x6d = hide_double(make_double(i6d));
   double x6r = hide_double(make_double(i6r));

   double x7n = hide_double(make_double(i7n));
   double x7d = hide_double(make_double(i7d));
   double x7r = hide_double(make_double(i7r));

   double x8n = hide_double(make_double(i8n));
   double x8d = hide_double(make_double(i8d));
   double x8r = hide_double(make_double(i8r));
   
   vec_double2 x0n_v = spu_splats(x0n);
   vec_double2 x0d_v = spu_splats(x0d);

   vec_double2 x1n_v = spu_splats(x1n);
   vec_double2 x1d_v = spu_splats(x1d);

   vec_double2 x2n_v = spu_splats(x2n);
   vec_double2 x2d_v = spu_splats(x2d);

   vec_double2 x3n_v = spu_splats(x3n);
   vec_double2 x3d_v = spu_splats(x3d);
   vec_double2 x3r_v = spu_splats(x3r);

   vec_double2 x4n_v = spu_splats(x4n);
   vec_double2 x4d_v = spu_splats(x4d);
   vec_double2 x4r_v = spu_splats(x4r);

   vec_double2 x5n_v = spu_splats(x5n);
   vec_double2 x5d_v = spu_splats(x5d);
   vec_double2 x5r_v = spu_splats(x5r);

   vec_double2 x6n_v = spu_splats(x6n);
   vec_double2 x6d_v = spu_splats(x6d);
   vec_double2 x6r_v = spu_splats(x6r);

   vec_double2 x7n_v = spu_splats(x7n);
   vec_double2 x7d_v = spu_splats(x7d);
   vec_double2 x7r_v = spu_splats(x7r);

   vec_double2 x8n_v = spu_splats(x8n);
   vec_double2 x8d_v = spu_splats(x8d);
   vec_double2 x8r_v = spu_splats(x8r);
   
   vec_double2 res_v;

   TEST_START("divd2");
   res_v = divd2(x0n_v, x0d_v);
   TEST_CHECK("20040927183001EJL", allnan_double2( res_v ), 0);
   res_v = divd2(x1n_v, x1d_v);
   TEST_CHECK("20040927183003EJL", allnan_double2( res_v ), 0);
   res_v = divd2(x2n_v, x2d_v);
   TEST_CHECK("20040927183006EJL", allnan_double2( res_v ), 0);
   res_v = divd2(x3n_v, x3d_v);
   TEST_CHECK("20040927183008EJL", allequal_ulps_double2( res_v, x3r_v, 1 ), 0 );
   res_v = divd2(x4n_v, x4d_v);
   TEST_CHECK("20040927183010EJL", allequal_ulps_double2( res_v, x4r_v, 1 ), 0 );
   res_v = divd2(x5n_v, x5d_v);
   TEST_CHECK("20040927183012EJL", allequal_ulps_double2( res_v, x5r_v, 1 ), 0 );
   res_v = divd2(x6n_v, x6d_v);
   TEST_CHECK("20040927183014EJL", allequal_ulps_double2( res_v, x6r_v, 1 ), 0 );
   res_v = divd2(x7n_v, x7d_v);
   TEST_CHECK("20040927183016EJL", allequal_ulps_double2( res_v, x7r_v, 1 ), 0 );
   res_v = divd2(x8n_v, x8d_v);
   TEST_CHECK("20040927183018EJL", allequal_ulps_double2( res_v, x8r_v, 1 ), 0 );
      
   TEST_SET_DONE();

   TEST_EXIT();
}
