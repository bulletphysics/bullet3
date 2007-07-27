/* Test fabsd2 for SPU
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
   TEST_SET_START("20040908022501EJL","EJL", "fabs");
   
   double x0n = hide_double(-0.0);
   double x0p = hide_double(0.0);
   double x1n = hide_double(-83532.96153153);
   double x1p = hide_double(83532.96153153);
   double x2n = hide_double(-0.0000000013152);
   double x2p = hide_double(0.0000000013152);
   double x3n = hide_double(-HUGE_VAL);
   double x3p = hide_double(HUGE_VAL);
   
   vec_double2 x0n_v = spu_splats(x0n);
   vec_double2 x0p_v = spu_splats(x0p);
   vec_double2 x1n_v = spu_splats(x1n);
   vec_double2 x1p_v = spu_splats(x1p);
   vec_double2 x2n_v = spu_splats(x2n);
   vec_double2 x2p_v = spu_splats(x2p);
   vec_double2 x3n_v = spu_splats(x3n);
   vec_double2 x3p_v = spu_splats(x3p);
   
   double res;
   vec_double2 res_v;

   TEST_START("fabsd2");
   res_v = fabsd2(x0n_v);
   TEST_CHECK("20040908022502EJL", allequal_double2( res_v, x0p_v ), 0);
   res_v = fabsd2(x0p_v);
   TEST_CHECK("20040908022503EJL", allequal_double2( res_v, x0p_v ), 0);
   res_v = fabsd2(x1n_v);
   TEST_CHECK("20040908022504EJL", allequal_double2( res_v, x1p_v ), 0);
   res_v = fabsd2(x1p_v);
   TEST_CHECK("20040908022505EJL", allequal_double2( res_v, x1p_v ), 0);
   res_v = fabsd2(x2n_v);
   TEST_CHECK("20040908022506EJL", allequal_double2( res_v, x2p_v ), 0);
   res_v = fabsd2(x2p_v);
   TEST_CHECK("20040908022507EJL", allequal_double2( res_v, x2p_v ), 0);
   res_v = fabsd2(x3n_v);
   TEST_CHECK("20040908022508EJL", allposinf_double2( res_v ), 0);
   res_v = fabsd2(x3p_v);
   TEST_CHECK("20040908022509EJL", allposinf_double2( res_v ), 0);
   
   TEST_START("fabs");
   res = fabs( x0n );
   TEST_CHECK("20040908022510EJL", res == x0p, 0);
   res = fabs( x0p );
   TEST_CHECK("20040908022511EJL", res == x0p, 0);
   res = fabs( x1n );
   TEST_CHECK("20040908022512EJL", res == x1p, 0);
   res = fabs( x1p );
   TEST_CHECK("20040908022513EJL", res == x1p, 0);
   res = fabs( x2n );
   TEST_CHECK("20040908022514EJL", res == x2p, 0);
   res = fabs( x2p );
   TEST_CHECK("20040908022515EJL", res == x2p, 0);
   res = fabs( x3n );
   TEST_CHECK("20040908022516EJL", isinf(res) == 1, 0);
   res = fabs( x3p );
   TEST_CHECK("20040908022517EJL", isinf(res) == 1, 0);
   
   TEST_SET_DONE();
   
   TEST_EXIT();
}
