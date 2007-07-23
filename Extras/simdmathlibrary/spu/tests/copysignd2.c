/* Test copysignd2 for SPU
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
   TEST_SET_START("20040928185245EJL","EJL", "copysign");
   
   double x0m = hide_double(1989.0);
   double x0s = hide_double(-319875.0);
   double x0c = hide_double(-1989.0);
   double x1m = hide_double(9013.0);
   double x1s = hide_double(185.0);
   double x1c = hide_double(9013.0);
   
   vec_double2 x0m_v = spu_splats(x0m);
   vec_double2 x0s_v = spu_splats(x0s);
   vec_double2 x0c_v = spu_splats(x0c);

   vec_double2 x1m_v = spu_splats(x1m);
   vec_double2 x1s_v = spu_splats(x1s);
   vec_double2 x1c_v = spu_splats(x1c);
   
   double res;
   vec_double2 res_v;

   TEST_START("copysignd2");
   res_v = copysignd2( x0m_v, x0s_v );
   TEST_CHECK("20040928185248EJL", allequal_double2( res_v, x0c_v ), 0);
   res_v = copysignd2( x1m_v, x1s_v );
   TEST_CHECK("20040928185251EJL", allequal_double2( res_v, x1c_v ), 0);
   
   TEST_START("copysign");
   res = copysign( x0m, x0s );
   TEST_CHECK("20040928185253EJL", res == x0c, 0);
   res = copysign( x1m, x1s );
   TEST_CHECK("20040928185256EJL", res == x1c, 0);
   
   TEST_SET_DONE();
   
   TEST_EXIT();
}
