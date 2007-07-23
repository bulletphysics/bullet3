/* Test modfd2 for SPU
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
   TEST_SET_START("20060901173000MH", "MH", "modfd2");
   
   unsigned long long i3 = 0x432fffffffffffffull;  // 2^52 - 0.5, largest truncatable value.           
   unsigned long long i3i = 0x432ffffffffffffeull;                                                      
   unsigned long long i4 = 0x4330000000000000ull;  // 2^53, no fractional part.                        
   unsigned long long i5 = 0xcff0000000000001ull;  // one more large, and negative, value.      

   double x0 = hide_double(0.91825);
   double x0i = hide_double(0.0);
   double x0f = hide_double(0.91825);

   double x1 = hide_double(-0.12958);
   double x1i = hide_double(0.0);
   double x1f = hide_double(-0.12958);

   double x2 = hide_double(-79615.1875);
   double x2i = hide_double(-79615.0);
   double x2f = hide_double(-0.1875);

   double x3 = hide_double(make_double(i3));
   double x3i = hide_double(make_double(i3i));
   double x3f = hide_double(0.5);

   double x4 = hide_double(make_double(i4));
   double x4i = hide_double(make_double(i4));
   double x4f = hide_double(0.0);

   double x5 = hide_double(make_double(i5));
   double x5i = hide_double(make_double(i5));
   double x5f = hide_double(0.0);

   vec_double2 x0_v = spu_splats(x0);
   vec_double2 x0i_v = spu_splats(x0i);
   vec_double2 x0f_v = spu_splats(x0f);
   vec_double2 x1_v = spu_splats(x1);
   vec_double2 x1i_v = spu_splats(x1i);
   vec_double2 x1f_v = spu_splats(x1f);
   vec_double2 x2_v = spu_splats(x2);
   vec_double2 x2i_v = spu_splats(x2i);
   vec_double2 x2f_v = spu_splats(x2f);
   vec_double2 x3_v = spu_splats(x3);
   vec_double2 x3i_v = spu_splats(x3i);
   vec_double2 x3f_v = spu_splats(x3f);
   vec_double2 x4_v = spu_splats(x4);
   vec_double2 x4i_v = spu_splats(x4i);
   vec_double2 x4f_v = spu_splats(x4f);
   vec_double2 x5_v = spu_splats(x5);
   vec_double2 x5i_v = spu_splats(x5i);
   vec_double2 x5f_v = spu_splats(x5f);
   
   vec_double2 integer_v, fraction_v;

   TEST_START("modff4");
   fraction_v = modfd2(x0_v, &integer_v);
   TEST_CHECK("20040916170647EJL", allequal_double2( integer_v, x0i_v ) && allequal_double2( fraction_v, x0f_v ), 0);
   fraction_v = modfd2(x1_v, &integer_v);
   TEST_CHECK("20040916170650EJL", allequal_double2( integer_v, x1i_v ) && allequal_double2( fraction_v, x1f_v ), 0);
   fraction_v = modfd2(x2_v, &integer_v);
   TEST_CHECK("20040916170653EJL", allequal_double2( integer_v, x2i_v ) && allequal_double2( fraction_v, x2f_v ), 0);
   fraction_v = modfd2(x3_v, &integer_v);
   TEST_CHECK("20040916170656EJL", allequal_double2( integer_v, x3i_v ) && allequal_double2( fraction_v, x3f_v ), 0);
   fraction_v = modfd2(x4_v, &integer_v);
   TEST_CHECK("20040916170658EJL", allequal_double2( integer_v, x4i_v ) && allequal_double2( fraction_v, x4f_v ), 0);
   fraction_v = modfd2(x5_v, &integer_v);
   TEST_CHECK("20040916170701EJL", allequal_double2( integer_v, x5i_v ) && allequal_double2( fraction_v, x5f_v ), 0);
   
   TEST_SET_DONE();
   
   TEST_EXIT();
}
