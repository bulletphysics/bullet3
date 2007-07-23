/* Test modff4 for SPU
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
   TEST_SET_START("20040916170642EJL", "EJL", "modff");
   
   unsigned int i3 = 0x4affffff;  // 2^23 - 0.5, largest truncatable value.           
   unsigned int i3i = 0x4afffffe;                                                      
   unsigned int i4 = 0x4b000000;  // 2^23, no fractional part.                        
   unsigned int i5 = 0xcf000001;  // -2^31, one more large, and negative, value.      

   float x0 = hide_float(0.91825f);
   float x0i = hide_float(0.0f);
   float x0f = hide_float(0.91825f);

   float x1 = hide_float(-0.12958f);
   float x1i = hide_float(0.0f);
   float x1f = hide_float(-0.12958f);

   float x2 = hide_float(-79615.1875f);
   float x2i = hide_float(-79615.0f);
   float x2f = hide_float(-0.1875f);

   float x3 = hide_float(make_float(i3));
   float x3i = hide_float(make_float(i3i));
   float x3f = hide_float(0.5f);

   float x4 = hide_float(make_float(i4));
   float x4i = hide_float(make_float(i4));
   float x4f = hide_float(0.0f);

   float x5 = hide_float(make_float(i5));
   float x5i = hide_float(make_float(i5));
   float x5f = hide_float(0.0f);

   vec_float4 x0_v = spu_splats(x0);
   vec_float4 x0i_v = spu_splats(x0i);
   vec_float4 x0f_v = spu_splats(x0f);
   vec_float4 x1_v = spu_splats(x1);
   vec_float4 x1i_v = spu_splats(x1i);
   vec_float4 x1f_v = spu_splats(x1f);
   vec_float4 x2_v = spu_splats(x2);
   vec_float4 x2i_v = spu_splats(x2i);
   vec_float4 x2f_v = spu_splats(x2f);
   vec_float4 x3_v = spu_splats(x3);
   vec_float4 x3i_v = spu_splats(x3i);
   vec_float4 x3f_v = spu_splats(x3f);
   vec_float4 x4_v = spu_splats(x4);
   vec_float4 x4i_v = spu_splats(x4i);
   vec_float4 x4f_v = spu_splats(x4f);
   vec_float4 x5_v = spu_splats(x5);
   vec_float4 x5i_v = spu_splats(x5i);
   vec_float4 x5f_v = spu_splats(x5f);
   
   float integer, fraction;
   vec_float4 integer_v, fraction_v;

   TEST_START("modff4");
   fraction_v = modff4(x0_v, &integer_v);
   TEST_CHECK("20040916170647EJL", allequal_float4( integer_v, x0i_v ) && allequal_float4( fraction_v, x0f_v ), 0);
   fraction_v = modff4(x1_v, &integer_v);
   TEST_CHECK("20040916170650EJL", allequal_float4( integer_v, x1i_v ) && allequal_float4( fraction_v, x1f_v ), 0);
   fraction_v = modff4(x2_v, &integer_v);
   TEST_CHECK("20040916170653EJL", allequal_float4( integer_v, x2i_v ) && allequal_float4( fraction_v, x2f_v ), 0);
   fraction_v = modff4(x3_v, &integer_v);
   TEST_CHECK("20040916170656EJL", allequal_float4( integer_v, x3i_v ) && allequal_float4( fraction_v, x3f_v ), 0);
   fraction_v = modff4(x4_v, &integer_v);
   TEST_CHECK("20040916170658EJL", allequal_float4( integer_v, x4i_v ) && allequal_float4( fraction_v, x4f_v ), 0);
   fraction_v = modff4(x5_v, &integer_v);
   TEST_CHECK("20040916170701EJL", allequal_float4( integer_v, x5i_v ) && allequal_float4( fraction_v, x5f_v ), 0);
   
   TEST_START("modff");
   fraction = modff(x0, &integer);
   TEST_CHECK("20040916170704EJL", integer == x0i && fraction == x0f, 0);
   fraction = modff(x1, &integer);
   TEST_CHECK("20040916170706EJL", integer == x1i && fraction == x1f, 0);
   fraction = modff(x2, &integer);
   TEST_CHECK("20040916170709EJL", integer == x2i && fraction == x2f, 0);
   fraction = modff(x3, &integer);
   TEST_CHECK("20040916170711EJL", integer == x3i && fraction == x3f, 0);
   fraction = modff(x4, &integer);
   TEST_CHECK("20040916170714EJL", integer == x4i && fraction == x4f, 0);
   fraction = modff(x5, &integer);
   TEST_CHECK("20040916170716EJL", integer == x5i && fraction == x5f, 0);
   
   TEST_SET_DONE();
   
   TEST_EXIT();
}
