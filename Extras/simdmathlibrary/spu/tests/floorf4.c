/* Test floorf4 for SPU
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
   TEST_SET_START("20040916145017EJL","EJL", "floorf");
   
   unsigned int i3 = 0x4affffff;  // 2^23 - 0.5, largest truncatable value.           
   unsigned int i3i = 0x4afffffe;                                                      
   unsigned int i4 = 0x4b000000;  // 2^23, no fractional part.                        
   unsigned int i5 = 0xcf000001;  // -2^31, one more large, and negative, value.      

   float x0 = hide_float(0.91825f);
   float x0i = hide_float(0.0f);
   float x1 = hide_float(-0.12958f);
   float x1i = hide_float(-1.0f);
   float x2 = hide_float(-79615.1875f);
   float x2i = hide_float(-79616.0f);
   float x3 = hide_float(make_float(i3));
   float x3i = hide_float(make_float(i3i));
   float x4 = hide_float(make_float(i4));
   float x4i = hide_float(make_float(i4));
   float x5 = hide_float(make_float(i5));
   float x5i = hide_float(make_float(i5));

   vec_float4 x0_v = spu_splats(x0);
   vec_float4 x0i_v = spu_splats(x0i);
   vec_float4 x1_v = spu_splats(x1);
   vec_float4 x1i_v = spu_splats(x1i);
   vec_float4 x2_v = spu_splats(x2);
   vec_float4 x2i_v = spu_splats(x2i);
   vec_float4 x3_v = spu_splats(x3);
   vec_float4 x3i_v = spu_splats(x3i);
   vec_float4 x4_v = spu_splats(x4);
   vec_float4 x4i_v = spu_splats(x4i);
   vec_float4 x5_v = spu_splats(x5);
   vec_float4 x5i_v = spu_splats(x5i);
   
   float res;
   vec_float4 res_v;

   TEST_START("floorf4");
   res_v = floorf4(x0_v);
   TEST_CHECK("20040916145022EJL", allequal_float4( res_v, x0i_v ), 0);
   res_v = floorf4(x1_v);
   TEST_CHECK("20040916145024EJL", allequal_float4( res_v, x1i_v ), 0);
   res_v = floorf4(x2_v);
   TEST_CHECK("20040916145027EJL", allequal_float4( res_v, x2i_v ), 0);
   res_v = floorf4(x3_v);
   TEST_CHECK("20040916145029EJL", allequal_float4( res_v, x3i_v ), 0);
   res_v = floorf4(x4_v);
   TEST_CHECK("20040916145032EJL", allequal_float4( res_v, x4i_v ), 0);
   res_v = floorf4(x5_v);
   TEST_CHECK("20040916145034EJL", allequal_float4( res_v, x5i_v ), 0);
   
   TEST_START("floorf");
   res = floorf(x0);
   TEST_CHECK("20040916155814EJL", res == x0i, 0);
   res = floorf(x1);
   TEST_CHECK("20040916155818EJL", res == x1i, 0);
   res = floorf(x2);
   TEST_CHECK("20040916155822EJL", res == x2i, 0);
   res = floorf(x3);
   TEST_CHECK("20040916155825EJL", res == x3i, 0);
   res = floorf(x4);
   TEST_CHECK("20040916155827EJL", res == x4i, 0);
   res = floorf(x5);
   TEST_CHECK("20040916155830EJL", res == x5i, 0);
   
   TEST_SET_DONE();
   
   TEST_EXIT();
}
