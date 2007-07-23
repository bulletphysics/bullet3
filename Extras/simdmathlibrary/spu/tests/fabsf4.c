/* Test fabsf4 for SPU
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
   TEST_SET_START("20040915032605EJL","EJL", "fabsf");
   
   unsigned int i3n = 0xffffffff;
   unsigned int i3p = 0x7fffffff;

   float x0n = hide_float(-0.0f);
   float x0p = hide_float(0.0f);
   float x1n = hide_float(-83532.96153153f);
   float x1p = hide_float(83532.96153153f);
   float x2n = hide_float(-0.0000000013152f);
   float x2p = hide_float(0.0000000013152f);
   float x3n = hide_float(make_float(i3n));
   float x3p = hide_float(make_float(i3p));
   
   vec_float4 x0n_v = spu_splats(x0n);
   vec_float4 x0p_v = spu_splats(x0p);
   vec_float4 x1n_v = spu_splats(x1n);
   vec_float4 x1p_v = spu_splats(x1p);
   vec_float4 x2n_v = spu_splats(x2n);
   vec_float4 x2p_v = spu_splats(x2p);
   vec_float4 x3n_v = spu_splats(x3n);
   vec_float4 x3p_v = spu_splats(x3p);
   
   float res;
   vec_float4 res_v;

   TEST_START("fabsf4");
   res_v = fabsf4(x0n_v);
   TEST_CHECK("20040915032618EJL", allequal_float4( res_v, x0p_v ), 0);
   res_v = fabsf4(x0p_v);
   TEST_CHECK("20040915032632EJL", allequal_float4( res_v, x0p_v ), 0);
   res_v = fabsf4(x1n_v);
   TEST_CHECK("20040915032643EJL", allequal_float4( res_v, x1p_v ), 0);
   res_v = fabsf4(x1p_v);
   TEST_CHECK("20040915032654EJL", allequal_float4( res_v, x1p_v ), 0);
   res_v = fabsf4(x2n_v);
   TEST_CHECK("20040915032704EJL", allequal_float4( res_v, x2p_v ), 0);
   res_v = fabsf4(x2p_v);
   TEST_CHECK("20040915032712EJL", allequal_float4( res_v, x2p_v ), 0);
   res_v = fabsf4(x3n_v);
   TEST_CHECK("20040915032719EJL", allequal_float4( res_v, x3p_v ), 0);
   res_v = fabsf4(x3p_v);
   TEST_CHECK("20040915032729EJL", allequal_float4( res_v, x3p_v ), 0);
   
   TEST_START("fabsf");
   res = fabsf( x0n );
   TEST_CHECK("20040915032739EJL", res == x0p, 0);
   res = fabsf( x0p );
   TEST_CHECK("20040915032747EJL", res == x0p, 0);
   res = fabsf( x1n );
   TEST_CHECK("20040915032755EJL", res == x1p, 0);
   res = fabsf( x1p );
   TEST_CHECK("20040915032806EJL", res == x1p, 0);
   res = fabsf( x2n );
   TEST_CHECK("20040915032814EJL", res == x2p, 0);
   res = fabsf( x2p );
   TEST_CHECK("20040915032826EJL", res == x2p, 0);
   res = fabsf( x3n );
   TEST_CHECK("20040915032834EJL", res == x3p, 0);
   res = fabsf( x3p );
   TEST_CHECK("20040915032841EJL", res == x3p, 0);
   
   TEST_SET_DONE();
   
   TEST_EXIT();
}
