/* Test divf4 for SPU
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
   TEST_SET_START("20040928105926EJL","EJL", "divf4");

   unsigned int i0n = 0x75013340;
   unsigned int i0d = 0x75e7753f;
   unsigned int i0r = 0x3e8ee64b;
   unsigned int i1n = 0x4c7fed5a;
   unsigned int i1d = 0x3a0731f0;
   unsigned int i1r = 0x51f24e86;
   unsigned int i2n = 0x5b08b303;
   unsigned int i2d = 0x562f5046;
   unsigned int i2r = 0x44479d24;
   unsigned int i3n = 0x748a9b87;
   unsigned int i3d = 0x6b014b46;
   unsigned int i3r = 0x49093864;
   unsigned int i4n = 0x35dcf9d8;
   unsigned int i4d = 0x6278d6e0;
   unsigned int i4r = 0x12e355b5;
   unsigned int i5n = 0x74d505fd;
   unsigned int i5d = 0x61ef565e;
   unsigned int i5r = 0x5263daa3;

   float x0n = hide_float(make_float(i0n));
   float x0d = hide_float(make_float(i0d));
   float x0r = hide_float(make_float(i0r));
                                       
   float x1n = hide_float(make_float(i1n));
   float x1d = hide_float(make_float(i1d));
   float x1r = hide_float(make_float(i1r));
                                       
   float x2n = hide_float(make_float(i2n));
   float x2d = hide_float(make_float(i2d));
   float x2r = hide_float(make_float(i2r));
                                       
   float x3n = hide_float(make_float(i3n));
   float x3d = hide_float(make_float(i3d));
   float x3r = hide_float(make_float(i3r));
                                       
   float x4n = hide_float(make_float(i4n));
   float x4d = hide_float(make_float(i4d));
   float x4r = hide_float(make_float(i4r));
                                       
   float x5n = hide_float(make_float(i5n));
   float x5d = hide_float(make_float(i5d));
   float x5r = hide_float(make_float(i5r));
   
   vec_float4 x0n_v = spu_splats(x0n);
   vec_float4 x0d_v = spu_splats(x0d);
   vec_float4 x0r_v = spu_splats(x0r);
                                   
   vec_float4 x1n_v = spu_splats(x1n);
   vec_float4 x1d_v = spu_splats(x1d);
   vec_float4 x1r_v = spu_splats(x1r);
                                   
   vec_float4 x2n_v = spu_splats(x2n);
   vec_float4 x2d_v = spu_splats(x2d);
   vec_float4 x2r_v = spu_splats(x2r);
                                   
   vec_float4 x3n_v = spu_splats(x3n);
   vec_float4 x3d_v = spu_splats(x3d);
   vec_float4 x3r_v = spu_splats(x3r);
                                   
   vec_float4 x4n_v = spu_splats(x4n);
   vec_float4 x4d_v = spu_splats(x4d);
   vec_float4 x4r_v = spu_splats(x4r);
                                   
   vec_float4 x5n_v = spu_splats(x5n);
   vec_float4 x5d_v = spu_splats(x5d);
   vec_float4 x5r_v = spu_splats(x5r);
   
   vec_float4 res_v;

   TEST_START("divf4");
   res_v = divf4(x0n_v, x0d_v);
   TEST_CHECK("20040928105932EJL", allequal_ulps_float4( res_v, x0r_v, 2 ), 0);
   res_v = divf4(x1n_v, x1d_v);
   TEST_CHECK("20040928105934EJL", allequal_ulps_float4( res_v, x1r_v, 2 ), 0);
   res_v = divf4(x2n_v, x2d_v);
   TEST_CHECK("20040928105936EJL", allequal_ulps_float4( res_v, x2r_v, 2 ), 0);
   res_v = divf4(x3n_v, x3d_v);
   TEST_CHECK("20040928105938EJL", allequal_ulps_float4( res_v, x3r_v, 2 ), 0);
   res_v = divf4(x4n_v, x4d_v);
   TEST_CHECK("20040928105940EJL", allequal_ulps_float4( res_v, x4r_v, 2 ), 0);
   res_v = divf4(x5n_v, x5d_v);
   TEST_CHECK("20040928105943EJL", allequal_ulps_float4( res_v, x5r_v, 2 ), 0);
   
   TEST_SET_DONE();
   
   TEST_EXIT();
}
