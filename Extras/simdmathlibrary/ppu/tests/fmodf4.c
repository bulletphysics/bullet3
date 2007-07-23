/* Testcase for fmodf4
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
#include "common-test.h"
#include "testutils.h"
#include "simdmath.h"



int main()
{
   TEST_SET_START("20040928191240EJL","EJL", "fmodf4");

   unsigned int i0n = 0x449edbc6;
   unsigned int i0d = 0x40cf799d;
   unsigned int i0r = 0x3daa7300;
   unsigned int i1n = 0x6bca107a;
   unsigned int i1d = 0x6c4a107a;
   unsigned int i1r = 0x6bca107a;
   unsigned int i2n = 0x1c123605;
   unsigned int i2d = 0x1c923602;
   unsigned int i2r = 0x1c123605;
   unsigned int i3n = 0x2b4c50fa;
   unsigned int i3d = 0x253a3ae3;
   unsigned int i3r = 0x25141df9;
   unsigned int i4n = 0x73addffc;
   unsigned int i4d = 0x742ddffc;
   unsigned int i4r = 0x73addffc;
   unsigned int i5n = 0x29d4d97c;
   unsigned int i5d = 0x2a546e77;
   unsigned int i5r = 0x29d4d97c;

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
   
   vec_float4 x0n_v = vec_splat_float(x0n);
   vec_float4 x0d_v = vec_splat_float(x0d);
   vec_float4 x0r_v = vec_splat_float(x0r);
                                   
   vec_float4 x1n_v = vec_splat_float(x1n);
   vec_float4 x1d_v = vec_splat_float(x1d);
   vec_float4 x1r_v = vec_splat_float(x1r);
                                   
   vec_float4 x2n_v = vec_splat_float(x2n);
   vec_float4 x2d_v = vec_splat_float(x2d);
   vec_float4 x2r_v = vec_splat_float(x2r);
                                   
   vec_float4 x3n_v = vec_splat_float(x3n);
   vec_float4 x3d_v = vec_splat_float(x3d);
   vec_float4 x3r_v = vec_splat_float(x3r);
                                   
   vec_float4 x4n_v = vec_splat_float(x4n);
   vec_float4 x4d_v = vec_splat_float(x4d);
   vec_float4 x4r_v = vec_splat_float(x4r);
                                   
   vec_float4 x5n_v = vec_splat_float(x5n);
   vec_float4 x5d_v = vec_splat_float(x5d);
   vec_float4 x5r_v = vec_splat_float(x5r);
   
   vec_float4 res_v;

   TEST_START("fmodf4");
   res_v = fmodf4(x0n_v, x0d_v);
   TEST_CHECK("20040928191245EJL", allequal_ulps_float4( res_v, x0r_v, 1 ), 0);
   res_v = fmodf4(x1n_v, x1d_v);
   TEST_CHECK("20040928191247EJL", allequal_ulps_float4( res_v, x1r_v, 1 ), 0);
   res_v = fmodf4(x2n_v, x2d_v);
   TEST_CHECK("20040928191249EJL", allequal_ulps_float4( res_v, x2r_v, 1 ), 0);
   res_v = fmodf4(x3n_v, x3d_v);
   TEST_CHECK("20040928191251EJL", allequal_ulps_float4( res_v, x3r_v, 1 ), 0);
   res_v = fmodf4(x4n_v, x4d_v);
   TEST_CHECK("20040928191253EJL", allequal_ulps_float4( res_v, x4r_v, 1 ), 0);
   res_v = fmodf4(x5n_v, x5d_v);
   TEST_CHECK("20040928191255EJL", allequal_ulps_float4( res_v, x5r_v, 1 ), 0);

   TEST_SET_DONE();
   
   TEST_EXIT();
}
