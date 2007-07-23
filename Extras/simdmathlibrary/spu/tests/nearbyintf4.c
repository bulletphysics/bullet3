/* Test nearbyintf4 for SPU
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

#define DEFINE_DATA(var,a,b)		\
  float var = hide_float(a);			\
  float var ## _out = hide_float(b);		\
  vec_float4 var ## _v = spu_splats(var);		\
  vec_float4 var  ## _out_v = spu_splats(var ## _out);

/*
 */
#define DEFINE_DATA_UNSIGNED(var,a,b)\
  unsigned int  var ## _ina = a ;		\
  unsigned int var ## _inb = b ;		\
  float var = make_float(var ## _ina);	\
  float var ## _out = make_float(var ## _inb);	\
  vec_float4 var ## _v = spu_splats(var);	\
  vec_float4 var  ## _out_v = spu_splats(var ## _out);

#define DO_TEST(var,id)			\
  res_v = nearbyintf4(var ## _v);			\
  TEST_CHECK(" #id ", allequal_float4( res_v, var ## _out_v ), 0);


int main()
{
   vec_float4 res_v;

   TEST_SET_START("625018616200","NBI", "nearbyintf4");




   //s=0, e=100, f=7fffff  --> s=0, e=100, f=7fffff
   DEFINE_DATA_UNSIGNED(x1,0x71ffffff,0x71ffffff)
     //s=0, e=22, f=0x7fffff  --> s=0,e=22,f=0x7ffffe     
     DEFINE_DATA_UNSIGNED(x2, 0x4affffff,0x4afffffe)
     //s=0, e=23, f=0  --> s=0,e=23,f=0
     DEFINE_DATA_UNSIGNED(x3, 0x4b000000,0x4b000000)
     //s=0, e=-126, f=0 --> 0
     DEFINE_DATA_UNSIGNED(x4, 0x800000,0x0)
     DEFINE_DATA(x5, 1.001f, 1.f)
     DEFINE_DATA(x6, -.05f, 0.f)
     DEFINE_DATA(x7, 0.9999f, 0.f)
     DEFINE_DATA(x8, 0.4999f, 0.f)

     TEST_START("nearbyintf4");  
   DO_TEST(x1,625018616201NBI)
     DO_TEST(x2,625018616202NBI)
     DO_TEST(x3,625018616203NBI)
     DO_TEST(x4,625018616204NBI)
     DO_TEST(x5,625018616205NBI)
     DO_TEST(x6,625018616206NBI)
     DO_TEST(x7,625018616207NBI)
     DO_TEST(x8,625018616208NBI)
     TEST_SET_DONE();
   
   
   TEST_EXIT();
}
