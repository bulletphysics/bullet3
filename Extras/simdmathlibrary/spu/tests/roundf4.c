/* Test roundf4 for SPU
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

// a: float b:float
#define DEFINE_DATA(var,a,b)		\
  float var = hide_float(a);			\
  float var ## _out = hide_float(b);		\
  vec_float4 var ## _v = spu_splats(var);		\
  vec_float4 var  ## _out_v = spu_splats(var ## _out);

// a: bit pattern     b: bit pattern
#define DEFINE_DATA_UNSIGNED(var,a,b)	\
  unsigned int  var ## _ina = a ;		\
  unsigned int var ## _inb = b ;		\
  float var = make_float (var ## _ina);	\
  float var ## _out = make_float(var ## _inb);	\
  vec_float4 var ## _v = spu_splats(var);	\
  vec_float4 var  ## _out_v = spu_splats(var ## _out);

#define DO_TEST(var,id)			\
  res_v = roundf4(var ## _v);			\
  TEST_CHECK(" #id ", allequal_float4( res_v, var ## _out_v ), 0);


int main()
{
   vec_float4 res_v;

   TEST_SET_START("164260798500","RUD", "roundf4");
 

 
   //s=0
   DEFINE_DATA(x1, 1.0, 1.0f)
     DEFINE_DATA(x2, -1.0,-1.0f)
     //s=-1
     DEFINE_DATA(x3, 0.5, 1.0f)
     DEFINE_DATA(x4, -0.5, -1.0f)
     //s=-2
     DEFINE_DATA(x5, 0.25, 0.0f)
     //s=-3
     DEFINE_DATA(x6, 0.125, 0.0f)
     //s=0, e=128, f=7fffff  --> s=0, e=128, f=7fffff 
     DEFINE_DATA_UNSIGNED(x7,0x7fffffff,0x7fffffff)
     //s=0, e=-126, f=0 --> 0
     DEFINE_DATA_UNSIGNED(x8, 0x800000,0x0)
     DEFINE_DATA(x9, 0.4999, 0.f)
     DEFINE_DATA(x10, 0.9999, 1.f)

     //TEST
     TEST_START("roundf4");
     DO_TEST(x1,164260798501RUD)
     DO_TEST(x2,164260798502RUD)
     DO_TEST(x3,164260798503RUD)
     DO_TEST(x4,164260798504RUD)
     DO_TEST(x5,164260798505RUD)
     DO_TEST(x6,164260798506RUD)
     DO_TEST(x7,164260798507RUD)
     DO_TEST(x8,164260798508RUD)
     DO_TEST(x9,164260798509RUD)
     DO_TEST(x10,164260798510RUD)
     TEST_SET_DONE();

   
   TEST_EXIT();
}
