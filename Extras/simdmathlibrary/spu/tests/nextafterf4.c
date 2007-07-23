/* Test nextafterf4 for SPU
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
#include <float.h>
#include "simdmath.h"
#include "common-test.h"
#include "testutils.h"

//a :float, b: float: c:bit pattern
#define DEFINE_DATA(var,a,b,c)	\
  float var ## _a = a;\
vec_float4 var ## _a_v = spu_splats(var ## _a);	\
  float var ## _b = b;\
vec_float4 var ## _b_v = spu_splats(var ## _b);	\
  unsigned int var ## _inc = c ;		\
  float var ## _out = make_float(var ## _inc);\
  vec_float4 var ## _out_v = spu_splats(var ## _out);\

//a :bit pattern, b: bit pattern: c:bit pattern
#define DEFINE_DATA_UNSIGNED(var,a,b,c)	\
  unsigned int  var ## _ina = a ;		\
  float var ## _a = make_float(var ## _ina);                   \
  vec_float4 var ## _a_v = spu_splats(var ## _a);	\
  unsigned int  var ## _inb = b ;		\
  float var ## _b = make_float(var ## _inb);                  \
  vec_float4 var ## _b_v = spu_splats(var ## _b);	\
  unsigned int var ## _inc = c ;		\
  float var ## _out = make_float(var ## _inc);             \
  vec_float4 var ## _out_v = spu_splats(var ## _out);

#define DO_TEST(var,id)	\
  res_v = nextafterf4(var ## _a_v, var ## _b_v);\
  TEST_CHECK(" #id ", allequal_float4( res_v, var ## _out_v ), 0);


int main()
{
   vec_float4 res_v;

   TEST_SET_START("958726589700","NAR", "nextafterf4");
 


   // ==
   // 1.0f --> 1.0f
   DEFINE_DATA(x1, 1.0f, 1.0f, 0x3f800000)
     DEFINE_DATA(x2, 0.0f, 0.0f, 0x0)
     
     // *  Icrement *

     // -FLT_MAX -> 
     DEFINE_DATA_UNSIGNED(x3,0xffffffff, 0x0, 0xfffffffe)
     //(1, 40, 0) --> (1, 39, 7fffff)
     DEFINE_DATA_UNSIGNED(x4,0xd3800000, 0x0, 0xd37fffff)
     // (1,-40,0 ) --> (1,-41,0x7fffff)
     DEFINE_DATA_UNSIGNED(x5,0xab800000, 0x0, 0xab7fffff)
     //-FLT_MIN --> 0
     DEFINE_DATA_UNSIGNED(x6,0x80800000, 0x0, 0x0)
     //0.0f --> FLT_MIN
     DEFINE_DATA(x7, 0.0f, 1.0f, 0x800000)
     //-0.0f --> FLT_MIN
     DEFINE_DATA_UNSIGNED(x8, 0x80000000,  0x7fffffff, 0x800000)
     //FLT_MIN -->
     DEFINE_DATA_UNSIGNED(x9, 0x800000,  0x7fffffff, 0x800001)
     // (0, -41, 7fffff) --> (0, -40, 0)
     DEFINE_DATA_UNSIGNED(x10, 0x2b7fffff, 0x7fffffff, 0x2b800000)
     // (0, 40, 7fffff) --> (0, 41, 0)
     DEFINE_DATA_UNSIGNED(x11, 0x53ffffff, 0x7fffffff, 0x54000000)
     // FLT_MAX --> 
     DEFINE_DATA_UNSIGNED(x12,0x7fffffff,0x7fffffff,0x7fffffff)
     
     // * Decrement *

     // FLT_MAX --> FLT_MAX
     DEFINE_DATA_UNSIGNED(x13,0x7fffffff,0x7fffffff,0x7fffffff)
     // FLT_MAX --> 
     DEFINE_DATA_UNSIGNED(x14,0x7fffffff,0x0,0x7ffffffe)
     // (0, 41, 0) -->  (0, 40, 7fffff)
     DEFINE_DATA_UNSIGNED(x15,  0x54000000, 0x0, 0x53ffffff)
     // (0, -40, 0) -->  (0, -41, 7fffff)
     DEFINE_DATA_UNSIGNED(x16, 0x2b800000,0x0, 0x2b7fffff)
     // -> FLT_MIN
     DEFINE_DATA_UNSIGNED(x17,  0x800001, 0x800000, 0x800000)
     // FLT_MIN --> 0
     DEFINE_DATA_UNSIGNED(x18, 0x800000, 0x0, 0x0)
     // 0.0 -> -FLT_MIN
     DEFINE_DATA_UNSIGNED(x19, 0x0, 0xffffffff, 0x80800000)
     // -0.0 -> FLT_MIN
     DEFINE_DATA_UNSIGNED(x20, 0x80000000, 0xffffffff, 0x80800000)
     //-FLT_MIN -->
     DEFINE_DATA_UNSIGNED(x21, 0x80800000, 0xffffffff, 0x80800001)
     //  (1,-41,0x7fffff) --> (1,-40,0 ) 
     DEFINE_DATA_UNSIGNED(x22, 0xab7fffff, 0xffffffff, 0xab800000)
     //(1, 40, 0) --> (1, 39, 7fffff)
     DEFINE_DATA_UNSIGNED(x23, 0xd37fffff, 0xffffffff, 0xd3800000)
     // --> -FLT_MAX 
     DEFINE_DATA_UNSIGNED(x24,0xfffffffe, 0xffffffff, 0xffffffff)
     

     //TEST
     TEST_START("nextafterf4");
     DO_TEST(x1,958726589701NAR)
     DO_TEST(x2,958726589702NAR)
     DO_TEST(x3,958726589703NAR)
     DO_TEST(x4,958726589704NAR)
     DO_TEST(x5,958726589705NAR)
     DO_TEST(x6,958726589706NAR)
     DO_TEST(x7,958726589707NAR)
     DO_TEST(x8,958726589708NAR)
     DO_TEST(x9,958726589709NAR)
     DO_TEST(x10,958726589710NAR)
     DO_TEST(x11,958726589711NAR)
     DO_TEST(x12,958726589712NAR)
     DO_TEST(x13,958726589713NAR)
     DO_TEST(x14,958726589714NAR)
     DO_TEST(x15,958726589715NAR)
     DO_TEST(x16,958726589716NAR)
     DO_TEST(x17,958726589717NAR)
     DO_TEST(x18,958726589718NAR)
     DO_TEST(x19,958726589719NAR)
     DO_TEST(x20,958726589720NAR)
     DO_TEST(x21,958726589721NAR)
     DO_TEST(x22,958726589722NAR)
     DO_TEST(x23,958726589723NAR)
     DO_TEST(x24,958726589724NAR)

       TEST_SET_DONE();

   
   TEST_EXIT();
}
