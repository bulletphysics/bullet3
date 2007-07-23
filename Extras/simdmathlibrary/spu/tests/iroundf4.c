/* Test iroundf4 for SPU
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

#define DEFINE_DATA(val,a,b)\
  float val = hide_float(a);  \
  signed int val ## _out =b;\
  vec_float4 val ## _v = spu_splats(val);\
  vec_int4 val  ## _out_v = spu_splats(val ## _out);

#define DEFINE_DATA_UNSIGNED(val,a,b)\
  unsigned int  val ## _in = a;\
  float val = make_float(val ## _in);\
  signed int val ## _out = b;\
  vec_float4 val ## _v = spu_splats(val);\
  vec_int4 val  ## _out_v = spu_splats(val ## _out);

#define DO_TEST(var,id)			\
  res_v = iroundf4(var ## _v);			\
  TEST_CHECK(" #id ", allequal_int4( res_v, var ## _out_v ), 0);


int main()
{
   vec_int4  res_v;

   TEST_SET_START("592642590100","RUD", "iroundf4");

   /*  
       Define original values and  the results 
   */
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
     //s=0, e=27, f=0  -> 134217728
     DEFINE_DATA_UNSIGNED(x7, 0x4d000000,134217728)
     //s=0, e=-126, f=0 --> 0
     DEFINE_DATA_UNSIGNED(x8, 0x800000,0)

     /*     TEST   */
   TEST_START("iroundf4");

   DO_TEST(x1,592642590101RUD)
     DO_TEST(x2,592642590102RUD)
     DO_TEST(x3,592642590103RUD)
     DO_TEST(x4,592642590104RUD)
     DO_TEST(x5,592642590105RUD)
     DO_TEST(x6,592642590106RUD)
     DO_TEST(x7,592642590107RUD)
     DO_TEST(x8,592642590108RUD)
     
   TEST_SET_DONE();

   
   TEST_EXIT();
}
