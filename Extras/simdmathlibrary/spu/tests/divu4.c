/* Test divu4 for SPU
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
   TEST_SET_START("20060901150000MH","MH", "divu4");

   unsigned int x0n = 0xffccb78d;
   unsigned int x0d = 0x0       ;
   unsigned int x0q = 0x0       ;
   unsigned int x0r = 0xffccb78d;
   unsigned int x1n = 0xff978333;
   unsigned int x1d = 0xff976bb6;
   unsigned int x1q = 0x1       ;
   unsigned int x1r = 0x177d    ;
   unsigned int x2n = 0x5e146   ;
   unsigned int x2d = 0xd14ebe0e;
   unsigned int x2q = 0x0       ;
   unsigned int x2r = 0x5e146   ;
   unsigned int x3n = 0xf0e91618;
   unsigned int x3d = 0xfddff7ac;
   unsigned int x3q = 0x0       ;
   unsigned int x3r = 0xf0e91618;
                       
   unsigned int x4n = 0xf2128d9d;
   unsigned int x4d = 0xe0f76   ;
   unsigned int x4q = 0x1137    ;
   unsigned int x4r = 0x66543   ;
   unsigned int x5n = 0xda1ba2ce;
   unsigned int x5d = 0x4c9     ;
   unsigned int x5q = 0x2d9482  ;
   unsigned int x5r = 0xbc      ;
   unsigned int x6n = 0xdd4426a6;
   unsigned int x6d = 0xf8d245cf;
   unsigned int x6q = 0x0       ;
   unsigned int x6r = 0xdd4426a6;
   unsigned int x7n = 0xd1d5ae9 ;
   unsigned int x7d = 0x333ab105;
   unsigned int x7q = 0x0       ;
   unsigned int x7r = 0xd1d5ae9 ;
                       
   unsigned int x8n = 0x3e0c6   ;
   unsigned int x8d = 0xfff24255;
   unsigned int x8q = 0x0       ;
   unsigned int x8r = 0x3e0c6   ;
   unsigned int x9n = 0xfd6fe27e;
   unsigned int x9d = 0xf32454  ;
   unsigned int x9q = 0x10a     ;
   unsigned int x9r = 0xcc2336  ;
   unsigned int x10n =0xfb150f79;
   unsigned int x10d =0xf521    ;
   unsigned int x10q =0x10637   ;
   unsigned int x10r =0x9f62    ;
   unsigned int x11n =0xfe88071f;
   unsigned int x11d =0xfff937c2;
   unsigned int x11q =0x0       ;
   unsigned int x11r =0xfe88071f;

   unsigned int x12n =0xc374fa4 ;
   unsigned int x12d =0x1234    ;
   unsigned int x12q =0xabcd    ;
   unsigned int x12r =0x0       ;
   unsigned int x13n =0xffffffff;
   unsigned int x13d =0x2       ;
   unsigned int x13q =0x7fffffff;
   unsigned int x13r =0x1       ;
   unsigned int x14n =0x0       ;
   unsigned int x14d =0x12345678;
   unsigned int x14q =0x0       ;
   unsigned int x14r =0x0       ;
   unsigned int x15n =0xffffffff;
   unsigned int x15d =0x1       ;
   unsigned int x15q =0xffffffff;
   unsigned int x15r =0x0       ;

   vec_uint4 x0n_v = (vec_uint4){ x0n, x1n, x2n, x3n };
   vec_uint4 x1n_v = (vec_uint4){ x4n, x5n, x6n, x7n };
   vec_uint4 x2n_v = (vec_uint4){ x8n, x9n, x10n, x11n };
   vec_uint4 x3n_v = (vec_uint4){ x12n, x13n, x14n, x15n };

   vec_uint4 x0d_v = (vec_uint4){ x0d, x1d, x2d, x3d };
   vec_uint4 x1d_v = (vec_uint4){ x4d, x5d, x6d, x7d };
   vec_uint4 x2d_v = (vec_uint4){ x8d, x9d, x10d, x11d };
   vec_uint4 x3d_v = (vec_uint4){ x12d, x13d, x14d, x15d };

   vec_uint4 x0q_v = (vec_uint4){ x0q, x1q, x2q, x3q };
   vec_uint4 x1q_v = (vec_uint4){ x4q, x5q, x6q, x7q };
   vec_uint4 x2q_v = (vec_uint4){ x8q, x9q, x10q, x11q };
   vec_uint4 x3q_v = (vec_uint4){ x12q, x13q, x14q, x15q };

   vec_uint4 x0r_v = (vec_uint4){ x0r, x1r, x2r, x3r };
   vec_uint4 x1r_v = (vec_uint4){ x4r, x5r, x6r, x7r };
   vec_uint4 x2r_v = (vec_uint4){ x8r, x9r, x10r, x11r };
   vec_uint4 x3r_v = (vec_uint4){ x12r, x13r, x14r, x15r };
   
   divu4_t res;

   TEST_START("divu4");
   res = divu4(x0n_v, x0d_v);
   TEST_CHECK("20060901150001MH", allequal_uint4( res.quot, x0q_v ) && allequal_uint4( res.rem,  x0r_v ), 0);
   res = divu4(x1n_v, x1d_v);
   TEST_CHECK("20060901150002MH", allequal_uint4( res.quot, x1q_v ) && allequal_uint4( res.rem,  x1r_v ), 0);
   res = divu4(x2n_v, x2d_v);
   TEST_CHECK("20060901150003MH", allequal_uint4( res.quot, x2q_v ) && allequal_uint4( res.rem,  x2r_v ), 0);
   res = divu4(x3n_v, x3d_v);
   TEST_CHECK("20060901150004MH", allequal_uint4( res.quot, x3q_v ) && allequal_uint4( res.rem,  x3r_v ), 0);
   
   TEST_SET_DONE();
   
   TEST_EXIT();
}
