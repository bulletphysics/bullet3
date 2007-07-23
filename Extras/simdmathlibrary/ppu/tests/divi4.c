/* Testcase for divi4
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
   TEST_SET_START("20040928161739EJL","EJL", "divi4");

   int x0n = 0xffccb78d;
   int x0d = 0x0       ;
   int x0q = 0x0       ;
   int x0r = 0xffccb78d;
   int x1n = 0x0;
   int x1d = 0xff976bb6;
   int x1q = 0x0       ;
   int x1r = 0x0;
   int x2n = 0x0;
   int x2d = 0x0;
   int x2q = 0x0       ;
   int x2r = 0x0;
   int x3n = 0xf0e91618;
   int x3d = 0xfddff7ac;
   int x3q = 0x7       ;
   int x3r = 0xffc95064;
                       
   int x4n = 0xf2128d9d;
   int x4d = 0xe0f76   ;
   int x4q = 0xffffff03;
   int x4r = 0xfff7d53b;
   int x5n = 0xda1ba2ce;
   int x5d = 0x4c9     ;
   int x5q = 0xfff814d3;
   int x5r = 0xfffffd23;
   int x6n = 0xdd4426a6;
   int x6d = 0xf8d245cf;
   int x6q = 0x4       ;
   int x6r = 0xf9fb0f6a;
   int x7n = 0xd1d5ae9 ;
   int x7d = 0x333ab105;
   int x7q = 0x0       ;
   int x7r = 0xd1d5ae9 ;
                       
   int x8n = 0x3e0c6   ;
   int x8d = 0xfff24255;
   int x8q = 0x0       ;
   int x8r = 0x3e0c6   ;
   int x9n = 0xfd6fe27e;
   int x9d = 0xf32454  ;
   int x9q = 0xfffffffe;
   int x9r = 0xff562b26;
   int x10n =0xfb150f79;
   int x10d =0xf521    ;
   int x10q =0xfffffade;
   int x10r =0xffff42db;
   int x11n =0xfe88071f;
   int x11d =0xfff937c2;
   int x11q =0x37      ;
   int x11r =0xfffd0c71;


   vec_int4 x0n_v = (vec_int4){ x0n, x1n, x2n, x3n };
   vec_int4 x1n_v = (vec_int4){ x4n, x5n, x6n, x7n };
   vec_int4 x2n_v = (vec_int4){ x8n, x9n, x10n, x11n };

   vec_int4 x0d_v = (vec_int4){ x0d, x1d, x2d, x3d };
   vec_int4 x1d_v = (vec_int4){ x4d, x5d, x6d, x7d };
   vec_int4 x2d_v = (vec_int4){ x8d, x9d, x10d, x11d };

   vec_int4 x0q_v = (vec_int4){ x0q, x1q, x2q, x3q };
   vec_int4 x1q_v = (vec_int4){ x4q, x5q, x6q, x7q };
   vec_int4 x2q_v = (vec_int4){ x8q, x9q, x10q, x11q };

   vec_int4 x0r_v = (vec_int4){ x0r, x1r, x2r, x3r };
   vec_int4 x1r_v = (vec_int4){ x4r, x5r, x6r, x7r };
   vec_int4 x2r_v = (vec_int4){ x8r, x9r, x10r, x11r };
   
   divi4_t res;

   TEST_START("divi4");
   res = divi4(x0n_v, x0d_v);
   TEST_CHECK("20040928161846EJL", allequal_int4( res.quot, x0q_v ) && allequal_int4( res.rem,  x0r_v ), 0);
   res = divi4(x1n_v, x1d_v);
   TEST_CHECK("20040928161851EJL", allequal_int4( res.quot, x1q_v ) && allequal_int4( res.rem,  x1r_v ), 0);
   res = divi4(x2n_v, x2d_v);
   TEST_CHECK("20040928161855EJL", allequal_int4( res.quot, x2q_v ) && allequal_int4( res.rem,  x2r_v ), 0);
   
   TEST_SET_DONE();
   
   TEST_EXIT();
}
