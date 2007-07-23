/* Test lldivi2 for SPU
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
   TEST_SET_START("20060908152000MH","MH", "lldivi2");

   signed long long x0n = 0x0c0e84c75f216c43ll;
   signed long long x0d = 0x00000000000abcdell;
   signed long long x0q = 0x0000011f71fb04cbll;
   signed long long x0r = 0x0000000000003039ll;
   signed long long x1n = 0x0c0e84c75f216c43ll;
   signed long long x1d = 0x0000011f71fb04cbll;
   signed long long x1q = 0x00000000000abcdell;
   signed long long x1r = 0x0000000000003039ll;

   signed long long x2n = 0x08e732f9d4baf903ll;
   signed long long x2d = 0x0000000000976bb6ll;
   signed long long x2q = 0x0000000f0d55f4d9ll;
   signed long long x2r = 0x00000000004933bdll;
   signed long long x3n = 0x08e732f9d4baf903ll;
   signed long long x3d = 0x0000000f0d55f4d9ll;
   signed long long x3q = 0x0000000000976bb6ll;
   signed long long x3r = 0x00000000004933bdll;

   signed long long x4n = 0xffffffffffffffffll;
   signed long long x4d = 0xfffffffffffffffell;
   signed long long x4q = 0x0000000000000000ll;
   signed long long x4r = 0xffffffffffffffffll;
   signed long long x5n = 0xffffffffffffffffll;
   signed long long x5d = 0x0000000000000001ll;
   signed long long x5q = 0xffffffffffffffffll;
   signed long long x5r = 0x0000000000000000ll;

   signed long long x6n = 0xffffffffffffffffll;
   signed long long x6d = 0x0000000000000002ll;
   signed long long x6q = 0x0000000000000000ll;
   signed long long x6r = 0xffffffffffffffffll;
   signed long long x7n = 0xffffffffffffffffll;
   signed long long x7d = 0x7fffffffffffffffll;
   signed long long x7q = 0x0000000000000000ll;
   signed long long x7r = 0xffffffffffffffffll;

   signed long long x8n = 0xf8c0d45d0ff344f0ll;
   signed long long x8d = 0x000019aa3e41e0bdll;
   signed long long x8q = 0xffffffffffffb7b8ll;
   signed long long x8r = 0xffffedc119afa218ll;
   signed long long x9n = 0xf8c0d45d0ff344f0ll;
   signed long long x9d = 0x000000000009b13bll;
   signed long long x9q = 0xffffff4097efb39ell;
   signed long long x9r = 0xfffffffffff6a186ll;

   vec_llong2 x0n_v = (vec_llong2){ x0n, x1n };
   vec_llong2 x0d_v = (vec_llong2){ x0d, x1d };
   vec_llong2 x0q_v = (vec_llong2){ x0q, x1q };
   vec_llong2 x0r_v = (vec_llong2){ x0r, x1r };
   vec_llong2 x1n_v = (vec_llong2){ x2n, x3n };
   vec_llong2 x1d_v = (vec_llong2){ x2d, x3d };
   vec_llong2 x1q_v = (vec_llong2){ x2q, x3q };
   vec_llong2 x1r_v = (vec_llong2){ x2r, x3r };
   vec_llong2 x2n_v = (vec_llong2){ x4n, x5n };
   vec_llong2 x2d_v = (vec_llong2){ x4d, x5d };
   vec_llong2 x2q_v = (vec_llong2){ x4q, x5q };
   vec_llong2 x2r_v = (vec_llong2){ x4r, x5r };
   vec_llong2 x3n_v = (vec_llong2){ x6n, x7n };
   vec_llong2 x3d_v = (vec_llong2){ x6d, x7d };
   vec_llong2 x3q_v = (vec_llong2){ x6q, x7q };
   vec_llong2 x3r_v = (vec_llong2){ x6r, x7r };
   vec_llong2 x4n_v = (vec_llong2){ x8n, x9n };
   vec_llong2 x4d_v = (vec_llong2){ x8d, x9d };
   vec_llong2 x4q_v = (vec_llong2){ x8q, x9q };
   vec_llong2 x4r_v = (vec_llong2){ x8r, x9r };
   
   lldivi2_t res;

   TEST_START("lldivi2");
   res = lldivi2(x0n_v, x0d_v);
   TEST_CHECK("20060908152001MH", allequal_llong2( res.quot, x0q_v ) && allequal_llong2( res.rem,  x0r_v ), 0);
   res = lldivi2(x1n_v, x1d_v);
   TEST_CHECK("20060908152002MH", allequal_llong2( res.quot, x1q_v ) && allequal_llong2( res.rem,  x1r_v ), 0);
   res = lldivi2(x2n_v, x2d_v);
   TEST_CHECK("20060908152003MH", allequal_llong2( res.quot, x2q_v ) && allequal_llong2( res.rem,  x2r_v ), 0);
   res = lldivi2(x3n_v, x3d_v);
   TEST_CHECK("20060908152004MH", allequal_llong2( res.quot, x3q_v ) && allequal_llong2( res.rem,  x3r_v ), 0);
   res = lldivi2(x4n_v, x4d_v);
   TEST_CHECK("20060908152005MH", allequal_llong2( res.quot, x4q_v ) && allequal_llong2( res.rem,  x4r_v ), 0);
   
   TEST_SET_DONE();
   
   TEST_EXIT();
}
