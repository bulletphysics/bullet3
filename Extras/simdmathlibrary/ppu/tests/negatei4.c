/* Testcase for negatei4
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
   TEST_SET_START("20040930102649EJL","EJL", "negatei4");
   
   int x0n = hide_int(0);
   int x0p = hide_int(0);
   int x1n = hide_int(-83532);
   int x1p = hide_int(83532);
   int x2n = hide_int(-13152);
   int x2p = hide_int(13152);
   int x3n = hide_int(-1);
   int x3p = hide_int(1);
   
   vec_int4 x0n_v = vec_splat_int(x0n);
   vec_int4 x0p_v = vec_splat_int(x0p);
   vec_int4 x1n_v = vec_splat_int(x1n);
   vec_int4 x1p_v = vec_splat_int(x1p);
   vec_int4 x2n_v = vec_splat_int(x2n);
   vec_int4 x2p_v = vec_splat_int(x2p);
   vec_int4 x3n_v = vec_splat_int(x3n);
   vec_int4 x3p_v = vec_splat_int(x3p);
   
   vec_int4 res_v;

   TEST_START("negatei4");
   res_v = negatei4(x0n_v);
   TEST_CHECK("20040930102652EJL", allequal_int4( res_v, x0p_v ), 0);
   res_v = negatei4(x0p_v);
   TEST_CHECK("20040930102653EJL", allequal_int4( res_v, x0n_v ), 0);
   res_v = negatei4(x1n_v);
   TEST_CHECK("20040930102655EJL", allequal_int4( res_v, x1p_v ), 0);
   res_v = negatei4(x1p_v);
   TEST_CHECK("20040930102657EJL", allequal_int4( res_v, x1n_v ), 0);
   res_v = negatei4(x2n_v);
   TEST_CHECK("20040930102659EJL", allequal_int4( res_v, x2p_v ), 0);
   res_v = negatei4(x2p_v);
   TEST_CHECK("20040930102701EJL", allequal_int4( res_v, x2n_v ), 0);
   res_v = negatei4(x3n_v);
   TEST_CHECK("20040930102703EJL", allequal_int4( res_v, x3p_v ), 0);
   res_v = negatei4(x3p_v);
   TEST_CHECK("20040930102705EJL", allequal_int4( res_v, x3n_v ), 0);
   
   TEST_SET_DONE();
   
   TEST_EXIT();
}
