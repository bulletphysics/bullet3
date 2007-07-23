/* Test absi4 for SPU
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
   TEST_SET_START("20040908101807EJL","EJL", "abs");
   
   int x0n = hide_int(0);
   int x0p = hide_int(0);
   int x1n = hide_int(-1);
   int x1p = hide_int(1);
   int x2n = hide_int(-83532);
   int x2p = hide_int(83532);

   vec_int4 x0n_v = spu_splats(x0n);
   vec_int4 x0p_v = spu_splats(x0p);
   vec_int4 x1n_v = spu_splats(x1n);
   vec_int4 x1p_v = spu_splats(x1p);
   vec_int4 x2n_v = spu_splats(x2n);
   vec_int4 x2p_v = spu_splats(x2p);
   
   int res;
   vec_int4 res_v;

   TEST_START("absi4");
   res_v = absi4(x0n_v);
   TEST_CHECK("20040908103824EJL", allequal_int4( res_v, x0p_v ), 0);
   res_v = absi4(x0p_v);
   TEST_CHECK("20040908103903EJL", allequal_int4( res_v, x0p_v ), 0);
   res_v = absi4(x1n_v);
   TEST_CHECK("20040908103905EJL", allequal_int4( res_v, x1p_v ), 0);
   res_v = absi4(x1p_v);
   TEST_CHECK("20040908114003EJL", allequal_int4( res_v, x1p_v ), 0);
   res_v = absi4(x2n_v);
   TEST_CHECK("20040908114714EJL", allequal_int4( res_v, x2p_v ), 0);
   res_v = absi4(x2p_v);
   TEST_CHECK("20040908114715EJL", allequal_int4( res_v, x2p_v ), 0);
   
   TEST_START("abs");
   res = abs(x0n);
   TEST_CHECK("20040908114718EJL", res == x0p, 0);
   res = abs(x0p);
   TEST_CHECK("20040908114719EJL", res == x0p, 0);
   res = abs(x1n);
   TEST_CHECK("20040908114720EJL", res == x1p, 0);
   res = abs(x1p);
   TEST_CHECK("20040908114721EJL", res == x1p, 0);
   res = abs(x2n);
   TEST_CHECK("20040908114722EJL", res == x2p, 0);
   res = abs(x2p);                  
   TEST_CHECK("20040908114723EJL", res == x2p, 0);
   
   TEST_SET_DONE();
   
   TEST_EXIT();
}
