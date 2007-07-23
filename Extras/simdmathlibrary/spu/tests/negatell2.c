/* Test negatell2 for SPU
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
   TEST_SET_START("20060731203500MH","MH", "negatell2");
   
   vec_llong2 x0n_v = spu_splats(0LL);
   vec_llong2 x0p_v = spu_splats(0LL);
   vec_llong2 x1n_v = spu_splats(-83582LL);
   vec_llong2 x1p_v = spu_splats(83582LL);
   vec_llong2 x2n_v = spu_splats(-13152LL);
   vec_llong2 x2p_v = spu_splats(13152LL);
   vec_llong2 x3n_v = spu_splats(-1LL);
   vec_llong2 x3p_v = spu_splats(1LL);
   vec_llong2 x4n_v = spu_splats((long long)0x8000000000000001ULL);
   vec_llong2 x4p_v = spu_splats(0x7fffffffffffffffLL);
   vec_llong2 x5n_v = spu_splats((long long)0x8000000100000000ULL);
   vec_llong2 x5p_v = spu_splats(0x7fffffff00000000LL);
   
   vec_llong2 res_v;

   TEST_START("negatell2");
   res_v = negatell2(x0n_v);
   TEST_CHECK("20060731203501MH", allequal_llong2( res_v, x0p_v ), 0);
   res_v = negatell2(x0p_v);
   TEST_CHECK("20060731203502MH", allequal_llong2( res_v, x0n_v ), 0);
   res_v = negatell2(x1n_v);
   TEST_CHECK("20060731203503MH", allequal_llong2( res_v, x1p_v ), 0);
   res_v = negatell2(x1p_v);
   TEST_CHECK("20060731203504MH", allequal_llong2( res_v, x1n_v ), 0);
   res_v = negatell2(x2n_v);
   TEST_CHECK("20060731203505MH", allequal_llong2( res_v, x2p_v ), 0);
   res_v = negatell2(x2p_v);
   TEST_CHECK("20060731203506MH", allequal_llong2( res_v, x2n_v ), 0);
   res_v = negatell2(x3n_v);
   TEST_CHECK("20060731203507MH", allequal_llong2( res_v, x3p_v ), 0);
   res_v = negatell2(x3p_v);
   TEST_CHECK("20060731203508MH", allequal_llong2( res_v, x3n_v ), 0);
   res_v = negatell2(x4n_v);
   TEST_CHECK("20060731203509MH", allequal_llong2( res_v, x4p_v ), 0);
   res_v = negatell2(x4p_v);
   TEST_CHECK("20060731203510MH", allequal_llong2( res_v, x4n_v ), 0);
   res_v = negatell2(x5n_v);
   TEST_CHECK("20060731203511MH", allequal_llong2( res_v, x5p_v ), 0);
   res_v = negatell2(x5p_v);
   TEST_CHECK("20060731203512MH", allequal_llong2( res_v, x5n_v ), 0);
   
   TEST_SET_DONE();
   
   TEST_EXIT();
}
