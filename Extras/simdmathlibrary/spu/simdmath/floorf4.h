/* floorf4 - for each of four float slots, round down to largest integer not greater than the value.
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

#ifndef ___SIMD_MATH_FLOORF4_H___
#define ___SIMD_MATH_FLOORF4_H___

#include <simdmath.h>
#include <spu_intrinsics.h>

static inline vector float
_floorf4 (vector float x)
{
  vec_int4   xi, xi1;
  vec_uint4  inrange;
  vec_float4 truncated, truncated1;
    
  // Find truncated value and one less.

  inrange = spu_cmpabsgt( (vec_float4)spu_splats(0x4b000000), x );

  xi = spu_convts( x, 0 );
  xi1 = spu_add( xi, -1 );

  truncated = spu_sel( x, spu_convtf( xi, 0 ), inrange );
  truncated1 = spu_sel( x, spu_convtf( xi1, 0 ), inrange );

  // If truncated value is greater than input, subtract one.

  return spu_sel( truncated, truncated1, spu_cmpgt( truncated, x ) );
}

#endif
