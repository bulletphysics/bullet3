/* modfd2 - for each of two double slots, compute fractional and integral parts.
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

#ifndef ___SIMD_MATH_MODFD2_H___
#define ___SIMD_MATH_MODFD2_H___

#include <simdmath.h>
#include <spu_intrinsics.h>

#include <simdmath/truncd2.h>

// Returns fractional part and stores integral part in *iptr.

static inline vector double
_modfd2 (vector double x, vector double *iptr)
{
  vec_double2 integral, fraction;
  vec_uint4 iszero;
  vec_uint4 sign = (vec_uint4){0x80000000, 0, 0x80000000, 0};
  vec_uchar16 pattern = (vec_uchar16){4,5,6,7, 0,1,2,3, 12,13,14,15, 8,9,10,11};

  integral = _truncd2( x );

  // if integral is zero, then fraction is x.
  iszero = spu_cmpeq(spu_andc((vec_uint4)integral, sign), 0);
  iszero = spu_and(iszero, spu_shuffle(iszero, iszero, pattern));
  fraction = spu_sel(spu_sub( x, integral ), x, (vec_ullong2)iszero);

  *iptr = integral;
  return fraction;
}

#endif
