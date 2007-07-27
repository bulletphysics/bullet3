/* exp2f4
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

#ifndef ___SIMD_MATH_EXP2F4_H___
#define ___SIMD_MATH_EXP2F4_H___

#include <simdmath.h>
#include <altivec.h>
#include <math.h>

#include <simdmath/_vec_utils.h>

/*
 * FUNCTION
 *	vector float _exp2_v(vector float x)
 *
 * DESCRIPTION
 *	_exp2_v computes 2 raised to the input vector x. Computation is 
 *	performed by observing the 2^(a+b) = 2^a * 2^b.
 *	We decompose x into a and b (above) by letting.
 *	a = ceil(x), b = x - a; 
 *
 *	2^a is easilty computed by placing a into the exponent
 *	or a floating point number whose mantissa is all zeros.
 *
 *	2^b is computed using the following polynomial approximation.
 *	(C. Hastings, Jr, 1955).
 *
 *                __7__
 *		  \
 *		   \ 
 *	2^(-x) =   /     Ci*x^i
 *                /____
 *                 i=1
 *
 *	for x in the range 0.0 to 1.0
 *
 *	C0 =  1.0
 *	C1 = -0.9999999995
 *	C2 =  0.4999999206
 *	C3 = -0.1666653019
 *	C4 =  0.0416573475
 *	C5 = -0.0083013598
 *	C6 =  0.0013298820
 *	C7 = -0.0001413161
 *
 *	This function does not handle out of range conditions. It
 *	assumes that x is in the range (-128.0, 127.0]. Values outside
 *	this range will produce undefined results.
 */


#define __EXP2F_LN2	0.69314718055995f	/* ln(2) */

static inline vector float
_exp2f4 (vector float x) 
{
  vector signed int ix;
  vector unsigned int overflow;
  vector unsigned int underflow;
  vector float frac, frac2, frac4;
  vector float exp_int, exp_frac;
  vector float result;
  vector float hi, lo;
  vector float zeros = __vec_splatsf4(0.0f);
  vector float bias;
  /* Break in the input x into two parts ceil(x), x - ceil(x).
   */
#if 1
  bias = (vector float)(vec_sra((vector signed int)x, __vec_splatsu4(31) ));
  bias = (vector float)(vec_andc(__vec_splatsu4(0x3F7FFFFF), (vector unsigned int)bias));
  ix = vec_cts(vec_add(x, bias), 0);  
#else
  bias = vec_sel(vec_floor(x), vec_ceil(x), vec_cmpgt(x, __vec_splatsf4(0.0f)));
  ix = vec_cts(bias, 0); 
#endif
  frac = vec_sub(vec_ctf(ix, 0), x);
  frac = vec_madd(frac, __vec_splatsf4(__EXP2F_LN2), zeros);

  overflow  = (vector unsigned int)vec_cmpgt(x, (vector float)(__vec_splatsi4(0x4300FFFF))); // !!! Biggest possible exponent to fit in range.
  underflow = (vector unsigned int)vec_cmpgt(__vec_splatsf4(-126.0f), x);

  exp_int = (vector float)(vec_sl(vec_add(ix, __vec_splatsi4(126)), __vec_splatsu4(23)));   // !!! HRD <- add with saturation

  /* Instruction counts can be reduced if the polynomial was
   * computed entirely from nested (dependent) fma's. However, 
   * to reduce the number of pipeline stalls, the polygon is evaluated 
   * in two halves (hi amd lo). 
   */
  frac2 = vec_madd(frac, frac, zeros);
  frac4 = vec_madd(frac2, frac2, zeros);

  hi = vec_madd(frac, __vec_splatsf4(-0.0001413161), __vec_splatsf4(0.0013298820));
  hi = vec_madd(frac, hi, __vec_splatsf4(-0.0083013598));
  hi = vec_madd(frac, hi, __vec_splatsf4(0.0416573475));
  lo = vec_madd(frac, __vec_splatsf4(-0.1666653019), __vec_splatsf4(0.4999999206));
  lo = vec_madd(frac, lo, __vec_splatsf4(-0.9999999995));
  lo = vec_madd(frac, lo, __vec_splatsf4(1.0));

  exp_frac = vec_madd(frac4, hi, lo);
  result = vec_madd(exp_frac, exp_int, zeros);
  result = vec_madd(exp_frac, exp_int, result); // !!! HRD

  /* Handle overflow */
  result = vec_sel(result, __vec_splatsf4(HUGE_VALF), overflow); 
  result = vec_sel(result, zeros, underflow);

  return (result);
}

#endif
