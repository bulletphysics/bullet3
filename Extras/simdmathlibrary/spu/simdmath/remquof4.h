/* remquof4 - 
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
/* remquof4 - This function returns the same vector float result as
 * remainderf4(). In addition a vector signed int is stored in
 * *pquo, that contains the corresponding element values whose sign is
 * the sign of xi / yi and whose magnitude is congruent modulo 2n to
 * the magnitude of the integral quotient of xi / yi, where n is an
 * implementation-defined integer
 * greater than or equal to 3.
 */

#ifndef ___SIMD_MATH_REMQUODF4_H___
#define ___SIMD_MATH_REMQUODF4_H___

#include <simdmath.h>
#include <spu_intrinsics.h>

static inline vector float 
_remquof4(vector float x, vector float y, vector signed int *quo)
{
  vec_int4 n;
  vec_int4 quotient;
  vec_uint4 z, y2, y4;
  vec_uint4 abs_x, abs_y, abs_2x, abs_8y;
  vec_uint4 exp_x, exp_y;
  vec_uint4 zero_x, zero_y;
  //  vec_uint4 logb_x, logb_y;
  vec_uint4 mant_x, mant_y;
  vec_uint4 not_ge, overflow, quo_pos, mask;
  vec_uint4 result, result0, resultx, cnt, sign, bias;
  vec_uint4 sign_mask = spu_splats((unsigned int)0x80000000);
  vec_uint4 implied_1 = spu_splats((unsigned int)0x00800000);
  vec_uint4 mant_mask = spu_splats((unsigned int)0x007FFFFF);

  abs_x = spu_andc((vec_uint4)x, sign_mask);
  abs_y = spu_andc((vec_uint4)y, sign_mask);

  abs_8y = spu_add(abs_y, spu_splats((unsigned int)0x01800000));	/* abs_8y = (2^3) * abs_y */

  sign = spu_and((vec_uint4)x, sign_mask);

  quo_pos = spu_cmpgt((vec_int4)spu_and(spu_xor((vec_uint4)x, (vec_uint4)y), sign_mask), -1);

  /* Compute abs_x = fmodf(abs_x, 8*abs_y). If y is greater than 0.125*SMAX 
   * (SMAX is the maximum representable float), then return abs_x.
   */
  {
    /* Determine ilogb of abs_x and abs_8y and 
     * extract the mantissas (mant_x, mant_y)
     */
    exp_x  = spu_rlmask(abs_x, -23);
    exp_y  = spu_rlmask(abs_8y, -23);

    resultx = spu_or(spu_cmpgt(abs_8y, abs_x), spu_cmpgt(abs_y, spu_splats((unsigned int)0x7E7FFFFF)));

    zero_x = spu_cmpeq(exp_x, 0);
    //    zero_y = spu_cmpeq(exp_y, 0);
    zero_y = spu_cmpgt(implied_1, abs_y );

    //    logb_x = spu_add(exp_x, -127);
    //    logb_y = spu_add(exp_y, -127);

    mant_x = spu_andc(spu_sel(implied_1, abs_x, mant_mask), zero_x);
    mant_y = spu_andc(spu_sel(implied_1, abs_8y, mant_mask), zero_y);

    /* Compute fixed point fmod of mant_x and mant_y. Set the flag,
     * result0, to all ones if we detect that the final result is 
     * ever 0.
     */
    result0 = spu_or(zero_x, zero_y);

    //    n = spu_sub((vec_int4)logb_x, (vec_int4)logb_y);
    n = spu_sub((vec_int4)exp_x, (vec_int4)exp_y);      // (exp_x-127)-(exp_y-127)=exp_x-exp_y
    mask = spu_cmpgt(n, 0);

    while (spu_extract(spu_gather(mask), 0)) {
      z = spu_sub(mant_x, mant_y);

      result0 = spu_or(spu_and(spu_cmpeq(z, 0), mask), result0);
    
      mant_x = spu_sel(mant_x, 
		       spu_sel(spu_add(mant_x, mant_x),
			       spu_add(z, z),
			       spu_cmpgt((vec_int4)z, -1)),
		       mask);

      n = spu_add(n, -1);
      mask = spu_cmpgt(n, 0);
    }

    z = spu_sub(mant_x, mant_y);
    mant_x = spu_sel(mant_x, z, spu_cmpgt((vec_int4)z, -1));

    result0 = spu_or(spu_cmpeq(mant_x, 0), result0);

    /* Convert the result back to floating point and restore
     * the sign. If we flagged the result to be zero (result0),
     * zero it. If we flagged the result to equal its input x,
     * (resultx) then return x.
     */
    cnt = spu_add(spu_cntlz(mant_x), -8);

    // hide hidden bit and shift left side zero
    mant_x = spu_rl(spu_andc(mant_x, implied_1), (vec_int4)cnt);
    
    exp_y = spu_sub(exp_y, cnt);  //adjust exponent
    result0 = spu_orc(result0, spu_cmpgt((vec_int4)exp_y, 0));	/* zero denorm results */
    exp_y = spu_rl(exp_y, 23);

    result = spu_sel(exp_y, mant_x, mant_mask);
    abs_x = spu_sel(spu_andc(result, spu_rlmask(result0, -1)), abs_x, resultx);
    result0 = spu_andc(result0, resultx);
  }

  /* if (x >= 4*y)
   * 	x -= 4*y
   *    quotient = 4
   * else 
   *	quotient = 0
   */
  y4 = spu_andc(spu_add(abs_y, spu_splats((unsigned int)0x01000000)), zero_y);

  overflow = spu_cmpgt(abs_y, spu_splats((unsigned int)0x7EFFFFFF));
  not_ge = spu_or(spu_cmpgt(y4, abs_x), overflow);

  abs_x = spu_sel((vec_uint4)spu_sub((vec_float4)abs_x, (vec_float4)y4), abs_x, not_ge);
  quotient = spu_andc(spu_splats((int)4), (vec_int4)not_ge);

  /* if (x >= 2*y
   *	x -= 2*y
   *    quotient += 2
   */
  y2 = spu_andc(spu_add(abs_y, implied_1), zero_y);
  not_ge = spu_cmpgt(y2, abs_x);
  
  abs_x = spu_sel((vec_uint4)spu_sub((vec_float4)abs_x, (vec_float4)y2), abs_x, not_ge);
  quotient = spu_sel(spu_add(quotient, 2), quotient, not_ge);

  /* if (2*x > y)
   *     x -= y
   *     if (2*x >= y) x -= y
   */
  abs_2x = spu_add(abs_x, implied_1);
  bias = spu_cmpgt(abs_2x, abs_y);
  abs_x = spu_sel(abs_x, (vec_uint4)spu_sub((vec_float4)abs_x, (vec_float4)abs_y), bias);
  quotient = spu_sub(quotient, (vec_int4)bias);

  bias = spu_andc(bias, spu_rlmaska((vec_uint4)spu_msub((vec_float4)abs_x, spu_splats(2.0f), (vec_float4)abs_y), -31));
  abs_x = spu_sel(abs_x, (vec_uint4)spu_sub((vec_float4)abs_x, (vec_float4)abs_y), bias);
  quotient = spu_sub(quotient, (vec_int4)bias);

  /* Generate a correct final sign 
   */
  result = spu_sel(abs_x, ((vec_uint4){0,0,0,0}), result0); // reminder 0
  result = spu_xor(result, sign);                           // set sign

  quotient = spu_and(quotient, 7);
  quotient = spu_sel(spu_sub(0, quotient), quotient, quo_pos);

  *quo = quotient;

  return ((vec_float4)result);
}

#endif
