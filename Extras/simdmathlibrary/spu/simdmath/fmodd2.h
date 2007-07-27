/* fmodd2 - 
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

#ifndef ___SIMD_MATH_FMODD2_H___
#define ___SIMD_MATH_FMODD2_H___

#include <simdmath.h>
#include <spu_intrinsics.h>

#include <simdmath/_vec_utils.h>

/* 
 * a vector is returned that contains the remainder of xi/yi,
 * for coresponding elements of vector double x and vector double y,
 * as described below:
 * if yi is 0, the result is 0
 * otherwise, the funciton determines the unique signed integer value i
 * such that the returned element is xi - i * yi with the same sign  as xi and
 * magnitude less than |yi|
 */

static inline vector double
_fmodd2(vector double x, vector double y)
{
  int shift0, shift1;
  vec_uchar16 swap_words = (vec_uchar16){4,5,6,7, 0,1,2,3, 12,13,14,15, 8,9,10,11};
  vec_uchar16 propagate = (vec_uchar16){4,5,6,7, 192,192,192,192, 12,13,14,15, 192,192,192,192};
  vec_uchar16 splat_hi = (vec_uchar16){0,1,2,3,0,1,2,3, 8,9,10,11, 8,9,10,11};
  vec_uchar16 merge = (vec_uchar16){8,9,10,11,12,13,14,15, 24,25,26,27,28,29,30,31};
  vec_int4 n, shift, power;
  vec_uint4 z;
  vec_uint4 x_hi, y_hi;
  vec_uint4 abs_x, abs_y;
  vec_uint4 exp_x, exp_y;
  vec_uint4 zero_x, zero_y;
  vec_uint4 mant_x, mant_x0, mant_x1, mant_y ;
  vec_uint4 norm, denorm, norm0, norm1, denorm0, denorm1;
  vec_uint4 result, result0, resultx, cnt, sign, borrow, mask;
  vec_uint4 x_7ff, x_inf, x_nan, y_7ff, y_inf, y_nan,  is_normal;
  vec_uint4  x_is_norm, y_is_norm, frac_x, frac_y, cnt_x, cnt_y, mant_x_norm, mant_y_norm;
  vec_uint4  mant_x_denorm0, mant_x_denorm1, mant_x_denorm;
  vec_uint4 mant_y_denorm0, mant_y_denorm1, mant_y_denorm;
  vec_uint4 lsb = (vec_uint4)(spu_splats(0x0000000000000001ULL));
  vec_uint4 sign_mask = (vec_uint4)(spu_splats(0x8000000000000000ULL));
  vec_uint4 implied_1 = (vec_uint4)(spu_splats(0x0010000000000000ULL));
  vec_uint4 mant_mask = (vec_uint4)(spu_splats(0x000FFFFFFFFFFFFFULL));

  sign = spu_and((vec_uint4)x, sign_mask);

  abs_x = spu_andc((vec_uint4)x, sign_mask);
  abs_y = spu_andc((vec_uint4)y, sign_mask);

  x_hi = spu_shuffle(abs_x, abs_x, splat_hi);
  y_hi = spu_shuffle(abs_y, abs_y, splat_hi);

  exp_x  = spu_rlmask(x_hi, -20);
  exp_y  = spu_rlmask(y_hi, -20);

  // y>x
  resultx = __vec_gt64(abs_y, abs_x);

  //is Inf,  is Nan
  x_7ff = spu_cmpgt(x_hi, spu_splats((unsigned int)0x7fefffff));
  x_inf = __vec_eq64_half(abs_x, ((vec_uint4){0x7ff00000,0x0,0x7ff00000,0x0}));
  x_nan = spu_andc(x_7ff,  x_inf);

  y_7ff = spu_cmpgt(y_hi, spu_splats((unsigned int)0x7fefffff));
  y_inf = __vec_eq64_half(abs_y, ((vec_uint4){0x7ff00000,0x0,0x7ff00000,0x0}));
  y_nan = spu_andc(y_7ff,  y_inf);
  
  // is zero
  zero_x = __vec_eq64_half(abs_x, spu_splats((unsigned int)0x0));
  zero_y = __vec_eq64_half(abs_y, spu_splats((unsigned int)0x0));


  /* Determine ilogb of abs_x and abs_y and 
   * extract the mantissas (mant_x, mant_y)
   */
  /* change form*/
  // 0 -> ! is_normal
  // 0 don't care  (because (x=0, y!=0)match x<y, (x!=0 && y=0)match y=0,  (x==0 && y==0) resultx)

  x_is_norm = spu_cmpgt(x_hi, spu_splats((unsigned int)0x000fffff));
  y_is_norm = spu_cmpgt(y_hi, spu_splats((unsigned int)0x000fffff));

  frac_x = spu_and((vec_uint4)x, mant_mask);
  frac_y = spu_and((vec_uint4)y, mant_mask);

  //cntlz(use when denorm)
  cnt_x = spu_cntlz(frac_x);
  cnt_x = spu_add(cnt_x, spu_and(spu_rlqwbyte(cnt_x, 4), spu_cmpeq(cnt_x, 32)));
  cnt_x = spu_add(spu_shuffle(cnt_x, cnt_x, splat_hi), -11);

  cnt_y = spu_cntlz(frac_y);
  cnt_y = spu_add(cnt_y, spu_and(spu_rlqwbyte(cnt_y, 4), spu_cmpeq(cnt_y, 32)));
  cnt_y = spu_add(spu_shuffle(cnt_y, cnt_y, splat_hi), -11);

  /*
    mant_x_norm = spu_andc(spu_sel(implied_1, abs_x, mant_mask), zero_x);
    mant_y_norm = spu_andc(spu_sel(implied_1, abs_y, mant_mask), zero_y);
  */
  //norm
  mant_x_norm = spu_or(implied_1, frac_x);
  mant_y_norm = spu_or(implied_1, frac_y);

  //denorm
  shift0 = spu_extract(cnt_x, 0);
  shift1 = spu_extract(cnt_x, 2);
  mant_x_denorm0 = spu_rlmaskqwbyte((vec_uint4)frac_x, -8);
  mant_x_denorm1 = spu_and((vec_uint4)frac_x, ((vec_uint4){0x0,0x0,-1,-1}));
  mant_x_denorm0 = spu_slqwbytebc(spu_slqw(mant_x_denorm0, shift0), shift0);
  mant_x_denorm1 = spu_slqwbytebc(spu_slqw(mant_x_denorm1, shift1), shift1);
  mant_x_denorm = spu_shuffle(mant_x_denorm0, mant_x_denorm1, merge);
  
  //  vec_int4 shift_y = (vec_int4)spu_sub(cnt_y, spu_splats((unsigned int)11));
  shift0 = spu_extract(cnt_y, 0);
  shift1 = spu_extract(cnt_y, 2);
  mant_y_denorm0 = spu_rlmaskqwbyte((vec_uint4)frac_y, -8);
  mant_y_denorm1 = spu_and((vec_uint4)frac_y, ((vec_uint4){0x0,0x0,-1,-1}));

  mant_y_denorm0 = spu_slqwbytebc(spu_slqw(mant_y_denorm0, shift0), shift0);
  mant_y_denorm1 = spu_slqwbytebc(spu_slqw(mant_y_denorm1, shift1), shift1);
  mant_y_denorm = spu_shuffle(mant_y_denorm0, mant_y_denorm1, merge);

  // mant_x, mant_y( norm | denorm )
  mant_x = spu_sel(mant_x_denorm, mant_x_norm, x_is_norm);
  mant_y = spu_sel(mant_y_denorm, mant_y_norm, y_is_norm);

  /*  power
   */
  vec_int4 power_x_norm = (vec_int4)exp_x;
  vec_int4 power_x_denorm = spu_sub(spu_splats((int)1), (vec_int4)cnt_x);
  vec_int4 power_x =   spu_sel(power_x_denorm, power_x_norm, x_is_norm);

  vec_int4 power_y_norm = (vec_int4)exp_y;
  vec_int4 power_y_denorm = spu_sub(spu_splats((int)1), (vec_int4)cnt_y);
  vec_int4 power_y =   spu_sel(power_y_denorm, power_y_norm, y_is_norm);


  /* Compute fixed point fmod of mant_x and mant_y. Set the flag,
   * result0, to all ones if we detect that the final result is 
   * ever 0.
   */
  result0 = spu_or(zero_x, zero_y);

  //  n = spu_sub((vec_int4)logb_x, (vec_int4)logb_y); //zhao--
  n = spu_sub(power_x, power_y);
  mask = spu_cmpgt(n, 0);

  while (spu_extract(spu_gather(mask), 0)) {
    borrow = spu_genb(mant_x, mant_y);
    borrow = spu_shuffle(borrow, borrow, propagate);
    z = spu_subx(mant_x, mant_y, borrow);

    result0 = spu_or(spu_and(spu_cmpeq(spu_or(z, spu_shuffle(z, z, swap_words)), 0), mask), result0);
    
    mant_x = spu_sel(mant_x, 
		     spu_sel(spu_slqw(mant_x, 1), spu_andc(spu_slqw(z, 1), lsb), spu_cmpgt((vec_int4)spu_shuffle(z, z, splat_hi), -1)),
		     mask);

    n = spu_add(n, -1);														   
    mask = spu_cmpgt(n, 0);
    
  }

  borrow = spu_genb(mant_x, mant_y);
  borrow = spu_shuffle(borrow, borrow, propagate);
  z = spu_subx(mant_x, mant_y, borrow);
  mant_x = spu_sel(mant_x, z, spu_cmpgt((vec_int4)spu_shuffle(z, z, splat_hi), -1));

  result0 = spu_or(spu_cmpeq(spu_or(mant_x, spu_shuffle(mant_x, mant_x, swap_words)), 0), result0);



  /* Convert the result back to floating point and restore
   * the sign. If we flagged the result to be zero (result0),
   * zero it. If we flagged the result to equal its input x,
   * (resultx) then return x.
   *
   * Double precision generates a denorm for an output.
   */

  //  normal = spu_cmpgt((vec_int4)exp_y, 0);//zhao--

  cnt = spu_cntlz(mant_x);
  cnt = spu_add(cnt, spu_and(spu_rlqwbyte(cnt, 4), spu_cmpeq(cnt, 32)));
  cnt = spu_add(spu_shuffle(cnt, cnt, splat_hi), -11);

  mant_x0 = spu_rlmaskqwbyte(mant_x, -8);
  mant_x1 = spu_and(mant_x,((vec_uint4){0x0,0x0,-1,-1}));

  power =spu_sub(power_y, (vec_int4)cnt);
  
  is_normal = spu_cmpgt(power, 0);



  //norm

  shift0 = spu_extract(cnt, 0);			    
  shift1 = spu_extract(cnt, 2);			    
  /*
    norm0 = spu_slqwbytebc(spu_slqw(spu_andc(mant_x0, implied_1), shift0), shift0);
    norm1 = spu_slqwbytebc(spu_slqw(spu_andc(mant_x1, implied_1), shift1), shift1);
  */
  norm0 = spu_slqwbytebc(spu_slqw(mant_x0, shift0), shift0);
  norm1 = spu_slqwbytebc(spu_slqw(mant_x1, shift1), shift1);

  norm   = spu_shuffle(norm0, norm1, merge);

  
  //denorm
  /*
    shift = spu_add((vec_int4)exp_y, -1);
    shift0 = spu_extract(shift, 0);
    shift1 = spu_extract(shift, 2);
    denorm0 = spu_slqwbytebc(spu_slqw(mant_x0, shift0), shift0);
    denorm1 = spu_slqwbytebc(spu_slqw(mant_x1, shift1), shift1);
  */
  shift = spu_add(power, -1);
  shift0 = spu_extract(shift, 0);
  shift1 = spu_extract(shift, 2);

  //  printf("result denorm: shift0=%d, shift1=%d\n",shift0, shift1);

  denorm0 = spu_rlmaskqwbytebc(spu_rlmaskqw(norm0, shift0), 7+shift0);
  denorm1 = spu_rlmaskqwbytebc(spu_rlmaskqw(norm1, shift1), 7+shift1);

  denorm = spu_shuffle(denorm0, denorm1, merge);
  
  // merge
  mant_x = spu_sel(denorm, norm, is_normal);

  exp_y = (vec_uint4)power;
  exp_y = spu_and(spu_rl(exp_y, 20), is_normal);

  result = spu_sel(exp_y, spu_or(sign, mant_x),((vec_uint4){0x800FFFFF, -1, 0x800FFFFF, -1}));

  //y>x  || y<=x
  result = spu_sel(spu_andc(result, spu_rlmask(result0, -1)),
		   (vec_uint4)x, resultx);
  //y=+-inf  => 0
  result = spu_sel(result, (vec_uint4)x, y_inf);
  //x=+-inf  => NaN
  result = spu_sel(result, ((vec_uint4){0x7ff80000, 0x0, 0x7ff80000, 0x0}), x_inf);
  //y=0          =>  0
  result = spu_andc(result, zero_y);

  //x=NaN or y=NaN   => 0
  result = spu_sel(result, (vec_uint4)x, x_nan);
  result = spu_sel(result, (vec_uint4)y, y_nan);

  return ((vec_double2)result);
}

#endif
