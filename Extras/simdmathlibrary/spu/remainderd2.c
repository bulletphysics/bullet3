/* A vector double is returned that contains the remainder xi REM yi,
        for the corresponding elements of vector double x and vector double y.
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

#include <simdmath.h>
#include <spu_intrinsics.h>


static inline vec_uint4 _sub_d_(vec_uint4 aa, vec_uint4 bb);
static inline vec_uint4 _vec_gt64_half(vec_uint4 aa, vec_uint4 bb);
static inline vec_uint4 _vec_gt64(vec_uint4 aa, vec_uint4 bb);
static inline vec_uint4 _twice(vec_uint4 aa);

vector double 
remainderd2(vector double x, vector double yy)
{
  vec_uchar16 splat_hi   = ((vec_uchar16){ 0,1,2,3,0,1,2,3, 8,9,10,11, 8,9,10,11});
  vec_uint4 y_hi;
  vec_uint4 abs_x, abs_yy, abs_2x, abs_2y;
  vec_uint4 bias;
  vec_uint4 nan_out, overflow;
  vec_uint4 result;
  vec_uint4 half_smax = spu_splats((unsigned int)0x7FEFFFFF);
  vec_uint4 sign_mask = (vec_uint4)(spu_splats(0x8000000000000000ULL));
  vec_uint4 exp_mask  = (vec_uint4)(spu_splats(0x7FF0000000000000ULL));
  vec_uint4 val_nan   = (vec_uint4)(spu_splats(0x7FF8000000000000ULL));
  vec_uint4 vec_zero = spu_splats((unsigned int)0);
  vec_uint4 is_zeroy;

  // cut sign
  abs_x = spu_andc((vec_uint4)x, sign_mask);
  abs_yy = spu_andc((vec_uint4)yy, sign_mask);
  y_hi = spu_shuffle(abs_yy, abs_yy, splat_hi);


  // check nan out
  is_zeroy = spu_cmpeq(abs_yy, vec_zero);
  is_zeroy = spu_and(is_zeroy, spu_rlqwbyte(is_zeroy, 4));
  nan_out = _vec_gt64_half(abs_yy, exp_mask);  // y > 7FF00000
  nan_out = spu_or(nan_out, spu_cmpgt(abs_x, half_smax)); // x >= 7FF0000000000000
  nan_out = spu_or(nan_out, is_zeroy);                    // y = 0
  nan_out = spu_shuffle(nan_out, nan_out, splat_hi);


  // make y x2
  abs_2y = _twice(abs_yy); // 2 x y

  /*
   * use fmodd2 function
   */
  // get remainder of y x2
//  result = (vec_uint4)_fmodd2( x, (vec_double2)abs_2y);
  {
    vec_double2 y = (vec_double2)abs_2y;

    int shiftx0, shiftx1, shifty0, shifty1;
    vec_uchar16 swap_words = ((vec_uchar16){ 4,5,6,7, 0,1,2,3, 12,13,14,15, 8,9,10,11});
    vec_uchar16 propagate = ((vec_uchar16){ 4,5,6,7, 192,192,192,192, 12,13,14,15, 192,192,192,192});
//    vec_uchar16 splat_hi = ((vec_uchar16){ 0,1,2,3,0,1,2,3, 8,9,10,11, 8,9,10,11});
    vec_int4 n, shift;
    vec_uint4 exp_x, exp_y;
//    , sign;
//    vec_uint4 abs_x, abs_y;
    vec_uint4 abs_y;
    vec_uint4 mant_x, mant_x0, mant_x1;
    vec_uint4 mant_y, mant_y0, mant_y1;
    vec_uint4 mant_0, mant_1;
    vec_uint4 mant_r, mant_l;
//    vec_uint4 result;
    vec_uint4 result0, resultx;
    vec_uint4 zero_x, zero_y;
    vec_uint4 denorm_x, denorm_y;
    vec_uint4 cnt, cnt_x, cnt_y;
    vec_uint4 shift_x, shift_y;
    vec_uint4 adj_x, adj_y;
    vec_uint4 z, borrow, mask;
    vec_uint4 lsb       = (vec_uint4)(spu_splats(0x0000000000000001ULL));
//    vec_uint4 sign_mask = (vec_uint4)(spu_splats(0x8000000000000000ULL));
    vec_uint4 implied_1 = (vec_uint4)(spu_splats(0x0010000000000000ULL));
    vec_uint4 mant_mask = (vec_uint4)(spu_splats(0x000FFFFFFFFFFFFFULL));
//    vec_uint4 exp_mask  = (vec_uint4)(spu_splats(0x7FF0000000000000ULL));
    vec_uint4 merge_sel = ((vec_uint4){0,0,-1,-1});
//    vec_uint4 vec_zero = spu_splats((unsigned int)0);

//    sign  = spu_and( (vec_uint4)x, sign_mask);
//    abs_x = spu_andc((vec_uint4)x, sign_mask);
    abs_y = spu_andc((vec_uint4)y, sign_mask);
    exp_x = spu_rlmask(abs_x, -20);
    exp_y = spu_rlmask(abs_y, -20);
    // get shift count for denorm
    cnt_x = spu_cntlz(abs_x);
    cnt_y = spu_cntlz(abs_y);
    cnt_x = spu_add(cnt_x, spu_sel( vec_zero, spu_rlqwbyte(cnt_x, 4), spu_cmpeq(cnt_x, 32)));
    cnt_y = spu_add(cnt_y, spu_sel( vec_zero, spu_rlqwbyte(cnt_y, 4), spu_cmpeq(cnt_y, 32)));

    zero_x = spu_cmpgt(cnt_x, 63);  // zero ?
    zero_y = spu_cmpgt(cnt_y, 63);  // zero ?
    result0 = spu_or(zero_x, zero_y);
    result0 = spu_shuffle(result0, result0, splat_hi);

    // 0 - (cnt_x - 11) = 11 - cnt_x
    shift_x= spu_add(cnt_x, -11);
    shift_y= spu_add(cnt_y, -11);
    cnt_x = spu_sub(11, cnt_x);
    cnt_y = spu_sub(11, cnt_y);

    // count to normalize
    adj_x = spu_sel(spu_add(exp_x, -1), cnt_x, spu_cmpeq(exp_x, 0));
    adj_y = spu_sel(spu_add(exp_y, -1), cnt_y, spu_cmpeq(exp_y, 0));
    adj_x = spu_shuffle(adj_x, adj_x, splat_hi);
    adj_y = spu_shuffle(adj_y, adj_y, splat_hi);

    // for denorm
    shiftx0 = spu_extract(shift_x, 0);
    shiftx1 = spu_extract(shift_x, 2);
    shifty0 = spu_extract(shift_y, 0);
    shifty1 = spu_extract(shift_y, 2);
    mant_x0 = spu_slqwbytebc( spu_slqw(spu_and(abs_x,((vec_uint4){-1,-1,0,0})),shiftx0), shiftx0);
    mant_y0 = spu_slqwbytebc( spu_slqw(spu_and(abs_y,((vec_uint4){-1,-1,0,0})),shifty0), shifty0);
    mant_x1 = spu_slqwbytebc( spu_slqw(abs_x,shiftx1), shiftx1);
    mant_y1 = spu_slqwbytebc( spu_slqw(abs_y,shifty1), shifty1);
    mant_x = spu_sel(mant_x0, mant_x1, merge_sel);
    mant_y = spu_sel(mant_y0, mant_y1, merge_sel);

    denorm_x = spu_cmpgt((vec_int4)vec_zero, (vec_int4)adj_x);
    denorm_y = spu_cmpgt((vec_int4)vec_zero, (vec_int4)adj_y);
    mant_x = spu_sel(spu_and(abs_x, mant_mask), mant_x, denorm_x);
    mant_y = spu_sel(spu_and(abs_y, mant_mask), mant_y, denorm_y);
    mant_x = spu_or(mant_x, implied_1); // hidden bit
    mant_y = spu_or(mant_y, implied_1); // hidden bit

    // x < y ?
    resultx = _vec_gt64(abs_y, abs_x);

    n = spu_sub((vec_int4)adj_x, (vec_int4)adj_y);
    mask = spu_cmpgt(n, 0);
    mask = spu_andc(mask, resultx);

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

    // bring back to original range
    mant_0 = spu_and(mant_x, ((vec_uint4){0x001FFFFF,-1,0,0}));
    mant_1 = spu_and(mant_x, ((vec_uint4){0,0,0x001FFFFF,-1}));

    // for adj_y < 0 exp max=1
    shiftx0 = spu_extract(adj_y, 0);
    shiftx1 = spu_extract(adj_y, 2);
    mant_x0 = spu_rlmaskqwbytebc(spu_rlmaskqw(mant_0, shiftx0), 7 + shiftx0);
    mant_x1 = spu_rlmaskqwbytebc(spu_rlmaskqw(mant_1, shiftx1), 7 + shiftx1);
    mant_r  = spu_sel(mant_x0, mant_x1, merge_sel);

    // for adj_y >= 0
    cnt = spu_cntlz(mant_x);
    cnt = spu_add(cnt, spu_sel( vec_zero, spu_rlqwbyte(cnt, 4), spu_cmpeq(cnt, 32)));
    cnt = spu_add(cnt, -11);
    cnt = spu_sel(vec_zero, cnt, spu_cmpgt(cnt, 0)); // for exp >= 1
    shift = (vec_int4)spu_sel(cnt, adj_y, spu_cmpgt(cnt, adj_y));
    shiftx0 = spu_extract(shift, 0);
    shiftx1 = spu_extract(shift, 2);
    mant_x0 = spu_slqwbytebc(spu_slqw(mant_0, shiftx0), shiftx0);
    mant_x1 = spu_slqwbytebc(spu_slqw(mant_1, shiftx1), shiftx1);
    mant_l  = spu_sel(mant_x0, mant_x1, merge_sel);
    cnt = spu_sub(adj_y, (vec_uint4)shift);
    mant_l = spu_add(mant_l, spu_and(spu_rl(cnt,20), exp_mask));

    result = spu_sel(mant_l, mant_r, denorm_y);
    result = spu_sel(result, vec_zero, result0); // reminder 0
    result = spu_sel(result, abs_x,    resultx); // x < y
//    result = spu_xor(result, sign);              // set sign

//    return ((vec_double2)result);
  }


//  abs_x = spu_sel(spu_andc(result, sign_mask), abs_x, spu_cmpgt(y_hi, spu_splats((unsigned int)0x7FBFFFFF)));
  abs_x = spu_sel(result, abs_x, spu_cmpgt(y_hi, spu_splats((unsigned int)0x7FEFFFFF)));

  /* if (2*x > y)
   *     x -= y
   *     if (2*x >= y) x -= y
   */
  overflow = spu_cmpgt(y_hi, spu_splats((unsigned int)0x7FEFFFFF));
  // make x2
  abs_2x = _twice(abs_x);  // 2 x x

  bias = _vec_gt64(abs_2x, abs_yy);  // abs_2x > abs_yy
  bias = spu_andc(bias, overflow);

  abs_x = spu_sel(abs_x, _sub_d_(abs_x, abs_yy), bias);


  overflow = spu_or(overflow, spu_shuffle(spu_rlmaska(abs_x, -31), vec_zero, splat_hi)); // minous

  // make x2
  abs_2x = _twice(spu_andc(abs_x, sign_mask));  // 2 x x  unsupport minous 
  bias = spu_andc(bias, spu_rlmaska(_sub_d_(abs_2x, abs_yy), -31));
  bias = spu_andc(spu_shuffle(bias, bias, splat_hi), overflow);
  abs_x = spu_sel(abs_x, _sub_d_(abs_x, abs_yy), bias);

  /* select final answer 
   */
  result = spu_xor(abs_x, spu_and((vec_uint4)x, sign_mask)); // set sign
  result = spu_sel(result, val_nan, nan_out); // if nan

  return ((vec_double2)result);
}

/*
 * subtraction function in limited confdition
 */
static inline vec_uint4 _sub_d_(vec_uint4 aa, vec_uint4 bb)
{
  // which is bigger input aa or bb
  vec_uint4 is_bigb = _vec_gt64(bb, aa);  // bb > aa

  // need denorm calc ?
  vec_uint4 norm_a, norm_b;
  norm_a = spu_cmpgt(aa, (vec_uint4)(spu_splats(0x000FFFFFFFFFFFFFULL)));
  norm_b = spu_cmpgt(bb, (vec_uint4)(spu_splats(0x000FFFFFFFFFFFFFULL)));
  norm_a = spu_and(norm_a, norm_b);
  norm_a = spu_shuffle(norm_a, norm_a,((vec_uchar16){ 0,1,2,3,0,1,2,3, 8,9,10,11, 8,9,10,11}));

  // calc (aa - bb) and (bb - aa)
  vec_uint4 res_a, res_b, res;
  vec_uint4 borrow_a, borrow_b;
  vec_uchar16 mask_b = ((vec_uchar16){4,5,6,7,192,192,192,192,12,13,14,15,192,192,192,192});
  borrow_a = spu_genb(aa, bb);
  borrow_b = spu_genb(bb, aa);
  borrow_a = spu_shuffle(borrow_a, borrow_a, mask_b);
  borrow_b = spu_shuffle(borrow_b, borrow_b, mask_b);
  res_a = spu_subx(aa, bb, borrow_a);
  res_b = spu_subx(bb, aa, borrow_b);
  res_b = spu_or(res_b, ((vec_uint4){0x80000000,0,0x80000000,0}));  // set sign

  res = spu_sel(res_a, res_b, is_bigb);  // select (aa - bb) or (bb - aa)
  // select normal calc or special
  res = spu_sel(res, (vec_uint4)spu_sub((vec_double2)aa, (vec_double2)bb), norm_a);

  return res;
}


/*
 * extend spu_cmpgt function to 64bit data
 */
static inline vec_uint4 _vec_gt64_half(vec_uint4 aa, vec_uint4 bb)
{
  vec_uint4 gt = spu_cmpgt(aa, bb);  // aa > bb
  vec_uint4 eq = spu_cmpeq(aa, bb);  // aa = bb
  return spu_or(gt, spu_and(eq, spu_rlqwbyte(gt, 4))); // only higher is right
}
static inline vec_uint4 _vec_gt64(vec_uint4 aa, vec_uint4 bb)
{
  vec_uint4 gt_hi = _vec_gt64_half(aa, bb); // only higher is right
  return spu_shuffle(gt_hi, gt_hi, ((vec_uchar16){ 0,1,2,3,0,1,2,3, 8,9,10,11, 8,9,10,11}));
}

/*
 * double formated x2
 */
static inline vec_uint4 _twice(vec_uint4 aa)
{
  vec_uint4 norm = spu_cmpgt(aa, (vec_uint4)(spu_splats(0x000FFFFFFFFFFFFFULL))); // exp > 0
  norm = spu_shuffle(norm, norm, ((vec_uchar16){ 0,1,2,3,0,1,2,3, 8,9,10,11, 8,9,10,11}));

  // if denorm or zero << 1 , if norm exp + 1
  return spu_sel(spu_slqw(aa, 1), spu_add(aa, (vec_uint4)(spu_splats(0x0010000000000000ULL))), norm); // x2
}
