/* ldexpd2 - Multiply Double by 2 Raised to its Power
             For large elements of ex (overflow), returns HUGE_VALF
             For small elements of ex (underflow), returns 0.
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

#ifndef ___SIMD_MATH_LDEXPD2_H___
#define ___SIMD_MATH_LDEXPD2_H___

#include <simdmath.h>
#include <spu_intrinsics.h>

static inline vector double 
_ldexpd2(vector double x, vector signed long long ex)
{
  vec_int4 e1, e2;
  vec_int4 min = spu_splats(-2099);
  //  vec_int4 min = spu_splats(-2044);
  vec_int4 max = spu_splats( 2098);
  //  vec_int4 max = spu_splats( 2046);
  vec_uint4 cmp_min, cmp_max;
  vec_uint4 shift = ((vec_uint4){20, 32, 20, 32});
  vec_double2 f1, f2;
  vec_double2 out;
  vec_double2 in = x;
  vec_int4 exp_in;

  // check input data range
  vec_int4 exp0 = spu_shuffle( (vec_int4)ex, (vec_int4)ex, ((vec_uchar16){4,5,6,7, 4,5,6,7, 12,13,14,15, 12,13,14,15}));
  vec_int4 dmy  = spu_shuffle( (vec_int4)spu_splats(0x10000), (vec_int4)ex, ((vec_uchar16){16,1,2,3, 16,1,2,3, 24,1,2,3,24,1,2,3}));
  // (-)0xFFFFFFFF80000000 or (+)0x000000007FFFFFFF
  vec_int4 msk_range = ((vec_int4){0,0x80000000, 0,0x80000000});
  vec_int4 inrange = spu_addx( (vec_int4)ex, msk_range, spu_rlqwbyte(spu_genc((vec_int4)ex, msk_range), 4));
  inrange = (vec_int4)spu_cmpeq( inrange, 0 );
  inrange = spu_shuffle(inrange,inrange,((vec_uchar16){0,1,2,3,0,1,2,3,8,9,10,11,8,9,10,11}));

  // select dummy over ranged data or input data
  vec_int4 exp = spu_sel( dmy, exp0, (vec_uint4)inrange);
  exp_in = exp;
  /* Clamp the specified exponent to the range -2044 to 2046.
   */
  cmp_min = spu_cmpgt(exp, min);
  cmp_max = spu_cmpgt(exp, max);
  exp = spu_sel(min, exp, cmp_min);
  exp = spu_sel(exp, max, cmp_max);

  /* Generate the factors f1 = 2^e1 and f2 = 2^e2
   */
  e1 = spu_rlmaska(exp, -1);
  e2 = spu_sub(exp, e1);

  f1 = (vec_double2)spu_sl(spu_add(e1, 1023), shift);

  vec_double2 otmp = spu_mul(x, f1);
  vec_uint4 fpscr1 = spu_mffpscr();

  f2 = (vec_double2)spu_sl(spu_add(e2, 1023), shift);

  out = spu_mul(otmp, f2);
  vec_uint4 fpscr2 = spu_mffpscr();

  /* Compute the product x * 2^e1 * 2^e2
   */
  //  out = spu_mul(spu_mul(x, f1), f2);

  // check floating point register DENORM bit
  vec_uint4 fpscr0, fpscr;
  fpscr0 = spu_or(fpscr1, fpscr2);
  fpscr = spu_shuffle(fpscr0, fpscr0, ((vec_uchar16){0x80,0x80,0x80,0x80,0x80,0x80,10,0x80,0x80,0x80,6,0x80,0x80,0x80,0x80,0x80}));
  fpscr = spu_or(fpscr0, fpscr);
  if ( __builtin_expect(spu_extract(fpscr, 1) == 0, 1) ) return out;


  //////////////////////
  // Denormalized calc//
  //////////////////////

  vec_uchar16 splat_msb = { 0,0,0,0,0,0,0,0, 8,8,8,8,8,8,8,8};
  vec_uint4 signmask = ((vec_uint4){0x80000000,0,0x80000000,0});
  vec_int4 zeros = spu_splats(0);
  vec_uchar16 msk_64_eq = ((vec_uchar16){4,5,6,7,0,1,2,3,12,13,14,15,8,9,10,11});

  //check input was zero
  vec_uint4 x_body = spu_and( (vec_uint4)x, ((vec_uint4){0x7FFFFFFF,-1,0x7FFFFFFF,-1}));
  vec_uint4 x_zero = spu_cmpeq( x_body, (vec_uint4)zeros );
  x_zero = spu_and( x_zero, spu_shuffle(x_zero,x_zero,msk_64_eq));

  // check Denormalized input
  vec_int4 cnt_zero = (vec_int4)spu_cntlz(x_body);
  vec_uint4 is_den = (vec_uint4)spu_cmpgt(cnt_zero, 11);  // Denormalized data 000XXXXX XXXXXXXX
  is_den = spu_shuffle( is_den, is_den, splat_msb);
  is_den = spu_sel(is_den, (vec_uint4)zeros, x_zero);  // exclude zero from denormalized

  // count 0bits for 64bit
  vec_uint4 cnt_ex = (vec_uint4)spu_cmpgt(cnt_zero, 31);  // Denormalized data 00000000 XXXXXXXX
  vec_int4 cnt_z = spu_shuffle( cnt_zero, cnt_zero, ((vec_uchar16){4,5,6,7,0,1,2,3,12,13,14,15,8,9,10,11}));
  cnt_zero = spu_add(cnt_zero, spu_sel(zeros, cnt_z, cnt_ex));
  cnt_zero = spu_shuffle(cnt_zero, cnt_zero, ((vec_uchar16){0,1,2,3,0,1,2,3,8,9,10,11,8,9,10,11}));

  // extract each 64bit data
  x_body = spu_and( (vec_uint4)x, ((vec_uint4){0x000FFFFF,-1,0x000FFFFF,-1}));
  vec_uint4 mant0 = spu_shuffle(x_body, x_body, ((vec_uchar16){0,1, 2, 3, 4, 5, 6, 7,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80}));
  vec_uint4 mant1 = spu_shuffle(x_body, x_body, ((vec_uchar16){8,9,10,11,12,13,14,15,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80}));
  vec_uint4 sign = (vec_uint4)spu_rlmaska((vec_int4)exp_in, -31);
  sign = spu_shuffle(sign, sign, splat_msb);

  // set max shift count
  vec_int4 sht = spu_add( cnt_zero, ((vec_int4){-11,-64,-11,-64}));

  // denorm & exp+ shift left
  vec_uint4 cmp = spu_cmpgt( sht, exp_in);
  vec_int4 sht_l = spu_sel(sht, exp_in, cmp);
  int shtl0 = spu_extract(sht_l, 0);
  int shtl1 = spu_extract(sht_l, 2);
  vec_uint4 mant0l = spu_slqwbytebc( spu_slqw(mant0, shtl0), shtl0 );
  vec_uint4 mant1l = spu_slqwbytebc( spu_slqw(mant1, shtl1), shtl1 );
  vec_int4 expp = spu_shuffle(spu_sub(exp_in, sht_l), zeros, ((vec_uchar16){0,1,2,3,0,1,2,3,8,9,10,11,8,9,10,11}));

  exp0 = spu_sel( expp, exp_in, sign );   // select plus or minus caluc
  vec_uint4 mantl = spu_shuffle( mant0l, mant1l, ((vec_uchar16){0,1,2,3,4,5,6,7,16,17,18,19,20,21,22,23}));
  vec_uint4 mant  = spu_sel( mantl, (vec_uint4)x, sign);
  exp  = spu_sel( exp_in, exp0, is_den );  // select denormalized
  x = (vec_double2)spu_sel( (vec_uint4)x, mant, is_den);


  //////////////////////////////////////////////////////////////////////////
  // from ldexpf4
  vec_int4 expmask = ((vec_int4){0x7FF00000, 0, 0x7FF00000, 0});
  e1 = spu_and((vec_int4)x, expmask);
  e2 = spu_rlmask(e1,-20);

  vec_uchar16 maxmask = (vec_uchar16)spu_cmpgt(exp, 2046);
  vec_uchar16 minmask = (vec_uchar16)spu_cmpgt(spu_splats(-2044), exp);
  minmask = spu_or (minmask, (vec_uchar16)x_zero);

  vec_int4 esum = spu_add(e2, exp);

  maxmask = spu_or (maxmask, (vec_uchar16)spu_cmpgt(esum, 2046));
  maxmask = spu_shuffle(maxmask, maxmask, splat_msb);
  //  maxmask = spu_and(maxmask, ((vec_uchar16)spu_splats((long long)0x7FFFFFFFFFFFFFFFLL)));
  minmask = spu_or (minmask, (vec_uchar16)spu_cmpgt(zeros, esum));
  minmask = spu_shuffle(minmask, minmask, splat_msb);
  
  // check denorm
  vec_uint4 mxmask = spu_and(spu_cmpgt(e2, 0), ((vec_uint4){0x00100000,0,0x00100000,0})); // not denorm
  vec_int4 esum2 = spu_sub(esum, (vec_int4)spu_rlmask(mxmask, -20));          // reverse to norm
  vec_uint4 mrange = spu_and(spu_cmpgt(zeros, esum2), spu_cmpgt(esum2, -55)); // denorm range
  mrange = spu_shuffle(mrange, mrange, splat_msb);

  vec_int4 sht_r = spu_sel(spu_splats(-54),  esum2, spu_cmpgt(esum2, spu_splats(-54)) );
  vec_int4 sht_rh = spu_add( sht_r, ((vec_int4){7,7,7,7}));

  x_body = spu_or( x_body, mxmask );
  mant0 = spu_shuffle(x_body, x_body, ((vec_uchar16){0,1, 2, 3, 4, 5, 6, 7,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80}));
  mant1 = spu_shuffle(x_body, x_body, ((vec_uchar16){8,9,10,11,12,13,14,15,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80}));
  vec_uint4 mant0r = spu_rlmaskqwbytebc( spu_rlmaskqw(mant0, spu_extract(sht_r, 0)), spu_extract(sht_rh,0) );
  vec_uint4 mant1r = spu_rlmaskqwbytebc( spu_rlmaskqw(mant1, spu_extract(sht_r, 2)), spu_extract(sht_rh,2) );

#ifdef LDEXPD2_ROUND
  // check current round mode
  fpscr = spu_shuffle(fpscr2, fpscr2, ((vec_uchar16){0x80,0x80,0x80,0x80,0,1,2,3,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80}));
  fpscr0 = spu_and(fpscr, ((vec_uint4){0,0xc00,0,0}));
  fpscr1 = spu_and(fpscr, ((vec_uint4){0,0x300,0,0}));

  // prepare round data
  vec_uint4 rnd0 = spu_slqwbytebc( spu_slqw( mant0r, 31), 31);
  vec_uint4 rnd1 = spu_slqwbytebc( spu_slqw( mant1r, 31), 31);
  vec_uint4 rnd0w = (vec_uint4)spu_cntb( (vec_uchar16)rnd0 );
  vec_uint4 rnd1w = (vec_uint4)spu_cntb( (vec_uchar16)rnd1 );
  rnd0w = spu_or( spu_slqwbyte(rnd0w,4), spu_slqwbyte(rnd0w,8));
  rnd1w = spu_or( spu_slqwbyte(rnd1w,4), spu_slqwbyte(rnd1w,8));
  rnd0 = spu_or( rnd0, rnd0w);
  rnd1 = spu_or( rnd1, rnd1w);

  // nearest
  // check half
  vec_uint4 hit0 = spu_cmpeq(rnd0, ((vec_uint4){0,0xc0000000,0,0}));  //odd + round out
  vec_uint4 hit1 = spu_cmpeq(rnd1, ((vec_uint4){0,0xc0000000,0,0}));  //odd + round out
  vec_uint4 add0 = spu_sel((vec_uint4)zeros, ((vec_uint4){0,1,0,0}), hit0);
  vec_uint4 add1 = spu_sel((vec_uint4)zeros, ((vec_uint4){0,1,0,0}), hit1);
  // check greater than half
  rnd0 = spu_and( rnd0, ((vec_uint4){0,0x7FFFFFFF,0,0}));
  rnd1 = spu_and( rnd1, ((vec_uint4){0,0x7FFFFFFF,0,0}));
  hit0 = spu_cmpgt(rnd0, ((vec_uint4){0,0x40000000,0,0}));
  hit1 = spu_cmpgt(rnd1, ((vec_uint4){0,0x40000000,0,0}));
  add0 = spu_sel(add0, ((vec_uint4){0,1,0,0}), hit0);
  add1 = spu_sel(add1, ((vec_uint4){0,1,0,0}), hit1);
  // select if fp0
  add0 = spu_sel((vec_uint4)zeros, add0, spu_cmpeq(fpscr0, (vec_uint4)zeros));
  add1 = spu_sel((vec_uint4)zeros, add1, spu_cmpeq(fpscr1, (vec_uint4)zeros));

  // toward zero do nothing
  // upward
  sign = spu_rlmaska((vec_uint4)in, -31);
  vec_uint4 sign0 = spu_shuffle(sign, sign, ((vec_uchar16){0x80,0x80,0x80,0x80,0,0,0,0,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80}));
  vec_uint4 sign1 = spu_shuffle(sign, sign, ((vec_uchar16){0x80,0x80,0x80,0x80,8,8,8,8,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80}));
  vec_uint4 hit0w = spu_cmpgt(rnd0, ((vec_uint4){0,0,0,0}));
  vec_uint4 hit1w = spu_cmpgt(rnd1, ((vec_uint4){0,0,0,0}));

  hit0 = spu_andc(hit0w, sign0);
  hit1 = spu_andc(hit1w, sign1);
  hit0 = spu_and(hit0, spu_cmpeq(fpscr0, ((vec_uint4){0,0x800,0,0})));
  hit1 = spu_and(hit1, spu_cmpeq(fpscr1, ((vec_uint4){0,0x200,0,0})));
  // select if fp2
  add0 = spu_sel(add0, ((vec_uint4){0,1,0,0}), hit0);
  add1 = spu_sel(add1, ((vec_uint4){0,1,0,0}), hit1);

  // downward
  hit0 = spu_and(hit0w, sign0);
  hit1 = spu_and(hit1w, sign1);
  hit0 = spu_and(hit0, spu_cmpeq(fpscr0, ((vec_uint4){0,0xc00,0,0})));
  hit1 = spu_and(hit1, spu_cmpeq(fpscr1, ((vec_uint4){0,0x300,0,0})));
  // select if fp3
  add0 = spu_sel(add0, ((vec_uint4){0,1,0,0}), hit0);
  add1 = spu_sel(add1, ((vec_uint4){0,1,0,0}), hit1);

  // calc round
  mant0r = spu_addx(mant0r, add0, spu_rlqwbyte(spu_genc(mant0r, add0), 4));
  mant1r = spu_addx(mant1r, add1, spu_rlqwbyte(spu_genc(mant1r, add1), 4));

#endif  // LDEXPD2_ROUND

  vec_uint4 mantr = spu_shuffle( mant0r, mant1r, ((vec_uchar16){0,1,2,3,4,5,6,7,16,17,18,19,20,21,22,23}));

  // select right answer
  x = spu_sel(x, (vec_double2)spu_sl(esum,20), (vec_ullong2)expmask);
  x = spu_sel(x, (vec_double2)zeros, (vec_ullong2)minmask);
  x = spu_sel(x, (vec_double2)spu_splats((long long)0x7FEFFFFFFFFFFFFFLL), (vec_ullong2)maxmask);

  out = (vec_double2)spu_sel((vec_uint4)x , mantr, mrange);

  // check Infinity,NaN
  vec_uint4 is_inf = spu_cmpeq(e1, expmask);
  is_inf = spu_and( is_inf, spu_shuffle(is_inf,is_inf,msk_64_eq));
  out = (vec_double2)spu_sel((vec_uint4)out , (vec_uint4)in, is_inf);

  out = spu_sel(out, in, (vec_ullong2)signmask);
  return out;
}

#endif
