/* This file contains a set of functions and macros that map the specific
   SPU intrinsic to the PPU/VMX system.  */

/* (C) Copyright
   Sony Computer Entertainment, Inc.,
   2001,2002,2003,2004,2005,2006.

   This file is free software; you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation; either version 2 of the License, or (at your option) 
   any later version.

   This file is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
   for more details.

   You should have received a copy of the GNU General Public License
   along with this file; see the file COPYING.  If not, write to the Free
   Software Foundation, 51 Franklin Street, Fifth Floor, Boston, MA
   02110-1301, USA.  */

/* As a special exception, if you include this header file into source files 
   compiled by GCC, this header file does not by itself cause  the resulting 
   executable to be covered by the GNU General Public License.  This exception 
   does not however invalidate any other reasons why the executable file might be 
   covered by the GNU General Public License.  */ 

#ifndef _SI2VMX_H_
#define _SI2VMX_H_	1

#ifndef __SPU__

#include <stdlib.h>
#include <vec_types.h>


/* Specify a default halt action for spu_hcmpeq and spu_hcmpgt intrinsics.
 * Users can override the action by defining it prior to including this 
 * header file.
 */
#ifndef SPU_HALT_ACTION
#define SPU_HALT_ACTION		abort()
#endif

/* Specify a default stop action for the spu_stop intrinsic.
 * Users can override the action by defining it prior to including this 
 * header file.
 */
#ifndef SPU_STOP_ACTION
#define SPU_STOP_ACTION		abort()
#endif


/* Specify a default action for unsupported intrinsic.
 * Users can override the action by defining it prior to including this 
 * header file.
 */
#ifndef SPU_UNSUPPORTED_ACTION
#define SPU_UNSUPPORTED_ACTION	abort()
#endif


/* Casting intrinsics - from scalar to quadword 
 */

static __inline qword si_from_uchar(unsigned char c) {
  union {
    qword q;
    unsigned char c[16];
  } x;
  x.c[3] = c;
  return (x.q);
}

static __inline qword si_from_char(signed char c) {
  union {
    qword q;
    signed char c[16];
  } x;
  x.c[3] = c;
  return (x.q);
}

static __inline qword si_from_ushort(unsigned short s) {
  union {
    qword q;
    unsigned short s[8];
  } x;
  x.s[1] = s;
  return (x.q);
}

static __inline qword si_from_short(short s) {
  union {
    qword q;
    short s[8];
  } x;
  x.s[1] = s;
  return (x.q);
}


static __inline qword si_from_uint(unsigned int i) {
  union {
    qword q;
    unsigned int i[4];
  } x;
  x.i[0] = i;
  return (x.q);
}

static __inline qword si_from_int(int i) {
  union {
    qword q;
    int i[4];
  } x;
  x.i[0] = i;
  return (x.q);
}

static __inline qword si_from_ullong(unsigned long long l) {
  union {
    qword q;
    unsigned long long l[2];
  } x;
  x.l[0] = l;
  return (x.q);
}

static __inline qword si_from_llong(long long l) {
  union {
    qword q;
    long long l[2];
  } x;
  x.l[0] = l;
  return (x.q);
}

static __inline qword si_from_float(float f) {
  union {
    qword q;
    float f[4];
  } x;
  x.f[0] = f;
  return (x.q);
}

static __inline qword si_from_double(double d) {
  union {
    qword q;
    double d[2];
  } x;
  x.d[0] = d;
  return (x.q);
}

static __inline qword si_from_ptr(void *ptr) {
  union {
    qword q;
    void *p;
  } x;
  x.p = ptr;
  return (x.q);
}


/* Casting intrinsics - from quadword to scalar
 */
static __inline unsigned char si_to_uchar(qword q) {
  union {
    qword q;
    unsigned char c[16];
  } x;
  x.q = q;
  return (x.c[3]);
}

static __inline signed char si_to_char(qword q) {
  union {
    qword q;
    signed char c[16];
  } x;
  x.q = q;
  return (x.c[3]);
}

static __inline unsigned short si_to_ushort(qword q) {
  union {
    qword q;
    unsigned short s[8];
  } x;
  x.q = q;
  return (x.s[1]);
}

static __inline short si_to_short(qword q) {
  union {
    qword q;
    short s[8];
  } x;
  x.q = q;
  return (x.s[1]);
}

static __inline unsigned int si_to_uint(qword q) {
  union {
    qword q;
    unsigned int i[4];
  } x;
  x.q = q;
  return (x.i[0]);
}

static __inline int si_to_int(qword q) {
  union {
    qword q;
    int i[4];
  } x;
  x.q = q;
  return (x.i[0]);
}

static __inline unsigned long long si_to_ullong(qword q) {
  union {
    qword q;
    unsigned long long l[2];
  } x;
  x.q = q;
  return (x.l[0]);
}

static __inline long long si_to_llong(qword q) {
  union {
    qword q;
    long long l[2];
  } x;
  x.q = q;
  return (x.l[0]);
}

static __inline float si_to_float(qword q) {
  union {
    qword q;
    float f[4];
  } x;
  x.q = q;
  return (x.f[0]);
}

static __inline double si_to_double(qword q) {
  union {
    qword q;
    double d[2];
  } x;
  x.q = q;
  return (x.d[0]);
}

static __inline void * si_to_ptr(qword q) {
  union {
    qword q;
    void *p;
  } x;
  x.q = q;
  return (x.p);
}


/* Absolute difference
 */
static __inline qword si_absdb(qword a, qword b)
{
  vec_uchar16 ac, bc, dc;

  ac = (vec_uchar16)(a);
  bc = (vec_uchar16)(b);
  dc = vec_sel(vec_sub(bc, ac), vec_sub(ac, bc), vec_cmpgt(ac, bc));

  return ((qword)(dc));
}

/* Add intrinsics 
 */
#define si_a(_a, _b)		((qword)(vec_add((vec_uint4)(_a), (vec_uint4)(_b))))

#define si_ah(_a, _b)		((qword)(vec_add((vec_ushort8)(_a), (vec_ushort8)(_b))))

static __inline qword si_ai(qword a, int b)
{
  return ((qword)(vec_add((vec_int4)(a), 
			  vec_splat((vec_int4)(si_from_int(b)), 0))));
}


static __inline qword si_ahi(qword a, short b)
{
  return ((qword)(vec_add((vec_short8)(a), 
			  vec_splat((vec_short8)(si_from_short(b)), 1))));
}


#define si_fa(_a, _b)	((qword)(vec_add((vec_float4)(_a), (vec_float4)(_b))))


static __inline qword si_dfa(qword a, qword b)
{
  union {
    vec_double2 v;
    double d[2];
  } ad, bd, dd;

  ad.v = (vec_double2)(a);
  bd.v = (vec_double2)(b);
  dd.d[0] = ad.d[0] + bd.d[0];
  dd.d[1] = ad.d[1] + bd.d[1];

  return ((qword)(dd.v));
}

/* Add word extended
 */
#define si_addx(_a, _b, _c)	((qword)(vec_add(vec_add((vec_uint4)(_a), (vec_uint4)(_b)), 	\
						 vec_and((vec_uint4)(_c), vec_splat_u32(1)))))


/* Bit-wise AND
 */
#define si_and(_a, _b)		((qword)(vec_and((vec_uint4)(_a), (vec_uint4)(_b))))


static __inline qword si_andbi(qword a, signed char b)
{
  return ((qword)(vec_and((vec_char16)(a), 
			  vec_splat((vec_char16)(si_from_char(b)), 3))));
}

static __inline qword si_andhi(qword a, signed short b)
{
  return ((qword)(vec_and((vec_short8)(a), 
			  vec_splat((vec_short8)(si_from_short(b)), 1))));
}


static __inline qword si_andi(qword a, signed int b)
{
  return ((qword)(vec_and((vec_int4)(a),
			  vec_splat((vec_int4)(si_from_int(b)), 0))));
}


/* Bit-wise AND with complement
 */
#define si_andc(_a, _b)		((qword)(vec_andc((vec_uchar16)(_a), (vec_uchar16)(_b))))


/* Average byte vectors
 */
#define si_avgb(_a, _b)		((qword)(vec_avg((vec_uchar16)(_a), (vec_uchar16)(_b))))


/* Branch indirect and set link on external data
 */
#define si_bisled(_func)	/* not mappable */
#define si_bisledd(_func)	/* not mappable */
#define si_bislede(_func)	/* not mappable */


/* Borrow generate
 */
#define si_bg(_a, _b)		((qword)(vec_subc((vec_uint4)(_b), (vec_uint4)(_a))))

#define si_bgx(_a, _b, _c)	((qword)(vec_and(vec_or(vec_cmpgt((vec_uint4)(_b), (vec_uint4)(_a)),		\
							vec_and(vec_cmpeq((vec_uint4)(_b), (vec_uint4)(_a)), 	\
								(vec_uint4)(_c))), vec_splat_u32(1))))

/* Compare absolute equal
 */
static __inline qword si_fcmeq(qword a, qword b)
{
  vec_float4 msb = (vec_float4)((vec_uint4){0x80000000, 0x80000000, 0x80000000, 0x80000000});
  
  return ((qword)(vec_cmpeq(vec_andc((vec_float4)(a), msb), 
				  vec_andc((vec_float4)(b), msb))));
}

static __inline qword si_dfcmeq(qword a, qword b)
{
  vec_uint4 sign_mask= (vec_uint4) { 0x7FFFFFFF, 0xFFFFFFFF, 0x7FFFFFFF, 0xFFFFFFFF };
  vec_uint4 nan_mask = (vec_uint4) { 0x7FF00000, 0x00000000, 0x7FF00000, 0x00000000 };
  vec_uchar16 hihi_promote = (vec_uchar16) { 0,1,2,3,  16,17,18,19,  8,9,10,11, 24,25,26,27};

  vec_uint4 biteq;
  vec_uint4 aabs;
  vec_uint4 babs;
  vec_uint4 a_gt;
  vec_uint4 ahi_inf;
  vec_uint4 anan;
  vec_uint4 result;

  union {
    vec_uchar16 v;
    int i[4];
  } x;

  /* Shift 4 bytes  */
  x.i[3] = 4 << 3;

  /*  Mask out sign bits */
  aabs = vec_and((vec_uint4)a,sign_mask);
  babs = vec_and((vec_uint4)b,sign_mask);

  /*  A)  Check for bit equality, store in high word */
  biteq = (vec_uint4) vec_cmpeq((vec_uint4)aabs,(vec_uint4)babs);
  biteq = vec_and(biteq,(vec_uint4)vec_slo((vec_uchar16)biteq,x.v));

  /*  
      B)  Check if a is NaN, store in high word
        
      B1) If the high word is greater than max_exp (indicates a NaN)
      B2) If the low word is greater than 0 
  */
  a_gt = (vec_uint4)vec_cmpgt(aabs,nan_mask);

  /*  B3) Check if the high word is equal to the inf exponent */
  ahi_inf = (vec_uint4)vec_cmpeq(aabs,nan_mask);

  /*  anan = B1[hi] or (B2[lo] and B3[hi]) */
  anan = (vec_uint4)vec_or(a_gt,vec_and((vec_uint4)vec_slo((vec_uchar16)a_gt,x.v),ahi_inf));

  /*  result = A and not B  */
  result = vec_andc(biteq, anan);

  /*  Promote high words to 64 bits and return  */
  return ((qword)(vec_perm((vec_uchar16)result, (vec_uchar16)result, hihi_promote)));
}


/* Compare absolute greater than
 */
static __inline qword si_fcmgt(qword a, qword b)
{
  vec_float4 msb = (vec_float4)((vec_uint4){0x80000000, 0x80000000, 0x80000000, 0x80000000});
  
  return ((qword)(vec_cmpgt(vec_andc((vec_float4)(a), msb),
				  vec_andc((vec_float4)(b), msb))));
}

static __inline qword si_dfcmgt(qword a, qword b)
{
  vec_uchar16 splat_hi = (vec_uchar16) { 0,1,2,3, 0,1,2,3, 8,9,10,11, 8,9,10,11 };
  vec_uint4 nan_mask = (vec_uint4) { 0x7FF00000, 0x0, 0x7FF00000, 0x0 };
  vec_uint4 sign_mask = (vec_uint4) { 0x7FFFFFFF, 0xFFFFFFFF, 0x7FFFFFFF, 0xFFFFFFFF };

  union {
    vec_uchar16 v;
    int i[4];
  } x;

  /* Shift 4 bytes  */
  x.i[3] = 4 << 3;

  // absolute value of a,b 
  vec_uint4 aabs = vec_and((vec_uint4)a, sign_mask);
  vec_uint4 babs = vec_and((vec_uint4)b, sign_mask);

  // check if a is nan
  vec_uint4 a_inf = (vec_uint4)vec_cmpeq(aabs, nan_mask);
  vec_uint4 a_nan = (vec_uint4)vec_cmpgt(aabs, nan_mask);
  a_nan = vec_or(a_nan, vec_and((vec_uint4)vec_slo((vec_uchar16)a_nan,x.v),a_inf));
  a_nan = (vec_uint4)vec_perm((vec_uchar16)a_nan, (vec_uchar16)a_nan, splat_hi);

  // check if b is nan
  vec_uint4 b_inf = (vec_uint4)vec_cmpeq(babs, nan_mask);
  vec_uint4 b_nan = (vec_uint4)vec_cmpgt(babs, nan_mask);
  b_nan = vec_or(b_nan, vec_and((vec_uint4)vec_slo((vec_uchar16)b_nan,x.v),b_inf));
  b_nan = (vec_uint4)vec_perm((vec_uchar16)b_nan, (vec_uchar16)b_nan, splat_hi);

  // A) Check if the exponents are different 
  vec_uint4 gt_hi = (vec_uint4)vec_cmpgt(aabs,babs);

  // B) Check if high word equal, and low word greater
  vec_uint4 gt_lo = (vec_uint4)vec_cmpgt((vec_uint4)aabs, (vec_uint4)babs);
  vec_uint4 eq = (vec_uint4)vec_cmpeq(aabs, babs);
  vec_uint4 eqgt = vec_and(eq,vec_slo(gt_lo,x.v));

  //  If either A or B is true, return true (unless NaNs detected) 
  vec_uint4 r = vec_or(gt_hi, eqgt);

  // splat the high words of the comparison step
  r = (vec_uint4)vec_perm((vec_uchar16)r,(vec_uchar16)r,splat_hi);

  // correct for NaNs in input
  return ((qword)vec_andc(r,vec_or(a_nan,b_nan)));
}


/* Compare equal
 */
static __inline qword si_ceqb(qword a, qword b)
{
  return ((qword)(vec_cmpeq((vec_uchar16)(a), (vec_uchar16)(b))));
}

static __inline qword si_ceqh(qword a, qword b)
{
  return ((qword)(vec_cmpeq((vec_ushort8)(a), (vec_ushort8)(b))));
}

static __inline qword si_ceq(qword a, qword b)
{
  return ((qword)(vec_cmpeq((vec_uint4)(a), (vec_uint4)(b))));
}

static __inline qword si_fceq(qword a, qword b)
{
  return ((qword)(vec_cmpeq((vec_float4)(a), (vec_float4)(b))));
}

static __inline qword si_ceqbi(qword a, signed char b)
{
  return ((qword)(vec_cmpeq((vec_char16)(a), 
			    vec_splat((vec_char16)(si_from_char(b)), 3))));
}

static __inline qword si_ceqhi(qword a, signed short b)
{
  return ((qword)(vec_cmpeq((vec_short8)(a), 
			  vec_splat((vec_short8)(si_from_short(b)), 1))));
}

static __inline qword si_ceqi(qword a, signed int b)
{
  return ((qword)(vec_cmpeq((vec_int4)(a), 
			  vec_splat((vec_int4)(si_from_int(b)), 0))));
}

static __inline qword si_dfceq(qword a, qword b)
{
  vec_uint4 sign_mask= (vec_uint4) { 0x7FFFFFFF, 0xFFFFFFFF, 0x7FFFFFFF, 0xFFFFFFFF };
  vec_uint4 nan_mask = (vec_uint4) { 0x7FF00000, 0x00000000, 0x7FF00000, 0x00000000 };
  vec_uchar16 hihi_promote = (vec_uchar16) { 0,1,2,3,  16,17,18,19,  8,9,10,11, 24,25,26,27};

  vec_uint4 biteq;
  vec_uint4 aabs;
  vec_uint4 babs;
  vec_uint4 a_gt;
  vec_uint4 ahi_inf;
  vec_uint4 anan;
  vec_uint4 iszero;
  vec_uint4 result;

  union {
    vec_uchar16 v;
    int i[4];
  } x;

  /* Shift 4 bytes  */
  x.i[3] = 4 << 3;

  /*  A)  Check for bit equality, store in high word */
  biteq = (vec_uint4) vec_cmpeq((vec_uint4)a,(vec_uint4)b);
  biteq = vec_and(biteq,(vec_uint4)vec_slo((vec_uchar16)biteq,x.v));

  /*  Mask out sign bits */
  aabs = vec_and((vec_uint4)a,sign_mask);
  babs = vec_and((vec_uint4)b,sign_mask);

  /*  
      B)  Check if a is NaN, store in high word
        
      B1) If the high word is greater than max_exp (indicates a NaN)
      B2) If the low word is greater than 0 
  */
  a_gt = (vec_uint4)vec_cmpgt(aabs,nan_mask);

  /*  B3) Check if the high word is equal to the inf exponent */
  ahi_inf = (vec_uint4)vec_cmpeq(aabs,nan_mask);

  /*  anan = B1[hi] or (B2[lo] and B3[hi]) */
  anan = (vec_uint4)vec_or(a_gt,vec_and((vec_uint4)vec_slo((vec_uchar16)a_gt,x.v),ahi_inf));

  /*  C)  Check for 0 = -0 special case */
  iszero =(vec_uint4)vec_cmpeq((vec_uint4)vec_or(aabs,babs),(vec_uint4)vec_splat_u32(0));
  iszero = vec_and(iszero,(vec_uint4)vec_slo((vec_uchar16)iszero,x.v));

  /*  result = (A or C) and not B  */
  result = vec_or(biteq,iszero);
  result = vec_andc(result, anan);

  /*  Promote high words to 64 bits and return  */
  return ((qword)(vec_perm((vec_uchar16)result, (vec_uchar16)result, hihi_promote))); 
}


/* Compare greater than
 */
static __inline qword si_cgtb(qword a, qword b)
{
  return ((qword)(vec_cmpgt((vec_char16)(a), (vec_char16)(b))));
}

static __inline qword si_cgth(qword a, qword b)
{
  return ((qword)(vec_cmpgt((vec_short8)(a), (vec_short8)(b))));
}

static __inline qword si_cgt(qword a, qword b)
{
  return ((qword)(vec_cmpgt((vec_int4)(a), (vec_int4)(b))));
}

static __inline qword si_clgtb(qword a, qword b)
{
  return ((qword)(vec_cmpgt((vec_uchar16)(a), (vec_uchar16)(b))));
}

static __inline qword si_clgth(qword a, qword b)
{
  return ((qword)(vec_cmpgt((vec_ushort8)(a), (vec_ushort8)(b))));
}

static __inline qword si_clgt(qword a, qword b)
{
  return ((qword)(vec_cmpgt((vec_uint4)(a), (vec_uint4)(b))));
}

static __inline qword si_fcgt(qword a, qword b)
{
  return ((qword)(vec_cmpgt((vec_float4)(a), (vec_float4)(b))));
}

static __inline qword si_dfcgt(qword a, qword b)
{
  vec_uchar16 splat_hi = (vec_uchar16) { 0,1,2,3, 0,1,2,3, 8,9,10,11, 8,9,10,11 };
  vec_uchar16 borrow_shuffle = (vec_uchar16) { 4,5,6,7, 192,192,192,192, 12,13,14,15, 192,192,192,192 };
  vec_uint4 nan_mask = (vec_uint4) { 0x7FF00000, 0x0, 0x7FF00000, 0x0 };
  vec_uint4 sign_mask = (vec_uint4) { 0x7FFFFFFF, 0xFFFFFFFF, 0x7FFFFFFF, 0xFFFFFFFF };

  union {
    vec_uchar16 v;
    int i[4];
  } x;

  /* Shift 4 bytes  */
  x.i[3] = 4 << 3;

  // absolute value of a,b 
  vec_uint4 aabs = vec_and((vec_uint4)a, sign_mask);
  vec_uint4 babs = vec_and((vec_uint4)b, sign_mask);

  // check if a is nan
  vec_uint4 a_inf = (vec_uint4)vec_cmpeq(aabs, nan_mask);
  vec_uint4 a_nan = (vec_uint4)vec_cmpgt(aabs, nan_mask);
  a_nan = vec_or(a_nan, vec_and((vec_uint4)vec_slo((vec_uchar16)a_nan,x.v),a_inf));
  a_nan = (vec_uint4)vec_perm((vec_uchar16)a_nan, (vec_uchar16)a_nan, splat_hi);

  // check if b is nan
  vec_uint4 b_inf = (vec_uint4)vec_cmpeq(babs, nan_mask);
  vec_uint4 b_nan = (vec_uint4)vec_cmpgt(babs, nan_mask);
  b_nan = vec_or(b_nan, vec_and((vec_uint4)vec_slo((vec_uchar16)b_nan,x.v),b_inf));
  b_nan = (vec_uint4)vec_perm((vec_uchar16)b_nan, (vec_uchar16)b_nan, splat_hi);

  // sign of a
  vec_uint4 asel = (vec_uint4)vec_sra((vec_int4)(a), (vec_uint4)vec_splat(((vec_uint4)si_from_int(31)), 0));
  asel = (vec_uint4)vec_perm((vec_uchar16)asel,(vec_uchar16)asel,splat_hi);

  // sign of b
  vec_uint4 bsel = (vec_uint4)vec_sra((vec_int4)(b), (vec_uint4)vec_splat(((vec_uint4)si_from_int(31)), 0));
  bsel = (vec_uint4)vec_perm((vec_uchar16)bsel,(vec_uchar16)bsel,splat_hi);

  // negative a
  vec_uint4 abor = vec_subc((vec_uint4)vec_splat_u32(0), aabs);
  vec_uchar16 pat = vec_sel(((vec_uchar16){0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15}), vec_sr(borrow_shuffle, vec_splat_u8(3)), vec_sra(borrow_shuffle, vec_splat_u8(7)));
  abor = (vec_uint4)(vec_perm(vec_perm((vec_uchar16)abor, (vec_uchar16)abor, borrow_shuffle),((vec_uchar16){0, 0, 0, 0, 0, 0, 0, 0, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x80, 0x80, 0x80}),pat));
  vec_uint4 aneg = vec_add(vec_add(vec_splat_u32(0), vec_nor(aabs, aabs)), vec_and(abor, vec_splat_u32(1)));

  // pick the one we want
  vec_int4 aval = (vec_int4)vec_sel((vec_uchar16)aabs, (vec_uchar16)aneg, (vec_uchar16)asel);

  // negative b
  vec_uint4 bbor = vec_subc((vec_uint4)vec_splat_u32(0), babs);
  bbor = (vec_uint4)(vec_perm(vec_perm((vec_uchar16)bbor, (vec_uchar16)bbor, borrow_shuffle),((vec_uchar16){0, 0, 0, 0, 0, 0, 0, 0, 0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x80, 0x80, 0x80}),pat));
  vec_uint4 bneg = vec_add(vec_nor(babs, babs), vec_and(bbor, vec_splat_u32(1)));

  // pick the one we want
  vec_int4 bval=(vec_int4)vec_sel((vec_uchar16)babs, (vec_uchar16)bneg, (vec_uchar16)bsel);

  // A) Check if the exponents are different 
  vec_uint4 gt_hi = (vec_uint4)vec_cmpgt(aval,bval);

  // B) Check if high word equal, and low word greater
  vec_uint4 gt_lo = (vec_uint4)vec_cmpgt((vec_uint4)aval, (vec_uint4)bval);
  vec_uint4 eq = (vec_uint4)vec_cmpeq(aval, bval);
  vec_uint4 eqgt = vec_and(eq,vec_slo(gt_lo,x.v));

  //  If either A or B is true, return true (unless NaNs detected) 
  vec_uint4 r = vec_or(gt_hi, eqgt);

  // splat the high words of the comparison step
  r = (vec_uint4)vec_perm((vec_uchar16)r,(vec_uchar16)r,splat_hi);

  // correct for NaNs in input
  return ((qword)vec_andc(r,vec_or(a_nan,b_nan)));
}

static __inline qword si_cgtbi(qword a, signed char b)
{
  return ((qword)(vec_cmpgt((vec_char16)(a), 
			    vec_splat((vec_char16)(si_from_char(b)), 3))));
}

static __inline qword si_cgthi(qword a, signed short b)
{
  return ((qword)(vec_cmpgt((vec_short8)(a), 
			    vec_splat((vec_short8)(si_from_short(b)), 1))));
}

static __inline qword si_cgti(qword a, signed int b)
{
  return ((qword)(vec_cmpgt((vec_int4)(a), 
			    vec_splat((vec_int4)(si_from_int(b)), 0))));
}

static __inline qword si_clgtbi(qword a, unsigned char b)
{
  return ((qword)(vec_cmpgt((vec_uchar16)(a), 
			    vec_splat((vec_uchar16)(si_from_uchar(b)), 3))));
}

static __inline qword si_clgthi(qword a, unsigned short b)
{
  return ((qword)(vec_cmpgt((vec_ushort8)(a),
			    vec_splat((vec_ushort8)(si_from_ushort(b)), 1))));
}

static __inline qword si_clgti(qword a, unsigned int b)
{
  return ((qword)(vec_cmpgt((vec_uint4)(a), 
			    vec_splat((vec_uint4)(si_from_uint(b)), 0))));
}

static __inline qword si_dftsv(qword a, char b)
{
  vec_uchar16 splat_hi = (vec_uchar16) { 0,1,2,3, 0,1,2,3, 8,9,10,11, 8,9,10,11 };
  vec_uint4 sign_mask = (vec_uint4) { 0x7FFFFFFF, 0xFFFFFFFF, 0x7FFFFFFF, 0xFFFFFFFF };
  vec_uint4 result = (vec_uint4){0};
  vec_uint4 sign = (vec_uint4)vec_sra((vec_int4)(a), (vec_uint4)vec_splat(((vec_uint4)si_from_int(31)), 0));
  sign = (vec_uint4)vec_perm((vec_uchar16)sign,(vec_uchar16)sign,splat_hi);
  vec_uint4 aabs = vec_and((vec_uint4)a,sign_mask);
  
  union {
    vec_uchar16 v;
    int i[4];
  } x;

  /* Shift 4 bytes  */
  x.i[3] = 4 << 3;
  
  /* Nan or +inf or -inf  */
  if (b & 0x70)
  {
    vec_uint4 nan_mask = (vec_uint4) { 0x7FF00000, 0x0, 0x7FF00000, 0x0 };
    vec_uint4 a_inf = (vec_uint4)vec_cmpeq(aabs, nan_mask);
     /* NaN  */
     if (b & 0x40)
     {
       vec_uint4 a_nan = (vec_uint4)vec_cmpgt(aabs, nan_mask);
       a_nan = vec_or(a_nan, vec_and((vec_uint4)vec_slo((vec_uchar16)a_nan,x.v),a_inf));
       a_nan = (vec_uint4)vec_perm((vec_uchar16)a_nan, (vec_uchar16)a_nan, splat_hi); 
       result = vec_or(result, a_nan);
     }
     /* inf  */ 
     if (b & 0x30)
     {
       a_inf = vec_and((vec_uint4)vec_slo((vec_uchar16)a_inf,x.v), a_inf);
       a_inf = (vec_uint4)vec_perm((vec_uchar16)a_inf, (vec_uchar16)a_inf, splat_hi); 
        /* +inf  */
        if (b & 0x20)
          result = vec_or(vec_andc(a_inf, sign), result);
        /* -inf  */
        if (b & 0x10)
          result = vec_or(vec_and(a_inf, sign), result);
     } 
  }
  /* 0 or denorm  */
  if (b & 0xF)
  {
    vec_uint4 iszero =(vec_uint4)vec_cmpeq(aabs,(vec_uint4)vec_splat_u32(0));
    iszero = vec_and(iszero,(vec_uint4)vec_slo((vec_uchar16)iszero,x.v));
    /* denorm  */
    if (b & 0x3)
    {
      vec_uint4 denorm_mask = (vec_uint4){0xFFFFF, 0xFFFFF, 0xFFFFF, 0xFFFFF};
      vec_uint4 isdenorm = vec_nor((vec_uint4)vec_cmpgt(aabs, denorm_mask), iszero);
      isdenorm = (vec_uint4)vec_perm((vec_uchar16)isdenorm, (vec_uchar16)isdenorm, splat_hi);
      /* +denorm  */
     if (b & 0x2)
        result = vec_or(vec_andc(isdenorm, sign), result);
      /* -denorm  */
     if (b & 0x1)
        result = vec_or(vec_and(isdenorm, sign), result);
    }
    /* 0  */
    if (b & 0xC)
    {
      iszero = (vec_uint4)vec_perm((vec_uchar16)iszero, (vec_uchar16)iszero, splat_hi);
      /* +0  */
     if (b & 0x8)
        result = vec_or(vec_andc(iszero, sign), result);
      /* -0  */
     if (b & 0x4)
        result = vec_or(vec_and(iszero, sign), result);
    }
  }
  return ((qword)result);
}


/* Carry generate
 */
#define si_cg(_a, _b)		((qword)(vec_addc((vec_uint4)(_a), (vec_uint4)(_b))))

#define si_cgx(_a, _b, _c)	((qword)(vec_or(vec_addc((vec_uint4)(_a), (vec_uint4)(_b)), 		\
						vec_addc(vec_add((vec_uint4)(_a), (vec_uint4)(_b)),	\
							 vec_and((vec_uint4)(_c), vec_splat_u32(1))))))


/* Count ones for bytes
 */
static __inline qword si_cntb(qword a)
{
  vec_uchar16 nib_cnt = (vec_uchar16){0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4};
  vec_uchar16 four = { 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4 };
  vec_uchar16 av;

  av = (vec_uchar16)(a);

  return ((qword)(vec_add(vec_perm(nib_cnt, nib_cnt, av),
			  vec_perm(nib_cnt, nib_cnt, vec_sr (av, four)))));
}

/* Count ones for bytes
 */
static __inline qword si_clz(qword a)
{
  vec_uchar16 av;
  vec_uchar16 cnt_hi, cnt_lo, cnt, tmp1, tmp2, tmp3;
  vec_uchar16 four    = vec_splat_u8(4);
  vec_uchar16 nib_cnt = (vec_uchar16){4, 3, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0};
  vec_uchar16 eight   = vec_splat_u8(8);
  vec_uchar16 sixteen = (vec_uchar16){16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16};
  vec_uchar16 twentyfour = (vec_uchar16){24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24};

  av = (vec_uchar16)(a);

  cnt_hi = vec_perm(nib_cnt, nib_cnt, vec_sr(av, four));
  cnt_lo = vec_perm(nib_cnt, nib_cnt, av);

  cnt = vec_add(cnt_hi, vec_and(cnt_lo, vec_cmpeq(cnt_hi, four)));

  tmp1 = (vec_uchar16)vec_sl((vec_uint4)(cnt), (vec_uint4)(eight));
  tmp2 = (vec_uchar16)vec_sl((vec_uint4)(cnt), (vec_uint4)(sixteen));
  tmp3 = (vec_uchar16)vec_sl((vec_uint4)(cnt), (vec_uint4)(twentyfour));

  cnt = vec_add(cnt, vec_and(tmp1, vec_cmpeq(cnt, eight)));
  cnt = vec_add(cnt, vec_and(tmp2, vec_cmpeq(cnt, sixteen)));
  cnt = vec_add(cnt, vec_and(tmp3, vec_cmpeq(cnt, twentyfour)));
  
  return (qword)((vec_sr((vec_uint4)(cnt), (vec_uint4)(twentyfour))));
}

/* Convert to float
 */
#define si_cuflt(_a, _b)	((qword)(vec_ctf((vec_uint4)(_a), _b)))
#define si_csflt(_a, _b)	((qword)(vec_ctf((vec_int4)(_a), _b)))

/* Convert to signed int
 */
#define si_cflts(_a, _b)	((qword)(vec_cts((vec_float4)(_a), _b)))

/* Convert to unsigned int
 */
#define si_cfltu(_a, _b)	((qword)(vec_ctu((vec_float4)(_a), _b)))

/* Synchronize
 */
#define si_dsync()		/* do nothing */
#define si_sync()		/* do nothing */
#define si_syncc()		/* do nothing */


/* Equivalence
 */
static __inline qword si_eqv(qword a, qword b)
{
  vec_uchar16 d;

  d = vec_xor((vec_uchar16)(a), (vec_uchar16)(b));
  return ((qword)(vec_nor(d, d)));
}

/* Extend
 */
static __inline qword si_xsbh(qword a)
{
  vec_char16 av;

  av = (vec_char16)(a);
  return ((qword)(vec_unpackh(vec_perm(av, av, ((vec_uchar16){1, 3, 5, 7, 9,11,13,15, 
						              0, 0, 0, 0, 0, 0, 0, 0})))));
}

static __inline qword si_xshw(qword a)
{
  vec_short8 av;

  av = (vec_short8)(a);
  return ((qword)(vec_unpackh(vec_perm(av, av, ((vec_uchar16){2, 3, 6, 7, 
					                      10,11,14,15,
							      0, 0, 0, 0, 
						              0, 0, 0, 0})))));
}

static __inline qword si_xswd(qword a)
{
  vec_int4 av;

  av = (vec_int4)(a);
  return ((qword)(vec_perm(av, vec_sra(av, ((vec_uint4){31,31,31,31})), 
			   ((vec_uchar16){20, 21, 22, 23,  
					   4,  5,  6,  7, 
				          28, 29, 30, 31, 
				          12, 13, 14, 15}))));
}

static __inline qword si_fesd(qword a)
{
  union {
    double d[2];
    vec_double2	vd;
  } out;
  union {
    float f[4];
    vec_float4 vf;
  } in;

  in.vf = (vec_float4)(a);
  out.d[0] = (double)(in.f[0]);
  out.d[1] = (double)(in.f[2]);
  return ((qword)(out.vd));
}

/* Gather
 */
static __inline qword si_gbb(qword a)
{
  vec_uchar16 bits;
  vec_uint4   bytes;

  bits  = vec_sl(vec_and((vec_uchar16)(a), vec_splat_u8(1)), ((vec_uchar16){7, 6, 5, 4, 3, 2, 1, 0,
								            7, 6, 5, 4, 3, 2, 1, 0}));
  bytes = (vec_uint4)vec_sum2s((vec_int4)(vec_sum4s(bits, ((vec_uint4){0}))), ((vec_int4){0}));

  return ((qword)(vec_perm(bytes, bytes, ((vec_uchar16){0, 0, 7,15, 0, 0, 0, 0,
					                0, 0, 0, 0, 0, 0, 0, 0}))));
}


static __inline qword si_gbh(qword a)
{
  vec_ushort8 bits;
  vec_uint4   bytes;

  bits  = vec_sl(vec_and((vec_ushort8)(a), vec_splat_u16(1)), ((vec_ushort8){7, 6, 5, 4, 3, 2, 1, 0}));

  bytes = (vec_uint4)vec_sums((vec_int4)(vec_sum4s((vec_short8)(bits), (vec_int4){0})), (vec_int4){0});

  return ((qword)(vec_sld(bytes, bytes, 12)));
}

static __inline qword si_gb(qword a)
{
  vec_uint4 bits;
  vec_uint4 bytes;

  bits  = vec_sl(vec_and((vec_uint4)(a), vec_splat_u32(1)), ((vec_uint4){3, 2, 1, 0}));
  bytes = (vec_uint4)vec_sums((vec_int4)(bits), ((vec_int4){0}));
  return ((qword)(vec_sld(bytes, bytes, 12)));
}


/* Compare and halt 
 */
static __inline void si_heq(qword a, qword b)
{
  union {
    vector unsigned int v;
    unsigned int i[4];
  } aa, bb;

  aa.v = (vector unsigned int)(a);
  bb.v = (vector unsigned int)(b);

  if (aa.i[0] == bb.i[0]) { SPU_HALT_ACTION; };
}

static __inline void si_heqi(qword a, unsigned int b)
{
  union {
    vector unsigned int v;
    unsigned int i[4];
  } aa;

  aa.v = (vector unsigned int)(a);

  if (aa.i[0] == b) { SPU_HALT_ACTION; };
}

static __inline void si_hgt(qword a, qword b)
{
  union {
    vector signed int v;
    signed int i[4];
  } aa, bb;

  aa.v = (vector signed int)(a);
  bb.v = (vector signed int)(b);

  if (aa.i[0] > bb.i[0]) { SPU_HALT_ACTION; };
}

static __inline void si_hgti(qword a, signed int b)
{
  union {
    vector signed int v;
    signed int i[4];
  } aa;

  aa.v = (vector signed int)(a);

  if (aa.i[0] > b) { SPU_HALT_ACTION; };
}

static __inline void si_hlgt(qword a, qword b)
{
  union {
    vector unsigned int v;
    unsigned int i[4];
  } aa, bb;

  aa.v = (vector unsigned int)(a);
  bb.v = (vector unsigned int)(b);

  if (aa.i[0] > bb.i[0]) { SPU_HALT_ACTION; };
}

static __inline void si_hlgti(qword a, unsigned int b)
{
  union {
    vector unsigned int v;
    unsigned int i[4];
  } aa;

  aa.v = (vector unsigned int)(a);

  if (aa.i[0] > b) { SPU_HALT_ACTION; };
}


/* Multiply and Add
 */
static __inline qword si_mpya(qword a, qword b, qword c)
{
  return ((qword)(vec_msum(vec_and((vec_short8)(a), 
				   ((vec_short8){0, -1, 0, -1, 0, -1, 0, -1})), 
			   (vec_short8)(b), (vec_int4)(c))));
}

static __inline qword si_fma(qword a, qword b, qword c)
{
  return ((qword)(vec_madd((vec_float4)(a), (vec_float4)(b), (vec_float4)(c))));
}

static __inline qword si_dfma(qword a, qword b, qword c)
{
  union {
    vec_double2 v;
    double d[2];
  } aa, bb, cc, dd;

  aa.v = (vec_double2)(a);
  bb.v = (vec_double2)(b);
  cc.v = (vec_double2)(c);
  dd.d[0] = aa.d[0] * bb.d[0] + cc.d[0];
  dd.d[1] = aa.d[1] * bb.d[1] + cc.d[1];
  return ((qword)(dd.v));
}

/* Form Mask
 */
#define si_fsmbi(_a)	si_fsmb(si_from_int(_a))

static __inline qword si_fsmb(qword a)
{
  vec_char16 mask;
  vec_ushort8 in;

  in = (vec_ushort8)(a);
  mask = (vec_char16)(vec_perm(in, in, ((vec_uchar16){2, 2, 2, 2, 2, 2, 2, 2,
					              3, 3, 3, 3, 3, 3, 3, 3})));
  return ((qword)(vec_sra(vec_sl(mask, ((vec_uchar16){0, 1, 2, 3, 4, 5, 6, 7,
				                      0, 1, 2, 3, 4, 5, 6, 7})),
			  vec_splat_u8(7))));
}


static __inline qword si_fsmh(qword a)
{
  vec_uchar16 in;
  vec_short8 mask;

  in = (vec_uchar16)(a);
  mask = (vec_short8)(vec_splat(in, 3));
  return ((qword)(vec_sra(vec_sl(mask, ((vec_ushort8){0, 1, 2, 3, 4, 5, 6, 7})), 
			  vec_splat_u16(15))));
}

static __inline qword si_fsm(qword a)
{
  vec_uchar16 in;
  vec_int4 mask;

  in = (vec_uchar16)(a);
  mask = (vec_int4)(vec_splat(in, 3));
  return ((qword)(vec_sra(vec_sl(mask, ((vec_uint4){28, 29, 30, 31})),
			  ((vec_uint4){31,31,31,31}))));
}

/* Move from/to registers
 */
#define si_fscrrd()		((qword)((vec_uint4){0}))
#define si_fscrwr(_a)

#define si_mfspr(_reg)		((qword)((vec_uint4){0}))
#define si_mtspr(_reg, _a)

/* Multiply High High Add
 */
static __inline qword si_mpyhha(qword a, qword b, qword c)
{
  return ((qword)(vec_add(vec_mule((vec_short8)(a), (vec_short8)(b)), (vec_int4)(c))));
}

static __inline qword si_mpyhhau(qword a, qword b, qword c)
{
  return ((qword)(vec_add(vec_mule((vec_ushort8)(a), (vec_ushort8)(b)), (vec_uint4)(c))));
}

/* Multiply Subtract
 */
static __inline qword si_fms(qword a, qword b, qword c)
{
  return ((qword)(vec_madd((vec_float4)(a), (vec_float4)(b), 
			   vec_sub(((vec_float4){0.0f}), (vec_float4)(c)))));
}

static __inline qword si_dfms(qword a, qword b, qword c)
{
  union {
    vec_double2 v;
    double d[2];
  } aa, bb, cc, dd;

  aa.v = (vec_double2)(a);
  bb.v = (vec_double2)(b);
  cc.v = (vec_double2)(c);
  dd.d[0] = aa.d[0] * bb.d[0] - cc.d[0];
  dd.d[1] = aa.d[1] * bb.d[1] - cc.d[1];
  return ((qword)(dd.v));
}

/* Multiply
 */
static __inline qword si_fm(qword a, qword b)
{
  return ((qword)(vec_madd((vec_float4)(a), (vec_float4)(b), ((vec_float4){0.0f}))));
}

static __inline qword si_dfm(qword a, qword b)
{
  union {
    vec_double2 v;
    double d[2];
  } aa, bb, dd;

  aa.v = (vec_double2)(a);
  bb.v = (vec_double2)(b);
  dd.d[0] = aa.d[0] * bb.d[0];
  dd.d[1] = aa.d[1] * bb.d[1];
  return ((qword)(dd.v));
}

/* Multiply High
 */
static __inline qword si_mpyh(qword a, qword b)
{
  vec_uint4 sixteen = (vec_uint4){16, 16, 16, 16};

  return ((qword)(vec_sl(vec_mule((vec_short8)(a), (vec_short8)(vec_sl((vec_uint4)(b), sixteen))), sixteen)));
}


/* Multiply High High
 */
static __inline qword si_mpyhh(qword a, qword b)
{
  return ((qword)(vec_mule((vec_short8)(a), (vec_short8)(b))));
}

static __inline qword si_mpyhhu(qword a, qword b)
{
  return ((qword)(vec_mule((vec_ushort8)(a), (vec_ushort8)(b))));
}

/* Multiply Odd
 */
static __inline qword si_mpy(qword a, qword b)
{
  return ((qword)(vec_mulo((vec_short8)(a), (vec_short8)(b))));
}

static __inline qword si_mpyu(qword a, qword b)
{
  return ((qword)(vec_mulo((vec_ushort8)(a), (vec_ushort8)(b))));
}

static __inline qword si_mpyi(qword a, short b)
{
  return ((qword)(vec_mulo((vec_short8)(a), 
			   vec_splat((vec_short8)(si_from_short(b)), 1))));
}

static __inline qword si_mpyui(qword a, unsigned short b)
{
  return ((qword)(vec_mulo((vec_ushort8)(a), 
			   vec_splat((vec_ushort8)(si_from_ushort(b)), 1))));
}

/* Multiply and Shift Right
 */
static __inline qword si_mpys(qword a, qword b)
{
  return ((qword)(vec_sra(vec_mulo((vec_short8)(a), (vec_short8)(b)), ((vec_uint4){16,16,16,16}))));
}

/* Nand
 */
static __inline qword si_nand(qword a, qword b)
{
  vec_uchar16 d;

  d = vec_and((vec_uchar16)(a), (vec_uchar16)(b));
  return ((qword)(vec_nor(d, d)));
}

/* Negative Multiply Add
 */
static __inline qword si_dfnma(qword a, qword b, qword c)
{
  union {
    vec_double2 v;
    double d[2];
  } aa, bb, cc, dd;

  aa.v = (vec_double2)(a);
  bb.v = (vec_double2)(b);
  cc.v = (vec_double2)(c);
  dd.d[0] = -cc.d[0] - aa.d[0] * bb.d[0];
  dd.d[1] = -cc.d[1] - aa.d[1] * bb.d[1];
  return ((qword)(dd.v));
}

/* Negative Multiply and Subtract
 */
static __inline qword si_fnms(qword a, qword b, qword c)
{
  return ((qword)(vec_nmsub((vec_float4)(a), (vec_float4)(b), (vec_float4)(c))));
}

static __inline qword si_dfnms(qword a, qword b, qword c)
{
  union {
    vec_double2 v;
    double d[2];
  } aa, bb, cc, dd;

  aa.v = (vec_double2)(a);
  bb.v = (vec_double2)(b);
  cc.v = (vec_double2)(c);
  dd.d[0] = cc.d[0] - aa.d[0] * bb.d[0];
  dd.d[1] = cc.d[1] - aa.d[1] * bb.d[1];
  return ((qword)(dd.v));
}

/* Nor
 */
static __inline qword si_nor(qword a, qword b)
{
  return ((qword)(vec_nor((vec_uchar16)(a), (vec_uchar16)(b))));
}

/* Or
 */
static __inline qword si_or(qword a, qword b)
{
  return ((qword)(vec_or((vec_uchar16)(a), (vec_uchar16)(b))));
}

static __inline qword si_orbi(qword a, unsigned char b)
{
  return ((qword)(vec_or((vec_uchar16)(a), 
			 vec_splat((vec_uchar16)(si_from_uchar(b)), 3))));
}

static __inline qword si_orhi(qword a, unsigned short b)
{
  return ((qword)(vec_or((vec_ushort8)(a), 
			  vec_splat((vec_ushort8)(si_from_ushort(b)), 1))));
}

static __inline qword si_ori(qword a, unsigned int b)
{
  return ((qword)(vec_or((vec_uint4)(a), 
			  vec_splat((vec_uint4)(si_from_uint(b)), 0))));
}

/* Or Complement
 */
static __inline qword si_orc(qword a, qword b)
{
  return ((qword)(vec_or((vec_uchar16)(a), vec_nor((vec_uchar16)(b), (vec_uchar16)(b)))));
}


/* Or Across
 */
static __inline qword si_orx(qword a)
{
  vec_uchar16 tmp;
  tmp = (vec_uchar16)(a);
  tmp = vec_or(tmp, vec_sld(tmp, tmp, 8));
  tmp = vec_or(tmp, vec_sld(tmp, tmp, 4));
  return ((qword)(vec_and(tmp, ((vec_uchar16){0xFF,0xFF,0xFF,0xFF, 0x00,0x00,0x00,0x00,
				              0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00}))));
}


/* Estimates
 */
static __inline qword si_frest(qword a)
{
  return ((qword)(vec_re((vec_float4)(a))));
}

static __inline qword si_frsqest(qword a)
{
  return ((qword)(vec_rsqrte((vec_float4)(a))));
}

#define si_fi(_a, _d)		(_d)

/* Channel Read and Write
 */
#define si_rdch(_channel)		((qword)(vec_splat_u8(0)))	/* not mappable */
#define si_rchcnt(_channel)		((qword)(vec_splat_u8(0)))	/* not mappable */
#define si_wrch(_channel, _a)		/* not mappable */

/* Rotate Left
 */
static __inline qword si_roth(qword a, qword b)
{
  return ((qword)(vec_rl((vec_ushort8)(a), (vec_ushort8)(b))));
}

static __inline qword si_rot(qword a, qword b)
{
  return ((qword)(vec_rl((vec_uint4)(a), (vec_uint4)(b))));
}

static __inline qword si_rothi(qword a, int b)
{
  return ((qword)(vec_rl((vec_ushort8)(a), 
			 vec_splat((vec_ushort8)(si_from_int(b)), 1))));
}

static __inline qword si_roti(qword a, int b)
{
  return ((qword)(vec_rl((vec_uint4)(a), 
			 vec_splat((vec_uint4)(si_from_int(b)), 0))));
}

/* Rotate Left with Mask
 */
static __inline qword si_rothm(qword a, qword b)
{
  vec_ushort8 neg_b;
  vec_ushort8 mask;

  neg_b = (vec_ushort8)vec_sub(vec_splat_s16(0), (vec_short8)(b));
  mask = vec_sra(vec_sl(neg_b, vec_splat_u16(11)), vec_splat_u16(15));
  return ((qword)(vec_andc(vec_sr((vec_ushort8)(a), neg_b), mask)));
}

static __inline qword si_rotm(qword a, qword b)
{
  vec_uint4 neg_b;
  vec_uint4 mask;

  neg_b = (vec_uint4)vec_sub(vec_splat_s32(0), (vec_int4)(b));
  mask = vec_sra(vec_sl(neg_b, ((vec_uint4){26,26,26,26})), ((vec_uint4){31,31,31,31}));
  return ((qword)(vec_andc(vec_sr((vec_uint4)(a), neg_b), mask)));
}

static __inline qword si_rothmi(qword a, int b)
{
  vec_ushort8 neg_b;
  vec_ushort8 mask;

  neg_b = vec_splat((vec_ushort8)(si_from_int(-b)), 1);
  mask = vec_sra(vec_sl(neg_b, vec_splat_u16(11)), vec_splat_u16(15));
  return ((qword)(vec_andc(vec_sr((vec_ushort8)(a), neg_b), mask)));
}

static __inline qword si_rotmi(qword a, int b)
{
  vec_uint4 neg_b;
  vec_uint4 mask;

  neg_b = vec_splat((vec_uint4)(si_from_int(-b)), 0);
  mask = vec_sra(vec_sl(neg_b, ((vec_uint4){26,26,26,26})), ((vec_uint4){31,31,31,31}));
  return ((qword)(vec_andc(vec_sr((vec_uint4)(a), neg_b), mask)));
}


/* Rotate Left Algebraic with Mask
 */
static __inline qword si_rotmah(qword a, qword b)
{
  vec_ushort8 neg_b;
  vec_ushort8 mask;

  neg_b = (vec_ushort8)vec_sub(vec_splat_s16(0), (vec_short8)(b));
  mask = vec_sra(vec_sl(neg_b, vec_splat_u16(11)), vec_splat_u16(15));
  return ((qword)(vec_sra((vec_short8)(a), (vec_ushort8)vec_or(neg_b, mask))));
}

static __inline qword si_rotma(qword a, qword b)
{
  vec_uint4 neg_b;
  vec_uint4 mask;

  neg_b = (vec_uint4)vec_sub(vec_splat_s32(0), (vec_int4)(b));
  mask = vec_sra(vec_sl(neg_b, ((vec_uint4){26,26,26,26})), ((vec_uint4){31,31,31,31}));
  return ((qword)(vec_sra((vec_int4)(a), (vec_uint4)vec_or(neg_b, mask))));
}


static __inline qword si_rotmahi(qword a, int b)
{
  vec_ushort8 neg_b;
  vec_ushort8 mask;

  neg_b = vec_splat((vec_ushort8)(si_from_int(-b)), 1);
  mask = vec_sra(vec_sl(neg_b, vec_splat_u16(11)), vec_splat_u16(15));
  return ((qword)(vec_sra((vec_short8)(a), (vec_ushort8)vec_or(neg_b, mask))));
}

static __inline qword si_rotmai(qword a, int b)
{
  vec_uint4 neg_b;
  vec_uint4 mask;

  neg_b = vec_splat((vec_uint4)(si_from_int(-b)), 0);
  mask = vec_sra(vec_sl(neg_b, ((vec_uint4){26,26,26,26})), ((vec_uint4){31,31,31,31}));
  return ((qword)(vec_sra((vec_int4)(a), (vec_uint4)vec_or(neg_b, mask))));
}


/* Rotate Left Quadword by Bytes with Mask
 */
static __inline qword si_rotqmbyi(qword a, int count)
{
  union {
    vec_uchar16 v;
    int i[4];
  } x;
  vec_uchar16 mask;

  count = 0 - count;
  x.i[3] = count << 3;
  mask = (count & 0x10) ? vec_splat_u8(0) : vec_splat_u8(-1);

  return ((qword)(vec_and(vec_sro((vec_uchar16)(a), x.v), mask)));
}


static __inline qword si_rotqmby(qword a, qword count)
{
  union {
    vec_uchar16 v;
    int i[4];
  } x;
  int cnt;
  vec_uchar16 mask;

  x.v = (vec_uchar16)(count);
  x.i[0] = cnt = (0 - x.i[0]) << 3;

  x.v = vec_splat(x.v, 3);
  mask = (cnt & 0x80) ? vec_splat_u8(0) : vec_splat_u8(-1);

  return ((qword)(vec_and(vec_sro((vec_uchar16)(a), x.v), mask)));
}


/* Rotate Left Quadword by Bytes
 */
static __inline qword si_rotqbyi(qword a, int count)
{
  union {
    vec_uchar16 v;
    int i[4];
  } left, right;
 
  count <<= 3;
  left.i[3] = count;
  right.i[3] = 0 - count;
  return ((qword)(vec_or(vec_slo((vec_uchar16)(a), left.v), vec_sro((vec_uchar16)(a), right.v))));
}

static __inline qword si_rotqby(qword a, qword count)
{
  vec_uchar16 left, right;
 
  left = vec_sl(vec_splat((vec_uchar16)(count), 3), vec_splat_u8(3));
  right = vec_sub(vec_splat_u8(0), left);
  return ((qword)(vec_or(vec_slo((vec_uchar16)(a), left), vec_sro((vec_uchar16)(a), right))));
}

/* Rotate Left Quadword by Bytes Bit Count
 */
static __inline qword si_rotqbybi(qword a, qword count)
{
  vec_uchar16 left, right;

  left = vec_splat((vec_uchar16)(count), 3);
  right = vec_sub(vec_splat_u8(7), left);
  return ((qword)(vec_or(vec_slo((vec_uchar16)(a), left), vec_sro((vec_uchar16)(a), right))));
}


/* Rotate Left Quadword by Bytes Bit Count
 */
static __inline qword si_rotqbii(qword a, int count)
{
  vec_uchar16 x, y;
  vec_uchar16 result;
 
  x = vec_splat((vec_uchar16)(si_from_int(count & 7)), 3);
  y = (vec_uchar16)(vec_sr((vec_uint4)vec_sro((vec_uchar16)(a), ((vec_uchar16)((vec_uint4){0,0,0,120}))),
			   (vec_uint4)vec_sub(vec_splat_u8(8), x)));
  result = vec_or(vec_sll((qword)(a), x), y);
  return ((qword)(result));
}

static __inline qword si_rotqbi(qword a, qword count)
{
  vec_uchar16 x, y;
  vec_uchar16 result;
 
  x = vec_and(vec_splat((vec_uchar16)(count), 3), vec_splat_u8(7));
  y = (vec_uchar16)(vec_sr((vec_uint4)vec_sro((vec_uchar16)(a), ((vec_uchar16)((vec_uint4){0,0,0,120}))),
			   (vec_uint4)vec_sub(vec_splat_u8(8), x)));
  
  result = vec_or(vec_sll((qword)(a), x), y);
  return ((qword)(result));
}


/* Rotate Left Quadword and Mask by Bits
 */
static __inline qword si_rotqmbii(qword a, int count)
{
  return ((qword)(vec_srl((vec_uchar16)(a), vec_splat((vec_uchar16)(si_from_int(0 - count)), 3))));
}

static __inline qword si_rotqmbi(qword a, qword count)
{
  return ((qword)(vec_srl((vec_uchar16)(a), vec_sub(vec_splat_u8(0), vec_splat((vec_uchar16)(count), 3)))));
}


/* Rotate Left Quadword and Mask by Bytes with Bit Count
 */
static __inline qword si_rotqmbybi(qword a, qword count)
{
  union {
    vec_uchar16 v;
    int i[4];
  } x;
  int cnt;
  vec_uchar16 mask;

  x.v = (vec_uchar16)(count);
  x.i[0] = cnt = 0 - (x.i[0] & ~7);
  x.v = vec_splat(x.v, 3);
  mask = (cnt & 0x80) ? vec_splat_u8(0) : vec_splat_u8(-1);

  return ((qword)(vec_and(vec_sro((vec_uchar16)(a), x.v), mask)));
}




/* Round Double to Float
 */
static __inline qword si_frds(qword a)
{
  union {
    vec_float4 v;
    float f[4];
  } d;
  union {
    vec_double2 v;
    double d[2];
  } in;

  in.v = (vec_double2)(a);
  d.v = (vec_float4){0.0f};
  d.f[0] = (float)in.d[0];
  d.f[2] = (float)in.d[1];

  return ((qword)(d.v));
}

/* Select Bits
 */
static __inline qword si_selb(qword a, qword b, qword c)
{
  return ((qword)(vec_sel((vec_uchar16)(a), (vec_uchar16)(b), (vec_uchar16)(c))));
}


/* Shuffle Bytes
 */
static __inline qword si_shufb(qword a, qword b, qword pattern)
{
  vec_uchar16 pat;

  pat = vec_sel(((vec_uchar16){0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10,11,12,13,14,15}), 
		vec_sr((vec_uchar16)(pattern), vec_splat_u8(3)),
		vec_sra((vec_uchar16)(pattern), vec_splat_u8(7)));
  return ((qword)(vec_perm(vec_perm(a, b, pattern), 
			   ((vec_uchar16){0, 0, 0, 0, 0, 0, 0, 0,
				          0xFF, 0xFF, 0xFF, 0xFF, 0x80, 0x80, 0x80, 0x80}),
			   pat)));
}


/* Shift Left
 */
static __inline qword si_shlh(qword a, qword b)
{
  vec_ushort8 mask;

  mask = (vec_ushort8)vec_sra(vec_sl((vec_ushort8)(b), vec_splat_u16(11)), vec_splat_u16(15));
  return ((qword)(vec_andc(vec_sl((vec_ushort8)(a), (vec_ushort8)(b)), mask)));
}

static __inline qword si_shl(qword a, qword b)
{
  vec_uint4 mask;

  mask = (vec_uint4)vec_sra(vec_sl((vec_uint4)(b), ((vec_uint4){26,26,26,26})), ((vec_uint4){31,31,31,31}));
  return ((qword)(vec_andc(vec_sl((vec_uint4)(a), (vec_uint4)(b)), mask)));
}


static __inline qword si_shlhi(qword a, unsigned int b)
{
  vec_ushort8 mask;
  vec_ushort8 bv;

  bv = vec_splat((vec_ushort8)(si_from_int(b)), 1);
  mask = (vec_ushort8)vec_sra(vec_sl(bv, vec_splat_u16(11)), vec_splat_u16(15));
  return ((qword)(vec_andc(vec_sl((vec_ushort8)(a), bv), mask)));
}

static __inline qword si_shli(qword a, unsigned int b)
{
  vec_uint4 bv;
  vec_uint4 mask;

  bv = vec_splat((vec_uint4)(si_from_uint(b)), 0);
  mask = (vec_uint4)vec_sra(vec_sl(bv, ((vec_uint4){26,26,26,26})), ((vec_uint4){31,31,31,31}));
  return ((qword)(vec_andc(vec_sl((vec_uint4)(a), bv), mask)));
}


/* Shift Left Quadword
 */
static __inline qword si_shlqbii(qword a, unsigned int count)
{
  vec_uchar16 x;

  x = vec_splat((vec_uchar16)(si_from_uint(count)), 3);
  return ((qword)(vec_sll((vec_uchar16)(a), x)));
}

static __inline qword si_shlqbi(qword a, qword count)
{
  vec_uchar16 x;

  x = vec_splat((vec_uchar16)(count), 3);
  return ((qword)(vec_sll((vec_uchar16)(a), x)));
}


/* Shift Left Quadword by Bytes
 */
static __inline qword si_shlqbyi(qword a, unsigned int count)
{
  union {
    vec_uchar16 v;
    int i[4];
  } x;
  vec_uchar16 mask;

  x.i[3] = count << 3;
  mask = (count & 0x10) ? vec_splat_u8(0) : vec_splat_u8(-1);
  return ((qword)(vec_and(vec_slo((vec_uchar16)(a), x.v), mask)));
}

static __inline qword si_shlqby(qword a, qword count)
{
  union {
    vec_uchar16 v;
    unsigned int i[4];
  } x;
  unsigned int cnt;
  vec_uchar16 mask;

  x.v = vec_sl(vec_splat((vec_uchar16)(count), 3), vec_splat_u8(3));
  cnt = x.i[0];
  mask = (cnt & 0x80) ? vec_splat_u8(0) : vec_splat_u8(-1);
  return ((qword)(vec_and(vec_slo((vec_uchar16)(a), x.v), mask)));
}

/* Shift Left Quadword by Bytes with Bit Count
 */
static __inline qword si_shlqbybi(qword a, qword count)
{
  union {
    vec_uchar16 v;
    int i[4];
  } x;
  unsigned int cnt;
  vec_uchar16 mask;

  x.v = vec_splat((vec_uchar16)(count), 3);
  cnt = x.i[0];
  mask = (cnt & 0x80) ? vec_splat_u8(0) : vec_splat_u8(-1);
  return ((qword)(vec_and(vec_slo((vec_uchar16)(a), x.v), mask)));
}


/* Stop and Signal
 */
#define si_stop(_type)		SPU_STOP_ACTION
#define si_stopd(a, b, c)	SPU_STOP_ACTION


/* Subtract
 */
static __inline qword si_sfh(qword a, qword b)
{
  return ((qword)(vec_sub((vec_ushort8)(b), (vec_ushort8)(a))));
}

static __inline qword si_sf(qword a, qword b)
{
  return ((qword)(vec_sub((vec_uint4)(b), (vec_uint4)(a))));
}

static __inline qword si_fs(qword a, qword b)
{
  return ((qword)(vec_sub((vec_float4)(a), (vec_float4)(b))));
}

static __inline qword si_dfs(qword a, qword b)
{
  union {
    vec_double2 v;
    double d[2];
  } aa, bb, dd;

  aa.v = (vec_double2)(a);
  bb.v = (vec_double2)(b);
  dd.d[0] = aa.d[0] - bb.d[0];
  dd.d[1] = aa.d[1] - bb.d[1];
  return ((qword)(dd.v));
}

static __inline qword si_sfhi(qword a, short b)
{
  return ((qword)(vec_sub(vec_splat((vec_short8)(si_from_short(b)), 1),
			  (vec_short8)(a))));
}

static __inline qword si_sfi(qword a, int b)
{
  return ((qword)(vec_sub(vec_splat((vec_int4)(si_from_int(b)), 0),
			  (vec_int4)(a))));
}

/* Subtract word extended
 */
#define si_sfx(_a, _b, _c)	((qword)(vec_add(vec_add((vec_uint4)(_b), 				\
							 vec_nor((vec_uint4)(_a), (vec_uint4)(_a))), 	\
						 vec_and((vec_uint4)(_c), vec_splat_u32(1)))))


/* Sum Bytes into Shorts
 */
static __inline qword si_sumb(qword a, qword b)
{
  vec_uint4 zero = (vec_uint4){0};
  vec_ushort8 sum_a, sum_b;
  
  sum_a = (vec_ushort8)vec_sum4s((vec_uchar16)(a), zero);
  sum_b = (vec_ushort8)vec_sum4s((vec_uchar16)(b), zero);

  return ((qword)(vec_perm(sum_a, sum_b, ((vec_uchar16){18, 19,  2,  3, 22, 23,  6,  7,
					                26, 27, 10, 11, 30, 31, 14, 15}))));
}

/* Exclusive OR
 */
static __inline qword si_xor(qword a, qword b)
{
  return ((qword)(vec_xor((vec_uchar16)(a), (vec_uchar16)(b))));
}

static __inline qword si_xorbi(qword a, unsigned char b)
{
  return ((qword)(vec_xor((vec_uchar16)(a), 
			  vec_splat((vec_uchar16)(si_from_uchar(b)), 3))));
}

static __inline qword si_xorhi(qword a, unsigned short b)
{
  return ((qword)(vec_xor((vec_ushort8)(a), 
			  vec_splat((vec_ushort8)(si_from_ushort(b)), 1))));
}

static __inline qword si_xori(qword a, unsigned int b)
{
  return ((qword)(vec_xor((vec_uint4)(a), 
			  vec_splat((vec_uint4)(si_from_uint(b)), 0))));
}


/* Generate Controls for Sub-Quadword Insertion
 */
static __inline qword si_cbd(qword a, int imm)
{
  union {
    vec_uint4 v;
    unsigned char c[16];
  } shmask;

  shmask.v = ((vec_uint4){0x10111213, 0x14151617, 0x18191A1B, 0x1C1D1E1F});
  shmask.c[(si_to_uint(a) + (unsigned int)(imm)) & 0xF] = 0x03;
  return ((qword)(shmask.v));
}

static __inline qword si_cdd(qword a, int imm)
{
  union {
    vec_uint4 v;
    unsigned long long ll[2];
  } shmask;

  shmask.v = ((vec_uint4){0x10111213, 0x14151617, 0x18191A1B, 0x1C1D1E1F});
  shmask.ll[((si_to_uint(a) + (unsigned int)(imm)) >> 3) & 0x1] = 0x0001020304050607ULL;
  return ((qword)(shmask.v));
}

static __inline qword si_chd(qword a, int imm)
{
  union {
    vec_uint4 v;
    unsigned short s[8];
  } shmask;

  shmask.v = ((vec_uint4){0x10111213, 0x14151617, 0x18191A1B, 0x1C1D1E1F});
  shmask.s[((si_to_uint(a) + (unsigned int)(imm)) >> 1) & 0x7] = 0x0203;
  return ((qword)(shmask.v));
}

static __inline qword si_cwd(qword a, int imm)
{
  union {
    vec_uint4 v;
    unsigned int i[4];
  } shmask;

  shmask.v = ((vec_uint4){0x10111213, 0x14151617, 0x18191A1B, 0x1C1D1E1F});
  shmask.i[((si_to_uint(a) + (unsigned int)(imm)) >> 2) & 0x3] = 0x00010203;
  return ((qword)(shmask.v));
}

static __inline qword si_cbx(qword a, qword b)
{
  union {
    vec_uint4 v;
    unsigned char c[16];
  } shmask;

  shmask.v = ((vec_uint4){0x10111213, 0x14151617, 0x18191A1B, 0x1C1D1E1F});
  shmask.c[si_to_uint((qword)(vec_add((vec_uint4)(a), (vec_uint4)(b)))) & 0xF] = 0x03;
  return ((qword)(shmask.v));
}


static __inline qword si_cdx(qword a, qword b)
{
  union {
    vec_uint4 v;
    unsigned long long ll[2];
  } shmask;

  shmask.v = ((vec_uint4){0x10111213, 0x14151617, 0x18191A1B, 0x1C1D1E1F});
  shmask.ll[(si_to_uint((qword)(vec_add((vec_uint4)(a), (vec_uint4)(b)))) >> 3) & 0x1] = 0x0001020304050607ULL;
  return ((qword)(shmask.v));
}

static __inline qword si_chx(qword a, qword b)
{
  union {
    vec_uint4 v;
    unsigned short s[8];
  } shmask;

  shmask.v = ((vec_uint4){0x10111213, 0x14151617, 0x18191A1B, 0x1C1D1E1F});
  shmask.s[(si_to_uint((qword)(vec_add((vec_uint4)(a), (vec_uint4)(b)))) >> 1) & 0x7] = 0x0203;
  return ((qword)(shmask.v));
}

static __inline qword si_cwx(qword a, qword b)
{
  union {
    vec_uint4 v;
    unsigned int i[4];
  } shmask;

  shmask.v = ((vec_uint4){0x10111213, 0x14151617, 0x18191A1B, 0x1C1D1E1F});
  shmask.i[(si_to_uint((qword)(vec_add((vec_uint4)(a), (vec_uint4)(b)))) >> 2) & 0x3] = 0x00010203;
  return ((qword)(shmask.v));
}


/* Constant Formation
 */
static __inline qword si_il(signed short imm)
{
  return ((qword)(vec_splat((vec_int4)(si_from_int((signed int)(imm))), 0)));
}


static __inline qword si_ila(unsigned int imm)
{
  return ((qword)(vec_splat((vec_uint4)(si_from_uint(imm)), 0)));
}

static __inline qword si_ilh(signed short imm)
{
  return ((qword)(vec_splat((vec_short8)(si_from_short(imm)), 1)));
}

static __inline qword si_ilhu(signed short imm)
{
  return ((qword)(vec_splat((vec_uint4)(si_from_uint((unsigned int)(imm) << 16)), 0)));
}

static __inline qword si_iohl(qword a, unsigned short imm)
{
  return ((qword)(vec_or((vec_uint4)(a), vec_splat((vec_uint4)(si_from_uint((unsigned int)(imm))), 0))));
}

/* No Operation
 */
#define si_lnop()		/* do nothing */
#define si_nop()		/* do nothing */


/* Memory Load and Store
 */
static __inline qword si_lqa(unsigned int imm)
{
  return ((qword)(vec_ld(0, (vector unsigned char *)(imm))));
}

static __inline qword si_lqd(qword a, unsigned int imm)
{
  return ((qword)(vec_ld(si_to_uint(a) & ~0xF, (vector unsigned char *)(imm))));
}

static __inline qword si_lqr(unsigned int imm)
{
  return ((qword)(vec_ld(0, (vector unsigned char *)(imm))));
}

static __inline qword si_lqx(qword a, qword b)
{
  return ((qword)(vec_ld(si_to_uint((qword)(vec_add((vec_uint4)(a), (vec_uint4)(b)))), (vector unsigned char *)(0))));
}

static __inline void si_stqa(qword a, unsigned int imm)
{
  vec_st((vec_uchar16)(a), 0, (vector unsigned char *)(imm));
}

static __inline void si_stqd(qword a, qword b, unsigned int imm)
{
  vec_st((vec_uchar16)(a), si_to_uint(b) & ~0xF, (vector unsigned char *)(imm));
}

static __inline void si_stqr(qword a, unsigned int imm)
{
  vec_st((vec_uchar16)(a), 0, (vector unsigned char *)(imm));
}

static __inline void si_stqx(qword a, qword b, qword c)
{
  vec_st((vec_uchar16)(a), 
	 si_to_uint((qword)(vec_add((vec_uint4)(b), (vec_uint4)(c)))),
	 (vector unsigned char *)(0));
}

#endif /* !__SPU__ */
#endif /* !_SI2VMX_H_ */

