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

#ifndef _SPU2VMX_H_
#define _SPU2VMX_H_	1

#ifdef __cplusplus

#ifndef __SPU__

#include <si2vmx.h>

/* spu_absd (absolute difference)
 * ========
 */
static __inline vec_uchar16 spu_absd(vec_uchar16 a, vec_uchar16 b)
{
  return ((vec_uchar16)(si_absdb((qword)(a), (qword)(b))));

}


/* spu_add
 * =======
 */
static __inline vec_uint4 spu_add(vec_uint4 a, vec_uint4 b)
{
  return ((vec_uint4)(si_a((qword)(a), (qword)(b))));
}

static __inline vec_int4 spu_add(vec_int4 a, vec_int4 b)
{
  return ((vec_int4)(si_a((qword)(a), (qword)(b))));
}

static __inline vec_ushort8 spu_add(vec_ushort8 a, vec_ushort8 b)
{
  return ((vec_ushort8)(si_ah((qword)(a), (qword)(b))));
}

static __inline vec_short8 spu_add(vec_short8 a, vec_short8 b)
{
  return ((vec_short8)(si_ah((qword)(a), (qword)(b))));
}

static __inline vec_uint4 spu_add(vec_uint4 a, unsigned int b)
{
  return ((vec_uint4)(si_ai((qword)(a), (int)(b))));
}

static __inline vec_int4 spu_add(vec_int4 a, int b)
{
  return ((vec_int4)(si_ai((qword)(a), b)));
}

static __inline vec_ushort8 spu_add(vec_ushort8 a, unsigned short b)
{
  return ((vec_ushort8)(si_ahi((qword)(a), (short)(b))));
}

static __inline vec_short8 spu_add(vec_short8 a, short b)
{
  return ((vec_short8)(si_ahi((qword)(a), b)));
}

static __inline vec_float4 spu_add(vec_float4 a, vec_float4 b)
{
  return ((vec_float4)(si_fa((qword)(a), (qword)(b))));
}

static __inline vec_double2 spu_add(vec_double2 a, vec_double2 b)
{
  return ((vec_double2)(si_dfa((qword)(a), (qword)(b))));
}


/* spu_addx
 * ========
 */
static __inline vec_uint4 spu_addx(vec_uint4 a, vec_uint4 b, vec_uint4 c)
{
  return ((vec_uint4)(si_addx((qword)(a), (qword)(b), (qword)(c))));
}

static __inline vec_int4 spu_addx(vec_int4 a, vec_int4 b, vec_int4 c)
{
  return ((vec_int4)(si_addx((qword)(a), (qword)(b), (qword)(c))));
}


/* spu_and
 * =======
 */
static __inline vec_uchar16 spu_and(vec_uchar16 a, vec_uchar16 b)
{
  return ((vec_uchar16)(si_and((qword)(a), (qword)(b))));
}

static __inline vec_char16 spu_and(vec_char16 a, vec_char16 b)
{
  return ((vec_char16)(si_and((qword)(a), (qword)(b))));
}

static __inline vec_ushort8 spu_and(vec_ushort8 a, vec_ushort8 b)
{
  return ((vec_ushort8)(si_and((qword)(a), (qword)(b))));
}

static __inline vec_short8 spu_and(vec_short8 a, vec_short8 b)
{
  return ((vec_short8)(si_and((qword)(a), (qword)(b))));
}

static __inline vec_uint4 spu_and(vec_uint4 a, vec_uint4 b)
{
  return ((vec_uint4)(si_and((qword)(a), (qword)(b))));
}

static __inline vec_int4 spu_and(vec_int4 a, vec_int4 b)
{
  return ((vec_int4)(si_and((qword)(a), (qword)(b))));
}

static __inline vec_float4 spu_and(vec_float4 a, vec_float4 b)
{
  return ((vec_float4)(si_and((qword)(a), (qword)(b))));
}

static __inline vec_ullong2 spu_and(vec_ullong2 a, vec_ullong2 b)
{
  return ((vec_ullong2)(si_and((qword)(a), (qword)(b))));
}

static __inline vec_llong2 spu_and(vec_llong2 a, vec_llong2 b)
{
  return ((vec_llong2)(si_and((qword)(a), (qword)(b))));
}

static __inline vec_double2 spu_and(vec_double2 a, vec_double2 b)
{
  return ((vec_double2)(si_and((qword)(a), (qword)(b))));
}

static __inline vec_uchar16 spu_and(vec_uchar16 a, unsigned char b)
{
  return ((vec_uchar16)(si_andbi((qword)(a), (signed char)(b))));
}


static __inline vec_char16 spu_and(vec_char16 a, signed char b)
{
  return ((vec_char16)(si_andbi((qword)(a), b)));
}

static __inline vec_ushort8 spu_and(vec_ushort8 a, unsigned short b)
{
  return ((vec_ushort8)(si_andhi((qword)(a), (signed short)(b))));
}

static __inline vec_short8 spu_and(vec_short8 a, signed short b)
{
  return ((vec_short8)(si_andhi((qword)(a), b)));
}

static __inline vec_uint4 spu_and(vec_uint4 a, unsigned int b)
{
  return ((vec_uint4)(si_andi((qword)(a), (signed int)(b))));
}

static __inline vec_int4 spu_and(vec_int4 a, signed int b)
{
  return ((vec_int4)(si_andi((qword)(a), b)));
}


/* spu_andc
 * ========
 */
#define spu_andc(_a, _b)	vec_andc(_a, _b)


/* spu_avg
 * =======
 */
#define spu_avg(_a, _b)		vec_avg(_a, _b)
  

/* spu_bisled
 * spu_bisled_d
 * spu_bisled_e
 * ============
 */
#define spu_bisled(_func)	/* not mappable */
#define spu_bisled_d(_func)	/* not mappable */
#define spu_bisled_e(_func)	/* not mappable */

/* spu_cmpabseq
 * ============
 */
static __inline vec_uint4 spu_cmpabseq(vec_float4 a, vec_float4 b)
{
  return ((vec_uint4)(si_fcmeq((qword)(a), (qword)(b))));

}

static __inline vec_ullong2 spu_cmpabseq(vec_double2 a, vec_double2 b)
{
  return ((vec_ullong2)(si_dfcmeq((qword)(a), (qword)(b))));
}


/* spu_cmpabsgt
 * ============
 */
static __inline vec_uint4 spu_cmpabsgt(vec_float4 a, vec_float4 b)
{
  return ((vec_uint4)(si_fcmgt((qword)(a), (qword)(b))));
}

static __inline vec_ullong2 spu_cmpabsgt(vec_double2 a, vec_double2 b)
{
  return ((vec_ullong2)(si_dfcmgt((qword)(a), (qword)(b))));
}


/* spu_cmpeq
 * ========
 */
static __inline vec_uchar16 spu_cmpeq(vec_uchar16 a, vec_uchar16 b)
{
  return ((vec_uchar16)(si_ceqb((qword)(a), (qword)(b))));
}

static __inline vec_uchar16 spu_cmpeq(vec_char16 a, vec_char16 b)
{
  return ((vec_uchar16)(si_ceqb((qword)(a), (qword)(b))));
}

static __inline vec_ushort8 spu_cmpeq(vec_ushort8 a, vec_ushort8 b)
{
  return ((vec_ushort8)(si_ceqh((qword)(a), (qword)(b))));
}

static __inline vec_ushort8 spu_cmpeq(vec_short8 a, vec_short8 b)
{
  return ((vec_ushort8)(si_ceqh((qword)(a), (qword)(b))));
}

static __inline vec_uint4 spu_cmpeq(vec_uint4 a, vec_uint4 b)
{
  return ((vec_uint4)(si_ceq((qword)(a), (qword)(b))));
}

static __inline vec_uint4 spu_cmpeq(vec_int4 a, vec_int4 b)
{
  return ((vec_uint4)(si_ceq((qword)(a), (qword)(b))));
}

static __inline vec_uint4 spu_cmpeq(vec_float4 a, vec_float4 b)
{
  return ((vec_uint4)(si_fceq((qword)(a), (qword)(b))));
}

static __inline vec_uchar16 spu_cmpeq(vec_uchar16 a, unsigned char b)
{
  return ((vec_uchar16)(si_ceqbi((qword)(a), (signed char)(b))));
}

static __inline vec_uchar16 spu_cmpeq(vec_char16 a, signed char b)
{
  return ((vec_uchar16)(si_ceqbi((qword)(a), b)));
}

static __inline vec_ushort8 spu_cmpeq(vec_ushort8 a, unsigned short b)
{
  return ((vec_ushort8)(si_ceqhi((qword)(a), (signed short)(b))));
}

static __inline vec_ushort8 spu_cmpeq(vec_short8 a, signed short b)
{
  return ((vec_ushort8)(si_ceqhi((qword)(a), b)));
}

static __inline vec_uint4 spu_cmpeq(vec_uint4 a, unsigned int b)
{
  return ((vec_uint4)(si_ceqi((qword)(a), (signed int)(b))));
}

static __inline vec_uint4 spu_cmpeq(vec_int4 a, signed int b)
{
  return ((vec_uint4)(si_ceqi((qword)(a), b)));
}

static __inline vec_ullong2 spu_cmpeq(vec_double2 a, vec_double2 b)
{
  return ((vec_ullong2)(si_dfceq((qword)(a), (qword)(b))));
}


/* spu_cmpgt
 * ========
 */
static __inline vec_uchar16 spu_cmpgt(vec_uchar16 a, vec_uchar16 b)
{
  return ((vec_uchar16)(si_clgtb((qword)(a), (qword)(b))));
}

static __inline vec_uchar16 spu_cmpgt(vec_char16 a, vec_char16 b)
{
  return ((vec_uchar16)(si_cgtb((qword)(a), (qword)(b))));
}

static __inline vec_ushort8 spu_cmpgt(vec_ushort8 a, vec_ushort8 b)
{
  return ((vec_ushort8)(si_clgth((qword)(a), (qword)(b))));
}

static __inline vec_ushort8 spu_cmpgt(vec_short8 a, vec_short8 b)
{
  return ((vec_ushort8)(si_cgth((qword)(a), (qword)(b))));
}

static __inline vec_uint4 spu_cmpgt(vec_uint4 a, vec_uint4 b)
{
  return ((vec_uint4)(si_clgt((qword)(a), (qword)(b))));
}

static __inline vec_uint4 spu_cmpgt(vec_int4 a, vec_int4 b)
{
  return ((vec_uint4)(si_cgt((qword)(a), (qword)(b))));
}

static __inline vec_uint4 spu_cmpgt(vec_float4 a, vec_float4 b)
{
  return ((vec_uint4)(si_fcgt((qword)(a), (qword)(b))));
}

static __inline vec_uchar16 spu_cmpgt(vec_uchar16 a, unsigned char b)
{
  return ((vec_uchar16)(si_clgtbi((qword)(a), b)));
}

static __inline vec_uchar16 spu_cmpgt(vec_char16 a, signed char b)
{
  return ((vec_uchar16)(si_cgtbi((qword)(a), b)));
}

static __inline vec_ushort8 spu_cmpgt(vec_ushort8 a, unsigned short b)
{
  return ((vec_ushort8)(si_clgthi((qword)(a), b)));
}

static __inline vec_ushort8 spu_cmpgt(vec_short8 a, signed short b)
{
  return ((vec_ushort8)(si_cgthi((qword)(a), b)));
}

static __inline vec_uint4 spu_cmpgt(vec_uint4 a, unsigned int b)
{
  return ((vec_uint4)(si_clgti((qword)(a), b)));
}

static __inline vec_uint4 spu_cmpgt(vec_int4 a, signed int b)
{
  return ((vec_uint4)(si_cgti((qword)(a), b)));
}

static __inline vec_ullong2 spu_cmpgt(vec_double2 a, vec_double2 b)
{
  return ((vec_ullong2)(si_dfcgt((qword)(a), (qword)(b))));
}


/* spu_cntb
 * ========
 */
static __inline vec_uchar16 spu_cntb(vec_uchar16 a)
{
  return ((vec_uchar16)(si_cntb((qword)(a))));
}


static __inline vec_uchar16 spu_cntb(vec_char16 a)
{
  return ((vec_uchar16)(si_cntb((qword)(a))));
}

/* spu_cntlz
 * =========
 */
static __inline vec_uint4 spu_cntlz(vec_uint4 a)
{
  return ((vec_uint4)(si_clz((qword)(a))));
}

static __inline vec_uint4 spu_cntlz(vec_int4 a)
{
  return ((vec_uint4)(si_clz((qword)(a))));
}

static __inline vec_uint4 spu_cntlz(vec_float4 a)
{
  return ((vec_uint4)(si_clz((qword)(a))));
}

/* spu_testsv
 * ==========
 */
static __inline vec_ullong2 spu_testsv(vec_double2 a, char b)
{
  return ((vec_ullong2)(si_dftsv((qword)(a), b)));
}

/* spu_convtf
 * ==========
 */
#define spu_convtf(_a, _b)	(vec_ctf(_a, _b))

/* spu_convts
 * ==========
 */
#define spu_convts(_a, _b)	(vec_cts(_a, _b))

/* spu_convtu
 * ==========
 */
#define spu_convtu(_a, _b)	(vec_ctu(_a, _b))


/* spu_dsync
 * ========
 */
#define spu_dsync()

/* spu_eqv
 * =======
 */
static __inline vec_uchar16 spu_eqv(vec_uchar16 a, vec_uchar16 b)
{
  return ((vec_uchar16)(si_eqv((qword)(a), (qword)(b))));
}

static __inline vec_char16 spu_eqv(vec_char16 a, vec_char16 b)
{
  return ((vec_char16)(si_eqv((qword)(a), (qword)(b))));
}

static __inline vec_ushort8 spu_eqv(vec_ushort8 a, vec_ushort8 b)
{
  return ((vec_ushort8)(si_eqv((qword)(a), (qword)(b))));
}

static __inline vec_short8 spu_eqv(vec_short8 a, vec_short8 b)
{
  return ((vec_short8)(si_eqv((qword)(a), (qword)(b))));
}

static __inline vec_uint4 spu_eqv(vec_uint4 a, vec_uint4 b)
{
  return ((vec_uint4)(si_eqv((qword)(a), (qword)(b))));
}

static __inline vec_int4 spu_eqv(vec_int4 a, vec_int4 b)
{
  return ((vec_int4)(si_eqv((qword)(a), (qword)(b))));
}

static __inline vec_float4 spu_eqv(vec_float4 a, vec_float4 b)
{
  return ((vec_float4)(si_eqv((qword)(a), (qword)(b))));
}

static __inline vec_ullong2 spu_eqv(vec_ullong2 a, vec_ullong2 b)
{
  return ((vec_ullong2)(si_eqv((qword)(a), (qword)(b))));
}

static __inline vec_llong2 spu_eqv(vec_llong2 a, vec_llong2 b)
{
  return ((vec_llong2)(si_eqv((qword)(a), (qword)(b))));
}

static __inline vec_double2 spu_eqv(vec_double2 a, vec_double2 b)
{
  return ((vec_double2)(si_eqv((qword)(a), (qword)(b))));
}

/* spu_extend
 * ========
 */
static __inline vec_short8 spu_extend(vec_char16 a)
{
  return ((vec_short8)(si_xsbh((qword)(a))));
}


static __inline vec_int4 spu_extend(vec_short8 a)
{
  return ((vec_int4)(si_xshw((qword)(a))));
}

static __inline vec_llong2 spu_extend(vec_int4 a)
{
  return ((vec_llong2)(si_xswd((qword)(a))));
}


static __inline vec_double2 spu_extend(vec_float4 a)
{
  return ((vec_double2)(si_fesd((qword)(a))));
}


/* spu_extract
 * ========
 */
static __inline unsigned char spu_extract(vec_uchar16 a, int element)
{
  union {
    vec_uchar16 v;
    unsigned char c[16];
  } in;

  in.v = a;
  return (in.c[element & 15]);
}

static __inline signed char spu_extract(vec_char16 a, int element)
{
  union {
    vec_char16 v;
    signed char c[16];
  } in;

  in.v = a;
  return (in.c[element & 15]);
}

static __inline unsigned short spu_extract(vec_ushort8 a, int element)
{
  union {
    vec_ushort8 v;
    unsigned short s[8];
  } in;

  in.v = a;
  return (in.s[element & 7]);
}

static __inline signed short spu_extract(vec_short8 a, int element)
{
  union {
    vec_short8 v;
    signed short s[8];
  } in;

  in.v = a;
  return (in.s[element & 7]);
}

static __inline unsigned int spu_extract(vec_uint4 a, int element)
{
  union {
    vec_uint4 v;
    unsigned int i[4];
  } in;

  in.v = a;
  return (in.i[element & 3]);
}

static __inline signed int spu_extract(vec_int4 a, int element)
{
  union {
    vec_int4 v;
    signed int i[4];
  } in;

  in.v = a;
  return (in.i[element & 3]);
}

static __inline float spu_extract(vec_float4 a, int element)
{
  union {
    vec_float4 v;
    float f[4];
  } in;

  in.v = a;
  return (in.f[element & 3]);
}

static __inline unsigned long long  spu_extract(vec_ullong2 a, int element)
{
  union {
    vec_ullong2 v;
    unsigned long long l[2];
  } in;

  in.v = a;
  return (in.l[element & 1]);
}

static __inline signed long long  spu_extract(vec_llong2 a, int element)
{
  union {
    vec_llong2 v;
    signed long long l[2];
  } in;

  in.v = a;
  return (in.l[element & 1]);
}

static __inline double spu_extract(vec_double2 a, int element)
{
  union {
    vec_double2 v;
    double d[2];
  } in;

  in.v = a;
  return (in.d[element & 1]);
}

/* spu_gather
 * ========
 */
static __inline vec_uint4 spu_gather(vec_uchar16 a)
{
  return ((vec_uint4)(si_gbb((qword)(a))));
}


static __inline vec_uint4 spu_gather(vec_char16 a)
{
  return ((vec_uint4)(si_gbb((qword)(a))));
}

static __inline vec_uint4 spu_gather(vec_ushort8 a)
{
  return ((vec_uint4)(si_gbh((qword)(a))));
}

static __inline vec_uint4 spu_gather(vec_short8 a)
{
  return ((vec_uint4)(si_gbh((qword)(a))));
}


static __inline vec_uint4 spu_gather(vec_uint4 a)
{
  return ((vec_uint4)(si_gb((qword)(a))));
}

static __inline vec_uint4 spu_gather(vec_int4 a)
{
  return ((vec_uint4)(si_gb((qword)(a))));
}

static __inline vec_uint4 spu_gather(vec_float4 a)
{
  return ((vec_uint4)(si_gb((qword)(a))));
}

/* spu_genb
 * ========
 */
static __inline vec_uint4 spu_genb(vec_uint4 a, vec_uint4 b)
{
  return ((vec_uint4)(si_bg((qword)(b), (qword)(a))));
}

static __inline vec_int4 spu_genb(vec_int4 a, vec_int4 b)
{
  return ((vec_int4)(si_bg((qword)(b), (qword)(a))));
}

/* spu_genbx
 * =========
 */
static __inline vec_uint4 spu_genbx(vec_uint4 a, vec_uint4 b, vec_uint4 c)
{
  return ((vec_uint4)(si_bgx((qword)(b), (qword)(a), (qword)(c))));
}

static __inline vec_int4 spu_genbx(vec_int4 a, vec_int4 b, vec_int4 c)
{
  return ((vec_int4)(si_bgx((qword)(b), (qword)(a), (qword)(c))));
}


/* spu_genc
 * ========
 */
static __inline vec_uint4 spu_genc(vec_uint4 a, vec_uint4 b)
{
  return ((vec_uint4)(si_cg((qword)(a), (qword)(b))));
}

static __inline vec_int4 spu_genc(vec_int4 a, vec_int4 b)
{
  return ((vec_int4)(si_cg((qword)(a), (qword)(b))));
}

/* spu_gencx
 * =========
 */
static __inline vec_uint4 spu_gencx(vec_uint4 a, vec_uint4 b, vec_uint4 c)
{
  return ((vec_uint4)(si_cgx((qword)(a), (qword)(b), (qword)(c))));
}

static __inline vec_int4 spu_gencx(vec_int4 a, vec_int4 b, vec_int4 c)
{
  return ((vec_int4)(si_cgx((qword)(a), (qword)(b), (qword)(c))));
}


/* spu_hcmpeq
 * ========
 */
#define spu_hcmpeq(_a, _b)	if (_a == _b) { SPU_HALT_ACTION; };


/* spu_hcmpgt
 * ========
 */
#define spu_hcmpgt(_a, _b)	if (_a > _b) { SPU_HALT_ACTION; };


/* spu_idisable
 * ============
 */
#define spu_idisable()		SPU_UNSUPPORTED_ACTION


/* spu_ienable
 * ===========
 */
#define spu_ienable()		SPU_UNSUPPORTED_ACTION


/* spu_insert
 * ========
 */
static __inline vec_uchar16 spu_insert(unsigned char a, vec_uchar16 b, int element)
{
  union {
    vec_uchar16 v;
    unsigned char c[16];
  } in;

  in.v = b;
  in.c[element & 15] = a;
  return (in.v);
}

static __inline vec_char16 spu_insert(signed char a, vec_char16 b, int element)
{
  return ((vec_char16)spu_insert((unsigned char)(a), (vec_uchar16)(b), element));
}

static __inline vec_ushort8 spu_insert(unsigned short a, vec_ushort8 b, int element)
{
  union {
    vec_ushort8 v;
    unsigned short s[8];
  } in;

  in.v = b;
  in.s[element & 7] = a;
  return (in.v);
}

static __inline vec_short8 spu_insert(signed short a, vec_short8 b, int element)
{
  return ((vec_short8)spu_insert((unsigned short)(a), (vec_ushort8)(b), element));
}

static __inline vec_uint4 spu_insert(unsigned int a, vec_uint4 b, int element)
{
  union {
    vec_uint4 v;
    unsigned int i[4];
  } in;

  in.v = b;
  in.i[element & 3] = a;
  return (in.v);
}

static __inline vec_int4 spu_insert(signed int a, vec_int4 b, int element)
{
  return ((vec_int4)spu_insert((unsigned int)(a), (vec_uint4)(b), element));
}

static __inline vec_float4 spu_insert(float a, vec_float4 b, int element)
{
  union {
    vec_float4 v;
    float f[4];
  } in;

  in.v = b;
  in.f[element & 3] = a;
  return (in.v);
}

static __inline vec_ullong2 spu_insert(unsigned long long a, vec_ullong2 b, int element)
{
  union {
    vec_ullong2 v;
    unsigned long long l[2];
  } in;

  in.v = b;
  in.l[element & 1] = a;
  return (in.v);
}

static __inline vec_llong2 spu_insert(signed long long a, vec_llong2 b, int element)
{
  return ((vec_llong2)spu_insert((unsigned long long)(a), (vec_ullong2)(b), element));
}

static __inline vec_double2 spu_insert(double a, vec_double2 b, int element)
{
  union {
    vec_double2 v;
    double d[2];
  } in;

  in.v = b;
  in.d[element & 1] = a;
  return (in.v);
}


/* spu_madd
 * ========
 */
static __inline vec_int4 spu_madd(vec_short8 a, vec_short8 b, vec_int4 c)
{
  return ((vec_int4)(si_mpya((qword)(a), (qword)(b), (qword)(c))));
}

static __inline vec_float4 spu_madd(vec_float4 a, vec_float4 b, vec_float4 c)
{
  return ((vec_float4)(si_fma((qword)(a), (qword)(b), (qword)(c))));
}

static __inline vec_double2 spu_madd(vec_double2 a, vec_double2 b, vec_double2 c)
{
  return ((vec_double2)(si_dfma((qword)(a), (qword)(b), (qword)(c))));
}


/* spu_maskb
 * ========
 */
#define spu_maskb(_a)	(vec_uchar16)(si_fsmb(si_from_int((int)(_a))))

/* spu_maskh
 * ========
 */
#define spu_maskh(_a)	(vec_ushort8)(si_fsmh(si_from_int((int)(_a))))


/* spu_maskw
 * ========
 */
#define spu_maskw(_a)	(vec_uint4)(si_fsm(si_from_int((int)(_a))))


/* spu_mfcdma32
 * ========
 */
#define spu_mfcdma32(_ls, _ea, _size, _tagid, _cmd)


/* spu_mfcdma64
 * ========
 */
#define spu_mfcdma64(_ls, _eahi, _ealow,  _size, _tagid, _cmd)

/* spu_mfcstat
 * ========
 */
#define spu_mfcstat(_type)	0xFFFFFFFF



/* spu_mffpscr
 * ===========
 */
#define spu_mffpscr()		(vec_uint4)(si_fscrrd())


/* spu_mfspr
 * ========
 */

#define spu_mfspr(_reg)		si_to_uint(si_mfspr(_reg))



/* spu_mhhadd
 * ==========
 */
static __inline vec_int4 spu_mhhadd(vec_short8 a, vec_short8 b, vec_int4 c)
{
  return ((vec_int4)(si_mpyhha((qword)(a), (qword)(b), (qword)(c))));
}


static __inline vec_uint4 spu_mhhadd(vec_ushort8 a, vec_ushort8 b, vec_uint4 c)
{
  return ((vec_uint4)(si_mpyhhau((qword)(a), (qword)(b), (qword)(c))));
}


/* spu_msub
 * ========
 */
static __inline vec_float4 spu_msub(vec_float4 a, vec_float4 b, vec_float4 c)
{
  return ((vec_float4)(si_fms((qword)(a), (qword)(b), (qword)(c))));
}

static __inline vec_double2 spu_msub(vec_double2 a, vec_double2 b, vec_double2 c)
{
  return ((vec_double2)(si_dfms((qword)(a), (qword)(b), (qword)(c))));
}


/* spu_mtfpscr
 * ===========
 */
#define spu_mtfpscr(_a)


/* spu_mtspr
 * ========
 */
#define spu_mtspr(_reg, _a)


/* spu_mul
 * ========
 */
static __inline vec_float4 spu_mul(vec_float4 a, vec_float4 b)
{
  return ((vec_float4)(si_fm((qword)(a), (qword)(b))));
}

static __inline vec_double2 spu_mul(vec_double2 a, vec_double2 b)
{
  return ((vec_double2)(si_dfm((qword)(a), (qword)(b))));
}


/* spu_mulh
 * ========
 */
static __inline vec_int4 spu_mulh(vec_short8 a, vec_short8 b)
{
  return ((vec_int4)(si_mpyh((qword)(a), (qword)(b))));
}

/* spu_mule
 * =========
 */
#define spu_mule(_a, _b)	vec_mule(_a, _b)



/* spu_mulo
 * ========
 */
static __inline vec_int4 spu_mulo(vec_short8 a, vec_short8 b)
{
  return ((vec_int4)(si_mpy((qword)(a), (qword)(b))));
}


static __inline vec_uint4 spu_mulo(vec_ushort8 a, vec_ushort8 b)
{
  return ((vec_uint4)(si_mpyu((qword)(a), (qword)(b))));
}


static __inline vec_int4 spu_mulo(vec_short8 a, short b)
{
  return ((vec_int4)(si_mpyi((qword)(a), b)));
}

static __inline vec_uint4 spu_mulo(vec_ushort8 a, unsigned short b)
{
  return ((vec_uint4)(si_mpyui((qword)(a), b)));
}


/* spu_mulsr
 * =========
 */
static __inline vec_int4 spu_mulsr(vec_short8 a, vec_short8 b)
{
  return ((vec_int4)(si_mpys((qword)(a), (qword)(b))));
}


/* spu_nand
 * ========
 */
static __inline vec_uchar16 spu_nand(vec_uchar16 a, vec_uchar16 b)
{
  return ((vec_uchar16)(si_nand((qword)(a), (qword)(b))));
}

static __inline vec_char16 spu_nand(vec_char16 a, vec_char16 b)
{
  return ((vec_char16)(si_nand((qword)(a), (qword)(b))));
}

static __inline vec_ushort8 spu_nand(vec_ushort8 a, vec_ushort8 b)
{
  return ((vec_ushort8)(si_nand((qword)(a), (qword)(b))));
}

static __inline vec_short8 spu_nand(vec_short8 a, vec_short8 b)
{
  return ((vec_short8)(si_nand((qword)(a), (qword)(b))));
}

static __inline vec_uint4 spu_nand(vec_uint4 a, vec_uint4 b)
{
  return ((vec_uint4)(si_nand((qword)(a), (qword)(b))));
}

static __inline vec_int4 spu_nand(vec_int4 a, vec_int4 b)
{
  return ((vec_int4)(si_nand((qword)(a), (qword)(b))));
}

static __inline vec_float4 spu_nand(vec_float4 a, vec_float4 b)
{
  return ((vec_float4)(si_nand((qword)(a), (qword)(b))));
}

static __inline vec_ullong2 spu_nand(vec_ullong2 a, vec_ullong2 b)
{
  return ((vec_ullong2)(si_nand((qword)(a), (qword)(b)))); 
}

static __inline vec_llong2 spu_nand(vec_llong2 a, vec_llong2 b)
{
  return ((vec_llong2)(si_nand((qword)(a), (qword)(b)))); 
}

static __inline vec_double2 spu_nand(vec_double2 a, vec_double2 b)
{
  return ((vec_double2)(si_nand((qword)(a), (qword)(b))));
}


/* spu_nmadd
 * =========
 */
static __inline vec_double2 spu_nmadd(vec_double2 a, vec_double2 b, vec_double2 c)
{
  return ((vec_double2)(si_dfnma((qword)(a), (qword)(b), (qword)(c))));
}


/* spu_nmsub
 * =========
 */
static __inline vec_float4 spu_nmsub(vec_float4 a, vec_float4 b, vec_float4 c)
{
  return ((vec_float4)(si_fnms((qword)(a), (qword)(b), (qword)(c))));
}

static __inline vec_double2 spu_nmsub(vec_double2 a, vec_double2 b, vec_double2 c)
{
  return ((vec_double2)(si_dfnms((qword)(a), (qword)(b), (qword)(c))));
}


/* spu_nor
 * =======
 */
#define spu_nor(_a, _b)		vec_nor(_a, _b)


/* spu_or
 * ======
 */
static __inline vec_uchar16 spu_or(vec_uchar16 a, vec_uchar16 b)
{
  return ((vec_uchar16)(si_or((qword)(a), (qword)(b))));
}

static __inline vec_char16 spu_or(vec_char16 a, vec_char16 b)
{
  return ((vec_char16)(si_or((qword)(a), (qword)(b))));
}

static __inline vec_ushort8 spu_or(vec_ushort8 a, vec_ushort8 b)
{
  return ((vec_ushort8)(si_or((qword)(a), (qword)(b))));
}

static __inline vec_short8 spu_or(vec_short8 a, vec_short8 b)
{
  return ((vec_short8)(si_or((qword)(a), (qword)(b))));
}

static __inline vec_uint4 spu_or(vec_uint4 a, vec_uint4 b)
{
  return ((vec_uint4)(si_or((qword)(a), (qword)(b))));
}

static __inline vec_int4 spu_or(vec_int4 a, vec_int4 b)
{
  return ((vec_int4)(si_or((qword)(a), (qword)(b))));
}

static __inline vec_float4 spu_or(vec_float4 a, vec_float4 b)
{
  return ((vec_float4)(si_or((qword)(a), (qword)(b))));
}

static __inline vec_ullong2 spu_or(vec_ullong2 a, vec_ullong2 b)
{
  return ((vec_ullong2)(si_or((qword)(a), (qword)(b))));
}

static __inline vec_llong2 spu_or(vec_llong2 a, vec_llong2 b)
{
  return ((vec_llong2)(si_or((qword)(a), (qword)(b))));
}

static __inline vec_double2 spu_or(vec_double2 a, vec_double2 b)
{
  return ((vec_double2)(si_or((qword)(a), (qword)(b))));
}


static __inline vec_uchar16 spu_or(vec_uchar16 a, unsigned char b)
{
  return ((vec_uchar16)(si_orbi((qword)(a), b)));
}

static __inline vec_char16 spu_or(vec_char16 a, signed char b)
{
  return ((vec_char16)(si_orbi((qword)(a), (unsigned char)(b))));
}

static __inline vec_ushort8 spu_or(vec_ushort8 a, unsigned short b)
{
  return ((vec_ushort8)(si_orhi((qword)(a), b)));
}

static __inline vec_short8 spu_or(vec_short8 a, signed short b)
{
  return ((vec_short8)(si_orhi((qword)(a), (unsigned short)(b))));
}

static __inline vec_uint4 spu_or(vec_uint4 a, unsigned int b)
{
  return ((vec_uint4)(si_ori((qword)(a), b)));
}

static __inline vec_int4 spu_or(vec_int4 a, signed int b)
{
  return ((vec_int4)(si_ori((qword)(a), (unsigned int)(b))));
}


/* spu_orc
 * =======
 */
#define spu_orc(_a, _b)		vec_or(_a, vec_nor(_b, _b))


/* spu_orx
 * =======
 */
static __inline vec_uint4 spu_orx(vec_uint4 a)
{
  return ((vec_uint4)(si_orx((qword)(a))));
}

static __inline vec_int4 spu_orx(vec_int4 a)
{
  return ((vec_int4)(si_orx((qword)(a))));
}


/* spu_promote
 * ===========
 */
static __inline vec_uchar16 spu_promote(unsigned char a, int element)
{
  union {
    vec_uchar16 v;
    unsigned char c[16];
  } in;

  in.c[element & 15] = a;
  return (in.v);
}

static __inline vec_char16 spu_promote(signed char a, int element)
{
  union {
    vec_char16 v;
    signed char c[16];
  } in;

  in.c[element & 15] = a;
  return (in.v);
}

static __inline vec_ushort8 spu_promote(unsigned short a, int element)
{
  union {
    vec_ushort8 v;
    unsigned short s[8];
  } in;

  in.s[element & 7] = a;
  return (in.v);
}

static __inline vec_short8 spu_promote(signed short a, int element)
{
  union {
    vec_short8 v;
    signed short s[8];
  } in;

  in.s[element & 7] = a;
  return (in.v);
}

static __inline vec_uint4 spu_promote(unsigned int a, int element)
{
  union {
    vec_uint4 v;
    unsigned int i[4];
  } in;

  in.i[element & 3] = a;
  return (in.v);
}

static __inline vec_int4 spu_promote(signed int a, int element)
{
  union {
    vec_int4 v;
    signed int i[4];
  } in;

  in.i[element & 3] = a;
  return (in.v);
}

static __inline vec_float4 spu_promote(float a, int element)
{
  union {
    vec_float4 v;
    float f[4];
  } in;

  in.f[element & 3] = a;
  return (in.v);
}

static __inline vec_ullong2 spu_promote(unsigned long long a, int element)
{
  union {
    vec_ullong2 v;
    unsigned long long l[2];
  } in;

  in.l[element & 1] = a;
  return (in.v);
}

static __inline vec_llong2 spu_promote(signed long long a, int element)
{
  union {
    vec_llong2 v;
    signed long long l[2];
  } in;

  in.l[element & 1] = a;
  return (in.v);
}

static __inline vec_double2 spu_promote(double a, int element)
{
  union {
    vec_double2 v;
    double d[2];
  } in;

  in.d[element & 1] = a;
  return (in.v);
}

/* spu_re
 * ======
 */
#define spu_re(_a)		vec_re(_a)


/* spu_readch
 * ==========
 */
#define spu_readch(_channel)		0	/* not mappable */


/* spu_readchcnt
 * =============
 */
#define spu_readchcnt(_channel)		0	/* not mappable */


/* spu_readchqw
 * ============
 */
#define spu_readchqw(_channel) __extension__ ({ vec_uint4 result = { 0, 0, 0, 0 }; result; })

/* spu_rl
 * ======
 */
static __inline vec_ushort8 spu_rl(vec_ushort8 a, vec_short8 b)
{
  return ((vec_ushort8)(si_roth((qword)(a), (qword)(b))));
}

static __inline vec_short8 spu_rl(vec_short8 a, vec_short8 b)
{
  return ((vec_short8)(si_roth((qword)(a), (qword)(b))));
}

static __inline vec_uint4 spu_rl(vec_uint4 a, vec_int4 b)
{
  return ((vec_uint4)(si_rot((qword)(a), (qword)(b))));
}

static __inline vec_int4 spu_rl(vec_int4 a, vec_int4 b)
{
  return ((vec_int4)(si_rot((qword)(a), (qword)(b))));
}

static __inline vec_ushort8 spu_rl(vec_ushort8 a, int b)
{
  return ((vec_ushort8)(si_rothi((qword)(a), b)));
}

static __inline vec_short8 spu_rl(vec_short8 a, int b)
{
  return ((vec_short8)(si_rothi((qword)(a), b)));
}

static __inline vec_uint4 spu_rl(vec_uint4 a, int b)
{
  return ((vec_uint4)(si_roti((qword)(a), b)));
}

static __inline vec_int4 spu_rl(vec_int4 a, int b)
{
  return ((vec_int4)(si_roti((qword)(a), b)));
}


/* spu_rlmask
 * ==========
 */
static __inline vec_ushort8 spu_rlmask(vec_ushort8 a, vec_short8 b)
{
  return ((vec_ushort8)(si_rothm((qword)(a), (qword)(b))));
}

static __inline vec_short8 spu_rlmask(vec_short8 a, vec_short8 b)
{
  return ((vec_short8)(si_rothm((qword)(a), (qword)(b))));
}

static __inline vec_uint4 spu_rlmask(vec_uint4 a, vec_int4 b)
{
  return ((vec_uint4)(si_rotm((qword)(a), (qword)(b))));
}

static __inline vec_int4 spu_rlmask(vec_int4 a, vec_int4 b)
{
  return ((vec_int4)(si_rotm((qword)(a), (qword)(b))));
}

static __inline vec_ushort8 spu_rlmask(vec_ushort8 a, int b)
{
  return ((vec_ushort8)(si_rothmi((qword)(a), b)));
}

static __inline vec_short8 spu_rlmask(vec_short8 a, int b)
{
  return ((vec_short8)(si_rothmi((qword)(a), b)));
}


static __inline vec_uint4 spu_rlmask(vec_uint4 a, int b)
{
  return ((vec_uint4)(si_rotmi((qword)(a), b)));
}

static __inline vec_int4 spu_rlmask(vec_int4 a, int b)
{
  return ((vec_int4)(si_rotmi((qword)(a), b)));
}

/* spu_rlmaska
 * ===========
 */
static __inline vec_short8 spu_rlmaska(vec_short8 a, vec_short8 b)
{
  return ((vec_short8)(si_rotmah((qword)(a), (qword)(b))));
}

static __inline vec_ushort8 spu_rlmaska(vec_ushort8 a, vec_short8 b)
{
  return ((vec_ushort8)(si_rotmah((qword)(a), (qword)(b))));
}


static __inline vec_int4 spu_rlmaska(vec_int4 a, vec_int4 b)
{
  return ((vec_int4)(si_rotma((qword)(a), (qword)(b))));
}

static __inline vec_uint4 spu_rlmaska(vec_uint4 a, vec_int4 b)
{
  return ((vec_uint4)(si_rotma((qword)(a), (qword)(b))));
}

static __inline vec_ushort8 spu_rlmaska(vec_ushort8 a, int b)
{
  return ((vec_ushort8)(si_rotmahi((qword)(a), b)));
}

static __inline vec_short8 spu_rlmaska(vec_short8 a, int b)
{
  return ((vec_short8)(si_rotmahi((qword)(a), b)));
}

static __inline vec_uint4 spu_rlmaska(vec_uint4 a, int b)
{
  return ((vec_uint4)(si_rotmai((qword)(a), b)));
}

static __inline vec_int4 spu_rlmaska(vec_int4 a, int b)
{
  return ((vec_int4)(si_rotmai((qword)(a), b)));
}


/* spu_rlmaskqw
 * ============
 */
static __inline vec_uchar16 spu_rlmaskqw(vec_uchar16 a, int count)
{
  return ((vec_uchar16)(si_rotqmbi((qword)(a), si_from_int(count))));
}

static __inline vec_char16 spu_rlmaskqw(vec_char16 a, int count)
{
  return ((vec_char16)(si_rotqmbi((qword)(a), si_from_int(count))));
}

static __inline vec_ushort8 spu_rlmaskqw(vec_ushort8 a, int count)
{
  return ((vec_ushort8)(si_rotqmbi((qword)(a), si_from_int(count))));
}

static __inline vec_short8 spu_rlmaskqw(vec_short8 a, int count)
{
  return ((vec_short8)(si_rotqmbi((qword)(a), si_from_int(count))));
}

static __inline vec_uint4 spu_rlmaskqw(vec_uint4 a, int count)
{
  return ((vec_uint4)(si_rotqmbi((qword)(a), si_from_int(count))));
}

static __inline vec_int4 spu_rlmaskqw(vec_int4 a, int count)
{
  return ((vec_int4)(si_rotqmbi((qword)(a), si_from_int(count))));
}

static __inline vec_float4 spu_rlmaskqw(vec_float4 a, int count)
{
  return ((vec_float4)(si_rotqmbi((qword)(a), si_from_int(count))));
}

static __inline vec_ullong2 spu_rlmaskqw(vec_ullong2 a, int count)
{
  return ((vec_ullong2)(si_rotqmbi((qword)(a), si_from_int(count))));
}

static __inline vec_llong2 spu_rlmaskqw(vec_llong2 a, int count)
{
  return ((vec_llong2)(si_rotqmbi((qword)(a), si_from_int(count))));
}

static __inline vec_double2 spu_rlmaskqw(vec_double2 a, int count)
{
  return ((vec_double2)(si_rotqmbi((qword)(a), si_from_int(count))));
}

/* spu_rlmaskqwbyte
 * ================
 */
static __inline vec_uchar16 spu_rlmaskqwbyte(vec_uchar16 a, int count)
{
  return ((vec_uchar16)(si_rotqmby((qword)(a), si_from_int(count))));
}

static __inline vec_char16 spu_rlmaskqwbyte(vec_char16 a, int count)
{
  return ((vec_char16)(si_rotqmby((qword)(a), si_from_int(count))));
}

static __inline vec_ushort8 spu_rlmaskqwbyte(vec_ushort8 a, int count)
{
  return ((vec_ushort8)(si_rotqmby((qword)(a), si_from_int(count))));
}

static __inline vec_short8 spu_rlmaskqwbyte(vec_short8 a, int count)
{
  return ((vec_short8)(si_rotqmby((qword)(a), si_from_int(count))));
}

static __inline vec_uint4 spu_rlmaskqwbyte(vec_uint4 a, int count)
{
  return ((vec_uint4)(si_rotqmby((qword)(a), si_from_int(count))));
}

static __inline vec_int4 spu_rlmaskqwbyte(vec_int4 a, int count)
{
  return ((vec_int4)(si_rotqmby((qword)(a), si_from_int(count))));
}

static __inline vec_float4 spu_rlmaskqwbyte(vec_float4 a, int count)
{
  return ((vec_float4)(si_rotqmby((qword)(a), si_from_int(count))));
}

static __inline vec_ullong2 spu_rlmaskqwbyte(vec_ullong2 a, int count)
{
  return ((vec_ullong2)(si_rotqmby((qword)(a), si_from_int(count))));
}

static __inline vec_llong2 spu_rlmaskqwbyte(vec_llong2 a, int count)
{
  return ((vec_llong2)(si_rotqmby((qword)(a), si_from_int(count))));
}

static __inline vec_double2 spu_rlmaskqwbyte(vec_double2 a, int count)
{
  return ((vec_double2)(si_rotqmby((qword)(a), si_from_int(count))));
}

/* spu_rlmaskqwbytebc
 * ==================
 */
static __inline vec_uchar16 spu_rlmaskqwbytebc(vec_uchar16 a, int count)
{
  return ((vec_uchar16)(si_rotqmbybi((qword)(a), si_from_int(count))));
}

static __inline vec_char16 spu_rlmaskqwbytebc(vec_char16 a, int count)
{
  return ((vec_char16)(si_rotqmbybi((qword)(a), si_from_int(count))));
}

static __inline vec_ushort8 spu_rlmaskqwbytebc(vec_ushort8 a, int count)
{
  return ((vec_ushort8)(si_rotqmbybi((qword)(a), si_from_int(count))));
}

static __inline vec_short8 spu_rlmaskqwbytebc(vec_short8 a, int count)
{
  return ((vec_short8)(si_rotqmbybi((qword)(a), si_from_int(count))));
}

static __inline vec_uint4 spu_rlmaskqwbytebc(vec_uint4 a, int count)
{
  return ((vec_uint4)(si_rotqmbybi((qword)(a), si_from_int(count))));
}

static __inline vec_int4 spu_rlmaskqwbytebc(vec_int4 a, int count)
{
  return ((vec_int4)(si_rotqmbybi((qword)(a), si_from_int(count))));
}

static __inline vec_float4 spu_rlmaskqwbytebc(vec_float4 a, int count)
{
  return ((vec_float4)(si_rotqmbybi((qword)(a), si_from_int(count))));
}

static __inline vec_ullong2 spu_rlmaskqwbytebc(vec_ullong2 a, int count)
{
  return ((vec_ullong2)(si_rotqmbybi((qword)(a), si_from_int(count))));
}

static __inline vec_llong2 spu_rlmaskqwbytebc(vec_llong2 a, int count)
{
  return ((vec_llong2)(si_rotqmbybi((qword)(a), si_from_int(count))));
}

static __inline vec_double2 spu_rlmaskqwbytebc(vec_double2 a, int count)
{
  return ((vec_double2)(si_rotqmbybi((qword)(a), si_from_int(count))));
}


/* spu_rlqwbyte
 * ============
 */
static __inline vec_uchar16 spu_rlqwbyte(vec_uchar16 a, int count)
{
  return ((vec_uchar16)(si_rotqby((qword)(a), si_from_int(count))));
}  

static __inline vec_char16 spu_rlqwbyte(vec_char16 a, int count)
{
  return ((vec_char16)(si_rotqby((qword)(a), si_from_int(count))));
}

static __inline vec_ushort8 spu_rlqwbyte(vec_ushort8 a, int count)
{
  return ((vec_ushort8)(si_rotqby((qword)(a), si_from_int(count))));
}  

static __inline vec_short8 spu_rlqwbyte(vec_short8 a, int count)
{
  return ((vec_short8)(si_rotqby((qword)(a), si_from_int(count))));
}

static __inline vec_uint4 spu_rlqwbyte(vec_uint4 a, int count)
{
  return ((vec_uint4)(si_rotqby((qword)(a), si_from_int(count))));
}

static __inline vec_int4 spu_rlqwbyte(vec_int4 a, int count)
{
  return ((vec_int4)(si_rotqby((qword)(a), si_from_int(count))));
}

static __inline vec_float4 spu_rlqwbyte(vec_float4 a, int count)
{
  return ((vec_float4)(si_rotqby((qword)(a), si_from_int(count))));
}

static __inline vec_ullong2 spu_rlqwbyte(vec_ullong2 a, int count)
{
  return ((vec_ullong2)(si_rotqby((qword)(a), si_from_int(count))));
}

static __inline vec_llong2 spu_rlqwbyte(vec_llong2 a, int count)
{
  return ((vec_llong2)(si_rotqby((qword)(a), si_from_int(count))));
}

static __inline vec_double2 spu_rlqwbyte(vec_double2 a, int count)
{
  return ((vec_double2)(si_rotqby((qword)(a), si_from_int(count))));
}


/* spu_rlqwbytebc
 * ==============
 */
static __inline vec_uchar16 spu_rlqwbytebc(vec_uchar16 a, int count)
{
  return ((vec_uchar16)(si_rotqbybi((qword)(a), si_from_int(count))));
}

static __inline vec_char16 spu_rlqwbytebc(vec_char16 a, int count)
{
  return ((vec_char16)(si_rotqbybi((qword)(a), si_from_int(count))));
}

static __inline vec_ushort8 spu_rlqwbytebc(vec_ushort8 a, int count)
{
  return ((vec_ushort8)(si_rotqbybi((qword)(a), si_from_int(count))));
}

static __inline vec_short8 spu_rlqwbytebc(vec_short8 a, int count)
{
  return ((vec_short8)(si_rotqbybi((qword)(a), si_from_int(count))));
}

static __inline vec_uint4 spu_rlqwbytebc(vec_uint4 a, int count)
{
  return ((vec_uint4)(si_rotqbybi((qword)(a), si_from_int(count))));
}

static __inline vec_int4 spu_rlqwbytebc(vec_int4 a, int count)
{
  return ((vec_int4)(si_rotqbybi((qword)(a), si_from_int(count))));
}

static __inline vec_float4 spu_rlqwbytebc(vec_float4 a, int count)
{
  return ((vec_float4)(si_rotqbybi((qword)(a), si_from_int(count))));
}

static __inline vec_ullong2 spu_rlqwbytebc(vec_ullong2 a, int count)
{
  return ((vec_ullong2)(si_rotqbybi((qword)(a), si_from_int(count))));
}

static __inline vec_llong2 spu_rlqwbytebc(vec_llong2 a, int count)
{
  return ((vec_llong2)(si_rotqbybi((qword)(a), si_from_int(count))));
}

static __inline vec_double2 spu_rlqwbytebc(vec_double2 a, int count)
{
  return ((vec_double2)(si_rotqbybi((qword)(a), si_from_int(count))));
}

/* spu_rlqw
 * ========
 */
static __inline vec_uchar16 spu_rlqw(vec_uchar16 a, int count)
{
  return ((vec_uchar16)(si_rotqbi((qword)(a), si_from_int(count))));
}

static __inline vec_char16 spu_rlqw(vec_char16 a, int count)
{
  return ((vec_char16)(si_rotqbi((qword)(a), si_from_int(count))));
}

static __inline vec_ushort8 spu_rlqw(vec_ushort8 a, int count)
{
  return ((vec_ushort8)(si_rotqbi((qword)(a), si_from_int(count))));
}

static __inline vec_short8 spu_rlqw(vec_short8 a, int count)
{
  return ((vec_short8)(si_rotqbi((qword)(a), si_from_int(count))));
}

static __inline vec_uint4 spu_rlqw(vec_uint4 a, int count)
{
  return ((vec_uint4)(si_rotqbi((qword)(a), si_from_int(count))));
}

static __inline vec_int4 spu_rlqw(vec_int4 a, int count)
{
  return ((vec_int4)(si_rotqbi((qword)(a), si_from_int(count))));
}

static __inline vec_float4 spu_rlqw(vec_float4 a, int count)
{
  return ((vec_float4)(si_rotqbi((qword)(a), si_from_int(count))));
}

static __inline vec_ullong2 spu_rlqw(vec_ullong2 a, int count)
{
  return ((vec_ullong2)(si_rotqbi((qword)(a), si_from_int(count))));
}

static __inline vec_llong2 spu_rlqw(vec_llong2 a, int count)
{
  return ((vec_llong2)(si_rotqbi((qword)(a), si_from_int(count))));
}

static __inline vec_double2 spu_rlqw(vec_double2 a, int count)
{
  return ((vec_double2)(si_rotqbi((qword)(a), si_from_int(count))));
}

/* spu_roundtf
 * ===========
 */
static __inline vec_float4 spu_roundtf(vec_double2 a)
{
  return ((vec_float4)(si_frds((qword)(a))));
}


/* spu_rsqrte
 * ==========
 */
#define spu_rsqrte(_a)		vec_rsqrte(_a)


/* spu_sel
 * =======
 */
static __inline vec_uchar16 spu_sel(vec_uchar16 a, vec_uchar16 b, vec_uchar16 pattern)
{
  return ((vec_uchar16)(si_selb((qword)(a), (qword)(b), (qword)(pattern))));
}

static __inline vec_char16 spu_sel(vec_char16 a, vec_char16 b, vec_uchar16 pattern)
{
  return ((vec_char16)(si_selb((qword)(a), (qword)(b), (qword)(pattern))));
}

static __inline vec_ushort8 spu_sel(vec_ushort8 a, vec_ushort8 b, vec_ushort8 pattern)
{
  return ((vec_ushort8)(si_selb((qword)(a), (qword)(b), (qword)(pattern))));
}

static __inline vec_short8 spu_sel(vec_short8 a, vec_short8 b, vec_ushort8 pattern)
{
  return ((vec_short8)(si_selb((qword)(a), (qword)(b), (qword)(pattern))));
}

static __inline vec_uint4 spu_sel(vec_uint4 a, vec_uint4 b, vec_uint4 pattern)
{
  return ((vec_uint4)(si_selb((qword)(a), (qword)(b), (qword)(pattern))));
}

static __inline vec_int4 spu_sel(vec_int4 a, vec_int4 b, vec_uint4 pattern)
{
  return ((vec_int4)(si_selb((qword)(a), (qword)(b), (qword)(pattern))));
}

static __inline vec_float4 spu_sel(vec_float4 a, vec_float4 b, vec_uint4 pattern)
{
  return ((vec_float4)(si_selb((qword)(a), (qword)(b), (qword)(pattern))));
}

static __inline vec_ullong2 spu_sel(vec_ullong2 a, vec_ullong2 b, vec_ullong2 pattern)
{
  return ((vec_ullong2)(si_selb((qword)(a), (qword)(b), (qword)(pattern))));
}

static __inline vec_llong2 spu_sel(vec_llong2 a, vec_llong2 b, vec_ullong2 pattern)
{
  return ((vec_llong2)(si_selb((qword)(a), (qword)(b), (qword)(pattern))));
}

static __inline vec_double2 spu_sel(vec_double2 a, vec_double2 b, vec_ullong2 pattern)
{
  return ((vec_double2)(si_selb((qword)(a), (qword)(b), (qword)(pattern))));
}



/* spu_shuffle
 * ===========
 */
static __inline vec_uchar16 spu_shuffle(vec_uchar16 a, vec_uchar16 b, vec_uchar16 pattern)
{
  return ((vec_uchar16)(si_shufb((qword)(a), (qword)(b), (qword)(pattern))));
}

static __inline vec_char16 spu_shuffle(vec_char16 a, vec_char16 b, vec_uchar16 pattern)
{
  return ((vec_char16)(si_shufb((qword)(a), (qword)(b), (qword)(pattern))));
}

static __inline vec_ushort8 spu_shuffle(vec_ushort8 a, vec_ushort8 b, vec_uchar16 pattern)
{
  return ((vec_ushort8)(si_shufb((qword)(a), (qword)(b), (qword)(pattern))));
}

static __inline vec_short8 spu_shuffle(vec_short8 a, vec_short8 b, vec_uchar16 pattern)
{
  return ((vec_short8)(si_shufb((qword)(a), (qword)(b), (qword)(pattern))));
}

static __inline vec_uint4 spu_shuffle(vec_uint4 a, vec_uint4 b, vec_uchar16 pattern)
{
  return ((vec_uint4)(si_shufb((qword)(a), (qword)(b), (qword)(pattern))));
}

static __inline vec_int4 spu_shuffle(vec_int4 a, vec_int4 b, vec_uchar16 pattern)
{
  return ((vec_int4)(si_shufb((qword)(a), (qword)(b), (qword)(pattern))));
}

static __inline vec_float4 spu_shuffle(vec_float4 a, vec_float4 b, vec_uchar16 pattern)
{
  return ((vec_float4)(si_shufb((qword)(a), (qword)(b), (qword)(pattern))));
}

static __inline vec_ullong2 spu_shuffle(vec_ullong2 a, vec_ullong2 b, vec_uchar16 pattern)
{
  return ((vec_ullong2)(si_shufb((qword)(a), (qword)(b), (qword)(pattern))));
}

static __inline vec_llong2 spu_shuffle(vec_llong2 a, vec_llong2 b, vec_uchar16 pattern)
{
  return ((vec_llong2)(si_shufb((qword)(a), (qword)(b), (qword)(pattern))));
}

static __inline vec_double2 spu_shuffle(vec_double2 a, vec_double2 b, vec_uchar16 pattern)
{
  return ((vec_double2)(si_shufb((qword)(a), (qword)(b), (qword)(pattern))));
}


/* spu_sl
 * ======
 */
static __inline vec_ushort8 spu_sl(vec_ushort8 a, vec_ushort8 b)
{
  return ((vec_ushort8)(si_shlh((qword)(a), (qword)(b))));
}

static __inline vec_short8 spu_sl(vec_short8 a, vec_ushort8 b)
{
  return ((vec_short8)(si_shlh((qword)(a), (qword)(b))));
}

static __inline vec_uint4 spu_sl(vec_uint4 a, vec_uint4 b)
{
  return ((vec_uint4)(si_shl((qword)(a), (qword)(b))));
}

static __inline vec_int4 spu_sl(vec_int4 a, vec_uint4 b)
{
  return ((vec_int4)(si_shl((qword)(a), (qword)(b))));
}

static __inline vec_ushort8 spu_sl(vec_ushort8 a, unsigned int b)
{
  return ((vec_ushort8)(si_shlhi((qword)(a), b)));
}

static __inline vec_short8 spu_sl(vec_short8 a, unsigned int b)
{
  return ((vec_short8)(si_shlhi((qword)(a), b)));
}

static __inline vec_uint4 spu_sl(vec_uint4 a, unsigned int b)
{
  return ((vec_uint4)(si_shli((qword)(a), b)));
}

static __inline vec_int4 spu_sl(vec_int4 a, unsigned int b)
{
  return ((vec_int4)(si_shli((qword)(a), b)));
}


/* spu_slqw
 * ========
 */
static __inline vec_uchar16 spu_slqw(vec_uchar16 a, unsigned int count)
{
  return ((vec_uchar16)(si_shlqbi((qword)(a), si_from_uint(count))));
}

static __inline vec_char16 spu_slqw(vec_char16 a, unsigned int count)
{
  return ((vec_char16)(si_shlqbi((qword)(a), si_from_uint(count))));
}

static __inline vec_ushort8 spu_slqw(vec_ushort8 a, unsigned int count)
{
  return ((vec_ushort8)(si_shlqbi((qword)(a), si_from_uint(count))));
}

static __inline vec_short8 spu_slqw(vec_short8 a, unsigned int count)
{
  return ((vec_short8)(si_shlqbi((qword)(a), si_from_uint(count))));
}

static __inline vec_uint4 spu_slqw(vec_uint4 a, unsigned int count)
{
  return ((vec_uint4)(si_shlqbi((qword)(a), si_from_uint(count))));
}

static __inline vec_int4 spu_slqw(vec_int4 a, unsigned int count)
{
  return ((vec_int4)(si_shlqbi((qword)(a), si_from_uint(count))));
}

static __inline vec_float4 spu_slqw(vec_float4 a, unsigned int count)
{
  return ((vec_float4)(si_shlqbi((qword)(a), si_from_uint(count))));
}

static __inline vec_ullong2 spu_slqw(vec_ullong2 a, unsigned int count)
{
  return ((vec_ullong2)(si_shlqbi((qword)(a), si_from_uint(count))));
}

static __inline vec_llong2 spu_slqw(vec_llong2 a, unsigned int count)
{
  return ((vec_llong2)(si_shlqbi((qword)(a), si_from_uint(count))));
}

static __inline vec_double2 spu_slqw(vec_double2 a, unsigned int count)
{
  return ((vec_double2)(si_shlqbi((qword)(a), si_from_uint(count))));
}

/* spu_slqwbyte
 * ============
 */
static __inline vec_uchar16 spu_slqwbyte(vec_uchar16 a, unsigned int count)
{
  return ((vec_uchar16)(si_shlqby((qword)(a), si_from_uint(count))));
}

static __inline vec_char16 spu_slqwbyte(vec_char16 a, unsigned int count)
{
  return ((vec_char16)(si_shlqby((qword)(a), si_from_uint(count))));
}

static __inline vec_ushort8 spu_slqwbyte(vec_ushort8 a, unsigned int count)
{
  return ((vec_ushort8)(si_shlqby((qword)(a), si_from_uint(count))));
}

static __inline vec_short8 spu_slqwbyte(vec_short8 a, unsigned int count)
{
  return ((vec_short8)(si_shlqby((qword)(a), si_from_uint(count))));
}

static __inline vec_uint4 spu_slqwbyte(vec_uint4 a, unsigned int count)
{
  return ((vec_uint4)(si_shlqby((qword)(a), si_from_uint(count))));
}

static __inline vec_int4 spu_slqwbyte(vec_int4 a, unsigned int count)
{
  return ((vec_int4)(si_shlqby((qword)(a), si_from_uint(count))));
}

static __inline vec_float4 spu_slqwbyte(vec_float4 a, unsigned int count)
{
  return ((vec_float4)(si_shlqby((qword)(a), si_from_uint(count))));
}

static __inline vec_ullong2 spu_slqwbyte(vec_ullong2 a, unsigned int count)
{
  return ((vec_ullong2)(si_shlqby((qword)(a), si_from_uint(count))));
}

static __inline vec_llong2 spu_slqwbyte(vec_llong2 a, unsigned int count)
{
  return ((vec_llong2)(si_shlqby((qword)(a), si_from_uint(count))));
}

static __inline vec_double2 spu_slqwbyte(vec_double2 a, unsigned int count)
{
  return ((vec_double2)(si_shlqby((qword)(a), si_from_uint(count))));
}

/* spu_slqwbytebc
 * ==============
 */
static __inline vec_uchar16 spu_slqwbytebc(vec_uchar16 a, unsigned int count)
{
  return ((vec_uchar16)(si_shlqbybi((qword)(a), si_from_uint(count))));
}

static __inline vec_char16 spu_slqwbytebc(vec_char16 a, unsigned int count)
{
  return ((vec_char16)(si_shlqbybi((qword)(a), si_from_uint(count))));
}

static __inline vec_ushort8 spu_slqwbytebc(vec_ushort8 a, unsigned int count)
{
  return ((vec_ushort8)(si_shlqbybi((qword)(a), si_from_uint(count))));
}

static __inline vec_short8 spu_slqwbytebc(vec_short8 a, unsigned int count)
{
  return ((vec_short8)(si_shlqbybi((qword)(a), si_from_uint(count))));
}

static __inline vec_uint4 spu_slqwbytebc(vec_uint4 a, unsigned int count)
{
  return ((vec_uint4)(si_shlqbybi((qword)(a), si_from_uint(count))));
}

static __inline vec_int4 spu_slqwbytebc(vec_int4 a, unsigned int count)
{
  return ((vec_int4)(si_shlqbybi((qword)(a), si_from_uint(count))));
}

static __inline vec_float4 spu_slqwbytebc(vec_float4 a, unsigned int count)
{
  return ((vec_float4)(si_shlqbybi((qword)(a), si_from_uint(count))));
}

static __inline vec_ullong2 spu_slqwbytebc(vec_ullong2 a, unsigned int count)
{
  return ((vec_ullong2)(si_shlqbybi((qword)(a), si_from_uint(count))));
}

static __inline vec_llong2 spu_slqwbytebc(vec_llong2 a, unsigned int count)
{
  return ((vec_llong2)(si_shlqbybi((qword)(a), si_from_uint(count))));
}

static __inline vec_double2 spu_slqwbytebc(vec_double2 a, unsigned int count)
{
  return ((vec_double2)(si_shlqbybi((qword)(a), si_from_uint(count))));
}

/* spu_splats
 * ==========
 */
static __inline vec_uchar16 spu_splats(unsigned char a)
{
  union {
    vec_uchar16 v;
    unsigned char c[16];
  } in;

  in.c[0] = a;
  return (vec_splat(in.v, 0));
}

static __inline vec_char16 spu_splats(signed char a)
{
  return ((vec_char16)spu_splats((unsigned char)(a)));
}

static __inline vec_ushort8 spu_splats(unsigned short a)
{
  union {
    vec_ushort8 v;
    unsigned short s[8];
  } in;

  in.s[0] = a;
  return (vec_splat(in.v, 0));
}

static __inline vec_short8 spu_splats(signed short a)
{
  return ((vec_short8)spu_splats((unsigned short)(a)));
}

static __inline vec_uint4 spu_splats(unsigned int a)
{
  union {
    vec_uint4 v;
    unsigned int i[4];
  } in;

  in.i[0] = a;
  return (vec_splat(in.v, 0));
}

static __inline vec_int4 spu_splats(signed int a)
{
  return ((vec_int4)spu_splats((unsigned int)(a)));
}

static __inline vec_float4 spu_splats(float a)
{
  union {
    vec_float4 v;
    float f[4];
  } in;

  in.f[0] = a;
  return (vec_splat(in.v, 0));
}

static __inline vec_ullong2 spu_splats(unsigned long long a)
{
  union {
    vec_ullong2 v;
    unsigned long long l[2];
  } in;

  in.l[0] = a;
  in.l[1] = a;
  return (in.v);
}

static __inline vec_llong2 spu_splats(signed long long a)
{
  return ((vec_llong2)spu_splats((unsigned long long)(a)));
}

static __inline vec_double2 spu_splats(double a)
{
  union {
    vec_double2 v;
    double d[2];
  } in;

  in.d[0] = a;
  in.d[1] = a;
  return (in.v);
}


/* spu_stop
 * ========
 */
#define spu_stop(_type)	si_stop(_type)


/* spu_sub
 * =======
 */
static __inline vec_ushort8 spu_sub(vec_ushort8 a, vec_ushort8 b)
{
  return ((vec_ushort8)(si_sfh((qword)(b), (qword)(a))));
}

static __inline vec_short8 spu_sub(vec_short8 a, vec_short8 b)
{
  return ((vec_short8)(si_sfh((qword)(b), (qword)(a))));
}

static __inline vec_uint4 spu_sub(vec_uint4 a, vec_uint4 b)
{
  return ((vec_uint4)(si_sf((qword)(b), (qword)(a))));
}

static __inline vec_int4 spu_sub(vec_int4 a, vec_int4 b)
{
  return ((vec_int4)(si_sf((qword)(b), (qword)(a))));
}

static __inline vec_float4 spu_sub(vec_float4 a, vec_float4 b)
{
  return ((vec_float4)(si_fs((qword)(a), (qword)(b))));
}

static __inline vec_double2 spu_sub(vec_double2 a, vec_double2 b)
{
  return ((vec_double2)(si_dfs((qword)(a), (qword)(b))));
}

static __inline vec_uint4 spu_sub(unsigned int a, vec_uint4 b)
{
  return ((vec_uint4)(si_sfi((qword)b, (int)a)));
}

static __inline vec_int4 spu_sub(signed int a, vec_int4 b)
{
  return ((vec_int4)(si_sfi((qword)b, (int)a)));
}

static __inline vec_ushort8 spu_sub(unsigned short a, vec_ushort8 b)
{
  return ((vec_ushort8)(si_sfhi((qword)b, (short)a)));
}

static __inline vec_short8 spu_sub(signed short a, vec_short8 b)
{
  return ((vec_short8)(si_sfhi((qword)b, (short)a)));
}

/* spu_subx
 * ========
 */
static __inline vec_uint4 spu_subx(vec_uint4 a, vec_uint4 b, vec_uint4 c)
{
  return ((vec_uint4)(si_sfx((qword)(b), (qword)(a), (qword)(c))));
}

static __inline vec_int4 spu_subx(vec_int4 a, vec_int4 b, vec_int4 c)
{
  return ((vec_int4)(si_sfx((qword)(b), (qword)(a), (qword)(c))));
}

/* spu_sumb
 * ========
 */
static __inline vec_ushort8 spu_sumb(vec_uchar16 a, vec_uchar16 b)
{
  return ((vec_ushort8)(si_sumb((qword)(a), (qword)(b))));
}  


/* spu_sync
 * spu_sync_c
 * ========
 */
#define spu_sync()	/* do nothing */

#define spu_sync_c()	/* do nothing */


/* spu_writech
 * ===========
 */
#define spu_writech(_channel, _a)	/* not mappable */

/* spu_writechqw
 * =============
 */
#define spu_writechqw(_channel, _a)	/* not mappable */


/* spu_xor
 * =======
 */
static __inline vec_uchar16 spu_xor(vec_uchar16 a, vec_uchar16 b)
{
  return ((vec_uchar16)(si_xor((qword)(a), (qword)(b))));
}

static __inline vec_char16 spu_xor(vec_char16 a, vec_char16 b)
{
  return ((vec_char16)(si_xor((qword)(a), (qword)(b))));
}

static __inline vec_ushort8 spu_xor(vec_ushort8 a, vec_ushort8 b)
{
  return ((vec_ushort8)(si_xor((qword)(a), (qword)(b))));
}

static __inline vec_short8 spu_xor(vec_short8 a, vec_short8 b)
{
  return ((vec_short8)(si_xor((qword)(a), (qword)(b))));
}

static __inline vec_uint4 spu_xor(vec_uint4 a, vec_uint4 b)
{
  return ((vec_uint4)(si_xor((qword)(a), (qword)(b))));
}

static __inline vec_int4 spu_xor(vec_int4 a, vec_int4 b)
{
  return ((vec_int4)(si_xor((qword)(a), (qword)(b))));
}

static __inline vec_float4 spu_xor(vec_float4 a, vec_float4 b)
{
  return ((vec_float4)(si_xor((qword)(a), (qword)(b))));
}

static __inline vec_ullong2 spu_xor(vec_ullong2 a, vec_ullong2 b)
{
  return ((vec_ullong2)(si_xor((qword)(a), (qword)(b))));
}

static __inline vec_llong2 spu_xor(vec_llong2 a, vec_llong2 b)
{
  return ((vec_llong2)(si_xor((qword)(a), (qword)(b))));
}

static __inline vec_double2 spu_xor(vec_double2 a, vec_double2 b)
{
  return ((vec_double2)(si_xor((qword)(a), (qword)(b))));
}

static __inline vec_uchar16 spu_xor(vec_uchar16 a, unsigned char b)
{
  return ((vec_uchar16)(si_xorbi((qword)(a), b)));
}

static __inline vec_char16 spu_xor(vec_char16 a, signed char b)
{
  return ((vec_char16)(si_xorbi((qword)(a), (unsigned char)(b))));
}

static __inline vec_ushort8 spu_xor(vec_ushort8 a, unsigned short b)
{
  return ((vec_ushort8)(si_xorhi((qword)(a), b)));
}

static __inline vec_short8 spu_xor(vec_short8 a, signed short b)
{
  return ((vec_short8)(si_xorhi((qword)(a), (unsigned short)(b))));
}

static __inline vec_uint4 spu_xor(vec_uint4 a, unsigned int b)
{
  return ((vec_uint4)(si_xori((qword)(a), b)));
}

static __inline vec_int4 spu_xor(vec_int4 a, signed int b)
{
  return ((vec_int4)(si_xori((qword)(a), (unsigned int)(b))));
}

#endif /* !__SPU__ */
#endif /* __cplusplus */
#endif /* !_SPU2VMX_H_ */
