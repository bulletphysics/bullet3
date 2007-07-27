/*  SIMD math library functions for both the PowerPC (PPU) and the SPU.
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



#ifndef ___SIMD_MATH_H___
#define ___SIMD_MATH_H___

#define __SIMD_MATH_HAVE_VECTOR_f4 0
#define __SIMD_MATH_HAVE_VECTOR_i4 0
#define __SIMD_MATH_HAVE_VECTOR_d2 0
#define __SIMD_MATH_HAVE_VECTOR_ll2 0

#ifdef __SPU__

/* SPU has vector float, vector double,
   vector {un,}signed long long, and vector {un,signed} int. */

#include <spu_intrinsics.h>

#undef __SIMD_MATH_HAVE_VECTOR_f4
#define __SIMD_MATH_HAVE_VECTOR_f4 1

#undef __SIMD_MATH_HAVE_VECTOR_i4
#define __SIMD_MATH_HAVE_VECTOR_i4 1

#undef __SIMD_MATH_HAVE_VECTOR_d2
#define __SIMD_MATH_HAVE_VECTOR_d2 1

#undef __SIMD_MATH_HAVE_VECTOR_ll2
#define __SIMD_MATH_HAVE_VECTOR_ll2 1

#elif defined(__ALTIVEC__)

#include <altivec.h>

/* PPU has vector float, and vector int. */
#undef __SIMD_MATH_HAVE_VECTOR_f4
#define __SIMD_MATH_HAVE_VECTOR_f4 1

#undef __SIMD_MATH_HAVE_VECTOR_i4
#define __SIMD_MATH_HAVE_VECTOR_i4 1

#else

/* Just in case someone tries to include this in say a i686 build.  */

#error "No functions defined"

#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Types */

#if __SIMD_MATH_HAVE_VECTOR_i4
typedef struct divi4_s {
  vector signed int quot;
  vector signed int rem;
} divi4_t;
#endif

#if __SIMD_MATH_HAVE_VECTOR_i4
typedef struct divu4_s {
  vector unsigned int quot;
  vector unsigned int rem;
} divu4_t;
#endif

#if __SIMD_MATH_HAVE_VECTOR_ll2
typedef struct lldivi2_s {
  vector signed long long quot;
  vector signed long long rem;
} lldivi2_t;
#endif

#if __SIMD_MATH_HAVE_VECTOR_ll2
typedef struct lldivu2_s {
  vector unsigned long long quot;
  vector unsigned long long rem;
} lldivu2_t;
#endif

#if __SIMD_MATH_HAVE_VECTOR_f4 && __SIMD_MATH_HAVE_VECTOR_ll2
typedef struct llroundf4_s {
	vector signed long long vll[2];
} llroundf4_t;
#endif

/* integer divide */

#if __SIMD_MATH_HAVE_VECTOR_i4
divi4_t divi4 (vector signed int, vector signed int);
#endif

#if __SIMD_MATH_HAVE_VECTOR_i4
divu4_t divu4 (vector unsigned int, vector unsigned int);
#endif

#if __SIMD_MATH_HAVE_VECTOR_ll2
lldivi2_t lldivi2 (vector signed long long, vector signed long long);
#endif

#if __SIMD_MATH_HAVE_VECTOR_ll2
lldivu2_t lldivu2 (vector unsigned long long, vector unsigned long long);
#endif

/* abs value */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float fabsf4 (vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double fabsd2 (vector double);
#endif

#if __SIMD_MATH_HAVE_VECTOR_i4
vector signed int absi4 (vector signed int);
#endif

#if __SIMD_MATH_HAVE_VECTOR_ll2
vector signed long long llabsi2 (vector signed long long);
#endif

/* negate value */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float negatef4 (vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double negated2 (vector double);
#endif

#if __SIMD_MATH_HAVE_VECTOR_i4
vector signed int negatei4 (vector signed int);
#endif

#if __SIMD_MATH_HAVE_VECTOR_ll2
vector signed long long negatell2 (vector signed long long);
#endif

/* trunc */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float truncf4 (vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double trund2 (vector double);
#endif

/* floor */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float floorf4 (vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double floord2 (vector double);
#endif

/* ceil */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float ceilf4 (vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double ceild2 (vector double);
#endif

/* exp */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float expf4 (vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double expd2 (vector double);
#endif

/* exp */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float exp2f4 (vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double exp2d2 (vector double);
#endif

/* expm1 */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float expm1f4 (vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double expm1d2 (vector double);
#endif

/* log */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float logf4 (vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double logd2 (vector double);
#endif

/* log10 */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float log10f4 (vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double log10d2 (vector double);
#endif

/* log1p */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float log1pf4 (vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double log1pd2 (vector double);
#endif

/* pow */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float powf4 (vector float, vector float);
#endif

/* fma */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float fmaf4 (vector float, vector float, vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double fmad2 (vector double, vector double, vector double);
#endif

/* fmax */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float fmaxf4 (vector float, vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double fmaxd2 (vector double, vector double);
#endif

/* fmin */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float fminf4 (vector float, vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double fmind2 (vector double, vector double);
#endif

/* fdim */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float fdimf4 (vector float, vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double fdimd2 (vector double, vector double);
#endif


/* fmod */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float fmodf4 (vector float, vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double fmodd2 (vector double, vector double);
#endif

/* log2 */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float log2f4 (vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double log2d2 (vector double);
#endif

/* logb */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float logbf4 (vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double logbd2 (vector double);
#endif

/* ilogb */
#if __SIMD_MATH_HAVE_VECTOR_f4 && __SIMD_MATH_HAVE_VECTOR_i4
vector signed int ilogbf4 (vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2 && __SIMD_MATH_HAVE_VECTOR_ll2
vector signed long long ilogbd2 (vector double);
#endif

/* modf */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float modff4 (vector float, vector float *);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double modfd2 (vector double, vector double *);
#endif

/* sqrt */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float sqrtf4 (vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double sqrtd2 (vector double);
#endif

/* hypot */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float hypotf4 (vector float, vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double hypotd2 (vector double, vector double);
#endif

/* cbrtf4 */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float cbrtf4 (vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double cbrtd2 (vector double);
#endif

/* sin */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float sinf4 (vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double sind2 (vector double);
#endif


/* asin */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float asinf4 (vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double asind2 (vector double);
#endif



/* divide */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float divf4 (vector float, vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double divd2 (vector double, vector double);
#endif

/* remainder */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float remainderf4 (vector float, vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double remainderd2 (vector double, vector double);
#endif

#if __SIMD_MATH_HAVE_VECTOR_f4 && __SIMD_MATH_HAVE_VECTOR_i4
vector float remquof4(vector float x, vector float y, vector signed int *quo);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2 && __SIMD_MATH_HAVE_VECTOR_ll2
vector double remquod2(vector double x, vector double y, vector signed long long *quo);
#endif

/* copysign */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float copysignf4 (vector float, vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double copysignd2 (vector double, vector double);
#endif

/* cos */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float cosf4 (vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double cosd2 (vector double);
#endif

/* acos */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float acosf4 (vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double acosd2 (vector double);
#endif

/* atan */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float atanf4 (vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double atand2 (vector double);
#endif

/* atan2 */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float atan2f4 (vector float, vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double atan2d2 (vector double, vector double);
#endif


/* tan */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float tanf4 (vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double tand2 (vector double);
#endif

/* sincos */
#if __SIMD_MATH_HAVE_VECTOR_f4
void sincosf4 (vector float, vector float *, vector float *);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
void sincosd2 (vector double, vector double *, vector double *);
#endif



/* recip */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float recipf4 (vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double recipd2 (vector double);
#endif


/* rsqrt */
#if __SIMD_MATH_HAVE_VECTOR_f4
vector float rsqrtf4 (vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double rsqrtd2 (vector double);
#endif


/* frexp */
#if __SIMD_MATH_HAVE_VECTOR_f4 && __SIMD_MATH_HAVE_VECTOR_i4
vector float frexpf4 (vector float, vector signed int *);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2 && __SIMD_MATH_HAVE_VECTOR_ll2
vector double frexpd2 (vector double, vector signed long long *);
#endif

/* ldexp */
#if __SIMD_MATH_HAVE_VECTOR_f4 && __SIMD_MATH_HAVE_VECTOR_i4
vector float ldexpf4 (vector float, vector signed int );
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2 && __SIMD_MATH_HAVE_VECTOR_ll2
vector double ldexpd2 (vector double, vector signed long long );
#endif

#if __SIMD_MATH_HAVE_VECTOR_f4 && __SIMD_MATH_HAVE_VECTOR_i4
vector float scalbnf4(vector float x, vector signed int n);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2 && __SIMD_MATH_HAVE_VECTOR_ll2
vector double scalbllnd2 (vector double, vector signed long long );
#endif


/* isnan */
#if __SIMD_MATH_HAVE_VECTOR_f4 && __SIMD_MATH_HAVE_VECTOR_i4
vector unsigned int isnanf4 (vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2 && __SIMD_MATH_HAVE_VECTOR_ll2
vector unsigned long long isnand2 (vector double);
#endif

/* isinf */
#if __SIMD_MATH_HAVE_VECTOR_f4 && __SIMD_MATH_HAVE_VECTOR_i4
vector unsigned int isinff4 (vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2 && __SIMD_MATH_HAVE_VECTOR_ll2
vector unsigned long long isinfd2 (vector double);
#endif

/* isfinite */
#if __SIMD_MATH_HAVE_VECTOR_f4 && __SIMD_MATH_HAVE_VECTOR_i4
vector unsigned int isfinitef4 (vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2 && __SIMD_MATH_HAVE_VECTOR_ll2
vector unsigned long long isfinited2 (vector double);
#endif

/* isnormal */
#if __SIMD_MATH_HAVE_VECTOR_f4 && __SIMD_MATH_HAVE_VECTOR_i4
vector unsigned int isnormalf4 (vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2 && __SIMD_MATH_HAVE_VECTOR_ll2
vector unsigned long long isnormald2 (vector double);
#endif

/* isunordered */
#if __SIMD_MATH_HAVE_VECTOR_f4 && __SIMD_MATH_HAVE_VECTOR_i4
vector unsigned int isunorderedf4 (vector float, vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2 && __SIMD_MATH_HAVE_VECTOR_ll2
vector unsigned long long isunorderedd2 (vector double, vector double);
#endif

/* is0denorm */
#if __SIMD_MATH_HAVE_VECTOR_f4 && __SIMD_MATH_HAVE_VECTOR_i4
vector unsigned int is0denormf4 (vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2 && __SIMD_MATH_HAVE_VECTOR_ll2
vector unsigned long long is0denormd2 (vector double);
#endif

/* signbit */
#if __SIMD_MATH_HAVE_VECTOR_f4 && __SIMD_MATH_HAVE_VECTOR_i4
vector unsigned int signbitf4 (vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2 && __SIMD_MATH_HAVE_VECTOR_ll2
vector unsigned long long signbitd2 (vector double);
#endif

/* isequal */
#if __SIMD_MATH_HAVE_VECTOR_f4 && __SIMD_MATH_HAVE_VECTOR_i4
vector unsigned int isequalf4 (vector float, vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2 && __SIMD_MATH_HAVE_VECTOR_ll2
vector unsigned long long isequald2 (vector double, vector double);
#endif

/* islessgreater */
#if __SIMD_MATH_HAVE_VECTOR_f4 && __SIMD_MATH_HAVE_VECTOR_i4
vector unsigned int islessgreaterf4 (vector float, vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2 && __SIMD_MATH_HAVE_VECTOR_ll2
vector unsigned long long islessgreaterd2 (vector double, vector double);
#endif

/* isless */
#if __SIMD_MATH_HAVE_VECTOR_f4 && __SIMD_MATH_HAVE_VECTOR_i4
vector unsigned int islessf4 (vector float, vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2 && __SIMD_MATH_HAVE_VECTOR_ll2
vector unsigned long long islessd2 (vector double, vector double);
#endif

/* isgreater */
#if __SIMD_MATH_HAVE_VECTOR_f4 && __SIMD_MATH_HAVE_VECTOR_i4
vector unsigned int isgreaterf4 (vector float, vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2 && __SIMD_MATH_HAVE_VECTOR_ll2
vector unsigned long long isgreaterd2 (vector double, vector double);
#endif

/* islessequal */
#if __SIMD_MATH_HAVE_VECTOR_f4 && __SIMD_MATH_HAVE_VECTOR_i4
vector unsigned int islessequalf4 (vector float, vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2 && __SIMD_MATH_HAVE_VECTOR_ll2
vector unsigned long long islessequald2 (vector double, vector double);
#endif

/* isgreaterequal */
#if __SIMD_MATH_HAVE_VECTOR_f4 && __SIMD_MATH_HAVE_VECTOR_i4
vector unsigned int isgreaterequalf4 (vector float, vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2 && __SIMD_MATH_HAVE_VECTOR_ll2
vector unsigned long long isgreaterequald2 (vector double, vector double);
#endif

/* fpclassify */
#if __SIMD_MATH_HAVE_VECTOR_f4 && __SIMD_MATH_HAVE_VECTOR_i4
vector signed int fpclassifyf4 (vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2 && __SIMD_MATH_HAVE_VECTOR_ll2
vector signed long long fpclassifyd2 (vector double);
#endif

/* round */
#if __SIMD_MATH_HAVE_VECTOR_d2 && __SIMD_MATH_HAVE_VECTOR_ll2
vector signed long long llroundd2 (vector double);
#endif

#if __SIMD_MATH_HAVE_VECTOR_f4 && __SIMD_MATH_HAVE_VECTOR_ll2
llroundf4_t llroundf4 (vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_f4 && __SIMD_MATH_HAVE_VECTOR_ll2
llroundf4_t llrintf4 (vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2 && __SIMD_MATH_HAVE_VECTOR_ll2
vector signed long long llrintd2 (vector double);
#endif

#if __SIMD_MATH_HAVE_VECTOR_f4
vector float roundf4(vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_f4 && __SIMD_MATH_HAVE_VECTOR_i4
vector signed int  iroundf4(vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_f4
vector float rintf4(vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_f4 && __SIMD_MATH_HAVE_VECTOR_i4
vector signed int  irintf4(vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double nextafterd2 (vector double, vector double);
#endif

#if __SIMD_MATH_HAVE_VECTOR_f4
vector float nextafterf4(vector float, vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double nearbyintd2 (vector double);
#endif

#if __SIMD_MATH_HAVE_VECTOR_f4
vector float nearbyintf4(vector float);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double truncd2 (vector double);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double roundd2 (vector double);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double rintd2 (vector double);
#endif

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double ceild2(vector double);
#endif 

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double floord2(vector double);
#endif 

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double fmodd2(vector double, vector double);
#endif 

#if __SIMD_MATH_HAVE_VECTOR_d2
vector double remainderd2(vector double, vector double);
#endif 

#ifdef __cplusplus
}
#endif

#endif
