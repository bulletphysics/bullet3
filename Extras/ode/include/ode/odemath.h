/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

#ifndef _ODE_ODEMATH_H_
#define _ODE_ODEMATH_H_

#include <ode/common.h>

#ifdef __GNUC__
#define PURE_INLINE extern inline
#else
#define PURE_INLINE inline
#endif

/*
 * macro to access elements i,j in an NxM matrix A, independent of the
 * matrix storage convention.
 */
#define dACCESS33(A,i,j) ((A)[(i)*4+(j)])


/*
 * 3-way dot product. dDOTpq means that elements of `a' and `b' are spaced
 * p and q indexes apart respectively. dDOT() means dDOT11.
 * in C++ we could use function templates to get all the versions of these
 * functions - but on some compilers this will result in sub-optimal code.
 */

#define dDOTpq(a,b,p,q) ((a)[0]*(b)[0] + (a)[p]*(b)[q] + (a)[2*(p)]*(b)[2*(q)])

#ifdef __cplusplus

PURE_INLINE dReal dDOT   (const dReal *a, const dReal *b) { return dDOTpq(a,b,1,1); }
PURE_INLINE dReal dDOT13 (const dReal *a, const dReal *b) { return dDOTpq(a,b,1,3); }
PURE_INLINE dReal dDOT31 (const dReal *a, const dReal *b) { return dDOTpq(a,b,3,1); }
PURE_INLINE dReal dDOT33 (const dReal *a, const dReal *b) { return dDOTpq(a,b,3,3); }
PURE_INLINE dReal dDOT14 (const dReal *a, const dReal *b) { return dDOTpq(a,b,1,4); }
PURE_INLINE dReal dDOT41 (const dReal *a, const dReal *b) { return dDOTpq(a,b,4,1); }
PURE_INLINE dReal dDOT44 (const dReal *a, const dReal *b) { return dDOTpq(a,b,4,4); }

#else

#define dDOT(a,b)   dDOTpq(a,b,1,1)
#define dDOT13(a,b) dDOTpq(a,b,1,3)
#define dDOT31(a,b) dDOTpq(a,b,3,1)
#define dDOT33(a,b) dDOTpq(a,b,3,3)
#define dDOT14(a,b) dDOTpq(a,b,1,4)
#define dDOT41(a,b) dDOTpq(a,b,4,1)
#define dDOT44(a,b) dDOTpq(a,b,4,4)

#endif /* __cplusplus */


/*
 * cross product, set a = b x c. dCROSSpqr means that elements of `a', `b'
 * and `c' are spaced p, q and r indexes apart respectively.
 * dCROSS() means dCROSS111. `op' is normally `=', but you can set it to
 * +=, -= etc to get other effects.
 */

#define dCROSS(a,op,b,c) \
do { \
  (a)[0] op ((b)[1]*(c)[2] - (b)[2]*(c)[1]); \
  (a)[1] op ((b)[2]*(c)[0] - (b)[0]*(c)[2]); \
  (a)[2] op ((b)[0]*(c)[1] - (b)[1]*(c)[0]); \
} while(0)
#define dCROSSpqr(a,op,b,c,p,q,r) \
do { \
  (a)[  0] op ((b)[  q]*(c)[2*r] - (b)[2*q]*(c)[  r]); \
  (a)[  p] op ((b)[2*q]*(c)[  0] - (b)[  0]*(c)[2*r]); \
  (a)[2*p] op ((b)[  0]*(c)[  r] - (b)[  q]*(c)[  0]); \
} while(0)
#define dCROSS114(a,op,b,c) dCROSSpqr(a,op,b,c,1,1,4)
#define dCROSS141(a,op,b,c) dCROSSpqr(a,op,b,c,1,4,1)
#define dCROSS144(a,op,b,c) dCROSSpqr(a,op,b,c,1,4,4)
#define dCROSS411(a,op,b,c) dCROSSpqr(a,op,b,c,4,1,1)
#define dCROSS414(a,op,b,c) dCROSSpqr(a,op,b,c,4,1,4)
#define dCROSS441(a,op,b,c) dCROSSpqr(a,op,b,c,4,4,1)
#define dCROSS444(a,op,b,c) dCROSSpqr(a,op,b,c,4,4,4)


/*
 * set a 3x3 submatrix of A to a matrix such that submatrix(A)*b = a x b.
 * A is stored by rows, and has `skip' elements per row. the matrix is
 * assumed to be already zero, so this does not write zero elements!
 * if (plus,minus) is (+,-) then a positive version will be written.
 * if (plus,minus) is (-,+) then a negative version will be written.
 */

#define dCROSSMAT(A,a,skip,plus,minus) \
do { \
  (A)[1] = minus (a)[2]; \
  (A)[2] = plus (a)[1]; \
  (A)[(skip)+0] = plus (a)[2]; \
  (A)[(skip)+2] = minus (a)[0]; \
  (A)[2*(skip)+0] = minus (a)[1]; \
  (A)[2*(skip)+1] = plus (a)[0]; \
} while(0)


/*
 * compute the distance between two 3-vectors
 */

#ifdef __cplusplus
PURE_INLINE float dDISTANCE (const float a[3], const float b[3])
	{ return (float) dSqrt( (a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]) + (a[2]-b[2])*(a[2]-b[2]) ); }
PURE_INLINE double dDISTANCE (const double a[3], const double b[3])
	{ return dSqrt( (a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]) + (a[2]-b[2])*(a[2]-b[2]) ); }
#else
#define dDISTANCE(a,b) \
	(dSqrt( ((a)[0]-(b)[0])*((a)[0]-(b)[0]) + ((a)[1]-(b)[1])*((a)[1]-(b)[1]) + ((a)[2]-(b)[2])*((a)[2]-(b)[2]) ))
#endif


/*
 * special case matrix multipication, with operator selection
 */

#define dMULTIPLYOP0_331(A,op,B,C) \
do { \
  (A)[0] op dDOT((B),(C)); \
  (A)[1] op dDOT((B+4),(C)); \
  (A)[2] op dDOT((B+8),(C)); \
} while(0)
#define dMULTIPLYOP1_331(A,op,B,C) \
do { \
  (A)[0] op dDOT41((B),(C)); \
  (A)[1] op dDOT41((B+1),(C)); \
  (A)[2] op dDOT41((B+2),(C)); \
} while(0)
#define dMULTIPLYOP0_133(A,op,B,C) \
do { \
  (A)[0] op dDOT14((B),(C)); \
  (A)[1] op dDOT14((B),(C+1)); \
  (A)[2] op dDOT14((B),(C+2)); \
} while(0)
#define dMULTIPLYOP0_333(A,op,B,C) \
do { \
  (A)[0] op dDOT14((B),(C)); \
  (A)[1] op dDOT14((B),(C+1)); \
  (A)[2] op dDOT14((B),(C+2)); \
  (A)[4] op dDOT14((B+4),(C)); \
  (A)[5] op dDOT14((B+4),(C+1)); \
  (A)[6] op dDOT14((B+4),(C+2)); \
  (A)[8] op dDOT14((B+8),(C)); \
  (A)[9] op dDOT14((B+8),(C+1)); \
  (A)[10] op dDOT14((B+8),(C+2)); \
} while(0)
#define dMULTIPLYOP1_333(A,op,B,C) \
do { \
  (A)[0] op dDOT44((B),(C)); \
  (A)[1] op dDOT44((B),(C+1)); \
  (A)[2] op dDOT44((B),(C+2)); \
  (A)[4] op dDOT44((B+1),(C)); \
  (A)[5] op dDOT44((B+1),(C+1)); \
  (A)[6] op dDOT44((B+1),(C+2)); \
  (A)[8] op dDOT44((B+2),(C)); \
  (A)[9] op dDOT44((B+2),(C+1)); \
  (A)[10] op dDOT44((B+2),(C+2)); \
} while(0)
#define dMULTIPLYOP2_333(A,op,B,C) \
do { \
  (A)[0] op dDOT((B),(C)); \
  (A)[1] op dDOT((B),(C+4)); \
  (A)[2] op dDOT((B),(C+8)); \
  (A)[4] op dDOT((B+4),(C)); \
  (A)[5] op dDOT((B+4),(C+4)); \
  (A)[6] op dDOT((B+4),(C+8)); \
  (A)[8] op dDOT((B+8),(C)); \
  (A)[9] op dDOT((B+8),(C+4)); \
  (A)[10] op dDOT((B+8),(C+8)); \
} while(0)

#ifdef __cplusplus

#define DECL template <class TA, class TB, class TC> PURE_INLINE void

DECL dMULTIPLY0_331(TA *A, const TB *B, const TC *C) { dMULTIPLYOP0_331(A,=,B,C); }
DECL dMULTIPLY1_331(TA *A, const TB *B, const TC *C) { dMULTIPLYOP1_331(A,=,B,C); }
DECL dMULTIPLY0_133(TA *A, const TB *B, const TC *C) { dMULTIPLYOP0_133(A,=,B,C); }
DECL dMULTIPLY0_333(TA *A, const TB *B, const TC *C) { dMULTIPLYOP0_333(A,=,B,C); }
DECL dMULTIPLY1_333(TA *A, const TB *B, const TC *C) { dMULTIPLYOP1_333(A,=,B,C); }
DECL dMULTIPLY2_333(TA *A, const TB *B, const TC *C) { dMULTIPLYOP2_333(A,=,B,C); }

DECL dMULTIPLYADD0_331(TA *A, const TB *B, const TC *C) { dMULTIPLYOP0_331(A,+=,B,C); }
DECL dMULTIPLYADD1_331(TA *A, const TB *B, const TC *C) { dMULTIPLYOP1_331(A,+=,B,C); }
DECL dMULTIPLYADD0_133(TA *A, const TB *B, const TC *C) { dMULTIPLYOP0_133(A,+=,B,C); }
DECL dMULTIPLYADD0_333(TA *A, const TB *B, const TC *C) { dMULTIPLYOP0_333(A,+=,B,C); }
DECL dMULTIPLYADD1_333(TA *A, const TB *B, const TC *C) { dMULTIPLYOP1_333(A,+=,B,C); }
DECL dMULTIPLYADD2_333(TA *A, const TB *B, const TC *C) { dMULTIPLYOP2_333(A,+=,B,C); }

#undef DECL

#else

#define dMULTIPLY0_331(A,B,C) dMULTIPLYOP0_331(A,=,B,C)
#define dMULTIPLY1_331(A,B,C) dMULTIPLYOP1_331(A,=,B,C)
#define dMULTIPLY0_133(A,B,C) dMULTIPLYOP0_133(A,=,B,C)
#define dMULTIPLY0_333(A,B,C) dMULTIPLYOP0_333(A,=,B,C)
#define dMULTIPLY1_333(A,B,C) dMULTIPLYOP1_333(A,=,B,C)
#define dMULTIPLY2_333(A,B,C) dMULTIPLYOP2_333(A,=,B,C)

#define dMULTIPLYADD0_331(A,B,C) dMULTIPLYOP0_331(A,+=,B,C)
#define dMULTIPLYADD1_331(A,B,C) dMULTIPLYOP1_331(A,+=,B,C)
#define dMULTIPLYADD0_133(A,B,C) dMULTIPLYOP0_133(A,+=,B,C)
#define dMULTIPLYADD0_333(A,B,C) dMULTIPLYOP0_333(A,+=,B,C)
#define dMULTIPLYADD1_333(A,B,C) dMULTIPLYOP1_333(A,+=,B,C)
#define dMULTIPLYADD2_333(A,B,C) dMULTIPLYOP2_333(A,+=,B,C)

#endif


#ifdef __cplusplus
extern "C" {
#endif

/*
 * normalize 3x1 and 4x1 vectors (i.e. scale them to unit length)
 */
void dNormalize3 (dVector3 a);
void dNormalize4 (dVector4 a);


/*
 * given a unit length "normal" vector n, generate vectors p and q vectors
 * that are an orthonormal basis for the plane space perpendicular to n.
 * i.e. this makes p,q such that n,p,q are all perpendicular to each other.
 * q will equal n x p. if n is not unit length then p will be unit length but
 * q wont be.
 */

void dPlaneSpace (const dVector3 n, dVector3 p, dVector3 q);

#ifdef __cplusplus
}
#endif

#endif
