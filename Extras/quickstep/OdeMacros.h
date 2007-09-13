/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser bteral Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       bteral Public License is included with this library in the     *
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

#define ODE_MACROS
#ifdef ODE_MACROS

#include "LinearMath/btScalar.h"

typedef btScalar dVector4[4];
typedef btScalar dMatrix3[4*3];
#define dInfinity FLT_MAX



#define dRecip(x) ((float)(1.0f/(x)))				/* reciprocal */



#define dMULTIPLY0_331NEW(A,op,B,C) \
{\
	btScalar tmp[3];\
	tmp[0] = C.getX();\
	tmp[1] = C.getY();\
	tmp[2] = C.getZ();\
	dMULTIPLYOP0_331(A,op,B,tmp);\
}

#define dMULTIPLY0_331(A,B,C) dMULTIPLYOP0_331(A,=,B,C)
#define dMULTIPLYOP0_331(A,op,B,C) \
  (A)[0] op dDOT1((B),(C)); \
  (A)[1] op dDOT1((B+4),(C)); \
  (A)[2] op dDOT1((B+8),(C));

#define dAASSERT btAssert
#define dIASSERT btAssert

#define REAL float
#define dDOTpq(a,b,p,q) ((a)[0]*(b)[0] + (a)[p]*(b)[q] + (a)[2*(p)]*(b)[2*(q)])
inline btScalar dDOT1 (const btScalar *a, const btScalar *b)
{ return dDOTpq(a,b,1,1); }
#define dDOT14(a,b) dDOTpq(a,b,1,4)

#define dCROSS(a,op,b,c) \
  (a)[0] op ((b)[1]*(c)[2] - (b)[2]*(c)[1]); \
  (a)[1] op ((b)[2]*(c)[0] - (b)[0]*(c)[2]); \
  (a)[2] op ((b)[0]*(c)[1] - (b)[1]*(c)[0]);

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


#define dMULTIPLYOP2_333(A,op,B,C) \
  (A)[0] op dDOT1((B),(C)); \
  (A)[1] op dDOT1((B),(C+4)); \
  (A)[2] op dDOT1((B),(C+8)); \
  (A)[4] op dDOT1((B+4),(C)); \
  (A)[5] op dDOT1((B+4),(C+4)); \
  (A)[6] op dDOT1((B+4),(C+8)); \
  (A)[8] op dDOT1((B+8),(C)); \
  (A)[9] op dDOT1((B+8),(C+4)); \
  (A)[10] op dDOT1((B+8),(C+8));
#define dMULTIPLYOP0_333(A,op,B,C) \
  (A)[0] op dDOT14((B),(C)); \
  (A)[1] op dDOT14((B),(C+1)); \
  (A)[2] op dDOT14((B),(C+2)); \
  (A)[4] op dDOT14((B+4),(C)); \
  (A)[5] op dDOT14((B+4),(C+1)); \
  (A)[6] op dDOT14((B+4),(C+2)); \
  (A)[8] op dDOT14((B+8),(C)); \
  (A)[9] op dDOT14((B+8),(C+1)); \
  (A)[10] op dDOT14((B+8),(C+2));

#define dMULTIPLY2_333(A,B,C) dMULTIPLYOP2_333(A,=,B,C)
#define dMULTIPLY0_333(A,B,C) dMULTIPLYOP0_333(A,=,B,C)
#define dMULTIPLYADD0_331(A,B,C) dMULTIPLYOP0_331(A,+=,B,C)


////////////////////////////////////////////////////////////////////
#define EFFICIENT_ALIGNMENT 16
#define dEFFICIENT_SIZE(x) ((((x)-1)|(EFFICIENT_ALIGNMENT-1))+1)
/* alloca aligned to the EFFICIENT_ALIGNMENT. note that this can waste
 * up to 15 bytes per allocation, depending on what alloca() returns.
 */

#define dALLOCA16(n) \
  ((char*)dEFFICIENT_SIZE(((size_t)(alloca((n)+(EFFICIENT_ALIGNMENT-1))))))



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

#ifdef DEBUG
#define ANSI_FTOL 1

extern "C" {
    __declspec(naked) void _ftol2() {
        __asm    {
#if ANSI_FTOL
            fnstcw   WORD PTR [esp-2]
            mov      ax, WORD PTR [esp-2]

            OR AX,	 0C00h

            mov      WORD PTR [esp-4], ax
            fldcw    WORD PTR [esp-4]
            fistp    QWORD PTR [esp-12]
            fldcw    WORD PTR [esp-2]
            mov      eax, DWORD PTR [esp-12]
            mov      edx, DWORD PTR [esp-8]
#else
            fistp    DWORD PTR [esp-12]
            mov		 eax, DWORD PTR [esp-12]
            mov		 ecx, DWORD PTR [esp-8]
#endif
            ret
        }
    }
}
#endif //DEBUG






#define ALLOCA dALLOCA16

typedef const btScalar *dRealPtr;
typedef btScalar *dRealMutablePtr;
#define dRealArray(name,n) btScalar name[n];
#define dRealAllocaArray(name,n) btScalar *name = (btScalar*) ALLOCA ((n)*sizeof(btScalar));

inline void dSetZero1 (btScalar *a, int n)
{
  dAASSERT (a && n >= 0);
  while (n > 0) {
    *(a++) = 0;
    n--;
  }
}

inline void dSetValue1 (btScalar *a, int n, btScalar value)
{
  dAASSERT (a && n >= 0);
  while (n > 0) {
    *(a++) = value;
    n--;
  }
}


#endif //USE_SOR_SOLVER

