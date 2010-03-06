/* @(#)86	1.3  src/include/vec_literal.h, sw.includes, sdk_pub 10/11/05 16:00:27 */
/* -------------------------------------------------------------- */
/* (C) Copyright 2001,2005,                                       */
/* International Business Machines Corporation,                   */
/* Sony Computer Entertainment Incorporated,                      */
/* Toshiba Corporation.                                           */
/*                                                                */
/* All Rights Reserved.                                           */
/* -------------------------------------------------------------- */
/* PROLOG END TAG zYx                                              */
#ifndef _VEC_LITERAL_H_
#define _VEC_LITERAL_H_

/* This header files provides an abstraction for the various implementations
 * of vector literal construction. The two formats are:
 *
 * 1) Altivec styled using parenthesis
 * 2) C grammer friendly styled using curly braces
 *
 * The macro, VEC_LITERAL has been developed to provide some portability
 * in these two styles. To achieve true portability, user must specify all
 * elements of the vector being initialized. A single element can be provided
 * but only the first element guarenteed across both construction styles.
 *
 * The VEC_SPLAT_* macros have been provided for portability of vector literal
 * construction when all the elements of the vector contain the same value.
 */

#ifdef __SPU__
#include <spu_intrinsics.h>
#endif


#ifdef __ALTIVEC_LITERAL_STYLE__
/* Use altivec style.
 */
#define VEC_LITERAL(_type, ...)	((_type)(__VA_ARGS__))

#define VEC_SPLAT_U8(_val)	((vector unsigned char)(_val))
#define VEC_SPLAT_S8(_val)	((vector signed char)(_val))

#define VEC_SPLAT_U16(_val)	((vector unsigned short)(_val))
#define VEC_SPLAT_S16(_val)	((vector signed short)(_val))

#define VEC_SPLAT_U32(_val)	((vector unsigned int)(_val))
#define VEC_SPLAT_S32(_val)	((vector signed int)(_val))
#define VEC_SPLAT_F32(_val)	((vector float)(_val))

#define VEC_SPLAT_U64(_val)	((vector unsigned long long)(_val))
#define VEC_SPLAT_S64(_val)	((vector signed long long)(_val))
#define VEC_SPLAT_F64(_val)	((vector double)(_val))

#else
/* Use curly brace style.
 */
#define VEC_LITERAL(_type, ...)	((_type){__VA_ARGS__})

#define VEC_SPLAT_U8(_val)	((vector unsigned char){_val, _val, _val, _val, _val, _val, _val, _val, _val, _val, _val, _val, _val, _val, _val, _val})
#define VEC_SPLAT_S8(_val)	((vector signed char){_val, _val, _val, _val, _val, _val, _val, _val, _val, _val, _val, _val, _val, _val, _val, _val})

#define VEC_SPLAT_U16(_val)	((vector unsigned short){_val, _val, _val, _val, _val, _val, _val, _val})
#define VEC_SPLAT_S16(_val)	((vector signed short){_val, _val, _val, _val, _val, _val, _val, _val})

#define VEC_SPLAT_U32(_val)	((vector unsigned int){_val, _val, _val, _val})
#define VEC_SPLAT_S32(_val)	((vector signed int){_val, _val, _val, _val})
#define VEC_SPLAT_F32(_val)	((vector float){_val, _val, _val, _val})

#define VEC_SPLAT_U64(_val)	((vector unsigned long long){_val, _val})
#define VEC_SPLAT_S64(_val)	((vector signed long long){_val, _val})
#define VEC_SPLAT_F64(_val)	((vector double){_val, _val})

#endif

#endif /* _VEC_LITERAL_H_ */
