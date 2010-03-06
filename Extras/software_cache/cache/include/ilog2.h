/* @(#)12	1.5  src/lib/math/ilog2.h, sw.lib, sdk_pub 10/11/05 15:35:56 */
/* -------------------------------------------------------------- */
/* (C) Copyright 2001,2005,                                       */
/* International Business Machines Corporation,                   */
/* Sony Computer Entertainment Incorporated,                      */
/* Toshiba Corporation.                                           */
/*                                                                */
/* All Rights Reserved.                                           */
/* -------------------------------------------------------------- */
/* PROLOG END TAG zYx                                              */
#ifndef _ILOG2_H_
#define _ILOG2_H_	1

/*
 * FUNCTION
 *	signed int _ilog2(signed int x)
 *
 * DESCRIPTION
 *	_ilog2 computes ceiling of log (base 2) of the input value x.
 *      The input value, x, must be a non-zero positive value.
 */

static __inline signed int _ilog2(signed int x)
{
#ifdef __SPU__
  return (32 - spu_extract(spu_cntlz(spu_promote(x - 1, 0)), 0));
#else
  signed int result;

  for (result=0, x--; x > 0; result++, x>>=1);
  return (result);
#endif
}

#endif /* _ILOG2_H_ */
