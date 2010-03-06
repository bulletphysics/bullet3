/* @(#)85	1.4  src/lib/c/memset.h, sw.lib, sdk_pub 10/13/05 10:17:09 */
/* -------------------------------------------------------------- */
/* (C) Copyright 2001,2005,                                       */
/* International Business Machines Corporation,                   */
/* Sony Computer Entertainment Incorporated,                      */
/* Toshiba Corporation.                                           */
/*                                                                */
/* All Rights Reserved.                                           */
/* -------------------------------------------------------------- */
/* PROLOG END TAG zYx                                              */
#include <spu_intrinsics.h>
#include <stddef.h>

/* Fills the first n bytes of the memory area pointed to by s 
 * with the constant byte c. Returns a pointer to the memory area s.
 */
static __inline void * _memset(void *s, int c, size_t n)
{
  int skip, cnt, i;
  vec_uchar16 *vs;
  vec_uchar16 vc, mask;

  vs = (vec_uchar16 *)(s);
  vc = spu_splats((unsigned char)c);
  cnt = (int)(n);

  /* Handle any leading partial quadwords as well a 
   * very short settings (ie, such that the n characters
   * all reside in a single quadword.
   */
  skip = (int)(s) & 15;
  if (skip) {
    mask = spu_rlmaskqwbyte((vec_uchar16)(-1), 0-skip);
    cnt -= 16 - skip;
    if (cnt < 0) {
      mask = spu_and(mask, spu_slqwbyte((vec_uchar16)(-1), (unsigned int)(-cnt)));
    }
    *vs = spu_sel(*vs, vc, mask);
    vs++;
  }

  /* Handle 8 quadwords at a time
   */
  for (i=127; i<cnt; cnt-=8*16) {
    vs[0] = vc;
    vs[1] = vc;
    vs[2] = vc;
    vs[3] = vc;
    vs[4] = vc;
    vs[5] = vc;
    vs[6] = vc;
    vs[7] = vc;
    vs += 8;
  }

  /* Finish all remaining complete quadwords
   */
  for (i=15; i<cnt; cnt-=16) *vs++ = vc;

  /* Handle any trailing partial quadwords
   */
  if (cnt > 0) {
    mask = spu_slqwbyte((vec_uchar16)(-1), (unsigned int)(16-cnt));
    *vs = spu_sel(*vs, vc, mask);
  }
  
  return (s);
}
