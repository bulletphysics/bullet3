/* --------------------------------------------------------------- */
/* PLEASE DO NOT MODIFY THIS SECTION                               */
/* This prolog section is automatically generated.                 */
/*                                                                 */
/* (C) Copyright 2001,2006,                                        */
/* International Business Machines Corporation,                    */
/*                                                                 */
/* All Rights Reserved.                                            */
/* --------------------------------------------------------------- */
/* PROLOG END TAG zYx                                              */
/* spe_cache_defs.h
 *
 * Copyright (C) 2005 IBM Corp.
 *
 * Internal definitions for software managed cache.
 */

#ifndef __SPE_CACHE_DEFS_H__
#define __SPE_CACHE_DEFS_H__

/**
 ** Defn's for number of cache sets.
 ** Default is 64 sets.
 */
#if (SPE_CACHE_NSETS==1024)
#define SPE_CACHE_NSETS_SHIFT	10
#elif (SPE_CACHE_NSETS==512)
#define SPE_CACHE_NSETS_SHIFT	9
#elif (SPE_CACHE_NSETS==256)
#define SPE_CACHE_NSETS_SHIFT	8
#elif (SPE_CACHE_NSETS==128)
#define SPE_CACHE_NSETS_SHIFT	7
#elif (SPE_CACHE_NSETS==64)
#define SPE_CACHE_NSETS_SHIFT	6
#elif (SPE_CACHE_NSETS==32)
#define SPE_CACHE_NSETS_SHIFT	5
#elif (SPE_CACHE_NSETS==16)
#define SPE_CACHE_NSETS_SHIFT	4
#elif (SPE_CACHE_NSETS==8)
#define SPE_CACHE_NSETS_SHIFT	3
#elif (SPE_CACHE_NSETS==4)
#define SPE_CACHE_NSETS_SHIFT	2
#elif (SPE_CACHE_NSETS==2)
#define SPE_CACHE_NSETS_SHIFT	1
#else
#undef SPE_CACHE_NSETS
#define SPE_CACHE_NSETS		64
#define SPE_CACHE_NSETS_SHIFT	6
#endif

/**
 ** Defn's for cachline size (bytes).
 ** Default is 128 bytes.
 */
#if (SPE_CACHELINE_SIZE==512)
#define SPE_CACHELINE_SHIFT	9
#elif (SPE_CACHELINE_SIZE==256)
#define SPE_CACHELINE_SHIFT	8
#elif (SPE_CACHELINE_SIZE==128)
#define SPE_CACHELINE_SHIFT	7
#elif (SPE_CACHELINE_SIZE==64)
#define SPE_CACHELINE_SHIFT	6
#elif (SPE_CACHELINE_SIZE==32)
#define SPE_CACHELINE_SHIFT	5
#else
#undef SPE_CACHELINE_SIZE
#define SPE_CACHELINE_SIZE	128
#define SPE_CACHELINE_SHIFT	7
#endif

/**
 ** Defn's derived from above settings.
 */
#define SPE_CACHE_NSETS_MASK    (SPE_CACHE_NSETS - 1)
#define SPE_CACHELINE_MASK      (SPE_CACHELINE_SIZE - 1)

/**
 ** Defn's for managing cacheline state.
 */
#define SPE_CACHELINE_DIRTY	  0x1
#define SPE_CACHELINE_LOCKED	  0x2
#define SPE_CACHELINE_STATE_MASK  (SPE_CACHELINE_DIRTY | SPE_CACHELINE_LOCKED)

#ifdef _XLC
/**
 * FIXME: For now disable manual branch hints 
 * on XLC due to performance degradation.
 */
#ifndef likely
#define likely(_c) 	(_c)
#define unlikely(_c) 	(_c)
#endif

#else /* !_XLC */

#ifndef likely
#define likely(_c) 	__builtin_expect((_c), 1)
#define unlikely(_c)	__builtin_expect((_c), 0)
#endif
#endif


/**
 ** Debug controls.  Set -DNDEBUG to
 ** disable both panic and assert.
 */
#include <assert.h>
#define _spe_cache_panic_(c)	assert(c)
#ifdef SPE_CACHE_DBG
#define _spe_cache_assert_(c)	assert(c)
#else
#define _spe_cache_assert_(c)	/* No-op. */
#endif

#define _spe_cacheline_byte_offset_(ea)	\
        ((ea) & SPE_CACHELINE_MASK)

#define _spe_cacheline_byte_offset_x4(ea)	\
	spu_and ((ea), SPE_CACHELINE_MASK)

#endif

static __inline vector unsigned int _load_vec_uint4(unsigned int ui1, unsigned int ui2, unsigned int ui3, unsigned int ui4)
{
  vector unsigned int result;
  vector unsigned int iv1, iv2, iv3, iv4;

  vector unsigned char shuffle = VEC_LITERAL(vector unsigned char,
              0x00, 0x01, 0x02, 0x03, 0x10, 0x11, 0x12, 0x13,
              0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80);
  iv1 = spu_promote(ui1, 0);
  iv2 = spu_promote(ui2, 0);
  iv3 = spu_promote(ui3, 0);
  iv4 = spu_promote(ui4, 0);

  result = spu_or(spu_shuffle(iv1, iv2, shuffle), spu_shuffle(iv3, iv4, spu_rlqwbyte(shuffle, 8)));
  return (result);
}

static __inline vector unsigned int _pack_vec_uint4(vector unsigned int ui1, vector unsigned int ui2, vector unsigned int ui3, vector unsigned int ui4)
{
  vector unsigned int result;
  vector unsigned char shuffle = VEC_LITERAL(vector unsigned char,
              0x00, 0x01, 0x02, 0x03, 0x10, 0x11, 0x12, 0x13,
              0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80);

  result = spu_or(spu_shuffle(ui1, ui2, shuffle), spu_shuffle(ui3, ui4, spu_rlqwbyte(shuffle, 8)));
  return (result);
}
