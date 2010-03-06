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
/* nway.h
 *
 * Copyright (C) 2005 IBM Corp.
 *
 * Support for n-way set associative software
 * managed cache.  The 4-way associative cache
 * is the only interface exposed currently.
 */

#ifndef __SPE_CACHE_NWAY_H_
#define __SPE_CACHE_NWAY_H_

/**
 ** Defn's for n-way set associativity.
 ** Default is 4-way.
 */
#define SPE_CACHE_NWAY          4
#define SPE_CACHE_NWAY_SHIFT    2

#define SPE_CACHE_NWAY_MASK     (SPE_CACHE_NWAY - 1)
#define SPE_CACHE_NENTRIES      (SPE_CACHE_NWAY * SPE_CACHE_NSETS)
#define SPE_CACHE_MEM_SIZE      (SPE_CACHE_NENTRIES * SPE_CACHELINE_SIZE)

#define _spe_cache_set_num_(ea)         				\
({									\
 	unsigned int ead, eadm, ret;					\
 	ead = ((ea) >> SPE_CACHELINE_SHIFT);				\
	eadm = ((ea) >> (SPE_CACHELINE_SHIFT+2));			\
	ret = (ead ^ eadm) & SPE_CACHE_NSETS_MASK;			\
	ret;								\
})

#define _spe_cache_set_num_x4(ea_x4)         				\
({									\
 	vector unsigned int tmp0;					\
 	vector unsigned int tmp1;					\
 	tmp0 = spu_rlmask (ea_x4, -SPE_CACHELINE_SHIFT);		\
 	tmp1 = spu_rlmask (ea_x4, -(SPE_CACHELINE_SHIFT+1));		\
        spu_and (spu_xor (tmp0, tmp1), SPE_CACHE_NSETS_MASK);		\
})

#define _spe_cache_idx_num_x4(found)                                    \
        spu_sub((unsigned int) 31, spu_cntlz(found))

#define _spe_cache_idx_num_(found)                                      \
        spu_extract(spu_sub((unsigned int) 31, spu_cntlz(found)), 0)

#define _spe_cacheline_num_(set, idx)                                   \
        (((set << SPE_CACHE_NWAY_SHIFT) + idx) << SPE_CACHELINE_SHIFT)

#define _spe_cacheline_num_x4(set, idx)                                   \
	spu_sl (spu_add (spu_sl (set, SPE_CACHE_NWAY_SHIFT), idx), SPE_CACHELINE_SHIFT)

#define _spe_cacheline_is_dirty_(set, idx)                              \
        (spe_cache_dir[set][SPE_CACHE_NWAY_MASK-(idx)] & SPE_CACHELINE_DIRTY)

#define _spe_cacheline_is_locked_(set, idx)                             \
        (spe_cache_dir[set][SPE_CACHE_NWAY_MASK-(idx)] & SPE_CACHELINE_LOCKED)

#define _spe_lock_cacheline_(set, idx)                                  \
        spe_cache_dir[set][SPE_CACHE_NWAY_MASK-(idx)] |= SPE_CACHELINE_LOCKED

#define _spe_unlock_cacheline_(set, idx)                                \
        spe_cache_dir[set][SPE_CACHE_NWAY_MASK-(idx)] &= ~SPE_CACHELINE_LOCKED


/**
 * spe_cache_dir
 *      This is the n-way set associative cache
 *      directory.  Entries are either zero (unused)
 *      or non-zero (used).  
 *
 *      State for one additional (dummy) set is
 *      allocated to improve efficiency of cache
 *      line locking.
 *	volatile seems not to be necessary here, the SCE toolchain guarantees a barrier after dma transfer
 */
static unsigned int spe_cache_dir[SPE_CACHE_NSETS+1][SPE_CACHE_NWAY]
    __attribute__ ((aligned(16)));

/**
 * spe_cache_mem
 *      A contiguous set of cachelines in LS memory,
 *      one line for each entry in the cache.
 *	volatile seems not to be necessary here, the SCE toolchain guarantees a barrier after dma transfer
 */
static char spe_cache_mem[SPE_CACHE_MEM_SIZE]
    __attribute__ ((aligned(128)));

#include "nway-lookup.h"
#include "nway-replace.h"
#include "nway-miss.h"
#include "nway-opt.h"

#endif 
