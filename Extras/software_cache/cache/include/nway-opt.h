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
/* nway-opt.h
 *
 * Copyright (C) 2006 IBM Corp.
 *
 * "Optimized" lookup operations for n-way set associative
 * software managed cache.
 */
#include <spu_intrinsics.h>

#ifndef __SPE_CACHE_NWAY_OPT_H_
#define __SPE_CACHE_NWAY_OPT_H_

/* __spe_cache_rd
 *      Look up and return data from the cache.  If the data
 *      is not currently in cache then transfer it from main
 *      storage.
 *
 *      This code uses a conditional branch to the cache miss
 *      handler in the event that the requested data is not
 *      in the cache.  A branch hint is used to avoid paying
 *      the branch stall penalty.
 */
#define __spe_cache_rd(type, ea)				\
({								\
    int set, idx, lnum, byte;					\
    type ret;							\
    _spe_cache_nway_lookup_(ea, set, idx);			\
	if (unlikely(idx < 0)) {					\
		idx = _spe_cache_miss_(ea, set, -1);			\
        spu_writech(22, SPE_CACHE_SET_TAGMASK(set));		\
        spu_mfcstat(MFC_TAG_UPDATE_ALL);			\
    } 								\
    lnum = _spe_cacheline_num_(set, idx);			\
    byte = _spe_cacheline_byte_offset_(ea);			\
    ret = *((type *) (&spe_cache_mem[lnum + byte])); \
	ret;							\
})

/**
 * __spe_cache_rd_x4
 *      Fetch four data elements from the cache.
 *
 *	This code uses one conditional branch in 
 *	the event that any of the four elements
 *	are missing.
 *
 *	On a miss, light weight locking is used to 
 *	avoid casting out entries that were found.
 *	Further, we wait just once for the transfers,
 *	allowing for parallel [rather than serial]
 *	transfers.
 */

#define __spe_cache_rd_x4(type, ea_x4)				\
({								\
    vector unsigned int missing;				\
    unsigned int ms;						\
    vector unsigned int cindex;					\
    unsigned int d0, d1, d2, d3;				\
    vector unsigned int s_x4;					\
    vector signed int i_x4;					\
    vector unsigned int ibyte, iline;				\
    vector unsigned int ret;					\
    unsigned int idx0, idx1, idx2, idx3;			\
								\
    _spe_cache_nway_lookup_x4(ea_x4, s_x4, i_x4);		\
    missing = spu_rlmask ((vector unsigned int)i_x4, -8);	\
    ms = spu_extract (spu_gather (missing), 0);			\
								\
    ibyte = _spe_cacheline_byte_offset_x4(ea_x4);		\
								\
    iline = _spe_cacheline_num_x4(s_x4, 			\
	    			(vector unsigned int)i_x4);	\
								\
    cindex = spu_add (iline, ibyte);				\
								\
    idx0 = spu_extract (cindex, 0);				\
    idx1 = spu_extract (cindex, 1);				\
    idx2 = spu_extract (cindex, 2);				\
    idx3 = spu_extract (cindex, 3);				\
								\
    d0 = *((type *) (&spe_cache_mem[idx0]));			\
    d1 = *((type *) (&spe_cache_mem[idx1]));			\
    d2 = *((type *) (&spe_cache_mem[idx2]));			\
    d3 = *((type *) (&spe_cache_mem[idx3]));			\
    								\
    ret = _load_vec_uint4 (d0, d1, d2, d3);			\
								\
    if (unlikely(ms)) { 					\
	int b0 = spu_extract (ibyte, 0);			\
	int b1 = spu_extract (ibyte, 1);			\
	int b2 = spu_extract (ibyte, 2);			\
	int b3 = spu_extract (ibyte, 3);			\
	int lnum0;						\
	int lnum1;						\
	int lnum2;						\
	int lnum3;						\
	int s0 = spu_extract (s_x4, 0);				\
	int s1 = spu_extract (s_x4, 1);				\
	int s2 = spu_extract (s_x4, 2);				\
	int s3 = spu_extract (s_x4, 3);				\
	int i0 = spu_extract (i_x4, 0);				\
	int i1 = spu_extract (i_x4, 1);				\
	int i2 = spu_extract (i_x4, 2);				\
	int i3 = spu_extract (i_x4, 3);				\
        unsigned int ea0 = spu_extract(ea_x4, 0);		\
        unsigned int ea1 = spu_extract(ea_x4, 1);		\
        unsigned int ea2 = spu_extract(ea_x4, 2);		\
        unsigned int ea3 = spu_extract(ea_x4, 3);		\
	int avail = -1;						\
								\
 	avail &= ~(((i0 < 0) ? 0 : (1 << i0)) |			\
 		   ((i1 < 0) ? 0 : (1 << i1)) |			\
 		   ((i2 < 0) ? 0 : (1 << i2)) |			\
 		   ((i3 < 0) ? 0 : (1 << i3)));			\
								\
	i0 = _spe_cache_miss_(ea0, s0, avail);			\
	avail &= ~(1 << i0);					\
	i1 = _spe_cache_miss_(ea1, s1, avail);			\
	avail &= ~(1 << i1);					\
	i2 = _spe_cache_miss_(ea2, s2, avail);			\
	avail &= ~(1 << i2);					\
	i3 = _spe_cache_miss_(ea3, s3, avail);			\
								\
	lnum0 = _spe_cacheline_num_(s0, i0);			\
	lnum1 = _spe_cacheline_num_(s1, i1);			\
	lnum2 = _spe_cacheline_num_(s2, i2);			\
	lnum3 = _spe_cacheline_num_(s3, i3);			\
								\
	spu_writech(22, SPE_CACHE_SET_TAGMASK(set));		\
	spu_mfcstat(MFC_TAG_UPDATE_ALL);			\
								\
	d0 = *((type *) (&spe_cache_mem[lnum0 + b0]));		\
	d1 = *((type *) (&spe_cache_mem[lnum1 + b1]));		\
	d2 = *((type *) (&spe_cache_mem[lnum2 + b2]));		\
	d3 = *((type *) (&spe_cache_mem[lnum3 + b3]));		\
								\
        ret = _load_vec_uint4 (d0, d1, d2, d3);			\
    }								\
    ret;							\
})

#endif /* _SPE_CACHE_NWAY_OPT_H_ */
