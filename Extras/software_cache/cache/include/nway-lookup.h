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
/* nway-lookup.h
 *
 * Copyright (C) 2005 IBM Corp.
 *
 * Internal lookup operations for software 
 * managed cache.
 *
 * See nway-opt.h for "optimized" nway
 * lookup operations.
 */

#ifndef __SPE_CACHE_NWAY_LOOKUP_H_
#define __SPE_CACHE_NWAY_LOOKUP_H_


/**
 * _decl_set_entries_ 
 * 	Load up set entries (by 4) from an n-way 
 *	set associative cache.  Mask off the dirty 
 *	bit, as needed.
 */
#define _decl_set_entries_(set, name, index)				\
    vec_uint4 name = *((vec_uint4 *) &spe_cache_dir[set][index])


#define _spe_cache_4_way_lookup_(set, ea)		\
({							\
    _decl_set_entries_(set, e0123, 0);			\
    spu_gather(spu_cmpeq(e0123, ea));			\
})

/**
 * _spe_cache_set_lookup_
 *	Compare 'ea' against all entries of
 *	a set, and return a result that is
 *	consistent with spu_gather().
 */
#define _spe_cache_set_lookup_(set, ea)	\
	_spe_cache_4_way_lookup_(set, ea)


/**
 * _spe_cache_nway_lookup_x4_
 * 	Declare local variables and lookup four addresses 
 *	in the n-way set associative cache.  Upon return, 
 * 	'idx_x4' contains the matching elements in the sets, 
 *	or -1 if not found.
 */
#define _spe_cache_nway_lookup_x4(ea_x4, set_x4, idx_x4)              		\
({                                                              		\
    vector unsigned int ea_aligned_x4 = spu_and ((ea_x4), ~SPE_CACHELINE_MASK); \
    vector unsigned char splat0 = VEC_LITERAL(vector unsigned char, 		\
    					      0x00, 0x01, 0x02, 0x03,		\
			 		      0x00, 0x01, 0x02, 0x03,		\
			 		      0x00, 0x01, 0x02, 0x03,		\
			 		      0x00, 0x01, 0x02, 0x03);		\
    vector unsigned char splat1 = VEC_LITERAL(vector unsigned char,		\
    					      0x04, 0x05, 0x06, 0x07, 		\
			 		      0x04, 0x05, 0x06, 0x07, 		\
			 		      0x04, 0x05, 0x06, 0x07, 		\
			 		      0x04, 0x05, 0x06, 0x07);		\
    vector unsigned char splat2 = VEC_LITERAL(vector unsigned char,		\
    					      0x08, 0x09, 0x0a, 0x0b,		\
			 		      0x08, 0x09, 0x0a, 0x0b, 		\
			 		      0x08, 0x09, 0x0a, 0x0b, 		\
			 		      0x08, 0x09, 0x0a, 0x0b);		\
    vector unsigned char splat3 = VEC_LITERAL(vector unsigned char,		\
    					      0x0c, 0x0d, 0x0e, 0x0f,		\
			 		      0x0c, 0x0d, 0x0e, 0x0f,		\
			 		      0x0c, 0x0d, 0x0e, 0x0f, 		\
			 		      0x0c, 0x0d, 0x0e, 0x0f);		\
    vec_uint4 ea_aligned0 = spu_shuffle(ea_aligned_x4, ea_aligned_x4, splat0);	\
    vec_uint4 ea_aligned1 = spu_shuffle(ea_aligned_x4, ea_aligned_x4, splat1);	\
    vec_uint4 ea_aligned2 = spu_shuffle(ea_aligned_x4, ea_aligned_x4, splat2);	\
    vec_uint4 ea_aligned3 = spu_shuffle(ea_aligned_x4, ea_aligned_x4, splat3);	\
    vec_uint4 found0, found1, found2, found3;					\
    vec_uint4 found_x4;								\
    (set_x4) = _spe_cache_set_num_x4(ea_x4);					\
    found0 = _spe_cache_set_lookup_(spu_extract (set_x4, 0), ea_aligned0);	\
    found1 = _spe_cache_set_lookup_(spu_extract (set_x4, 1), ea_aligned1);	\
    found2 = _spe_cache_set_lookup_(spu_extract (set_x4, 2), ea_aligned2);	\
    found3 = _spe_cache_set_lookup_(spu_extract (set_x4, 3), ea_aligned3);	\
    found_x4 = _pack_vec_uint4 (found0, found1, found2, found3);		\
    (idx_x4) = (vector signed int)_spe_cache_idx_num_x4(found_x4);		\
})

#define _spe_cache_nway_lookup_(ea, set, idx)                   \
({                                                              \
    unsigned int ea_aligned = (ea) & ~SPE_CACHELINE_MASK;	\
    vec_uint4 ea_aligned4 = spu_splats(ea_aligned);		\
    vec_uint4 found;						\
    (set) = _spe_cache_set_num_(ea);				\
    found = _spe_cache_set_lookup_(set, ea_aligned4);		\
    (idx) = _spe_cache_idx_num_(found);				\
})

/**
 * _spe_cache_lookup_
 *	Lookup and return the LSA of an EA 
 *	that is known to be in the cache.
 */
#define _spe_cache_lookup_(ea, is_write) 			\
({                             					\
    int set, idx, line, byte;					\
    _spe_cache_nway_lookup_(ea, set, idx);                      \
								\
    line = _spe_cacheline_num_(set, idx);			\
    byte = _spe_cacheline_byte_offset_(ea);			\
    (void *) &spe_cache_mem[line + byte]; 			\
})

/**
 * _spe_cache_wait_
 *	Wait for transfer of a cache line 
 *	to complete.
 */
#define _spe_cache_wait_(_lsa) 					\
({								\
    spu_writech(22, _SPE_CACHELINE_TAGMASK(_lsa));		\
    spu_mfcstat(MFC_TAG_UPDATE_ALL);				\
})

/**
 * _spe_cache_lookup_wait_
 *	Lookup and return the LSA of an EA
 *	that is known to be in the cache,
 *	and guarantee that its transfer is
 *	complete. 
 */
#define _spe_cache_lookup_wait_(ea, is_write) 			\
({                 						\
    int set, idx, line, byte;					\
    _spe_cache_nway_lookup_(ea, set, idx); 			\
								\
    line = _spe_cacheline_num_(set, idx);			\
    byte = _spe_cacheline_byte_offset_(ea);			\
    spu_writech(22, SPE_CACHE_SET_TAGMASK(set));		\
    spu_mfcstat(MFC_TAG_UPDATE_ALL);				\
    (void *) &spe_cache_mem[line + byte]; 			\
})

/**
 * _spe_cache_lookup_xfer_
 *	Lookup and return the LSA of an EA, where 
 *	the line may either be in the cache or not.  
 *	If not, initiate transfer but do not wait 
 *	for completion.
 */
#define _spe_cache_lookup_xfer_(ea, is_write, rb) 		\
({         							\
    int set, idx, line, byte;					\
    _spe_cache_nway_lookup_(ea, set, idx); 			\
								\
    if (unlikely(idx < 0)) {                     		\
        idx = _spe_cache_miss_(ea, set, -1);			\
    }                                                           \
    line = _spe_cacheline_num_(set, idx);			\
    byte = _spe_cacheline_byte_offset_(ea);			\
    (void *) &spe_cache_mem[line + byte];                       \
})

/**
 * _spe_cache_lookup_xfer_wait_
 *	Lookup and return the LSA of an EA, where 
 *	the line may either be in the cache or not.  
 *	If not, initiate transfer and guarantee
 *	completion.
 */
#define _spe_cache_lookup_xfer_wait_(ea, is_write, rb)		\
({								\
    int set, idx, line, byte;					\
    _spe_cache_nway_lookup_(ea, set, idx);			\
								\
    if (unlikely(idx < 0)) {					\
	idx = _spe_cache_miss_(ea, set, -1);			\
        spu_writech(22, SPE_CACHE_SET_TAGMASK(set));		\
        spu_mfcstat(MFC_TAG_UPDATE_ALL);			\
    } 								\
    line = _spe_cacheline_num_(set, idx);			\
    byte = _spe_cacheline_byte_offset_(ea);			\
    (void *) &spe_cache_mem[line + byte];			\
})

#endif
