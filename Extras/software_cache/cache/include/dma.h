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
/* dma.h
 *
 * Copyright (C) 2005 IBM Corp.
 *
 * Internal DMA utilities for software 
 * managed cache.
 */

#ifndef __SPE_CACHE_DMA_H__
#define __SPE_CACHE_DMA_H__

#define SPE_CACHE_TAGID_SHIFT	(SPE_CACHELINE_SHIFT + SPE_CACHE_NWAY_SHIFT)

#define _SPE_CACHELINE_TAGID(_ptr)      (16)
#define _SPE_CACHELINE_TAGMASK(_ptr)    (1 << 16)

#define SPE_CACHELINE_TAGID(_line) \
        _SPE_CACHELINE_TAGID(&spe_cache_mem[_line])
#define SPE_CACHELINE_TAGMASK(_line) \
        _SPE_CACHELINE_TAGMASK(&spe_cache_mem[_line])

#ifndef SPE_CACHE_SET_TAGID
#define SPE_CACHE_SET_TAGID(set) 	((set) & 0x1f)
#endif
#define SPE_CACHE_SET_TAGMASK(set) 	(1 << SPE_CACHE_SET_TAGID(set))

#define SPE_CACHE_PUT	MFC_PUTF_CMD
#define SPE_CACHE_GET	MFC_GET_CMD

#endif
