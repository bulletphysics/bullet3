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
/* api.h
 *
 * Copyright (C) 2005 IBM Corp.
 *
 * Simple API for software managed cache on SPEs.
 * A sophisticated application would not use these,
 * but rather use the low-level lookup functions.
 */

#ifndef __SPE_CACHE_API_H__
#define __SPE_CACHE_API_H__

typedef void *spe_cache_entry_t;

#define spe_cache_rd(ea)		_spe_cache_lookup_xfer_wait_(ea, 0, 1)
#define spe_cache_tr(ea)		_spe_cache_lookup_xfer_(ea, 0, 1)
#define spe_cache_lr(ea)		_spe_cache_lookup_(ea, 0)

#define spe_cache_wait(entry)		_spe_cache_wait_(entry)

#endif
