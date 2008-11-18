/*
 *   SCE CONFIDENTIAL                                      
 *   PLAYSTATION(R)3 Programmer Tool Runtime Library 085.007
 *   Copyright (C) 2005 Sony Computer Entertainment Inc.   
 *   All Rights Reserved.                                  
 */
#include <sys/spu_thread.h>
#include <spu_printf.h>

#include <spu_intrinsics.h>
#include <sys/spu_thread.h>
#include <sys/spu_event.h>
#include <stdint.h>

#define SPE_CACHE_NWAY   		4
#define SPE_CACHE_NSETS 		32
#define SPE_CACHELINE_SIZE 		512
#define SPE_CACHE_SET_TAGID(set) 	16

#define USE_SOFTWARE_CACHE 1
#ifdef USE_SOFTWARE_CACHE

#include "cache/include/spe_cache.h"

void *	spe_readcache(unsigned int ea)
{
	
	int set, idx, line, byte;					
    _spe_cache_nway_lookup_(ea, set, idx);			
								
    if (unlikely(idx < 0)) {					
	idx = _spe_cache_miss_(ea, set, -1);			
        spu_writech(22, SPE_CACHE_SET_TAGMASK(set));		
        spu_mfcstat(MFC_TAG_UPDATE_ALL);			
    } 								
    line = _spe_cacheline_num_(set, idx);			
    byte = _spe_cacheline_byte_offset_(ea);			
    return (void *) &spe_cache_mem[line + byte];
}
#endif //USE_SOFTWARE_CACHE

int main(int spu_num,uint64_t mainmemPtr)
{
	int memPtr = (int) mainmemPtr;


#define MAX_BUF 256
	char spuBuffer[MAX_BUF];
	spuBuffer[0] = 0;
	
	char* result,*result2; //= spe_cache_rd(mainmemPtr);
	
#ifdef USE_SOFTWARE_CACHE

	//this is a brute-force sample.
	//you can use the software cache more efficient using __spe_cache_rd_x4 to read 4 elements at a time

	int i=0;
	do
	{
		result = spe_readcache(mainmemPtr+i);
		//spe_readcache is the expanded version of spe_cache_rd MACRO

		spuBuffer[i] = result[0];
		i++;
	} while (result[0] && (i<MAX_BUF)); //assume that the buffer ends with [0] in main memory
	

	//result = _spe_cache_lookup_xfer_wait_(mainmemPtr, 0, 1);

	result = spe_readcache(mainmemPtr);
	result2 = spe_readcache(mainmemPtr);
#endif //USE_SOFTWARE_CACHE
	
	if (i>= MAX_BUF)
	{
		spu_printf("spe_readcache buffer overflow. is the buffer 0-terminated?\n");
	}
	spu_printf("spe_cache_rd(%x)  =  %s\n", memPtr,spuBuffer);
	
	sys_spu_thread_exit(0);
}

