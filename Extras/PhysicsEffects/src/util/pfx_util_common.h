/*
Physics Effects Copyright(C) 2010 Sony Computer Entertainment Inc.
All rights reserved.

Physics Effects is open software; you can redistribute it and/or
modify it under the terms of the BSD License.

Physics Effects is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the BSD License for more details.

A copy of the BSD License is distributed with
Physics Effects under the filename: physics_effects_license.txt
*/

#ifndef _SCE_PFX_UTIL_COMMON_H
#define _SCE_PFX_UTIL_COMMON_H

#include <string.h>

#ifdef _WIN32
	#define SCE_PFX_UTIL_ALLOC(align,size) _aligned_malloc(size,align)
	#define SCE_PFX_UTIL_REALLOC(ptr,align,size) _aligned_realloc(ptr,size,align)
	#define SCE_PFX_UTIL_FREE(ptr) if(ptr) {_aligned_free(ptr);ptr=NULL;}
#else
	#define SCE_PFX_UTIL_ALLOC(align,size) malloc(size)
        #define SCE_PFX_UTIL_REALLOC(ptr,align,size) realloc(ptr,size)
        #define SCE_PFX_UTIL_FREE(ptr) if(ptr) {free(ptr);ptr=NULL;}
#endif

#endif // _SCE_PFX_UTIL_COMMON_H
