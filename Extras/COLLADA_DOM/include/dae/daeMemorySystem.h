/*
 * Copyright 2006 Sony Computer Entertainment Inc.
 *
 * Licensed under the SCEA Shared Source License, Version 1.0 (the "License"); you may not use this 
 * file except in compliance with the License. You may obtain a copy of the License at:
 * http://research.scea.com/scea_shared_source_license.html
 *
 * Unless required by applicable law or agreed to in writing, software distributed under the License 
 * is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or 
 * implied. See the License for the specific language governing permissions and limitations under the 
 * License. 
 */

#ifndef __DAE_MEMORY_SYSTEM_H__
#define __DAE_MEMORY_SYSTEM_H__

#include <dae/daeTypes.h>

/**
 * The @c daeMemorySystem class is a simple wrapper for memory operations.
 * Every allocation passes a string pool name such that
 * in the future different pools can be used based on allocation type.
 * Currently the system is just a pass-through to system @c malloc.
 */
class daeMemorySystem
{
public:
	/**
	 * Provides a wrapper malloc with pool field.
	 * @param pool String name of the pool to use for this allocation.
	 * @param n Number of bytes to allocate.
	 * @return Returns the memory allocated if successful, or NULL if not.
	 */
	static daeRawRef malloc(daeString pool, size_t n);

	/**
	 * Provides a wrapper free with pool argument.
	 * @param pool Pool the memory should be freed from.
	 * @param mem Memory to be freed.
	 */
	static void free(daeString pool, daeRawRef mem);
};

// Shorthand for defining new and delete overrides for classes, bad use of macros!

#define DAE_ALLOC \
	inline void* operator new(size_t n) { \
		return daeMemorySystem::malloc("meta",n); \
	} \
	inline void* operator new(size_t , size_t runtimeSize) { \
		return daeMemorySystem::malloc("meta",runtimeSize); \
	} \
	inline void operator delete(void* p) { \
		daeMemorySystem::free("meta",p); \
	} \
	inline void operator delete(void* p, size_t runtimeSize) { \
	(void)runtimeSize; \
	daeMemorySystem::free("meta",p); \
	}



#endif // __DAE_MEMORY_H__
