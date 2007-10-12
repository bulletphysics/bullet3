/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "btAlignedAllocator.h"


#if defined (BT_HAS_ALIGNED_ALLOCATOR)

#include <malloc.h>
void*	btAlignedAlloc	(size_t size, int alignment)
{
	void* ptr = _aligned_malloc(size,alignment);
//	printf("btAlignedAlloc %d, %x\n",size,ptr);
	return ptr;
}

void	btAlignedFree	(void* ptr)
{
//	printf("btAlignedFree %x\n",ptr);
	_aligned_free(ptr);
}

#else

#ifdef __CELLOS_LV2__

#include <stdlib.h>

int numAllocs = 0;
int numFree = 0;

void*	btAlignedAlloc	(size_t size, int alignment)
{
	numAllocs++;
	return memalign(alignment, size);
}

void	btAlignedFree	(void* ptr)
{
	numFree++;
	free(ptr);
}

#else

void*	btAlignedAlloc	(size_t size, int alignment)
{
 void *ret;
  char *real;
  unsigned long offset;
  
  real = (char *)malloc(size + sizeof(void *) + (alignment-1));
  if (real) {
    offset = (alignment - (unsigned long)(real + sizeof(void *))) & (alignment-1);
    ret = (void *)((real + sizeof(void *)) + offset);
    *((void **)(ret)-1) = (void *)(real);
  } else {
    ret = (void *)(real);
  }
  return (ret);
}

void	btAlignedFree	(void* ptr)
{

 void* real;

  if (ptr) {
    real = *((void **)(ptr)-1);
    free(real);
  }
}
#endif //

#endif


