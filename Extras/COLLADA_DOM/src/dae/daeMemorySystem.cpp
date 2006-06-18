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

#include <dae/daeMemorySystem.h>
//#include <malloc.h>

daeRawRef
daeMemorySystem::malloc(daeString pool, size_t n)
{
	(void)pool;
	void *mem = ::malloc(n);
//	memset(mem,0,n);
//	printf("alloc[%s] - %d = 0x%x\n",pool,n,mem);
	return (daeRawRef)mem;
}

void
daeMemorySystem::free(daeString pool, daeRawRef mem)
{
	(void)pool;
//	printf("free[%s] - 0x%x\n",pool,mem);
	::free(mem);
}

