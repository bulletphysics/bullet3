//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2018 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#include "PsFoundation.h"
#include "PsAllocator.h"
#include "PsHashMap.h"
#include "PsArray.h"
#include "PsMutex.h"

namespace physx
{
namespace shdfnd
{

#if PX_USE_NAMED_ALLOCATOR
namespace
{
typedef HashMap<const NamedAllocator*, const char*, Hash<const NamedAllocator*>, NonTrackingAllocator> AllocNameMap;
PX_INLINE AllocNameMap& getMap()
{
	return getFoundation().getNamedAllocMap();
}
PX_INLINE Foundation::Mutex& getMutex()
{
	return getFoundation().getNamedAllocMutex();
}
}

NamedAllocator::NamedAllocator(const PxEMPTY)
{
	Foundation::Mutex::ScopedLock lock(getMutex());
	getMap().insert(this, 0);
}

NamedAllocator::NamedAllocator(const char* name)
{
	Foundation::Mutex::ScopedLock lock(getMutex());
	getMap().insert(this, name);
}

NamedAllocator::NamedAllocator(const NamedAllocator& other)
{
	Foundation::Mutex::ScopedLock lock(getMutex());
	const AllocNameMap::Entry* e = getMap().find(&other);
	PX_ASSERT(e);
	const char* name = e->second; // The copy is important because insert might invalidate the referenced hash entry
	getMap().insert(this, name);
}

NamedAllocator::~NamedAllocator()
{
	Foundation::Mutex::ScopedLock lock(getMutex());
	bool erased = getMap().erase(this);
	PX_UNUSED(erased);
	PX_ASSERT(erased);
}

NamedAllocator& NamedAllocator::operator=(const NamedAllocator& other)
{
	Foundation::Mutex::ScopedLock lock(getMutex());
	const AllocNameMap::Entry* e = getMap().find(&other);
	PX_ASSERT(e);
	getMap()[this] = e->second;
	return *this;
}

void* NamedAllocator::allocate(size_t size, const char* filename, int line)
{
	if(!size)
		return 0;
	Foundation::Mutex::ScopedLock lock(getMutex());
	const AllocNameMap::Entry* e = getMap().find(this);
	PX_ASSERT(e);
	return getAllocator().allocate(size, e->second, filename, line);
}

void NamedAllocator::deallocate(void* ptr)
{
	if(ptr)
		getAllocator().deallocate(ptr);
}

#endif // PX_DEBUG

void* Allocator::allocate(size_t size, const char* file, int line)
{
	if(!size)
		return 0;
	return getAllocator().allocate(size, "", file, line);
}
void Allocator::deallocate(void* ptr)
{
	if(ptr)
		getAllocator().deallocate(ptr);
}

} // namespace shdfnd
} // namespace physx
