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

#ifndef PXCUDACONTEXTMANAGER_PXCUDAMEMORYMANAGER_H
#define PXCUDACONTEXTMANAGER_PXCUDAMEMORYMANAGER_H

#include "foundation/PxPreprocessor.h"

#if PX_SUPPORT_GPU_PHYSX

#include "task/PxTaskDefine.h"

// some macros to keep the source code more readable
#define PX_ALLOC_INFO(name, ID) __FILE__, __LINE__, name, physx::PxAllocId::ID
#define PX_ALLOC_INFO_PARAMS_DECL(p0, p1, p2, p3)  const char* file = p0, int line = p1, const char* allocName = p2, physx::PxAllocId::Enum allocId = physx::PxAllocId::p3
#define PX_ALLOC_INFO_PARAMS_DEF()  const char* file, int line, const char* allocName, physx::PxAllocId::Enum allocId
#define PX_ALLOC_INFO_PARAMS_INPUT()  file, line, allocName, allocId
#define PX_ALLOC_INFO_PARAMS_INPUT_INFO(info) info.getFileName(), info.getLine(), info.getAllocName(), info.getAllocId()

#ifndef NULL // don't want to include <string.h>
#define NULL 0
#endif

namespace physx
{

PX_PUSH_PACK_DEFAULT

/** \brief ID of the Feature which owns/allocated memory from the heap
 *
 * Maximum of 64k IDs allowed.
 */
struct PxAllocId
{
    /**
     * \brief ID of the Feature which owns/allocated memory from the heap
     */
	enum Enum
	{
		UNASSIGNED,		//!< default
		APEX,			//!< APEX stuff not further classified
		PARTICLES,		//!< all particle related
		GPU_UTIL,		//!< e.g. RadixSort (used in SPH and deformable self collision)
		CLOTH,			//!< all cloth related
		NUM_IDS			//!< number of IDs, be aware that ApexHeapStats contains PxAllocIdStats[NUM_IDS]
	};
};

/// \brief memory type managed by a heap
struct PxCudaBufferMemorySpace
{
    /**
     * \brief memory type managed by a heap
     */
	enum Enum
	{
		T_GPU,
		T_PINNED_HOST,
		T_WRITE_COMBINED,
		T_HOST,
		COUNT
	};
};

/// \brief class to track allocation statistics, see PxgMirrored
class PxAllocInfo
{
public:
    /**
     * \brief AllocInfo default constructor
     */
	PxAllocInfo() {}

    /**
     * \brief AllocInfo constructor that initializes all of the members
     */
	PxAllocInfo(const char* file, int line, const char* allocName, PxAllocId::Enum allocId)
		: mFileName(file)
		, mLine(line)
		, mAllocName(allocName)
		, mAllocId(allocId)
	{}

	/// \brief get the allocation file name
	inline	const char*			getFileName() const
	{
		return mFileName;
	}

	/// \brief get the allocation line
	inline	int					getLine() const
	{
		return mLine;
	}

	/// \brief get the allocation name
	inline	const char*			getAllocName() const
	{
		return mAllocName;
	}

	/// \brief get the allocation ID
	inline	PxAllocId::Enum		getAllocId() const
	{
		return mAllocId;
	}

private:
	const char*			mFileName;
	int					mLine;
	const char*			mAllocName;
	PxAllocId::Enum		mAllocId;
};

/// \brief statistics collected per AllocationId by HeapManager.
struct PxAllocIdStats
{
	size_t size;		//!< currently allocated memory by this ID
	size_t maxSize;		//!< max allocated memory by this ID
	size_t elements;	//!< number of current allocations by this ID
	size_t maxElements;	//!< max number of allocations by this ID
};

class PxCudaMemoryManager;
typedef size_t PxCudaBufferPtr;

/// \brief Hint flag to tell how the buffer will be used
struct PxCudaBufferFlags
{
/// \brief Enumerations for the hint flag to tell how the buffer will be used
	enum Enum
	{
		F_READ			= (1 << 0),
		F_WRITE			= (1 << 1),
		F_READ_WRITE	= F_READ | F_WRITE
	};
};


/// \brief Memory statistics struct returned by CudaMemMgr::getStats()
struct PxCudaMemoryManagerStats
{

	size_t			heapSize;		//!< Size of all pages allocated for this memory type (allocated + free).
	size_t			totalAllocated; //!< Size occupied by the current allocations.
	size_t			maxAllocated;	//!< High water mark of allocations since the SDK was created.
	PxAllocIdStats	allocIdStats[PxAllocId::NUM_IDS]; //!< Stats for each allocation ID, see PxAllocIdStats
};


/// \brief Buffer type: made of hint flags and the memory space (Device Memory, Pinned Host Memory, ...)
struct PxCudaBufferType
{
	/// \brief PxCudaBufferType copy constructor
	PX_INLINE PxCudaBufferType(const PxCudaBufferType& t)
		: memorySpace(t.memorySpace)
		, flags(t.flags)
	{}
	
	/// \brief PxCudaBufferType constructor to explicitely assign members
	PX_INLINE PxCudaBufferType(PxCudaBufferMemorySpace::Enum _memSpace, PxCudaBufferFlags::Enum _flags)
		: memorySpace(_memSpace)
		, flags(_flags)
	{}

	PxCudaBufferMemorySpace::Enum	memorySpace; 	//!< specifies which memory space for the buffer 
	PxCudaBufferFlags::Enum			flags;			//!< specifies the usage flags for the buffer
};


/// \brief Buffer which keeps informations about allocated piece of memory.
class PxCudaBuffer
{
public:
	/// Retrieves the manager over which the buffer was allocated.
	virtual	PxCudaMemoryManager*			getCudaMemoryManager() const = 0;

	/// Releases the buffer and the memory it used, returns true if successful.
	virtual	bool						free() = 0;

	/// Realloc memory. Use to shrink or resize the allocated chunk of memory of this buffer.
	/// Returns true if successful. Fails if the operation would change the address and need a memcopy.
	/// In that case the user has to allocate, copy and free the memory with separate steps.
	/// Realloc to size 0 always returns false and doesn't change the state.
	virtual	bool						realloc(size_t size, PX_ALLOC_INFO_PARAMS_DECL(NULL, 0, NULL, UNASSIGNED)) = 0;

	/// Returns the type of the allocated memory.
	virtual	const PxCudaBufferType&		getType() const = 0;

	/// Returns the pointer to the allocated memory.
	virtual	PxCudaBufferPtr				getPtr() const = 0;

	/// Returns the size of the allocated memory.
	virtual	size_t						getSize() const = 0;

protected:
    /// \brief protected destructor
    virtual ~PxCudaBuffer() {}
};


/// \brief Allocator class for different kinds of CUDA related memory.
class PxCudaMemoryManager
{
public:
	/// Allocate memory of given type and size. Returns a CudaBuffer if successful. Returns NULL if failed.
	virtual PxCudaBuffer*				alloc(const PxCudaBufferType& type, size_t size, PX_ALLOC_INFO_PARAMS_DECL(NULL, 0, NULL, UNASSIGNED)) = 0;

	/// Basic heap allocator without PxCudaBuffer
	virtual PxCudaBufferPtr				alloc(PxCudaBufferMemorySpace::Enum memorySpace, size_t size, PX_ALLOC_INFO_PARAMS_DECL(NULL, 0, NULL, UNASSIGNED)) = 0;

	/// Basic heap deallocator without PxCudaBuffer
	virtual bool						free(PxCudaBufferMemorySpace::Enum memorySpace, PxCudaBufferPtr addr) = 0;

	/// Basic heap realloc without PxCudaBuffer
	virtual bool						realloc(PxCudaBufferMemorySpace::Enum memorySpace, PxCudaBufferPtr addr, size_t size, PX_ALLOC_INFO_PARAMS_DECL(NULL, 0, NULL, UNASSIGNED)) = 0;

	/// Retrieve stats for the memory of given type. See PxCudaMemoryManagerStats.
	virtual void						getStats(const PxCudaBufferType& type, PxCudaMemoryManagerStats& outStats) = 0;

	/// Ensure that a given amount of free memory is available. Triggers CUDA allocations in size of (2^n * pageSize) if necessary.
	/// Returns false if page allocations failed.
	virtual bool						reserve(const PxCudaBufferType& type, size_t size) = 0;

	/// Set the page size. The managed memory grows by blocks 2^n * pageSize. Page allocations trigger CUDA driver allocations,
	/// so the page size should be reasonably big. Returns false if input size was invalid, i.e. not power of two.
	/// Default is 2 MB.
	virtual bool						setPageSize(const PxCudaBufferType& type, size_t size) = 0;

	/// Set the upper limit until which pages of a given memory type can be allocated.
	/// Reducing the max when it is already hit does not shrink the memory until it is deallocated by releasing the buffers which own the memory.
	virtual bool						setMaxMemorySize(const PxCudaBufferType& type, size_t size) = 0;

	/// Returns the base size. The base memory block stays persistently allocated over the SDKs life time.
	virtual size_t						getBaseSize(const PxCudaBufferType& type) = 0;

	/// Returns the currently set page size. The memory grows and shrinks in blocks of size (2^n pageSize)
	virtual size_t						getPageSize(const PxCudaBufferType& type) = 0;

	/// Returns the upper limit until which the manager is allowed to allocate additional pages from the CUDA driver.
	virtual size_t						getMaxMemorySize(const PxCudaBufferType& type) = 0;

	/// Get device mapped pinned host mem ptr. Operation only valid for memory space PxCudaBufferMemorySpace::T_PINNED_HOST.
	virtual PxCudaBufferPtr				getMappedPinnedPtr(PxCudaBufferPtr hostPtr) = 0;

protected:
    /// \brief protected destructor
    virtual ~PxCudaMemoryManager() {}
};

PX_POP_PACK


} // end physx namespace

#endif // PX_SUPPORT_GPU_PHYSX
#endif // PXCUDACONTEXTMANAGER_PXCUDAMEMORYMANAGER_H
