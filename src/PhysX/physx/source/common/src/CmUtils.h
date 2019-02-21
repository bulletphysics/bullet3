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
// Copyright (c) 2008-2019 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  


#ifndef PX_PHYSICS_COMMON_UTILS
#define PX_PHYSICS_COMMON_UTILS


#include "foundation/PxVec3.h"
#include "foundation/PxMat33.h"
#include "foundation/PxBounds3.h"
#include "CmPhysXCommon.h"
#include "PxBase.h"
#include "PsInlineArray.h"
#include "PsArray.h"
#include "PsAllocator.h"

namespace physx
{
namespace Cm
{

template<class DstType, class SrcType>
PX_FORCE_INLINE PxU32 getArrayOfPointers(DstType** PX_RESTRICT userBuffer, PxU32 bufferSize, PxU32 startIndex, SrcType*const* PX_RESTRICT src, PxU32 size)
{
	const PxU32 remainder = PxU32(PxMax<PxI32>(PxI32(size - startIndex), 0));
	const PxU32 writeCount = PxMin(remainder, bufferSize);
	src += startIndex;
	for(PxU32 i=0;i<writeCount;i++)
		userBuffer[i] = static_cast<DstType*>(src[i]);
	return writeCount;
}

PX_INLINE void transformInertiaTensor(const PxVec3& invD, const PxMat33& M, PxMat33& mIInv)
{
	const float	axx = invD.x*M(0,0), axy = invD.x*M(1,0), axz = invD.x*M(2,0);
	const float	byx = invD.y*M(0,1), byy = invD.y*M(1,1), byz = invD.y*M(2,1);
	const float	czx = invD.z*M(0,2), czy = invD.z*M(1,2), czz = invD.z*M(2,2);

	mIInv(0,0) = axx*M(0,0) + byx*M(0,1) + czx*M(0,2);
	mIInv(1,1) = axy*M(1,0) + byy*M(1,1) + czy*M(1,2);
	mIInv(2,2) = axz*M(2,0) + byz*M(2,1) + czz*M(2,2);

	mIInv(0,1) = mIInv(1,0)	= axx*M(1,0) + byx*M(1,1) + czx*M(1,2);
	mIInv(0,2) = mIInv(2,0)	= axx*M(2,0) + byx*M(2,1) + czx*M(2,2);
	mIInv(1,2) = mIInv(2,1)	= axy*M(2,0) + byy*M(2,1) + czy*M(2,2);
}

// PT: TODO: refactor this with PxBounds3 header
PX_FORCE_INLINE PxVec3 basisExtent(const PxVec3& basis0, const PxVec3& basis1, const PxVec3& basis2, const PxVec3& extent)
{
	// extended basis vectors
	const PxVec3 c0 = basis0 * extent.x;
	const PxVec3 c1 = basis1 * extent.y;
	const PxVec3 c2 = basis2 * extent.z;

	// find combination of base vectors that produces max. distance for each component = sum of abs()
	return PxVec3 (	PxAbs(c0.x) + PxAbs(c1.x) + PxAbs(c2.x),
					PxAbs(c0.y) + PxAbs(c1.y) + PxAbs(c2.y),
					PxAbs(c0.z) + PxAbs(c1.z) + PxAbs(c2.z));
}

PX_FORCE_INLINE PxBounds3 basisExtent(const PxVec3& center, const PxVec3& basis0, const PxVec3& basis1, const PxVec3& basis2, const PxVec3& extent)
{
	const PxVec3 w = basisExtent(basis0, basis1, basis2, extent);
	return PxBounds3(center - w, center + w);
}

PX_FORCE_INLINE	bool isValid(const PxVec3& c, const PxVec3& e)
{
	return (c.isFinite() && e.isFinite() && (((e.x >= 0.0f) && (e.y >= 0.0f) && (e.z >= 0.0f)) ||
											((e.x == -PX_MAX_BOUNDS_EXTENTS) &&
											(e.y == -PX_MAX_BOUNDS_EXTENTS) &&
											(e.z == -PX_MAX_BOUNDS_EXTENTS))));
}

PX_FORCE_INLINE	bool isEmpty(const PxVec3& c, const PxVec3& e)
{
	PX_UNUSED(c);
	PX_ASSERT(isValid(c, e));
	return e.x<0.0f;
}

// Array with externally managed storage.
// Allocation and resize policy are managed by the owner, 
// Very minimal functionality right now, just POD types

template <typename T, 
		  typename Owner, 
		  typename IndexType,
		  void (Owner::*realloc)(T*& currentMem, IndexType& currentCapacity, IndexType size, IndexType requiredMinCapacity)>
class OwnedArray
{
public:
	OwnedArray() 
	: mData(0)
	, mCapacity(0) 
	, mSize(0)
	{}

	~OwnedArray()		// owner must call releaseMem before destruction
	{
		PX_ASSERT(mCapacity==0);
	}

	void pushBack(T& element, Owner& owner)
	{
		// there's a failure case if here if we push an existing element which causes a resize -
		// a rare case not worth coding around; if you need it, copy the element then push it.

		PX_ASSERT(&element<mData || &element>=mData+mSize);
		if(mSize==mCapacity)
			(owner.*realloc)(mData, mCapacity, mSize, IndexType(mSize+1));

		PX_ASSERT(mData && mSize<mCapacity);
		mData[mSize++] = element;
	}

	IndexType size() const
	{
		return mSize;
	}

	void replaceWithLast(IndexType index)
	{
		PX_ASSERT(index<mSize);
		mData[index] = mData[--mSize];
	}

	T* begin() const
	{
		return mData;
	}

	T* end() const
	{
		return mData+mSize;
	}

	T& operator [](IndexType index)
	{
		PX_ASSERT(index<mSize);
		return mData[index];
	}

	const T& operator [](IndexType index) const
	{
		PX_ASSERT(index<mSize);
		return mData[index];
	}

	void reserve(IndexType capacity, Owner &owner)
	{
		if(capacity>=mCapacity)
			(owner.*realloc)(mData, mCapacity, mSize, capacity);
	}

	void releaseMem(Owner &owner)
	{
		mSize = 0;
		(owner.*realloc)(mData, mCapacity, 0, 0);
	}

private:
	T*					mData;
	IndexType			mCapacity;
	IndexType			mSize;

	// just in case someone tries to use a non-POD in here
	union FailIfNonPod
	{
		T t;
		int x;
	};
};

/**
Any object deriving from PxBase needs to call this function instead of 'delete object;'. 

We don't want implement 'operator delete' in PxBase because that would impose how
memory of derived classes is allocated. Even though most or all of the time derived classes will 
be user allocated, we don't want to put UserAllocatable into the API and derive from that.
*/
template<typename T>
PX_INLINE void deletePxBase(T* object)
{
	if(object->getBaseFlags() & PxBaseFlag::eOWNS_MEMORY)
		PX_DELETE(object);
	else
		object->~T();
}

#if PX_CHECKED
/**
Mark a specified amount of memory with 0xcd pattern. This is used to check that the meta data 
definition for serialized classes is complete in checked builds.
*/
PX_INLINE void markSerializedMem(void* ptr, PxU32 byteSize)
{
	for (PxU32 i = 0; i < byteSize; ++i)
      reinterpret_cast<PxU8*>(ptr)[i] = 0xcd;
}

/**
Macro to instantiate a type for serialization testing. 
Note: Only use PX_NEW_SERIALIZED once in a scope.
*/
#define PX_NEW_SERIALIZED(v,T) 															        \
    void* _buf = physx::shdfnd::ReflectionAllocator<T>().allocate(sizeof(T),__FILE__,__LINE__);  \
	Cm::markSerializedMem(_buf, sizeof(T));                                                      \
    v = PX_PLACEMENT_NEW(_buf, T)

#else
PX_INLINE void markSerializedMem(void*, PxU32){}

#define PX_NEW_SERIALIZED(v,T)  v = PX_NEW(T)
#endif

template<typename T, class Alloc>
struct ArrayAccess: public Ps::Array<T, Alloc> 
{
	void store(PxSerializationContext& context) const
	{
		if(this->mData && (this->mSize || this->capacity()))
			context.writeData(this->mData, this->capacity()*sizeof(T));
	}

	void load(PxDeserializationContext& context)
	{
		if(this->mData && (this->mSize || this->capacity()))
			this->mData = context.readExtraData<T>(this->capacity());
	}
};

template<typename T, typename Alloc>
void exportArray(const Ps::Array<T, Alloc>& a, PxSerializationContext& context)
{
	static_cast<const ArrayAccess<T, Alloc>&>(a).store(context);
}

template<typename T, typename Alloc>
void importArray(Ps::Array<T, Alloc>& a, PxDeserializationContext& context)
{
	static_cast<ArrayAccess<T, Alloc>&>(a).load(context);
}

template<typename T, PxU32 N, typename Alloc>
void exportInlineArray(const Ps::InlineArray<T, N, Alloc>& a, PxSerializationContext& context)
{
	if(!a.isInlined())
		Cm::exportArray(a, context);
}

template<typename T, PxU32 N, typename Alloc>
void importInlineArray(Ps::InlineArray<T, N, Alloc>& a, PxDeserializationContext& context)
{
	if(!a.isInlined())
		Cm::importArray(a, context);
}

template<class T>
static PX_INLINE T* reserveContainerMemory(Ps::Array<T>& container, PxU32 nb)
{
	const PxU32 maxNbEntries = container.capacity();
	const PxU32 requiredSize = container.size() + nb;

	if(requiredSize>maxNbEntries)
	{
		const PxU32 naturalGrowthSize = maxNbEntries ? maxNbEntries*2 : 2;
		const PxU32 newSize = PxMax(requiredSize, naturalGrowthSize);
		container.reserve(newSize);
	}

	T* buf = container.end();
	container.forceSize_Unsafe(requiredSize);
	return buf;
}

} // namespace Cm



}

#endif
