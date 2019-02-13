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

#include "foundation/PxMemory.h"
#include "CctObstacleContext.h"
#include "CctCharacterControllerManager.h"
#include "PsUtilities.h"

using namespace physx;
using namespace Cct;

//! Initial list size
#define DEFAULT_HANDLEMANAGER_SIZE	2

HandleManager::HandleManager() : mCurrentNbObjects(0), mNbFreeIndices(0)
{
	mMaxNbObjects	= DEFAULT_HANDLEMANAGER_SIZE;
	mObjects		= reinterpret_cast<void**>(PX_ALLOC(sizeof(void*)*mMaxNbObjects, "HandleManager"));
	mOutToIn		= reinterpret_cast<PxU16*>(PX_ALLOC(sizeof(PxU16)*mMaxNbObjects, "HandleManager"));
	mInToOut		= reinterpret_cast<PxU16*>(PX_ALLOC(sizeof(PxU16)*mMaxNbObjects, "HandleManager"));
	mStamps			= reinterpret_cast<PxU16*>(PX_ALLOC(sizeof(PxU16)*mMaxNbObjects, "HandleManager"));
	PxMemSet(mOutToIn, 0xff, mMaxNbObjects*sizeof(PxU16));
	PxMemSet(mInToOut, 0xff, mMaxNbObjects*sizeof(PxU16));
	PxMemZero(mStamps, mMaxNbObjects*sizeof(PxU16));
}

HandleManager::~HandleManager()
{
	SetupLists();
}

bool HandleManager::SetupLists(void** objects, PxU16* oti, PxU16* ito, PxU16* stamps)
{
	// Release old data
	PX_FREE_AND_RESET(mStamps);
	PX_FREE_AND_RESET(mInToOut);
	PX_FREE_AND_RESET(mOutToIn);
	PX_FREE_AND_RESET(mObjects);
	// Assign new data
	mObjects	= objects;
	mOutToIn	= oti;
	mInToOut	= ito;
	mStamps		= stamps;
	return true;
}

Handle HandleManager::Add(void* object)
{
	// Are there any free indices I should recycle?
	if(mNbFreeIndices)
	{
		const PxU16 FreeIndex = mInToOut[mCurrentNbObjects];// Get the recycled virtual index
		mObjects[mCurrentNbObjects]	= object;				// The physical location is always at the end of the list (it never has holes).
		mOutToIn[FreeIndex]			= Ps::to16(mCurrentNbObjects++);	// Update virtual-to-physical remapping table.
		mNbFreeIndices--;
		return Handle((mStamps[FreeIndex]<<16)|FreeIndex);			// Return virtual index (handle) to the client app
	}
	else
	{
		PX_ASSERT_WITH_MESSAGE(mCurrentNbObjects<0xffff ,"Internal error - 64K objects in HandleManager!");

		// Is the array large enough for another entry?
		if(mCurrentNbObjects==mMaxNbObjects)
		{
			// Nope! Resize all arrays (could be avoided with linked lists... one day)
			mMaxNbObjects<<=1;													// The more you eat, the more you're given
			if(mMaxNbObjects>0xffff)	mMaxNbObjects = 0xffff;					// Clamp to 64K
			void** NewList		= reinterpret_cast<void**>(PX_ALLOC(sizeof(void*)*mMaxNbObjects, "HandleManager"));		// New physical list
			PxU16* NewOTI		= reinterpret_cast<PxU16*>(PX_ALLOC(sizeof(PxU16)*mMaxNbObjects, "HandleManager"));		// New remapping table
			PxU16* NewITO		= reinterpret_cast<PxU16*>(PX_ALLOC(sizeof(PxU16)*mMaxNbObjects, "HandleManager"));		// New remapping table
			PxU16* NewStamps	= reinterpret_cast<PxU16*>(PX_ALLOC(sizeof(PxU16)*mMaxNbObjects, "HandleManager"));		// New stamps
			PxMemCopy(NewList, mObjects,	mCurrentNbObjects*sizeof(void*));	// Copy old data
			PxMemCopy(NewOTI, mOutToIn,	mCurrentNbObjects*sizeof(PxU16));	// Copy old data
			PxMemCopy(NewITO, mInToOut,	mCurrentNbObjects*sizeof(PxU16));	// Copy old data
			PxMemCopy(NewStamps, mStamps,	mCurrentNbObjects*sizeof(PxU16));	// Copy old data
			PxMemSet(NewOTI+mCurrentNbObjects, 0xff, (mMaxNbObjects-mCurrentNbObjects)*sizeof(PxU16));
			PxMemSet(NewITO+mCurrentNbObjects, 0xff, (mMaxNbObjects-mCurrentNbObjects)*sizeof(PxU16));
			PxMemZero(NewStamps+mCurrentNbObjects, (mMaxNbObjects-mCurrentNbObjects)*sizeof(PxU16));
			SetupLists(NewList, NewOTI, NewITO, NewStamps);
		}

		mObjects[mCurrentNbObjects]	= object;						// Store object at mCurrentNbObjects = physical index = virtual index
		mOutToIn[mCurrentNbObjects]	= Ps::to16(mCurrentNbObjects);	// Update virtual-to-physical remapping table.
		mInToOut[mCurrentNbObjects]	= Ps::to16(mCurrentNbObjects);	// Update physical-to-virtual remapping table.
		PxU32 tmp = mCurrentNbObjects++;
		return (mStamps[tmp]<<16)|tmp;								// Return virtual index (handle) to the client app
	}
}

void HandleManager::Remove(Handle handle)
{
	const PxU16 VirtualIndex = PxU16(handle);
	if(VirtualIndex>=mMaxNbObjects)		return;			// Invalid handle
	const PxU16 PhysicalIndex = mOutToIn[VirtualIndex];	// Get the physical index
	if(PhysicalIndex==0xffff)			return;			// Value has already been deleted
	if(PhysicalIndex>=mMaxNbObjects)	return;			// Invalid index

	// There must be at least one valid entry.
	if(mCurrentNbObjects)
	{
		if(mStamps[VirtualIndex]!=handle>>16)	return;							// Stamp mismatch => index has been recycled

		// Update list so that there's no hole
		mObjects[PhysicalIndex]					= mObjects[--mCurrentNbObjects];// Move the real object so that the array has no holes.
		mOutToIn[mInToOut[mCurrentNbObjects]]	= PhysicalIndex;				// Update virtual-to-physical remapping table.
		mInToOut[PhysicalIndex]					= mInToOut[mCurrentNbObjects];	// Update physical-to-virtual remapping table.
		// Keep track of the recyclable virtual index
		mInToOut[mCurrentNbObjects]				= VirtualIndex;					// Store the free virtual index/handle at the end of mInToOut
		mOutToIn[VirtualIndex]					= 0xffff;						// Invalidate the entry
		mNbFreeIndices++;														// One more free index
		mStamps[VirtualIndex]++;												// Update stamp
	}
}

void* HandleManager::GetObject(Handle handle) const
{
	const PxU16 VirtualIndex = PxU16(handle);
	if(VirtualIndex>=mMaxNbObjects)			return NULL;	// Invalid handle
	const PxU16 PhysicalIndex = mOutToIn[VirtualIndex];		// Get physical index
	if(PhysicalIndex==0xffff)				return NULL;	// Object has been deleted
	if(PhysicalIndex>=mMaxNbObjects)		return NULL;	// Index is invalid
	if(mStamps[VirtualIndex]!=handle>>16)	return NULL;	// Index has been recycled
	return mObjects[PhysicalIndex];							// Returns stored pointer
}

bool HandleManager::UpdateObject(Handle handle, void* object)
{
	const PxU16 VirtualIndex = PxU16(handle);
	if(VirtualIndex>=mMaxNbObjects)			return false;	// Invalid handle
	const PxU16 PhysicalIndex = mOutToIn[VirtualIndex];		// Get physical index
	if(PhysicalIndex==0xffff)				return false;	// Object has been deleted
	if(PhysicalIndex>=mMaxNbObjects)		return false;	// Index is invalid
	if(mStamps[VirtualIndex]!=handle>>16)	return false;	// Index has been recycled
	mObjects[PhysicalIndex] = object;						// Updates stored pointer
	return true;
}


// PT: please do not expose these functions outside of this class,
// as we want to retain the ability to easily modify the handle format if necessary.

#define NEW_ENCODING
#ifdef NEW_ENCODING
static PX_FORCE_INLINE void* encodeInternalHandle(PxU32 index, PxGeometryType::Enum type)
{
	PX_ASSERT(index<=0xffff);
	PX_ASSERT(PxU32(type)<=0xffff);
	// PT: we do type+1 to ban internal handles with a 0 value, since it's reserved for NULL pointers
	return reinterpret_cast<void*>((size_t(index)<<16)|(size_t(type)+1));
}

static PX_FORCE_INLINE PxGeometryType::Enum decodeInternalType(void* handle)
{
	return PxGeometryType::Enum((PxU32(reinterpret_cast<size_t>(handle)) & 0xffff)-1);
}

static PX_FORCE_INLINE PxU32 decodeInternalIndex(void* handle)
{
	return (PxU32(size_t(handle)))>>16;
}
#else
static PX_FORCE_INLINE ObstacleHandle encodeHandle(PxU32 index, PxGeometryType::Enum type)
{
	PX_ASSERT(index<=0xffff);
	PX_ASSERT(type<=0xffff);
	return (PxU16(index)<<16)|PxU32(type);
}

static PX_FORCE_INLINE PxGeometryType::Enum decodeType(ObstacleHandle handle)
{
	return PxGeometryType::Enum(handle & 0xffff);
}

static PX_FORCE_INLINE PxU32 decodeIndex(ObstacleHandle handle)
{
	return handle>>16;
}
#endif

ObstacleContext::ObstacleContext(CharacterControllerManager& cctMan)
	: mCCTManager(cctMan)
{
}

ObstacleContext::~ObstacleContext()
{
}

void ObstacleContext::release()
{
	mCCTManager.releaseObstacleContext(*this);
}

PxControllerManager& ObstacleContext::getControllerManager() const
{
	return mCCTManager;
}

ObstacleHandle ObstacleContext::addObstacle(const PxObstacle& obstacle)
{
	const PxGeometryType::Enum type = obstacle.getType();
	if(type==PxGeometryType::eBOX)
	{
		const PxU32 index = mBoxObstacles.size();
#ifdef NEW_ENCODING
		const ObstacleHandle handle = mHandleManager.Add(encodeInternalHandle(index, type));
#else
		const ObstacleHandle handle = encodeHandle(index, type);
#endif
		mBoxObstacles.pushBack(InternalBoxObstacle(handle, static_cast<const PxBoxObstacle&>(obstacle)));
		mCCTManager.onObstacleAdded(handle, this);
		return handle;
	}
	else if(type==PxGeometryType::eCAPSULE)
	{
		const PxU32 index = mCapsuleObstacles.size();
#ifdef NEW_ENCODING
		const ObstacleHandle handle = mHandleManager.Add(encodeInternalHandle(index, type));
#else
		const ObstacleHandle handle = encodeHandle(index, type);
#endif
		mCapsuleObstacles.pushBack(InternalCapsuleObstacle(handle, static_cast<const PxCapsuleObstacle&>(obstacle)));
		mCCTManager.onObstacleAdded(handle, this);
		return handle;
	}
	else return INVALID_OBSTACLE_HANDLE;
}

#ifdef NEW_ENCODING
template<class T>
static PX_FORCE_INLINE void remove(HandleManager& manager, void* object, ObstacleHandle handle, PxU32 index, PxU32 size, const Ps::Array<T>& obstacles)
{
	manager.Remove(handle);
	if(index!=size-1)
	{
		bool status = manager.UpdateObject(obstacles[size-1].mHandle, object);
		PX_ASSERT(status);
		PX_UNUSED(status);
	}
}
#endif

bool ObstacleContext::removeObstacle(ObstacleHandle handle)
{
#ifdef NEW_ENCODING
	void* object = mHandleManager.GetObject(handle);
	if(!object)
		return false;
	const PxGeometryType::Enum type = decodeInternalType(object);
	const PxU32 index = decodeInternalIndex(object);
#else
	const PxGeometryType::Enum type = decodeType(handle);
	const PxU32 index = decodeIndex(handle);
#endif

	if(type==PxGeometryType::eBOX)
	{
		const PxU32 size = mBoxObstacles.size();
		PX_ASSERT(index<size);
		if(index>=size)
			return false;

#ifdef NEW_ENCODING
		remove<InternalBoxObstacle>(mHandleManager, object, handle, index, size, mBoxObstacles);
#endif
		mBoxObstacles.replaceWithLast(index);
#ifdef NEW_ENCODING
		mCCTManager.onObstacleRemoved(handle);
#else
		mCCTManager.onObstacleRemoved(handle, encodeHandle(size-1, type));
#endif
		return true;
	}
	else if(type==PxGeometryType::eCAPSULE)
	{
		const PxU32 size = mCapsuleObstacles.size();
		PX_ASSERT(index<size);
		if(index>=size)
			return false;

#ifdef NEW_ENCODING
		remove<InternalCapsuleObstacle>(mHandleManager, object, handle, index, size, mCapsuleObstacles);
#endif

		mCapsuleObstacles.replaceWithLast(index);
#ifdef NEW_ENCODING
		mCCTManager.onObstacleRemoved(handle);
#else
		mCCTManager.onObstacleRemoved(handle, encodeHandle(size-1, type));
#endif
		return true;
	}
	else return false;
}

bool ObstacleContext::updateObstacle(ObstacleHandle handle, const PxObstacle& obstacle)
{
#ifdef NEW_ENCODING
	void* object = mHandleManager.GetObject(handle);
	if(!object)
		return false;
	const PxGeometryType::Enum type = decodeInternalType(object);
	PX_ASSERT(type==obstacle.getType());
	if(type!=obstacle.getType())
		return false;
	const PxU32 index = decodeInternalIndex(object);
#else
	const PxGeometryType::Enum type = decodeType(handle);
	PX_ASSERT(type==obstacle.getType());
	if(type!=obstacle.getType())
		return false;
	const PxU32 index = decodeIndex(handle);
#endif

	if(type==PxGeometryType::eBOX)
	{
		const PxU32 size = mBoxObstacles.size();
		PX_ASSERT(index<size);
		if(index>=size)
			return false;

		mBoxObstacles[index].mData = static_cast<const PxBoxObstacle&>(obstacle);
		mCCTManager.onObstacleUpdated(handle,this);
		return true;
	}
	else if(type==PxGeometryType::eCAPSULE)
	{
		const PxU32 size = mCapsuleObstacles.size();
		PX_ASSERT(index<size);
		if(index>=size)
			return false;

		mCapsuleObstacles[index].mData = static_cast<const PxCapsuleObstacle&>(obstacle);
		mCCTManager.onObstacleUpdated(handle,this);
		return true;
	}
	else return false;
}

PxU32 ObstacleContext::getNbObstacles() const
{
	return mBoxObstacles.size() + mCapsuleObstacles.size();
}

const PxObstacle* ObstacleContext::getObstacle(PxU32 i) const
{
	const PxU32 nbBoxes = mBoxObstacles.size();
	if(i<nbBoxes)
		return &mBoxObstacles[i].mData;
	i -= nbBoxes;

	const PxU32 nbCapsules = mCapsuleObstacles.size();
	if(i<nbCapsules)
		return &mCapsuleObstacles[i].mData;

	return NULL;
}

const PxObstacle* ObstacleContext::getObstacleByHandle(ObstacleHandle handle) const
{
#ifdef NEW_ENCODING
	void* object = mHandleManager.GetObject(handle);
	if(!object)
		return NULL;
	const PxGeometryType::Enum type = decodeInternalType(object);
	const PxU32 index = decodeInternalIndex(object);
#else
	const PxGeometryType::Enum type = decodeType(handle);
	const PxU32 index = decodeIndex(handle);
#endif

	if(type == PxGeometryType::eBOX)
	{
		const PxU32 size = mBoxObstacles.size();
		if(index>=size)
			return NULL;
		PX_ASSERT(mBoxObstacles[index].mHandle==handle);
		return &mBoxObstacles[index].mData;
	}
	else if(type==PxGeometryType::eCAPSULE)
	{
		const PxU32 size = mCapsuleObstacles.size();
		if(index>=size)
			return NULL;
		PX_ASSERT(mCapsuleObstacles[index].mHandle==handle);
		return &mCapsuleObstacles[index].mData;
	}
	else return NULL;
}

#include "GuRaycastTests.h"
#include "PxBoxGeometry.h"
#include "PxCapsuleGeometry.h"
#include "PsMathUtils.h"
using namespace Gu;
const PxObstacle* ObstacleContext::raycastSingle(PxRaycastHit& hit, const PxVec3& origin, const PxVec3& unitDir, const PxReal distance, ObstacleHandle& obstacleHandle) const
{
	PxRaycastHit localHit;
	PxF32 t = FLT_MAX;
	const PxObstacle* touchedObstacle = NULL;

	const PxHitFlags hitFlags = PxHitFlags(0);

	{
		const RaycastFunc raycastFunc = Gu::getRaycastFuncTable()[PxGeometryType::eBOX];
		PX_ASSERT(raycastFunc);

		const PxU32 nbExtraBoxes = mBoxObstacles.size();
		for(PxU32 i=0;i<nbExtraBoxes;i++)
		{
			const PxBoxObstacle& userBoxObstacle = mBoxObstacles[i].mData;

			PxU32 status = raycastFunc(	PxBoxGeometry(userBoxObstacle.mHalfExtents),
										PxTransform(toVec3(userBoxObstacle.mPos), userBoxObstacle.mRot),
										origin, unitDir, distance,
										hitFlags,
										1, &localHit);
			if(status && localHit.distance<t)
			{
				t = localHit.distance;
				hit = localHit;
				obstacleHandle = mBoxObstacles[i].mHandle;
				touchedObstacle = &userBoxObstacle;
			}
		}
	}

	{
		const RaycastFunc raycastFunc = Gu::getRaycastFuncTable()[PxGeometryType::eCAPSULE];
		PX_ASSERT(raycastFunc);

		const PxU32 nbExtraCapsules = mCapsuleObstacles.size();
		for(PxU32 i=0;i<nbExtraCapsules;i++)
		{
			const PxCapsuleObstacle& userCapsuleObstacle = mCapsuleObstacles[i].mData;

			PxU32 status = raycastFunc(	PxCapsuleGeometry(userCapsuleObstacle.mRadius, userCapsuleObstacle.mHalfHeight),
										PxTransform(toVec3(userCapsuleObstacle.mPos), userCapsuleObstacle.mRot),
										origin, unitDir, distance,
										hitFlags,
										1, &localHit);
			if(status && localHit.distance<t)
			{
				t = localHit.distance;
				hit = localHit;
				obstacleHandle = mCapsuleObstacles[i].mHandle;
				touchedObstacle = &userCapsuleObstacle;
			}
		}
	}
	return touchedObstacle;
}


const PxObstacle* ObstacleContext::raycastSingle(PxRaycastHit& hit, const ObstacleHandle& obstacleHandle, const PxVec3& origin, const PxVec3& unitDir, const PxReal distance) const
{	
	const PxHitFlags hitFlags = PxHitFlags(0);

#ifdef NEW_ENCODING
	void* object = mHandleManager.GetObject(obstacleHandle);
	if(!object)
		return NULL;
	const PxGeometryType::Enum geomType = decodeInternalType(object);
	const PxU32 index = decodeInternalIndex(object);
#else
	const PxGeometryType::Enum geomType = decodeType(obstacleHandle);
	const PxU32 index = decodeIndex(obstacleHandle);
#endif
	if(geomType == PxGeometryType::eBOX)
	{
		const PxBoxObstacle& userBoxObstacle = mBoxObstacles[index].mData;

		PxU32 status = Gu::getRaycastFuncTable()[PxGeometryType::eBOX](
			PxBoxGeometry(userBoxObstacle.mHalfExtents),
			PxTransform(toVec3(userBoxObstacle.mPos), userBoxObstacle.mRot),
			origin, unitDir, distance,
			hitFlags,
			1, &hit);

		if(status)
		{								
			return &userBoxObstacle;
		}
	}
	else
	{
		PX_ASSERT(geomType == PxGeometryType::eCAPSULE);
		const PxCapsuleObstacle& userCapsuleObstacle = mCapsuleObstacles[index].mData;

		PxU32 status = Gu::getRaycastFuncTable()[PxGeometryType::eCAPSULE](
			PxCapsuleGeometry(userCapsuleObstacle.mRadius, userCapsuleObstacle.mHalfHeight),
			PxTransform(toVec3(userCapsuleObstacle.mPos), userCapsuleObstacle.mRot),
			origin, unitDir, distance,
			hitFlags,
			1, &hit);
		if(status)
		{
			return &userCapsuleObstacle;
		}
	}

	return NULL;
}

void ObstacleContext::onOriginShift(const PxVec3& shift)
{
	for(PxU32 i=0; i < mBoxObstacles.size(); i++)
		mBoxObstacles[i].mData.mPos -= shift;

	for(PxU32 i=0; i < mCapsuleObstacles.size(); i++)
		mCapsuleObstacles[i].mData.mPos -= shift;
}
