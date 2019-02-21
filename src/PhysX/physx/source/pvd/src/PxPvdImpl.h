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

#ifndef PXPVDSDK_PXPVDIMPL_H
#define PXPVDSDK_PXPVDIMPL_H

#include "foundation/PxProfiler.h"

#include "PsAllocator.h"
#include "PsPvd.h"
#include "PsArray.h"
#include "PsMutex.h"
#include "PxPvdCommStreamTypes.h"
#include "PxPvdFoundation.h"
#include "PxPvdObjectModelMetaData.h"
#include "PxPvdObjectRegistrar.h"

namespace physx
{

namespace profile
{
	class PxProfileZoneManager;
}

namespace pvdsdk
{
class PvdMemClient;
class PvdProfileZoneClient;

struct MetaDataProvider : public PvdOMMetaDataProvider, public shdfnd::UserAllocated
{
    typedef shdfnd::Mutex::ScopedLock TScopedLockType;
    typedef shdfnd::HashMap<const void*, int32_t> TInstTypeMap;
	PvdObjectModelMetaData& mMetaData;
    shdfnd::Mutex mMutex;
	uint32_t mRefCount;
	TInstTypeMap mTypeMap;

	MetaDataProvider()
	: mMetaData(PvdObjectModelMetaData::create()), mRefCount(0), mTypeMap("MetaDataProvider::mTypeMap")
	{
		mMetaData.addRef();
	}
	virtual ~MetaDataProvider()
	{
		mMetaData.release();
	}

	virtual void addRef()
	{
		TScopedLockType locker(mMutex);
		++mRefCount;
	}
	virtual void release()
	{
		{
			TScopedLockType locker(mMutex);
			if(mRefCount)
				--mRefCount;
		}
		if(!mRefCount)
			PVD_DELETE(this);
	}
	virtual PvdObjectModelMetaData& lock()
	{
		mMutex.lock();
		return mMetaData;
	}
	virtual void unlock()
	{
		mMutex.unlock();
	}

	virtual bool createInstance(const NamespacedName& clsName, const void* instance)
	{
		TScopedLockType locker(mMutex);
		Option<ClassDescription> cls(mMetaData.findClass(clsName));
		if(cls.hasValue() == false)
			return false;
		int32_t instType = cls->mClassId;
		mTypeMap.insert(instance, instType);
		return true;
	}
	virtual bool isInstanceValid(const void* instance)
	{
		TScopedLockType locker(mMutex);
		ClassDescription classDesc;
		bool retval = mTypeMap.find(instance) != NULL;
#if PX_DEBUG
		if(retval)
			classDesc = mMetaData.getClass(mTypeMap.find(instance)->second);
#endif
		return retval;
	}
	virtual void destroyInstance(const void* instance)
	{
		{
			TScopedLockType locker(mMutex);
			mTypeMap.erase(instance);
		}
	}
	virtual int32_t getInstanceClassType(const void* instance)
	{
		TScopedLockType locker(mMutex);
		const TInstTypeMap::Entry* entry = mTypeMap.find(instance);
		if(entry)
			return entry->second;
		return -1;
	}

  private:
	MetaDataProvider& operator=(const MetaDataProvider&);
	MetaDataProvider(const MetaDataProvider&);
};

//////////////////////////////////////////////////////////////////////////
/*!
PvdImpl is the realization of PxPvd.
It implements the interface methods and provides richer functionality for advanced users or internal clients (such as
PhysX or APEX), including handler notification for clients.
*/
//////////////////////////////////////////////////////////////////////////
class PvdImpl : public PsPvd, public shdfnd::UserAllocated
{
	PX_NOCOPY(PvdImpl)

    typedef shdfnd::Mutex::ScopedLock TScopedLockType;
	typedef void (PvdImpl::*TAllocationHandler)(size_t size, const char* typeName, const char* filename, int line,
	                                            void* allocatedMemory);
	typedef void (PvdImpl::*TDeallocationHandler)(void* allocatedMemory);

  public:
	PvdImpl();
	virtual ~PvdImpl();
	void release();

	bool connect(PxPvdTransport& transport, PxPvdInstrumentationFlags flags);
	void disconnect();
	bool isConnected(bool useCachedStatus = true);
	void flush();

	PxPvdTransport* getTransport();
	PxPvdInstrumentationFlags getInstrumentationFlags();

	void addClient(PvdClient* client);
	void removeClient(PvdClient* client);

	PvdOMMetaDataProvider& getMetaDataProvider();

	bool registerObject(const void* inItem);
	bool unRegisterObject(const void* inItem);

	//AllocationListener
	void onAllocation(size_t size, const char* typeName, const char* filename, int line, void* allocatedMemory);
	void onDeallocation(void* addr);

	uint64_t getNextStreamId();

	static bool initialize();
	static PvdImpl* getInstance();

	// Profiling

	virtual void* zoneStart(const char* eventName, bool detached, uint64_t contextId);

	virtual void zoneEnd(void* profilerData, const char *eventName, bool detached, uint64_t contextId);

  private:
	void sendTransportInitialization();

	PxPvdTransport*						mPvdTransport;
	physx::shdfnd::Array<PvdClient*>	mPvdClients;

	MetaDataProvider*					mSharedMetaProvider; // shared between clients
	ObjectRegistrar						mObjectRegistrar;

	PvdMemClient*						mMemClient;

	PxPvdInstrumentationFlags			mFlags;
	bool								mIsConnected;
	bool								mIsNVTXSupportEnabled;
	uint32_t							mNVTXContext;
	uint64_t							mNextStreamId;
	physx::profile::PxProfileZoneManager*mProfileZoneManager;
	PvdProfileZoneClient*				mProfileClient;
	physx::profile::PxProfileZone*		mProfileZone;
	static PvdImpl*						sInstance;
	static uint32_t						sRefCount;
};

} // namespace pvdsdk
}

#endif // PXPVDSDK_PXPVDIMPL_H
