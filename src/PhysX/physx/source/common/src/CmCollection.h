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


#ifndef PX_PHYSICS_CM_COLLECTION
#define PX_PHYSICS_CM_COLLECTION

#include "CmPhysXCommon.h"
#include "PxCollection.h"
#include "PsHashMap.h"
#include "PsUserAllocated.h"
#include "PsAllocator.h"

namespace physx
{
namespace Cm
{	
	template <class Key, 
			  class Value,
			  class HashFn = Ps::Hash<Key>, 
			  class Allocator = Ps::NonTrackingAllocator >
	class CollectionHashMap : public Ps::CoalescedHashMap< Key, Value, HashFn, Allocator>
	{
		typedef physx::shdfnd::internal::HashMapBase< Key, Value, HashFn, Allocator> MapBase;	
		typedef Ps::Pair<const Key,Value> EntryData;

		public:
			CollectionHashMap(PxU32 initialTableSize = 64, float loadFactor = 0.75f):
			    Ps::CoalescedHashMap< Key, Value, HashFn, Allocator>(initialTableSize,loadFactor) {}

			void insertUnique(const Key& k, const Value& v)
			{
				PX_PLACEMENT_NEW(MapBase::mBase.insertUnique(k), EntryData)(k,v);
			}
	};

	
	
	class Collection : public PxCollection, public Ps::UserAllocated
	{
	public:
		typedef CollectionHashMap<PxBase*, PxSerialObjectId> ObjectToIdMap;
		typedef CollectionHashMap<PxSerialObjectId, PxBase*> IdToObjectMap;
					
		virtual void						add(PxBase& object, PxSerialObjectId ref);
		virtual	void						remove(PxBase& object);	
		virtual bool						contains(PxBase& object) const;
		virtual void						addId(PxBase& object, PxSerialObjectId id);
		virtual void						removeId(PxSerialObjectId id);
		virtual PxBase*						find(PxSerialObjectId ref) const;
		virtual void						add(PxCollection& collection);
		virtual void						remove(PxCollection& collection);		
		virtual	PxU32						getNbObjects() const;
		virtual PxBase&						getObject(PxU32 index) const;
		virtual	PxU32						getObjects(PxBase** userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const;

		virtual PxU32						getNbIds() const;		
		virtual PxSerialObjectId			getId(const PxBase& object) const;
		virtual	PxU32						getIds(PxSerialObjectId* userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const;

		void								release() { PX_DELETE(this); }


		// Only for internal use. Bypasses virtual calls, specialized behaviour.
		PX_INLINE	void		            internalAdd(PxBase* s, PxSerialObjectId id = PX_SERIAL_OBJECT_ID_INVALID)				{ mObjects.insertUnique(s, id);	                   }
		PX_INLINE	PxU32		            internalGetNbObjects()		 const	{ return mObjects.size();								               }
		PX_INLINE	PxBase*		            internalGetObject(PxU32 i)	 const	{ PX_ASSERT(i<mObjects.size());	return mObjects.getEntries()[i].first; }
		PX_INLINE	const ObjectToIdMap::Entry*	internalGetObjects() const  { return mObjects.getEntries(); 			                           }
			
		IdToObjectMap					    mIds;
		ObjectToIdMap                       mObjects;
		
	};
}
}

#endif
