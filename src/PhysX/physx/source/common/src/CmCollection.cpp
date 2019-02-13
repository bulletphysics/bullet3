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

#include "CmCollection.h"
#include "PsFoundation.h"

using namespace physx;
using namespace Cm;

void Collection::add(PxBase& object, PxSerialObjectId id)
{
	PxSerialObjectId originId = getId(object);
	if( originId != PX_SERIAL_OBJECT_ID_INVALID)
	{
		if( originId != id)
		{
			 physx::shdfnd::getFoundation().error(physx::PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__,
		        "PxCollection::add called for an object that has an associated id already present in the collection!");
		}
		return;		   
	}
	
	if(id != PX_SERIAL_OBJECT_ID_INVALID)
	{		
		if(!mIds.insert(id, &object))
		{
		   physx::shdfnd::getFoundation().error(physx::PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__,
		        "PxCollection::add called with an id which is already used in the collection");
		   return;	
		}
	}
    
	mObjects[&object] = id;	
}

void Collection::remove(PxBase& object)
{
	PX_CHECK_AND_RETURN(contains(object), "PxCollection::remove called for an object not contained in the collection!");
	
	const ObjectToIdMap::Entry* e = mObjects.find(&object);
	if(e)
	{
		mIds.erase(e->second);
	    mObjects.erase(&object);
	}
}

bool Collection::contains(PxBase& object) const
{	
	return  mObjects.find(&object) != NULL;
}

void Collection::addId(PxBase& object, PxSerialObjectId id)
{
	PX_CHECK_AND_RETURN(contains(object), "PxCollection::addId called for object that is not contained in the collection!");
	PX_CHECK_AND_RETURN(id != PX_SERIAL_OBJECT_ID_INVALID, "PxCollection::addId called with PxSerialObjectId being set to PX_SERIAL_OBJECT_ID_INVALID!");
	PX_CHECK_AND_RETURN(mIds.find(id) == NULL, "PxCollection::addId called with an id which is already used in the collection!");
	
	const ObjectToIdMap::Entry* e = mObjects.find(&object);
	if(e  && e->second != PX_SERIAL_OBJECT_ID_INVALID)
		mIds.erase(e->second);

	mIds.insert(id, &object);	
	mObjects[&object] = id;
}

void Collection::removeId(PxSerialObjectId id)
{
	PX_CHECK_AND_RETURN(id != PX_SERIAL_OBJECT_ID_INVALID, "PxCollection::removeId called with PxSerialObjectId being set to PX_SERIAL_OBJECT_ID_INVALID!");
	PX_CHECK_AND_RETURN(mIds.find(id), "PxCollection::removeId called with PxSerialObjectId not contained in the collection!");
	const IdToObjectMap::Entry* e = mIds.find(id);
	if(e)
	{	
		mObjects[e->second] = PX_SERIAL_OBJECT_ID_INVALID;
	    mIds.erase(id);	   
	}
}

PxBase* Collection::find(PxSerialObjectId id) const
{
	PX_CHECK_AND_RETURN_NULL(id != PX_SERIAL_OBJECT_ID_INVALID, "PxCollection::find called with PxSerialObjectId being set to PX_SERIAL_OBJECT_ID_INVALID!");
	const IdToObjectMap::Entry* e = mIds.find(id);
	return e ? static_cast<PxBase*>(e->second) : NULL;
}

void Collection::add(PxCollection& _collection)
{
	Collection& collection = static_cast<Collection&>(_collection);
	PX_CHECK_AND_RETURN(this != &collection, "PxCollection::add(PxCollection&) called with itself!");

	mObjects.reserve(mObjects.capacity() + collection.mObjects.size());
	const ObjectToIdMap::Entry* e = collection.mObjects.getEntries();
	for (PxU32 i = 0; i < collection.mObjects.size(); ++i)
	{
        PxSerialObjectId id = e[i].second;
		if( id != PX_SERIAL_OBJECT_ID_INVALID)
		{
			if(!mIds.insert(id, e[i].first))
		    {
			    if(mIds[id] != e[i].first)
				{
				    PX_CHECK_MSG( false, "PxCollection::add(PxCollection&) called with conflicting id!");
				    mObjects.insert(e[i].first, PX_SERIAL_OBJECT_ID_INVALID);
				}
		    }
			else
			   mObjects[ e[i].first ] = id;
		}
		else
		    mObjects.insert(e[i].first, PX_SERIAL_OBJECT_ID_INVALID);				
	}
}

void Collection::remove(PxCollection& _collection)
{
	Collection& collection = static_cast<Collection&>(_collection);
	PX_CHECK_AND_RETURN(this != &collection, "PxCollection::remove(PxCollection&) called with itself!");

	const ObjectToIdMap::Entry* e = collection.mObjects.getEntries();
	for (PxU32 i = 0; i < collection.mObjects.size(); ++i)
	{
		const ObjectToIdMap::Entry* e1 = mObjects.find(e[i].first);
		if(e1)
		{
		    mIds.erase(e1->second);
	        mObjects.erase(e1->first);
		}
	}
}

PxU32 Collection::getNbObjects() const
{
	return mObjects.size();
}

PxBase& Collection::getObject(PxU32 index) const
{
	PX_ASSERT(index < mObjects.size());
	return *mObjects.getEntries()[index].first;
}

PxU32 Collection::getObjects(PxBase** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	PX_CHECK_AND_RETURN_NULL(userBuffer != NULL, "PxCollection::getObjects called with userBuffer NULL!");
	PX_CHECK_AND_RETURN_NULL(bufferSize != 0, "PxCollection::getObjects called with bufferSize 0!");
	PxU32 dstIndex = 0;
	const ObjectToIdMap::Entry* e  = mObjects.getEntries();
	for (PxU32 srcIndex = startIndex; srcIndex < mObjects.size() && dstIndex < bufferSize; ++srcIndex)
		userBuffer[dstIndex++] = e[srcIndex].first;

	return dstIndex;
}

PxU32 Collection::getNbIds() const
{
	return mIds.size();
}

PxSerialObjectId Collection::getId(const PxBase& object) const
{	
	const ObjectToIdMap::Entry* e =  mObjects.find(const_cast<PxBase*>(&object));
	return e ? e->second : PX_SERIAL_OBJECT_ID_INVALID;
}

PxU32 Collection::getIds(PxSerialObjectId* userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	PX_CHECK_AND_RETURN_NULL(userBuffer != NULL, "PxCollection::getIds called with userBuffer NULL!");
	PX_CHECK_AND_RETURN_NULL(bufferSize != 0, "PxCollection::getIds called with bufferSize 0!");
	PxU32 dstIndex = 0;

	IdToObjectMap::Iterator srcIt = (const_cast<IdToObjectMap&>(mIds)).getIterator();

	while (!srcIt.done() &&  dstIndex < bufferSize)
	{
		if(srcIt->first != PX_SERIAL_OBJECT_ID_INVALID)
		{
			if(startIndex > 0)
				startIndex--;
			else
				userBuffer[dstIndex++] = srcIt->first;
		}
		srcIt++;
	}

	return dstIndex;
}

PxCollection*	PxCreateCollection()
{
	return PX_NEW(Collection);
}
