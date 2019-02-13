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

#include "PxPhysicsAPI.h"
#include "PxConstraintExt.h"
#include "PxPhysicsVersion.h"
#include "PsFoundation.h"
#include "PxMetaData.h"
#include "SnConvX.h"
#include "SnSerializationRegistry.h"
#include "SnSerialUtils.h"
#include "ExtSerialization.h"
#include "PxSerializer.h"
#include "CmCollection.h"

using namespace physx;
using namespace Sn;

namespace
{
	struct RequiresCallback : public PxProcessPxBaseCallback
	{
		RequiresCallback(physx::PxCollection& c) : collection(c) {}
		void process(PxBase& base)
		{			
			  if(!collection.contains(base))
				  collection.add(base);
		}
    
		PxCollection& collection;
		PX_NOCOPY(RequiresCallback)	
	};

	struct CompleteCallback : public PxProcessPxBaseCallback
	{
		CompleteCallback(physx::PxCollection& r, physx::PxCollection& c, const physx::PxCollection* e) :
		required(r),  complete(c), external(e)	{}
		void process(PxBase& base)
		{
			if(complete.contains(base) || (external && external->contains(base)))
			   return;
			if(!required.contains(base))
				  required.add(base);
		}

		PxCollection& required;
		PxCollection& complete;
		const PxCollection* external;
		PX_NOCOPY(CompleteCallback)
	};

	void getRequiresCollection(PxCollection& required, PxCollection& collection, PxCollection& complete, const PxCollection* external, PxSerializationRegistry& sr, bool followJoints)
	{
		CompleteCallback callback(required, complete, external);
		for (PxU32 i = 0; i < collection.getNbObjects(); ++i)
		{
			PxBase& s = collection.getObject(i);			
			const PxSerializer* serializer = sr.getSerializer(s.getConcreteType());
			PX_ASSERT(serializer);
			serializer->requiresObjects(s, callback);

			if(followJoints)
			{
				PxRigidActor* actor = s.is<PxRigidActor>();
				if(actor)
				{
					Ps::Array<PxConstraint*> objects(actor->getNbConstraints());
					actor->getConstraints(objects.begin(), objects.size());

					for(PxU32 j=0;j<objects.size();j++)
					{
						PxU32 typeId;
						PxJoint* joint = reinterpret_cast<PxJoint*>(objects[j]->getExternalReference(typeId));				
						if(typeId == PxConstraintExtIDs::eJOINT)
						{							
							const PxSerializer* sj = sr.getSerializer(joint->getConcreteType());
							PX_ASSERT(sj);
							sj->requiresObjects(*joint, callback);
							if(!required.contains(*joint))
								required.add(*joint);
						}
					}
				}
			}
		}	
	}
}

bool PxSerialization::isSerializable(PxCollection& collection, PxSerializationRegistry& sr, const PxCollection* externalReferences) 
{		
	PxCollection* subordinateCollection = PxCreateCollection();
	PX_ASSERT(subordinateCollection);

	for(PxU32 i = 0; i < collection.getNbObjects(); ++i)
	{
		PxBase& s = collection.getObject(i);
		const PxSerializer* serializer = sr.getSerializer(s.getConcreteType());
		PX_ASSERT(serializer);
		if(serializer->isSubordinate())
			subordinateCollection->add(s);

		if(externalReferences)
		{
			PxSerialObjectId id = collection.getId(s);
			if(id != PX_SERIAL_OBJECT_ID_INVALID)
			{
				PxBase* object = externalReferences->find(id);
				if(object && (object != &s))
				{					
					subordinateCollection->release();					
					Ps::getFoundation().error(physx::PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, 
						"PxSerialization::isSerializable: Reference id %" PX_PRIu64 " used both in current collection and in externalReferences. "
						"Please use unique identifiers.", id);	
					return false;
				}
			}
		}		
	}

	PxCollection* requiresCollection = PxCreateCollection();
	PX_ASSERT(requiresCollection);
		
	RequiresCallback requiresCallback0(*requiresCollection);

	for (PxU32 i = 0; i < collection.getNbObjects(); ++i)
	{
		PxBase& s = collection.getObject(i);
		const PxSerializer* serializer = sr.getSerializer(s.getConcreteType());
		PX_ASSERT(serializer);
		serializer->requiresObjects(s, requiresCallback0);

		Cm::Collection* cmRequiresCollection = static_cast<Cm::Collection*>(requiresCollection);

		for(PxU32 j = 0; j < cmRequiresCollection->getNbObjects(); ++j)
		{
			PxBase& s0 = cmRequiresCollection->getObject(j);

			if(subordinateCollection->contains(s0))
			{
				subordinateCollection->remove(s0);
				continue;
			}

			bool requiredIsInCollection = collection.contains(s0);
			if(!requiredIsInCollection)
			{
				if(externalReferences)
				{
					if(!externalReferences->contains(s0))
					{						
						Ps::getFoundation().error(physx::PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, 
							"PxSerialization::isSerializable: Object of type %s references a missing object of type %s. "
							"The missing object needs to be added to either the current collection or the externalReferences collection.",
							s.getConcreteTypeName(), s0.getConcreteTypeName());						
					}
					else if(externalReferences->getId(s0) == PX_SERIAL_OBJECT_ID_INVALID)
					{						
						Ps::getFoundation().error(physx::PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, 
							"PxSerialization::isSerializable: Object of type %s in externalReferences collection requires an id.", 
							s0.getConcreteTypeName());
					}
					else
						continue;
				}
				else
				{				
					Ps::getFoundation().error(physx::PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, 
						"PxSerialization::isSerializable: Object of type %s references a missing serial object of type %s. "
						"Please completed the collection or specify an externalReferences collection containing the object.",
						s.getConcreteTypeName(), s0.getConcreteTypeName());					
				}
				subordinateCollection->release();
				requiresCollection->release();
				return false;	
			}		
		}
		cmRequiresCollection->mObjects.clear();
	}
	requiresCollection->release();
	
	PxU32 numOrphans = subordinateCollection->getNbObjects();
	
	for(PxU32 j = 0; j < numOrphans; ++j)
	{
		PxBase& subordinate = subordinateCollection->getObject(j);

		Ps::getFoundation().error(physx::PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, 
			"PxSerialization::isSerializable: An object of type %s is subordinate but not required "
			"by other objects in the collection (orphan). Please remove the object from the collection or add its owner.", 
			subordinate.getConcreteTypeName());
	}
	
	subordinateCollection->release();

	if(numOrphans>0)
		return false;

	if(externalReferences)
	{
		PxCollection* oppositeRequiresCollection = PxCreateCollection();
		PX_ASSERT(oppositeRequiresCollection);

		RequiresCallback requiresCallback(*oppositeRequiresCollection);

		for (PxU32 i = 0; i < externalReferences->getNbObjects(); ++i)
		{
			PxBase& s = externalReferences->getObject(i);			
			const PxSerializer* serializer = sr.getSerializer(s.getConcreteType());
			PX_ASSERT(serializer);
			serializer->requiresObjects(s, requiresCallback);
		
			Cm::Collection* cmCollection = static_cast<Cm::Collection*>(oppositeRequiresCollection);

			for(PxU32 j = 0; j < cmCollection->getNbObjects(); ++j)
			{
				PxBase& s0 = cmCollection->getObject(j);

				if(collection.contains(s0))
				{
					oppositeRequiresCollection->release();
					Ps::getFoundation().error(physx::PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, 
						"PxSerialization::isSerializable: Object of type %s in externalReferences references an object "
						"of type %s in collection (circular dependency).",
						s.getConcreteTypeName(), s0.getConcreteTypeName());
					return false;
				}
			}
			cmCollection->mObjects.clear();
		}
		oppositeRequiresCollection->release();
	}

	return true;
}

void PxSerialization::complete(PxCollection& collection, PxSerializationRegistry& sr, const PxCollection* exceptFor, bool followJoints)
{	
	PxCollection* curCollection = PxCreateCollection();
	PX_ASSERT(curCollection);	
	curCollection->add(collection);

	PxCollection* requiresCollection = PxCreateCollection();
	PX_ASSERT(requiresCollection);

	do
	{		
		getRequiresCollection(*requiresCollection, *curCollection, collection, exceptFor, sr, followJoints);		
		
		collection.add(*requiresCollection);
		PxCollection* swap = curCollection;	
		curCollection = requiresCollection;
		requiresCollection = swap;
		(static_cast<Cm::Collection*>(requiresCollection))->mObjects.clear();

	}while(curCollection->getNbObjects() > 0);

	requiresCollection->release();
	curCollection->release();
	
}

void PxSerialization::createSerialObjectIds(PxCollection& collection, const PxSerialObjectId base)
{
	PxSerialObjectId localBase = base;
	PxU32 nbObjects = collection.getNbObjects();

	for (PxU32 i = 0; i < nbObjects; ++i)
	{
		while(collection.find(localBase))
		{
			localBase++;
		}

		PxBase& s = collection.getObject(i);		
		if(PX_SERIAL_OBJECT_ID_INVALID == collection.getId(s))
		{
			collection.addId(s, localBase);
			localBase++;
		}
	}
}

namespace physx { namespace Sn
{
	static PxU32 addToStringTable(physx::shdfnd::Array<char>& stringTable, const char* str)
	{
		if(!str)
			return 0xffffffff;

		PxI32 length = PxI32(stringTable.size());
		const char* table = stringTable.begin();
		const char* start = table;
		while(length)
		{
			if(strcmp(table, str)==0)
				return PxU32(table - start);

			const char* saved = table;
			while(*table++);
			length -= PxU32(table - saved);
			PX_ASSERT(length>=0);
		}

		const PxU32 offset = stringTable.size();

		while(*str)
			stringTable.pushBack(*str++);
		stringTable.pushBack(0);
		return offset;
	}
} }

void PxSerialization::dumpBinaryMetaData(PxOutputStream& outputStream, PxSerializationRegistry& sr)
{
	class MetaDataStream : public PxOutputStream
	{
	public:
		bool addNewType(const char* typeName)
		{			
			for(PxU32 i=0;i<types.size();i++)
			{			
				if(strcmp(types[i], typeName)==0)
					return false;
			}
			types.pushBack(typeName);
			return true;
		}
		virtual	PxU32 write(const void* src, PxU32 count)
		{
			PX_ASSERT(count==sizeof(PxMetaDataEntry));
			const PxMetaDataEntry* entry = reinterpret_cast<const PxMetaDataEntry*>(src);
			if(( entry->flags & PxMetaDataFlag::eTYPEDEF) || ((entry->flags & PxMetaDataFlag::eCLASS) && (!entry->name)) )
                 newType = addNewType(entry->type);
			if(newType)
			   metaData.pushBack(*entry);
			return count;
		}		
		shdfnd::Array<PxMetaDataEntry> metaData;
		shdfnd::Array<const char*> types;
		bool newType;
	}s;

	SerializationRegistry& sn = static_cast<SerializationRegistry&>(sr);
	sn.getBinaryMetaData(s);

	shdfnd::Array<char>	stringTable;

	PxU32 nb = s.metaData.size();
	PxMetaDataEntry* entries = s.metaData.begin();
	for(PxU32 i=0;i<nb;i++)
	{
		entries[i].type = reinterpret_cast<const char*>(size_t(addToStringTable(stringTable, entries[i].type)));
		entries[i].name = reinterpret_cast<const char*>(size_t(addToStringTable(stringTable, entries[i].name)));
	}

	PxU32 platformTag = getBinaryPlatformTag();

	const PxU32 gaussMapLimit = 32;

	const PxU32 header = PX_MAKE_FOURCC('M','E','T','A');
	const PxU32 version = PX_PHYSICS_VERSION;
	const PxU32 binaryVersion = PX_BINARY_SERIAL_VERSION;
	const PxU32 ptrSize = sizeof(void*);
	PxU32 buildNumber = 0;
#if defined(PX_BUILD_NUMBER)
	buildNumber =  PX_BUILD_NUMBER;
#endif
	outputStream.write(&header, 4);
	outputStream.write(&version, 4);
	outputStream.write(&binaryVersion, 4);
	outputStream.write(&buildNumber, 4);
	outputStream.write(&ptrSize, 4);
	outputStream.write(&platformTag, 4);
	outputStream.write(&gaussMapLimit, 4);

	outputStream.write(&nb, 4);
	outputStream.write(entries, nb*sizeof(PxMetaDataEntry));
	
	//concreteTypes to name	
	PxU32 num = sn.getNbSerializers();
	outputStream.write(&num, 4);
	for(PxU32 i=0; i<num; i++)
	{
		PxU16 type = sn.getSerializerType(i);
		PxU32 nameOffset = addToStringTable(stringTable, sn.getSerializerName(i)); 
		outputStream.write(&type, 2);
		outputStream.write(&nameOffset, 4);
	}

	PxU32 length = stringTable.size();
	const char* table = stringTable.begin();
	outputStream.write(&length, 4);
	outputStream.write(table, length);
}

PxBinaryConverter* PxSerialization::createBinaryConverter()
{
	return PX_NEW(ConvX)();
}
