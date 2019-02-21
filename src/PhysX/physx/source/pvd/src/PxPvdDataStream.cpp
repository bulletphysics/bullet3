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

#include "foundation/PxAssert.h"
#include "PxPvdCommStreamEventSink.h"
#include "PxPvdDataStreamHelpers.h"
#include "PxPvdObjectModelInternalTypes.h"
#include "PxPvdImpl.h"
#include "PsFoundation.h"

using namespace physx;
using namespace physx::pvdsdk;
using namespace physx::shdfnd;

namespace
{

struct ScopedMetaData
{
	PvdOMMetaDataProvider& mProvider;
	PvdObjectModelMetaData& mMeta;
	ScopedMetaData(PvdOMMetaDataProvider& provider) : mProvider(provider), mMeta(provider.lock())
	{
	}
	~ScopedMetaData()
	{
		mProvider.unlock();
	}
	PvdObjectModelMetaData* operator->()
	{
		return &mMeta;
	}

  private:
	ScopedMetaData& operator=(const ScopedMetaData&);
};

struct PropertyDefinitionHelper : public PvdPropertyDefinitionHelper
{
	PvdDataStream* mStream;
	PvdOMMetaDataProvider& mProvider;
    Array<char> mNameBuffer;
    Array<uint32_t> mNameStack;
    Array<NamedValue> mNamedValues;
    Array<PropertyMessageArg> mPropertyMessageArgs;

	PropertyDefinitionHelper(PvdOMMetaDataProvider& provider)
	: mStream(NULL)
	, mProvider(provider)
	, mNameBuffer("PropertyDefinitionHelper::mNameBuffer")
	, mNameStack("PropertyDefinitionHelper::mNameStack")
	, mNamedValues("PropertyDefinitionHelper::mNamedValues")
	, mPropertyMessageArgs("PropertyDefinitionHelper::mPropertyMessageArgs")
	{
	}
	void setStream(PvdDataStream* stream)
	{
		mStream = stream;
	}

	inline void appendStrToBuffer(const char* str)
	{
		if(str == NULL)
			return;
		size_t strLen = strlen(str);
		size_t endBufOffset = mNameBuffer.size();
		size_t resizeLen = endBufOffset;
		// account for null
		if(mNameBuffer.empty())
			resizeLen += 1;
		else
			endBufOffset -= 1;

		mNameBuffer.resize(static_cast<uint32_t>(resizeLen + strLen));
		char* endPtr = mNameBuffer.begin() + endBufOffset;
		PxMemCopy(endPtr, str, static_cast<uint32_t>(strLen));
	}

	virtual void pushName(const char* nm, const char* appender = ".")
	{
		size_t nameBufLen = mNameBuffer.size();
		mNameStack.pushBack(static_cast<uint32_t>(nameBufLen));
		if(mNameBuffer.empty() == false)
			appendStrToBuffer(appender);
		appendStrToBuffer(nm);
		mNameBuffer.back() = 0;
	}

	virtual void pushBracketedName(const char* inName, const char* leftBracket = "[", const char* rightBracket = "]")
	{
		size_t nameBufLen = mNameBuffer.size();
		mNameStack.pushBack(static_cast<uint32_t>(nameBufLen));
		appendStrToBuffer(leftBracket);
		appendStrToBuffer(inName);
		appendStrToBuffer(rightBracket);
		mNameBuffer.back() = 0;
	}

	virtual void popName()
	{
		if(mNameStack.empty())
			return;
		mNameBuffer.resize(static_cast<uint32_t>(mNameStack.back()));
		mNameStack.popBack();
		if(mNameBuffer.empty() == false)
			mNameBuffer.back() = 0;
	}

	virtual const char* getTopName()
	{
		if(mNameBuffer.size())
			return mNameBuffer.begin();
		return "";
	}
	virtual void clearNameStack()
	{
		mNameBuffer.clear();
		mNameStack.clear();
	}

	virtual void addNamedValue(const char* name, uint32_t value)
	{
		mNamedValues.pushBack(NamedValue(name, value));
	}
	virtual void clearNamedValues()
	{
		mNamedValues.clear();
	}

	virtual DataRef<NamedValue> getNamedValues()
	{
		return DataRef<NamedValue>(mNamedValues.begin(), mNamedValues.size());
	}

	virtual void createProperty(const NamespacedName& clsName, const char* inSemantic, const NamespacedName& dtypeName,
	                            PropertyType::Enum propType)
	{
		mStream->createProperty(clsName, getTopName(), inSemantic, dtypeName, propType, getNamedValues());
		clearNamedValues();
	}
	const char* registerStr(const char* str)
	{
		ScopedMetaData scopedProvider(mProvider);
		return scopedProvider->getStringTable().registerStr(str);
	}
	virtual void addPropertyMessageArg(const NamespacedName& inDatatype, uint32_t inOffset, uint32_t inSize)
	{
		mPropertyMessageArgs.pushBack(PropertyMessageArg(registerStr(getTopName()), inDatatype, inOffset, inSize));
	}
	virtual void addPropertyMessage(const NamespacedName& clsName, const NamespacedName& msgName,
	                                uint32_t inStructSizeInBytes)
	{
		if(mPropertyMessageArgs.empty())
		{
			PX_ASSERT(false);
			return;
		}
		mStream->createPropertyMessage(
		    clsName, msgName, DataRef<PropertyMessageArg>(mPropertyMessageArgs.begin(), mPropertyMessageArgs.size()),
		    inStructSizeInBytes);
	}
	virtual void clearPropertyMessageArgs()
	{
		mPropertyMessageArgs.clear();
	}

  private:
	PropertyDefinitionHelper& operator=(const PropertyDefinitionHelper&);
};

class PvdMemPool
{
	// Link List
    Array<uint8_t*> mMemBuffer;
	uint32_t mLength;
	uint32_t mBufIndex;

	// 4k for one page
	static const int BUFFER_LENGTH = 4096;
	PX_NOCOPY(PvdMemPool)
  public:
	PvdMemPool(const char* bufDataName) : mMemBuffer(bufDataName), mLength(0), mBufIndex(0)
	{
		grow();
	}

	~PvdMemPool()
	{
		for(uint32_t i = 0; i < mMemBuffer.size(); i++)
		{
			PX_FREE(mMemBuffer[i]);
		}
	}

	void grow()
	{
		if(mBufIndex + 1 < mMemBuffer.size())
		{
			mBufIndex++;
		}
		else
		{
			uint8_t* Buf = reinterpret_cast<uint8_t*>(PX_ALLOC(BUFFER_LENGTH, "PvdMemPool::mMemBuffer.buf"));
			mMemBuffer.pushBack(Buf);
			mBufIndex = mMemBuffer.size() - 1;
		}
		mLength = 0;
	}

	void* allocate(uint32_t length)
	{
		if(length > uint32_t(BUFFER_LENGTH))
			return NULL;

		if(length + mLength > uint32_t(BUFFER_LENGTH))
			grow();

		void* mem = reinterpret_cast<void*>(&mMemBuffer[mBufIndex][mLength]);
		mLength += length;
		return mem;
	}

	void clear()
	{
		mLength = 0;
		mBufIndex = 0;
	}
};
struct PvdOutStream : public PvdDataStream, public UserAllocated
{
    HashMap<String, uint32_t> mStringHashMap;
	PvdOMMetaDataProvider& mMetaDataProvider;
    Array<uint8_t> mTempBuffer;
	PropertyDefinitionHelper mPropertyDefinitionHelper;
	DataStreamState::Enum mStreamState;

	ClassDescription mSPVClass;
	PropertyMessageDescription mMessageDesc;
	// Set property value and SetPropertyMessage calls require
	// us to write the data out to a separate buffer
	// when strings are involved.
	ForwardingMemoryBuffer mSPVBuffer;
	uint32_t mEventCount;
	uint32_t mPropertyMessageSize;
	bool mConnected;
	uint64_t mStreamId;
    Array<PvdCommand*> mPvdCommandArray;
	PvdMemPool mPvdCommandPool;
	PxPvdTransport& mTransport;

	PvdOutStream(PxPvdTransport& transport, PvdOMMetaDataProvider& provider, uint64_t streamId)
	: mStringHashMap("PvdOutStream::mStringHashMap")
	, mMetaDataProvider(provider)
	, mTempBuffer("PvdOutStream::mTempBuffer")
	, mPropertyDefinitionHelper(mMetaDataProvider)
	, mStreamState(DataStreamState::Open)
	, mSPVBuffer("PvdCommStreamBufferedEventSink::mSPVBuffer")
	, mEventCount(0)
	, mPropertyMessageSize(0)
	, mConnected(true)
	, mStreamId(streamId)
	, mPvdCommandArray("PvdCommStreamBufferedEventSink::mPvdCommandArray")
	, mPvdCommandPool("PvdCommStreamBufferedEventSink::mPvdCommandPool")
	, mTransport(transport)
	{
		mPropertyDefinitionHelper.setStream(this);
	}
	virtual ~PvdOutStream()
	{
	}

	virtual void release()
	{
		PVD_DELETE(this);
	}

	StringHandle toStream(String nm)
	{
		if(nm == NULL || *nm == 0)
			return 0;
        const HashMap<String, uint32_t>::Entry* entry(mStringHashMap.find(nm));
		if(entry)
			return entry->second;
		ScopedMetaData meta(mMetaDataProvider);
		StringHandle hdl = meta->getStringTable().strToHandle(nm);
		nm = meta->getStringTable().handleToStr(hdl);
		handlePvdEvent(StringHandleEvent(nm, hdl));
		mStringHashMap.insert(nm, hdl);
		return hdl;
	}

	StreamNamespacedName toStream(const NamespacedName& nm)
	{
		return StreamNamespacedName(toStream(nm.mNamespace), toStream(nm.mName));
	}

	bool isClassExist(const NamespacedName& nm)
	{
		ScopedMetaData meta(mMetaDataProvider);
		return meta->findClass(nm).hasValue();
	}
	
	bool createMetaClass(const NamespacedName& nm)
	{
		ScopedMetaData meta(mMetaDataProvider);
		meta->getOrCreateClass(nm);
		return true;
	}

	bool deriveMetaClass(const NamespacedName& parent, const NamespacedName& child)
	{
		ScopedMetaData meta(mMetaDataProvider);
		return meta->deriveClass(parent, child);
	}
	
// You will notice that some functions are #pragma'd out throughout this file.
// This is because they are only called from asserts which means they aren't
// called in release.  This causes warnings when building using snc which break
// the build.
#if PX_DEBUG

	bool propertyExists(const NamespacedName& nm, String pname)
	{
		ScopedMetaData meta(mMetaDataProvider);
		return meta->findProperty(nm, pname).hasValue();
	}

#endif

	PvdError boolToError(bool val)
	{
		if(val)
			return PvdErrorType::Success;
		return PvdErrorType::NetworkError;
	}

	// PvdMetaDataStream
	virtual PvdError createClass(const NamespacedName& nm)
	{
		PX_ASSERT(mStreamState == DataStreamState::Open);
#if PX_DEBUG
		PX_ASSERT(isClassExist(nm) == false);
#endif
		createMetaClass(nm);
		return boolToError(handlePvdEvent(CreateClass(toStream(nm))));
	}

	virtual PvdError deriveClass(const NamespacedName& parent, const NamespacedName& child)
	{
		PX_ASSERT(mStreamState == DataStreamState::Open);
#if PX_DEBUG
		PX_ASSERT(isClassExist(parent));
		PX_ASSERT(isClassExist(child));
#endif
		deriveMetaClass(parent, child);
		return boolToError(handlePvdEvent(DeriveClass(toStream(parent), toStream(child))));
	}

	template <typename TDataType>
	TDataType* allocTemp(uint32_t numItems)
	{
		uint32_t desiredBytes = numItems * sizeof(TDataType);
		if(desiredBytes > mTempBuffer.size())
			mTempBuffer.resize(desiredBytes);
		TDataType* retval = reinterpret_cast<TDataType*>(mTempBuffer.begin());
		if(numItems)
		{
			PVD_FOREACH(idx, numItems) new (retval + idx) TDataType();
		}
		return retval;
	}

#if PX_DEBUG

	// Property datatypes need to be uniform.
	// At this point, the data stream cannot handle properties that
	// A struct with a float member and a char member would work.
	// A struct with a float member and a long member would work (more efficiently).
	bool isValidPropertyDatatype(const NamespacedName& dtypeName)
	{
		ScopedMetaData meta(mMetaDataProvider);
		ClassDescription clsDesc(meta->findClass(dtypeName));
		return clsDesc.mRequiresDestruction == false;
	}

#endif

	NamespacedName createMetaProperty(const NamespacedName& clsName, String name, String semantic,
	                                  const NamespacedName& dtypeName, PropertyType::Enum propertyType)
	{
		ScopedMetaData meta(mMetaDataProvider);
		int32_t dtypeType = meta->findClass(dtypeName)->mClassId;
		NamespacedName typeName = dtypeName;
		if(dtypeType == getPvdTypeForType<String>())
		{
			dtypeType = getPvdTypeForType<StringHandle>();
			typeName = getPvdNamespacedNameForType<StringHandle>();
		}
		Option<PropertyDescription> propOpt =
		    meta->createProperty(meta->findClass(clsName)->mClassId, name, semantic, dtypeType, propertyType);
		PX_ASSERT(propOpt.hasValue());
		PX_UNUSED(propOpt);
		return typeName;
	}

	virtual PvdError createProperty(const NamespacedName& clsName, String name, String semantic,
	                                const NamespacedName& incomingDtypeName, PropertyType::Enum propertyType,
	                                DataRef<NamedValue> values)
	{
		PX_ASSERT(mStreamState == DataStreamState::Open);
#if PX_DEBUG
		PX_ASSERT(isClassExist(clsName));
		PX_ASSERT(propertyExists(clsName, name) == false);
#endif
		NamespacedName dtypeName(incomingDtypeName);
		if(safeStrEq(dtypeName.mName, "VoidPtr"))
			dtypeName.mName = "ObjectRef";
#if PX_DEBUG
		PX_ASSERT(isClassExist(dtypeName));
		PX_ASSERT(isValidPropertyDatatype(dtypeName));
#endif
		NamespacedName typeName = createMetaProperty(clsName, name, semantic, dtypeName, propertyType);
		// Can't have arrays of strings or arrays of string handles due to the difficulty
		// of quickly dealing with them on the network receiving side.
		if(propertyType == PropertyType::Array && safeStrEq(typeName.mName, "StringHandle"))
		{
			PX_ASSERT(false);
			return PvdErrorType::ArgumentError;
		}
		uint32_t numItems = values.size();
		NameHandleValue* streamValues = allocTemp<NameHandleValue>(numItems);
		PVD_FOREACH(idx, numItems)
		streamValues[idx] = NameHandleValue(toStream(values[idx].mName), values[idx].mValue);
		CreateProperty evt(toStream(clsName), toStream(name), toStream(semantic), toStream(typeName), propertyType,
		                   DataRef<NameHandleValue>(streamValues, numItems));
		return boolToError(handlePvdEvent(evt));
	}

	bool createMetaPropertyMessage(const NamespacedName& cls, const NamespacedName& msgName,
	                               DataRef<PropertyMessageArg> entries, uint32_t messageSizeInBytes)
	{
		ScopedMetaData meta(mMetaDataProvider);
		return meta->createPropertyMessage(cls, msgName, entries, messageSizeInBytes).hasValue();
	}
#if PX_DEBUG

	bool messageExists(const NamespacedName& msgName)
	{
		ScopedMetaData meta(mMetaDataProvider);
		return meta->findPropertyMessage(msgName).hasValue();
	}

#endif

	virtual PvdError createPropertyMessage(const NamespacedName& cls, const NamespacedName& msgName,
	                                       DataRef<PropertyMessageArg> entries, uint32_t messageSizeInBytes)
	{
		PX_ASSERT(mStreamState == DataStreamState::Open);
#if PX_DEBUG
		PX_ASSERT(isClassExist(cls));
		PX_ASSERT(messageExists(msgName) == false);
#endif
		createMetaPropertyMessage(cls, msgName, entries, messageSizeInBytes);
		uint32_t numItems = entries.size();
		StreamPropMessageArg* streamValues = allocTemp<StreamPropMessageArg>(numItems);
		PVD_FOREACH(idx, numItems)
		streamValues[idx] =
		    StreamPropMessageArg(toStream(entries[idx].mPropertyName), toStream(entries[idx].mDatatypeName),
		                         entries[idx].mMessageOffset, entries[idx].mByteSize);
		CreatePropertyMessage evt(toStream(cls), toStream(msgName),
		                          DataRef<StreamPropMessageArg>(streamValues, numItems), messageSizeInBytes);
		return boolToError(handlePvdEvent(evt));
	}

	uint64_t toStream(const void* instance)
	{
		return PVD_POINTER_TO_U64(instance);
	}
	virtual PvdError createInstance(const NamespacedName& cls, const void* instance)
	{
		PX_ASSERT(isInstanceValid(instance) == false);
		PX_ASSERT(mStreamState == DataStreamState::Open);
		bool success = mMetaDataProvider.createInstance(cls, instance);
		PX_ASSERT(success);
		(void)success;
		return boolToError(handlePvdEvent(CreateInstance(toStream(cls), toStream(instance))));
	}

	virtual bool isInstanceValid(const void* instance)
	{
		return mMetaDataProvider.isInstanceValid(instance);
	}

#if PX_DEBUG

	// If the property will fit or is already completely in memory
	bool checkPropertyType(const void* instance, String name, const NamespacedName& incomingType)
	{
		int32_t instType = mMetaDataProvider.getInstanceClassType(instance);
		ScopedMetaData meta(mMetaDataProvider);
		Option<PropertyDescription> prop = meta->findProperty(instType, name);
		if(prop.hasValue() == false)
			return false;
		int32_t propType = prop->mDatatype;
		int32_t incomingTypeId = meta->findClass(incomingType)->mClassId;
		if(incomingTypeId != getPvdTypeForType<VoidPtr>())
		{
			MarshalQueryResult result = meta->checkMarshalling(incomingTypeId, propType);
			bool possible = result.needsMarshalling == false || result.canMarshal;
			return possible;
		}
		else
		{
			if(propType != getPvdTypeForType<ObjectRef>())
				return false;
		}
		return true;
	}

#endif

	DataRef<const uint8_t> bufferPropertyValue(ClassDescriptionSizeInfo info, DataRef<const uint8_t> data)
	{
		uint32_t realSize = info.mByteSize;
		uint32_t numItems = data.size() / realSize;
		if(info.mPtrOffsets.size() != 0)
		{
			mSPVBuffer.clear();
			PVD_FOREACH(item, numItems)
			{
				const uint8_t* itemPtr = data.begin() + item * realSize;
				mSPVBuffer.write(itemPtr, realSize);
				PVD_FOREACH(stringIdx, info.mPtrOffsets.size())
				{
					PtrOffset offset(info.mPtrOffsets[stringIdx]);
					if(offset.mOffsetType == PtrOffsetType::VoidPtrOffset)
						continue;
					const char* strPtr;
					physx::intrinsics::memCopy(&strPtr, itemPtr + offset.mOffset, sizeof(char*));
					strPtr = nonNull(strPtr);
					uint32_t len = safeStrLen(strPtr) + 1;
					mSPVBuffer.write(strPtr, len);
				}
			}
			data = DataRef<const uint8_t>(mSPVBuffer.begin(), mSPVBuffer.size());
		}
		return data;
	}

	virtual PvdError setPropertyValue(const void* instance, String name, DataRef<const uint8_t> data,
	                                  const NamespacedName& incomingTypeName)
	{

		PX_ASSERT(isInstanceValid(instance));
#if PX_DEBUG
		PX_ASSERT(isClassExist(incomingTypeName));
#endif
		PX_ASSERT(mStreamState == DataStreamState::Open);
		ClassDescription clsDesc;
		{
			ScopedMetaData meta(mMetaDataProvider);
			clsDesc = meta->findClass(incomingTypeName);
		}
		uint32_t realSize = clsDesc.getNativeSize();
		uint32_t numItems = data.size() / realSize;
		data = bufferPropertyValue(clsDesc.getNativeSizeInfo(), data);
		SetPropertyValue evt(toStream(instance), toStream(name), data, toStream(incomingTypeName), numItems);
		return boolToError(handlePvdEvent(evt));
	}

	// Else if the property is very large (contact reports) you can send it in chunks.
	virtual PvdError beginSetPropertyValue(const void* instance, String name, const NamespacedName& incomingTypeName)
	{
		PX_ASSERT(isInstanceValid(instance));
#if PX_DEBUG
		PX_ASSERT(isClassExist(incomingTypeName));
		PX_ASSERT(checkPropertyType(instance, name, incomingTypeName));
#endif
		PX_ASSERT(mStreamState == DataStreamState::Open);
		mStreamState = DataStreamState::SetPropertyValue;
		{
			ScopedMetaData meta(mMetaDataProvider);
			mSPVClass = meta->findClass(incomingTypeName);
		}
		BeginSetPropertyValue evt(toStream(instance), toStream(name), toStream(incomingTypeName));
		return boolToError(handlePvdEvent(evt));
	}

	virtual PvdError appendPropertyValueData(DataRef<const uint8_t> data)
	{
		uint32_t realSize = mSPVClass.getNativeSize();
		uint32_t numItems = data.size() / realSize;
		data = bufferPropertyValue(mSPVClass.getNativeSizeInfo(), data);
		PX_ASSERT(mStreamState == DataStreamState::SetPropertyValue);
		return boolToError(handlePvdEvent(AppendPropertyValueData(data, numItems)));
	}
	virtual PvdError endSetPropertyValue()
	{
		PX_ASSERT(mStreamState == DataStreamState::SetPropertyValue);
		mStreamState = DataStreamState::Open;
		return boolToError(handlePvdEvent(EndSetPropertyValue()));
	}

#if PX_DEBUG

	bool checkPropertyMessage(const void* instance, const NamespacedName& msgName)
	{
		int32_t clsId = mMetaDataProvider.getInstanceClassType(instance);
		ScopedMetaData meta(mMetaDataProvider);
		PropertyMessageDescription desc(meta->findPropertyMessage(msgName));
		bool retval = meta->isDerivedFrom(clsId, desc.mClassId);
		return retval;
	}

#endif

	DataRef<const uint8_t> bufferPropertyMessage(const PropertyMessageDescription& desc, DataRef<const uint8_t> data)
	{
		if(desc.mStringOffsets.size())
		{
			mSPVBuffer.clear();
			mSPVBuffer.write(data.begin(), data.size());
			PVD_FOREACH(idx, desc.mStringOffsets.size())
			{
				const char* strPtr;
				physx::intrinsics::memCopy(&strPtr, data.begin() + desc.mStringOffsets[idx], sizeof(char*));
				strPtr = nonNull(strPtr);
				uint32_t len = safeStrLen(strPtr) + 1;
				mSPVBuffer.write(strPtr, len);
			}
			data = DataRef<const uint8_t>(mSPVBuffer.begin(), mSPVBuffer.end());
		}
		return data;
	}

	virtual PvdError setPropertyMessage(const void* instance, const NamespacedName& msgName, DataRef<const uint8_t> data)
	{
		ScopedMetaData meta(mMetaDataProvider);
		PX_ASSERT(isInstanceValid(instance));
#if PX_DEBUG
		PX_ASSERT(messageExists(msgName));
		PX_ASSERT(checkPropertyMessage(instance, msgName));
#endif
		PropertyMessageDescription desc(meta->findPropertyMessage(msgName));
		if(data.size() < desc.mMessageByteSize)
		{
			PX_ASSERT(false);
			return PvdErrorType::ArgumentError;
		}
		data = bufferPropertyMessage(desc, data);
		PX_ASSERT(mStreamState == DataStreamState::Open);
		return boolToError(handlePvdEvent(SetPropertyMessage(toStream(instance), toStream(msgName), data)));
	}

#if PX_DEBUG

	bool checkBeginPropertyMessageGroup(const NamespacedName& msgName)
	{
		ScopedMetaData meta(mMetaDataProvider);
		PropertyMessageDescription desc(meta->findPropertyMessage(msgName));
		return desc.mStringOffsets.size() == 0;
	}

#endif
	// If you need to send of lot of identical messages, this avoids a hashtable lookup per message.
	virtual PvdError beginPropertyMessageGroup(const NamespacedName& msgName)
	{
#if PX_DEBUG
		PX_ASSERT(messageExists(msgName));
		PX_ASSERT(checkBeginPropertyMessageGroup(msgName));
#endif
		PX_ASSERT(mStreamState == DataStreamState::Open);
		mStreamState = DataStreamState::PropertyMessageGroup;
		ScopedMetaData meta(mMetaDataProvider);
		mMessageDesc = meta->findPropertyMessage(msgName);
		return boolToError(handlePvdEvent(BeginPropertyMessageGroup(toStream(msgName))));
	}

	virtual PvdError sendPropertyMessageFromGroup(const void* instance, DataRef<const uint8_t> data)
	{
		PX_ASSERT(mStreamState == DataStreamState::PropertyMessageGroup);
		PX_ASSERT(isInstanceValid(instance));
#if PX_DEBUG
		PX_ASSERT(checkPropertyMessage(instance, mMessageDesc.mMessageName));
#endif
		if(mMessageDesc.mMessageByteSize != data.size())
		{
			PX_ASSERT(false);
			return PvdErrorType::ArgumentError;
		}
		if(data.size() < mMessageDesc.mMessageByteSize)
			return PvdErrorType::ArgumentError;
		data = bufferPropertyMessage(mMessageDesc, data);
		return boolToError(handlePvdEvent(SendPropertyMessageFromGroup(toStream(instance), data)));
	}
	virtual PvdError endPropertyMessageGroup()
	{
		PX_ASSERT(mStreamState == DataStreamState::PropertyMessageGroup);
		mStreamState = DataStreamState::Open;
		return boolToError(handlePvdEvent(EndPropertyMessageGroup()));
	}
	virtual PvdError pushBackObjectRef(const void* instance, String propName, const void* data)
	{
		PX_ASSERT(isInstanceValid(instance));
		PX_ASSERT(isInstanceValid(data));
		PX_ASSERT(mStreamState == DataStreamState::Open);
		return boolToError(handlePvdEvent(PushBackObjectRef(toStream(instance), toStream(propName), toStream(data))));
	}
	virtual PvdError removeObjectRef(const void* instance, String propName, const void* data)
	{
		PX_ASSERT(isInstanceValid(instance));
		PX_ASSERT(isInstanceValid(data));
		PX_ASSERT(mStreamState == DataStreamState::Open);
		return boolToError(handlePvdEvent(RemoveObjectRef(toStream(instance), toStream(propName), toStream(data))));
	}
	// Instance elimination.
	virtual PvdError destroyInstance(const void* instance)
	{
		PX_ASSERT(isInstanceValid(instance));
		PX_ASSERT(mStreamState == DataStreamState::Open);
		mMetaDataProvider.destroyInstance(instance);
		return boolToError(handlePvdEvent(DestroyInstance(toStream(instance))));
	}

	// Profiling hooks
	virtual PvdError beginSection(const void* instance, String name)
	{
		PX_ASSERT(mStreamState == DataStreamState::Open);
		return boolToError(handlePvdEvent(
            BeginSection(toStream(instance), toStream(name), Time::getCurrentCounterValue())));
	}

	virtual PvdError endSection(const void* instance, String name)
	{
		PX_ASSERT(mStreamState == DataStreamState::Open);
		return boolToError(handlePvdEvent(
            EndSection(toStream(instance), toStream(name), Time::getCurrentCounterValue())));
	}

	virtual PvdError originShift(const void* scene, PxVec3 shift)
	{
		PX_ASSERT(mStreamState == DataStreamState::Open);
		return boolToError(handlePvdEvent(OriginShift(toStream(scene), shift)));
	}

	virtual void addProfileZone(void* zone, const char* name)
	{
		handlePvdEvent(AddProfileZone(toStream(zone), name));
	}
	virtual void addProfileZoneEvent(void* zone, const char* name, uint16_t eventId, bool compileTimeEnabled)
	{
		handlePvdEvent(AddProfileZoneEvent(toStream(zone), name, eventId, compileTimeEnabled));
	}

	// add a variable sized event
	void addEvent(const EventSerializeable& evt, PvdCommStreamEventTypes::Enum evtType)
	{
		MeasureStream measure;
		PvdCommStreamEventSink::writeStreamEvent(evt, evtType, measure);
        EventGroup evtGroup(measure.mSize, 1, mStreamId, Time::getCurrentCounterValue());
		EventStreamifier<PxPvdTransport> streamifier(mTransport.lock());
		evtGroup.serialize(streamifier);
		PvdCommStreamEventSink::writeStreamEvent(evt, evtType, mTransport);
		mTransport.unlock();
	}

	void setIsTopLevelUIElement(const void* instance, bool topLevel)
	{
		addEvent(SetIsTopLevel(static_cast<uint64_t>(reinterpret_cast<size_t>(instance)), topLevel),
		         getCommStreamEventType<SetIsTopLevel>());
	}

	void sendErrorMessage(uint32_t code, const char* message, const char* file, uint32_t line)
	{
		addEvent(ErrorMessage(code, message, file, line), getCommStreamEventType<ErrorMessage>());
	}

	void updateCamera(const char* name, const PxVec3& origin, const PxVec3& up, const PxVec3& target)
	{
		addEvent(SetCamera(name, origin, up, target), getCommStreamEventType<SetCamera>());
	}

	template <typename TEventType>
	bool handlePvdEvent(const TEventType& evt)
	{
		addEvent(evt, getCommStreamEventType<TEventType>());
		return mConnected;
	}

	virtual PvdPropertyDefinitionHelper& getPropertyDefinitionHelper()
	{
		mPropertyDefinitionHelper.clearBufferedData();
		return mPropertyDefinitionHelper;
	}

	virtual bool isConnected()
	{
		return mConnected;
	}

	virtual void* allocateMemForCmd(uint32_t length)
	{
		return mPvdCommandPool.allocate(length);
	}

	virtual void pushPvdCommand(PvdCommand& cmd)
	{
		mPvdCommandArray.pushBack(&cmd);
	}

	virtual void flushPvdCommand()
	{
		uint32_t cmdQueueSize = mPvdCommandArray.size();
		for(uint32_t i = 0; i < cmdQueueSize; i++)
		{
			if(mPvdCommandArray[i])
			{
				// if(mPvdCommandArray[i]->canRun(*this))
				mPvdCommandArray[i]->run(*this);
				mPvdCommandArray[i]->~PvdCommand();
			}
		}
		mPvdCommandArray.clear();
		mPvdCommandPool.clear();
	}

	PX_NOCOPY(PvdOutStream)
};
}

PvdDataStream* PvdDataStream::create(PxPvd* pvd)
{
	if(pvd == NULL)
	{
        getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "PvdDataStream::create - pvd must be non-NULL!");
	    return NULL;
	}

	PvdImpl* pvdImpl = static_cast<PvdImpl*>(pvd);
	return PVD_NEW(PvdOutStream)(*pvdImpl->getTransport(), pvdImpl->getMetaDataProvider(), pvdImpl->getNextStreamId());
}
