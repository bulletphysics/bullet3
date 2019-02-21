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

#include "PxPvdObjectModelInternalTypes.h"
#include "PxPvdObjectModelMetaData.h"
#include "PxPvdInternalByteStreams.h"
#include "PxPvdMarshalling.h"

using namespace physx;
using namespace pvdsdk;
using namespace shdfnd;

namespace
{

struct PropDescImpl : public PropertyDescription, public UserAllocated
{
    Array<NamedValue> mValueNames;
	PropDescImpl(const PropertyDescription& inBase, StringTable& table)
	: PropertyDescription(inBase), mValueNames("NamedValue")
	{
		mName = table.registerStr(mName);
	}
	PropDescImpl() : mValueNames("NamedValue")
	{
	}

	template <typename TSerializer>
	void serialize(TSerializer& serializer)
	{
		serializer.streamify(mOwnerClassName);
		serializer.streamify(mOwnerClassId);
		serializer.streamify(mSemantic);
		serializer.streamify(mDatatype);
		serializer.streamify(mDatatypeName);
		serializer.streamify(mPropertyType);
		serializer.streamify(mPropertyId);
		serializer.streamify(m32BitOffset);
		serializer.streamify(m64BitOffset);
		serializer.streamify(mValueNames);
		serializer.streamify(mName);
	}
};

struct ClassDescImpl : public ClassDescription, public UserAllocated
{
    Array<PropDescImpl*> mPropImps;
    Array<PtrOffset> m32OffsetArray;
	Array<PtrOffset> m64OffsetArray;
	ClassDescImpl(const ClassDescription& inBase)
	: ClassDescription(inBase)
	, mPropImps("PropDescImpl*")
	, m32OffsetArray("ClassDescImpl::m32OffsetArray")
	, m64OffsetArray("ClassDescImpl::m64OffsetArray")
	{
		PVD_FOREACH(idx, get32BitSizeInfo().mPtrOffsets.size())
		m32OffsetArray.pushBack(get32BitSizeInfo().mPtrOffsets[idx]);
		PVD_FOREACH(idx, get64BitSizeInfo().mPtrOffsets.size())
		m64OffsetArray.pushBack(get64BitSizeInfo().mPtrOffsets[idx]);
	}
	ClassDescImpl()
	: mPropImps("PropDescImpl*")
	, m32OffsetArray("ClassDescImpl::m32OffsetArray")
	, m64OffsetArray("ClassDescImpl::m64OffsetArray")
	{
	}
	PropDescImpl* findProperty(String name)
	{
		PVD_FOREACH(idx, mPropImps.size())
		{
			if(safeStrEq(mPropImps[idx]->mName, name))
				return mPropImps[idx];
		}
		return NULL;
	}
	void addProperty(PropDescImpl* prop)
	{
		mPropImps.pushBack(prop);
	}

	void addPtrOffset(PtrOffsetType::Enum type, uint32_t offset32, uint32_t offset64)
	{
		m32OffsetArray.pushBack(PtrOffset(type, offset32));
		m64OffsetArray.pushBack(PtrOffset(type, offset64));
		get32BitSizeInfo().mPtrOffsets = DataRef<PtrOffset>(m32OffsetArray.begin(), m32OffsetArray.end());
		get64BitSizeInfo().mPtrOffsets = DataRef<PtrOffset>(m64OffsetArray.begin(), m64OffsetArray.end());
	}

	template <typename TSerializer>
	void serialize(TSerializer& serializer)
	{
		serializer.streamify(mName);
		serializer.streamify(mClassId);
		serializer.streamify(mBaseClass);
		serializer.streamify(mPackedUniformWidth);
		serializer.streamify(mPackedClassType);
		serializer.streamify(mLocked);
		serializer.streamify(mRequiresDestruction);
		serializer.streamify(get32BitSize());
		serializer.streamify(get32BitSizeInfo().mDataByteSize);
		serializer.streamify(get32BitSizeInfo().mAlignment);
		serializer.streamify(get64BitSize());
		serializer.streamify(get64BitSizeInfo().mDataByteSize);
		serializer.streamify(get64BitSizeInfo().mAlignment);
		serializer.streamifyLinks(mPropImps);
		serializer.streamify(m32OffsetArray);
		serializer.streamify(m64OffsetArray);
		get32BitSizeInfo().mPtrOffsets = DataRef<PtrOffset>(m32OffsetArray.begin(), m32OffsetArray.end());
		get64BitSizeInfo().mPtrOffsets = DataRef<PtrOffset>(m64OffsetArray.begin(), m64OffsetArray.end());
	}
};

class StringTableImpl : public StringTable, public UserAllocated
{
	HashMap<const char*, char*> mStrings;
	uint32_t mNextStrHandle;
	HashMap<uint32_t, char*> mHandleToStr;
	HashMap<const char*, uint32_t> mStrToHandle;

  public:
	StringTableImpl()
	: mStrings("StringTableImpl::mStrings")
	, mNextStrHandle(1)
	, mHandleToStr("StringTableImpl::mHandleToStr")
	, mStrToHandle("StringTableImpl::mStrToHandle")
	{
	}
	uint32_t nextHandleValue()
	{
		return mNextStrHandle++;
	}
	virtual ~StringTableImpl()
	{
		for(HashMap<const char*, char*>::Iterator iter = mStrings.getIterator(); !iter.done(); ++iter)
			PX_FREE(iter->second);
		mStrings.clear();
	}
	virtual uint32_t getNbStrs()
	{
		return mStrings.size();
	}
	virtual uint32_t getStrs(const char** outStrs, uint32_t bufLen, uint32_t startIdx = 0)
	{
		startIdx = PxMin(getNbStrs(), startIdx);
		uint32_t numStrs(PxMin(getNbStrs() - startIdx, bufLen));
		HashMap<const char*, char*>::Iterator iter(mStrings.getIterator());
		for(uint32_t idx = 0; idx < startIdx; ++idx, ++iter)
			;
		for(uint32_t idx = 0; idx < numStrs && !iter.done(); ++idx, ++iter)
			outStrs[idx] = iter->second;
		return numStrs;
	}
	void addStringHandle(char* str, uint32_t hdl)
	{
		mHandleToStr.insert(hdl, str);
		mStrToHandle.insert(str, hdl);
	}

	uint32_t addStringHandle(char* str)
	{
		uint32_t theNewHandle = nextHandleValue();
		addStringHandle(str, theNewHandle);
		return theNewHandle;
	}
	const char* doRegisterStr(const char* str, bool& outAdded)
	{
		PX_ASSERT(isMeaningful(str));
		const HashMap<const char*, char*>::Entry* entry(mStrings.find(str));
        if(entry == NULL)
		{
			outAdded = true;
			char* retval(copyStr(str));
			mStrings.insert(retval, retval);
			return retval;
		}
		return entry->second;
	}
	virtual const char* registerStr(const char* str, bool& outAdded)
	{
		outAdded = false;
		if(isMeaningful(str) == false)
			return "";
		const char* retval = doRegisterStr(str, outAdded);
		if(outAdded)
			addStringHandle(const_cast<char*>(retval));
		return retval;
	}

	NamespacedName registerName(const NamespacedName& nm)
	{
		return NamespacedName(registerStr(nm.mNamespace), registerStr(nm.mName));
	}
	const char* registerStr(const char* str)
	{
		bool ignored;
		return registerStr(str, ignored);
	}

	virtual StringHandle strToHandle(const char* str)
	{
		if(isMeaningful(str) == false)
			return 0;
		const HashMap<const char*, uint32_t>::Entry* entry(mStrToHandle.find(str));
		if(entry)
			return entry->second;
		bool added = false;
		const char* registeredStr = doRegisterStr(str, added);
		uint32_t theNewHandle = addStringHandle(const_cast<char*>(registeredStr));
		PX_ASSERT(mStrToHandle.find(str));
		PX_ASSERT(added);
		return theNewHandle;
	}

	virtual const char* handleToStr(uint32_t hdl)
	{
		if(hdl == 0)
			return "";
		const HashMap<uint32_t, char*>::Entry* entry(mHandleToStr.find(hdl));
		if(entry)
			return entry->second;
		// unregistered handle...
		return "";
	}

	void write(PvdOutputStream& stream)
	{
		uint32_t numStrs = static_cast<uint32_t>(mHandleToStr.size());
		stream << numStrs;
		stream << mNextStrHandle;
		for(HashMap<uint32_t, char*>::Iterator iter = mHandleToStr.getIterator(); !iter.done(); ++iter)
		{
			stream << iter->first;
			uint32_t len = static_cast<uint32_t>(strlen(iter->second) + 1);
			stream << len;
			stream.write(reinterpret_cast<uint8_t*>(iter->second), len);
		}
	}

	template <typename TReader>
	void read(TReader& stream)
	{
		mHandleToStr.clear();
		mStrToHandle.clear();
		uint32_t numStrs;
		stream >> numStrs;
		stream >> mNextStrHandle;
		Array<uint8_t> readBuffer("StringTable::read::readBuffer");
		uint32_t bufSize = 0;
		for(uint32_t idx = 0; idx < numStrs; ++idx)
		{
			uint32_t handleValue;
			uint32_t bufLen;
			stream >> handleValue;
			stream >> bufLen;
			if(bufSize < bufLen)
				readBuffer.resize(bufLen);
			bufSize = PxMax(bufSize, bufLen);
			stream.read(readBuffer.begin(), bufLen);
			bool ignored;
			const char* newStr = doRegisterStr(reinterpret_cast<const char*>(readBuffer.begin()), ignored);
			addStringHandle(const_cast<char*>(newStr), handleValue);
		}
	}

	virtual void release()
	{
		PVD_DELETE(this);
	}

  private:
	StringTableImpl& operator=(const StringTableImpl&);
};

struct NamespacedNameHasher
{
	uint32_t operator()(const NamespacedName& nm)
	{
		return Hash<const char*>()(nm.mNamespace) ^ Hash<const char*>()(nm.mName);
	}
	bool equal(const NamespacedName& lhs, const NamespacedName& rhs)
	{
		return safeStrEq(lhs.mNamespace, rhs.mNamespace) && safeStrEq(lhs.mName, rhs.mName);
	}
};

struct ClassPropertyName
{
	NamespacedName mName;
	String mPropName;
	ClassPropertyName(const NamespacedName& name = NamespacedName(), String propName = "")
	: mName(name), mPropName(propName)
	{
	}
};

struct ClassPropertyNameHasher
{
	uint32_t operator()(const ClassPropertyName& nm)
	{
		return NamespacedNameHasher()(nm.mName) ^ Hash<const char*>()(nm.mPropName);
	}
	bool equal(const ClassPropertyName& lhs, const ClassPropertyName& rhs)
	{
		return NamespacedNameHasher().equal(lhs.mName, rhs.mName) && safeStrEq(lhs.mPropName, rhs.mPropName);
	}
};

struct PropertyMessageEntryImpl : public PropertyMessageEntry
{
	PropertyMessageEntryImpl(const PropertyMessageEntry& data) : PropertyMessageEntry(data)
	{
	}
	PropertyMessageEntryImpl()
	{
	}
	template <typename TSerializerType>
	void serialize(TSerializerType& serializer)
	{
		serializer.streamify(mDatatypeName);
		serializer.streamify(mDatatypeId);
		serializer.streamify(mMessageOffset);
		serializer.streamify(mByteSize);
		serializer.streamify(mDestByteSize);
		serializer.streamify(mProperty);
	}
};

struct PropertyMessageDescriptionImpl : public PropertyMessageDescription, public UserAllocated
{
	Array<PropertyMessageEntryImpl> mEntryImpls;
	Array<PropertyMessageEntry> mEntries;
	Array<uint32_t> mStringOffsetArray;
	PropertyMessageDescriptionImpl(const PropertyMessageDescription& data)
	: PropertyMessageDescription(data)
	, mEntryImpls("PropertyMessageDescriptionImpl::mEntryImpls")
	, mEntries("PropertyMessageDescriptionImpl::mEntries")
	, mStringOffsetArray("PropertyMessageDescriptionImpl::mStringOffsets")
	{
	}
	PropertyMessageDescriptionImpl()
	: mEntryImpls("PropertyMessageDescriptionImpl::mEntryImpls")
	, mEntries("PropertyMessageDescriptionImpl::mEntries")
	, mStringOffsetArray("PropertyMessageDescriptionImpl::mStringOffsets")
	{
	}

	~PropertyMessageDescriptionImpl()
	{
	}

	void addEntry(const PropertyMessageEntryImpl& entry)
	{
		mEntryImpls.pushBack(entry);
		mEntries.pushBack(entry);
		mProperties = DataRef<PropertyMessageEntry>(mEntries.begin(), mEntries.end());
	}

	template <typename TSerializerType>
	void serialize(TSerializerType& serializer)
	{
		serializer.streamify(mClassName);
		serializer.streamify(mClassId); // No other class has this id, it is DB-unique
		serializer.streamify(mMessageName);
		serializer.streamify(mMessageId);
		serializer.streamify(mMessageByteSize);
		serializer.streamify(mEntryImpls);
		serializer.streamify(mStringOffsetArray);
		if(mEntries.size() != mEntryImpls.size())
		{
			mEntries.clear();
			uint32_t numEntries = static_cast<uint32_t>(mEntryImpls.size());
			for(uint32_t idx = 0; idx < numEntries; ++idx)
				mEntries.pushBack(mEntryImpls[idx]);
		}
		mProperties = DataRef<PropertyMessageEntry>(mEntries.begin(), mEntries.end());
		mStringOffsets = DataRef<uint32_t>(mStringOffsetArray.begin(), mStringOffsetArray.end());
	}

  private:
	PropertyMessageDescriptionImpl& operator=(const PropertyMessageDescriptionImpl&);
};

struct PvdObjectModelMetaDataImpl : public PvdObjectModelMetaData, public UserAllocated
{
	typedef HashMap<NamespacedName, ClassDescImpl*, NamespacedNameHasher> TNameToClassMap;
	typedef HashMap<ClassPropertyName, PropDescImpl*, ClassPropertyNameHasher> TNameToPropMap;
	typedef HashMap<NamespacedName, PropertyMessageDescriptionImpl*, NamespacedNameHasher> TNameToPropertyMessageMap;

	TNameToClassMap mNameToClasses;
	TNameToPropMap mNameToProperties;
	Array<ClassDescImpl*> mClasses;
	Array<PropDescImpl*> mProperties;
    StringTableImpl* mStringTable;
	TNameToPropertyMessageMap mPropertyMessageMap;
	Array<PropertyMessageDescriptionImpl*> mPropertyMessages;
	int32_t mNextClassId;
	uint32_t mRefCount;

	PvdObjectModelMetaDataImpl()
	: mNameToClasses("NamespacedName->ClassDescImpl*")
	, mNameToProperties("ClassPropertyName->PropDescImpl*")
	, mClasses("ClassDescImpl*")
	, mProperties("PropDescImpl*")
    , mStringTable(PVD_NEW(StringTableImpl)())
	, mPropertyMessageMap("PropertyMessageMap")
	, mPropertyMessages("PvdObjectModelMetaDataImpl::mPropertyMessages")
	, mNextClassId(1)
	, mRefCount(0)
	{
	}

  private:
	PvdObjectModelMetaDataImpl& operator=(const PvdObjectModelMetaDataImpl&);

  public:
	int32_t nextClassId()
	{
		return mNextClassId++;
	}
	void initialize()
	{
		// Create the default classes.
		{
			ClassDescImpl& aryData = getOrCreateClassImpl(getPvdNamespacedNameForType<ArrayData>(),
			                                              DataTypeToPvdTypeMap<ArrayData>::BaseTypeEnum);
			aryData.get32BitSize() = sizeof(ArrayData);
			aryData.get32BitSizeInfo().mAlignment = sizeof(void*);
			aryData.get64BitSize() = sizeof(ArrayData);
			aryData.get64BitSizeInfo().mAlignment = sizeof(void*);
			aryData.mLocked = true;
		}
#define CREATE_BASIC_PVD_CLASS(type)                                                                                   \
	{                                                                                                                  \
		ClassDescImpl& cls = getOrCreateClassImpl(getPvdNamespacedNameForType<type>(), getPvdTypeForType<type>());     \
		cls.get32BitSize() = sizeof(type);                                                                             \
		cls.get32BitSizeInfo().mAlignment = sizeof(type);                                                              \
		cls.get64BitSize() = sizeof(type);                                                                             \
		cls.get64BitSizeInfo().mAlignment = sizeof(type);                                                              \
		cls.mLocked = true;                                                                                            \
		cls.mPackedUniformWidth = sizeof(type);                                                                        \
		cls.mPackedClassType = getPvdTypeForType<type>();                                                              \
	}
		CREATE_BASIC_PVD_CLASS(int8_t)
		CREATE_BASIC_PVD_CLASS(uint8_t)
		CREATE_BASIC_PVD_CLASS(bool)
		CREATE_BASIC_PVD_CLASS(int16_t)
		CREATE_BASIC_PVD_CLASS(uint16_t)
		CREATE_BASIC_PVD_CLASS(int32_t)
		CREATE_BASIC_PVD_CLASS(uint32_t)
		// CREATE_BASIC_PVD_CLASS(uint32_t)
		CREATE_BASIC_PVD_CLASS(int64_t)
		CREATE_BASIC_PVD_CLASS(uint64_t)
		CREATE_BASIC_PVD_CLASS(float)
		CREATE_BASIC_PVD_CLASS(double)
#undef CREATE_BASIC_PVD_CLASS

#define CREATE_PTR_TYPE_PVD_CLASS(type, ptrType)                                                                       \
	{                                                                                                                  \
		ClassDescImpl& cls = getOrCreateClassImpl(getPvdNamespacedNameForType<type>(), getPvdTypeForType<type>());     \
		cls.get32BitSize() = 4;                                                                                        \
		cls.get32BitSizeInfo().mAlignment = 4;                                                                         \
		cls.get64BitSize() = 8;                                                                                        \
		cls.get64BitSizeInfo().mAlignment = 8;                                                                         \
		cls.mLocked = true;                                                                                            \
		cls.addPtrOffset(PtrOffsetType::ptrType, 0, 0);                                                                \
	}

		CREATE_PTR_TYPE_PVD_CLASS(String, StringOffset)
		CREATE_PTR_TYPE_PVD_CLASS(VoidPtr, VoidPtrOffset)
		CREATE_PTR_TYPE_PVD_CLASS(StringHandle, StringOffset)
		CREATE_PTR_TYPE_PVD_CLASS(ObjectRef, VoidPtrOffset)

#undef CREATE_64BIT_ADJUST_PVD_CLASS

		int32_t fltClassType = getPvdTypeForType<float>();
		int32_t u32ClassType = getPvdTypeForType<uint32_t>();
		int32_t v3ClassType = getPvdTypeForType<PxVec3>();
		int32_t v4ClassType = getPvdTypeForType<PxVec4>();
		int32_t qtClassType = getPvdTypeForType<PxQuat>();
		{
			ClassDescImpl& cls =
			    getOrCreateClassImpl(getPvdNamespacedNameForType<PvdColor>(), getPvdTypeForType<PvdColor>());
			createProperty(cls.mClassId, "r", "", getPvdTypeForType<uint8_t>(), PropertyType::Scalar);
			createProperty(cls.mClassId, "g", "", getPvdTypeForType<uint8_t>(), PropertyType::Scalar);
			createProperty(cls.mClassId, "b", "", getPvdTypeForType<uint8_t>(), PropertyType::Scalar);
			createProperty(cls.mClassId, "a", "", getPvdTypeForType<uint8_t>(), PropertyType::Scalar);
			PX_ASSERT(cls.get32BitSizeInfo().mAlignment == 1);
			PX_ASSERT(cls.get32BitSize() == 4);
			PX_ASSERT(cls.get64BitSizeInfo().mAlignment == 1);
			PX_ASSERT(cls.get64BitSize() == 4);
			PX_ASSERT(cls.mPackedUniformWidth == 1);
			PX_ASSERT(cls.mPackedClassType == getPvdTypeForType<uint8_t>());
			cls.mLocked = true;
		}

		{
			ClassDescImpl& cls = getOrCreateClassImpl(getPvdNamespacedNameForType<PxVec2>(), getPvdTypeForType<PxVec2>());
			createProperty(cls.mClassId, "x", "", fltClassType, PropertyType::Scalar);
			createProperty(cls.mClassId, "y", "", fltClassType, PropertyType::Scalar);
			PX_ASSERT(cls.get32BitSizeInfo().mAlignment == 4);
			PX_ASSERT(cls.get32BitSize() == 8);
			PX_ASSERT(cls.get64BitSizeInfo().mAlignment == 4);
			PX_ASSERT(cls.get64BitSize() == 8);
			PX_ASSERT(cls.mPackedUniformWidth == 4);
			PX_ASSERT(cls.mPackedClassType == fltClassType);
			cls.mLocked = true;
		}
		{
			ClassDescImpl& cls = getOrCreateClassImpl(getPvdNamespacedNameForType<PxVec3>(), getPvdTypeForType<PxVec3>());
			createProperty(cls.mClassId, "x", "", fltClassType, PropertyType::Scalar);
			createProperty(cls.mClassId, "y", "", fltClassType, PropertyType::Scalar);
			createProperty(cls.mClassId, "z", "", fltClassType, PropertyType::Scalar);
			PX_ASSERT(cls.get32BitSizeInfo().mAlignment == 4);
			PX_ASSERT(cls.get32BitSize() == 12);
			PX_ASSERT(cls.get64BitSizeInfo().mAlignment == 4);
			PX_ASSERT(cls.get64BitSize() == 12);
			PX_ASSERT(cls.mPackedUniformWidth == 4);
			PX_ASSERT(cls.mPackedClassType == fltClassType);
			cls.mLocked = true;
		}
		{
			ClassDescImpl& cls = getOrCreateClassImpl(getPvdNamespacedNameForType<PxVec4>(), getPvdTypeForType<PxVec4>());
			createProperty(cls.mClassId, "x", "", fltClassType, PropertyType::Scalar);
			createProperty(cls.mClassId, "y", "", fltClassType, PropertyType::Scalar);
			createProperty(cls.mClassId, "z", "", fltClassType, PropertyType::Scalar);
			createProperty(cls.mClassId, "w", "", fltClassType, PropertyType::Scalar);
			PX_ASSERT(cls.get32BitSizeInfo().mAlignment == 4);
			PX_ASSERT(cls.get32BitSize() == 16);
			PX_ASSERT(cls.get64BitSizeInfo().mAlignment == 4);
			PX_ASSERT(cls.get64BitSize() == 16);
			PX_ASSERT(cls.mPackedUniformWidth == 4);
			PX_ASSERT(cls.mPackedClassType == fltClassType);
			cls.mLocked = true;
		}

		{
			ClassDescImpl& cls = getOrCreateClassImpl(getPvdNamespacedNameForType<PxQuat>(), getPvdTypeForType<PxQuat>());
			createProperty(cls.mClassId, "x", "", fltClassType, PropertyType::Scalar);
			createProperty(cls.mClassId, "y", "", fltClassType, PropertyType::Scalar);
			createProperty(cls.mClassId, "z", "", fltClassType, PropertyType::Scalar);
			createProperty(cls.mClassId, "w", "", fltClassType, PropertyType::Scalar);
			PX_ASSERT(cls.get32BitSizeInfo().mAlignment == 4);
			PX_ASSERT(cls.get32BitSize() == 16);
			PX_ASSERT(cls.get64BitSizeInfo().mAlignment == 4);
			PX_ASSERT(cls.get64BitSize() == 16);
			PX_ASSERT(cls.mPackedUniformWidth == 4);
			PX_ASSERT(cls.mPackedClassType == fltClassType);
			cls.mLocked = true;
		}

		{
			ClassDescImpl& cls =
			    getOrCreateClassImpl(getPvdNamespacedNameForType<PxBounds3>(), getPvdTypeForType<PxBounds3>());
			createProperty(cls.mClassId, "minimum", "", v3ClassType, PropertyType::Scalar);
			createProperty(cls.mClassId, "maximum", "", v3ClassType, PropertyType::Scalar);
			PX_ASSERT(cls.get32BitSizeInfo().mAlignment == 4);
			PX_ASSERT(cls.get32BitSize() == 24);
			PX_ASSERT(cls.mPackedUniformWidth == 4);
			PX_ASSERT(cls.mPackedClassType == fltClassType);
			cls.mLocked = true;
		}

		{
			ClassDescImpl& cls =
			    getOrCreateClassImpl(getPvdNamespacedNameForType<PxTransform>(), getPvdTypeForType<PxTransform>());
			createProperty(cls.mClassId, "q", "", qtClassType, PropertyType::Scalar);
			createProperty(cls.mClassId, "p", "", v3ClassType, PropertyType::Scalar);
			PX_ASSERT(cls.get32BitSizeInfo().mAlignment == 4);
			PX_ASSERT(cls.get32BitSize() == 28);
			PX_ASSERT(cls.mPackedUniformWidth == 4);
			PX_ASSERT(cls.mPackedClassType == fltClassType);
			cls.mLocked = true;
		}

		{
			ClassDescImpl& cls =
			    getOrCreateClassImpl(getPvdNamespacedNameForType<PxMat33>(), getPvdTypeForType<PxMat33>());
			createProperty(cls.mClassId, "column0", "", v3ClassType, PropertyType::Scalar);
			createProperty(cls.mClassId, "column1", "", v3ClassType, PropertyType::Scalar);
			createProperty(cls.mClassId, "column2", "", v3ClassType, PropertyType::Scalar);
			PX_ASSERT(cls.get32BitSizeInfo().mAlignment == 4);
			PX_ASSERT(cls.get32BitSize() == 36);
			PX_ASSERT(cls.mPackedUniformWidth == 4);
			PX_ASSERT(cls.mPackedClassType == fltClassType);
			cls.mLocked = true;
		}

		{
			ClassDescImpl& cls =
			    getOrCreateClassImpl(getPvdNamespacedNameForType<PxMat44>(), getPvdTypeForType<PxMat44>());
			createProperty(cls.mClassId, "column0", "", v4ClassType, PropertyType::Scalar);
			createProperty(cls.mClassId, "column1", "", v4ClassType, PropertyType::Scalar);
			createProperty(cls.mClassId, "column2", "", v4ClassType, PropertyType::Scalar);
			createProperty(cls.mClassId, "column3", "", v4ClassType, PropertyType::Scalar);
			PX_ASSERT(cls.get32BitSizeInfo().mAlignment == 4);
			PX_ASSERT(cls.get32BitSize() == 64);
			PX_ASSERT(cls.mPackedUniformWidth == 4);
			PX_ASSERT(cls.mPackedClassType == fltClassType);
			cls.mLocked = true;
		}

		{
			ClassDescImpl& cls =
			    getOrCreateClassImpl(getPvdNamespacedNameForType<U32Array4>(), getPvdTypeForType<U32Array4>());
			createProperty(cls.mClassId, "d0", "", u32ClassType, PropertyType::Scalar);
			createProperty(cls.mClassId, "d1", "", u32ClassType, PropertyType::Scalar);
			createProperty(cls.mClassId, "d2", "", u32ClassType, PropertyType::Scalar);
			createProperty(cls.mClassId, "d3", "", u32ClassType, PropertyType::Scalar);
			cls.mLocked = true;
		}
	}
	virtual ~PvdObjectModelMetaDataImpl()
	{
        mStringTable->release();
		PVD_FOREACH(idx, mClasses.size())
		{
			if(mClasses[idx] != NULL)
				PVD_DELETE(mClasses[idx]);
		}
		mClasses.clear();
		PVD_FOREACH(idx, mProperties.size()) PVD_DELETE(mProperties[idx]);
		mProperties.clear();
		PVD_FOREACH(idx, mPropertyMessages.size()) PVD_DELETE(mPropertyMessages[idx]);
		mPropertyMessages.clear();
	}

	ClassDescImpl& getOrCreateClassImpl(const NamespacedName& nm, int32_t idx)
	{
		ClassDescImpl* impl(getClassImpl(idx));
		if(impl)
			return *impl;
        NamespacedName safeName(mStringTable->registerStr(nm.mNamespace), mStringTable->registerStr(nm.mName));
		while(idx >= int32_t(mClasses.size()))
			mClasses.pushBack(NULL);
		mClasses[uint32_t(idx)] = PVD_NEW(ClassDescImpl)(ClassDescription(safeName, idx));
		mNameToClasses.insert(nm, mClasses[uint32_t(idx)]);
		mNextClassId = PxMax(mNextClassId, idx + 1);
		return *mClasses[uint32_t(idx)];
	}

	ClassDescImpl& getOrCreateClassImpl(const NamespacedName& nm)
	{
		ClassDescImpl* retval = findClassImpl(nm);
		if(retval)
			return *retval;
		return getOrCreateClassImpl(nm, nextClassId());
	}
	virtual ClassDescription getOrCreateClass(const NamespacedName& nm)
	{
		return getOrCreateClassImpl(nm);
	}
	// get or create parent, lock parent. deriveFrom getOrCreatechild.
	virtual bool deriveClass(const NamespacedName& parent, const NamespacedName& child)
	{
		ClassDescImpl& p(getOrCreateClassImpl(parent));
		ClassDescImpl& c(getOrCreateClassImpl(child));

		if(c.mBaseClass >= 0)
		{
			PX_ASSERT(c.mBaseClass == p.mClassId);
			return false;
		}
		p.mLocked = true;
		c.mBaseClass = p.mClassId;
		c.get32BitSizeInfo() = p.get32BitSizeInfo();
		c.get64BitSizeInfo() = p.get64BitSizeInfo();
		c.mPackedClassType = p.mPackedClassType;
		c.mPackedUniformWidth = p.mPackedUniformWidth;
		c.mRequiresDestruction = p.mRequiresDestruction;
		c.m32OffsetArray = p.m32OffsetArray;
		c.m64OffsetArray = p.m64OffsetArray;
		// Add all the parent propertes to this class in the global name map.
		for(ClassDescImpl* parent0 = &p; parent0 != NULL; parent0 = getClassImpl(parent0->mBaseClass))
		{
			PVD_FOREACH(idx, parent0->mPropImps.size())
			mNameToProperties.insert(ClassPropertyName(c.mName, parent0->mPropImps[idx]->mName), parent0->mPropImps[idx]);

			if(parent0->mBaseClass < 0)
				break;
		}

		return true;
	}
	ClassDescImpl* findClassImpl(const NamespacedName& nm) const
	{
		const TNameToClassMap::Entry* entry(mNameToClasses.find(nm));
		if(entry)
			return entry->second;
		return NULL;
	}
	virtual Option<ClassDescription> findClass(const NamespacedName& nm) const
	{
		ClassDescImpl* retval = findClassImpl(nm);
		if(retval)
			return *retval;
		return Option<ClassDescription>();
	}

	ClassDescImpl* getClassImpl(int32_t classId) const
	{
		if(classId < 0)
            return NULL;
		uint32_t idx = uint32_t(classId);
		if(idx < mClasses.size())
			return mClasses[idx];
        return NULL;
	}

	virtual Option<ClassDescription> getClass(int32_t classId) const
	{
		ClassDescImpl* impl(getClassImpl(classId));
		if(impl)
			return *impl;
		return None();
	}

	virtual ClassDescription* getClassPtr(int32_t classId) const
	{
		return getClassImpl(classId);
	}

	virtual Option<ClassDescription> getParentClass(int32_t classId) const
	{
		ClassDescImpl* impl(getClassImpl(classId));
		if(impl == NULL)
			return None();
		return getClass(impl->mBaseClass);
	}

	virtual void lockClass(int32_t classId)
	{
		ClassDescImpl* impl(getClassImpl(classId));
		PX_ASSERT(impl);
		if(impl)
			impl->mLocked = true;
	}
	virtual uint32_t getNbClasses() const
	{
		uint32_t total = 0;
		PVD_FOREACH(idx, mClasses.size()) if(mClasses[idx])++ total;
		return total;
	}

	virtual uint32_t getClasses(ClassDescription* outClasses, uint32_t requestCount, uint32_t startIndex = 0) const
	{
		uint32_t classCount(getNbClasses());
		startIndex = PxMin(classCount, startIndex);
		uint32_t retAmount = PxMin(requestCount, classCount - startIndex);

		uint32_t idx = 0;
		while(startIndex)
		{
			if(mClasses[idx] != NULL)
				--startIndex;
			++idx;
		}

		uint32_t inserted = 0;
		uint32_t classesSize = static_cast<uint32_t>(mClasses.size());
		while(inserted < retAmount && idx < classesSize)
		{
			if(mClasses[idx] != NULL)
			{
				outClasses[inserted] = *mClasses[idx];
				++inserted;
			}
			++idx;
		}
		return inserted;
	}

	uint32_t updateByteSizeAndGetPropertyAlignment(ClassDescriptionSizeInfo& dest, const ClassDescriptionSizeInfo& src)
	{
		uint32_t alignment = src.mAlignment;
		dest.mAlignment = PxMax(dest.mAlignment, alignment);
		uint32_t offset = align(dest.mDataByteSize, alignment);
		dest.mDataByteSize = offset + src.mByteSize;
		dest.mByteSize = align(dest.mDataByteSize, dest.mAlignment);
		return offset;
	}

	void transferPtrOffsets(ClassDescriptionSizeInfo& destInfo, Array<PtrOffset>& destArray,
	                        const Array<PtrOffset>& src, uint32_t offset)
	{
		PVD_FOREACH(idx, src.size())
		destArray.pushBack(PtrOffset(src[idx].mOffsetType, src[idx].mOffset + offset));
		destInfo.mPtrOffsets = DataRef<PtrOffset>(destArray.begin(), destArray.end());
	}

	virtual Option<PropertyDescription> createProperty(int32_t classId, String name, String semantic, int32_t datatype,
	                                                   PropertyType::Enum propertyType)
	{
		ClassDescImpl* cls(getClassImpl(classId));
		PX_ASSERT(cls);
		if(!cls)
			return None();
		if(cls->mLocked)
		{
            PX_ASSERT(false);
			return None();
		}
		PropDescImpl* impl(cls->findProperty(name));
		// duplicate property definition
		if(impl)
		{
			PX_ASSERT(false);
			return None();
		}
		if(datatype == getPvdTypeForType<String>())
		{
			PX_ASSERT(false);
			return None();
		}
		// The datatype for this property has not been declared.
		ClassDescImpl* propDType(getClassImpl(datatype));
		PX_ASSERT(propDType);
		if(!propDType)
			return None();
		NamespacedName propClsName(propDType->mName);
		int32_t propPackedWidth = propDType->mPackedUniformWidth;
		int32_t propPackedType = propDType->mPackedClassType;
		// The implications of properties being complex types aren't major
		//*until* you start trying to undue a property event that set values
		// of those complex types.  Then things just get too complex.
		if(propDType->mRequiresDestruction)
		{
			PX_ASSERT(false);
			return None();
		}
		bool requiresDestruction = propDType->mRequiresDestruction || cls->mRequiresDestruction;

		if(propertyType == PropertyType::Array)
		{
			int32_t tempId = DataTypeToPvdTypeMap<ArrayData>::BaseTypeEnum;
			propDType = getClassImpl(tempId);
			PX_ASSERT(propDType);
			if(!propDType)
				return None();
			requiresDestruction = true;
		}
		uint32_t offset32 = updateByteSizeAndGetPropertyAlignment(cls->get32BitSizeInfo(), propDType->get32BitSizeInfo());
		uint32_t offset64 = updateByteSizeAndGetPropertyAlignment(cls->get64BitSizeInfo(), propDType->get64BitSizeInfo());
		transferPtrOffsets(cls->get32BitSizeInfo(), cls->m32OffsetArray, propDType->m32OffsetArray, offset32);
		transferPtrOffsets(cls->get64BitSizeInfo(), cls->m64OffsetArray, propDType->m64OffsetArray, offset64);
		propDType->mLocked = true; // Can't add members to the property type.
		cls->mRequiresDestruction = requiresDestruction;
		int32_t propId = int32_t(mProperties.size());
		PropertyDescription newDesc(cls->mName, cls->mClassId, name, semantic, datatype, propClsName, propertyType,
		                            propId, offset32, offset64);
        mProperties.pushBack(PVD_NEW(PropDescImpl)(newDesc, *mStringTable));
		mNameToProperties.insert(ClassPropertyName(cls->mName, mProperties.back()->mName), mProperties.back());
		cls->addProperty(mProperties.back());
		bool firstProp = cls->mPropImps.size() == 1;

		if(firstProp)
		{
			cls->mPackedUniformWidth = propPackedWidth;
			cls->mPackedClassType = propPackedType;
		}
		else
		{
			bool packed = (propPackedWidth > 0) && (cls->get32BitSizeInfo().mDataByteSize % propPackedWidth) == 0;
			if(cls->mPackedClassType >= 0) // maybe uncheck packed class type
			{
				if(propPackedType < 0 || cls->mPackedClassType != propPackedType
				                             // Object refs require conversion from stream to db id
				   ||
				   datatype == getPvdTypeForType<ObjectRef>()
				       // Strings also require conversion from stream to db id.
				   ||
				   datatype == getPvdTypeForType<StringHandle>() || packed == false)
					cls->mPackedClassType = -1;
			}
			if(cls->mPackedUniformWidth >= 0) // maybe uncheck packed class width
			{
				if(propPackedWidth < 0 || cls->mPackedUniformWidth != propPackedWidth
				                              // object refs, because they require special treatment during parsing,
				                              // cannot be packed
				   ||
				   datatype == getPvdTypeForType<ObjectRef>()
				       // Likewise, string handles are special because the data needs to be sent *after*
				       // the
				   ||
				   datatype == getPvdTypeForType<StringHandle>() || packed == false)
					cls->mPackedUniformWidth = -1; // invalid packed width.
			}
		}
		return *mProperties.back();
	}

	PropDescImpl* findPropImpl(const NamespacedName& clsName, String prop) const
	{
		const TNameToPropMap::Entry* entry = mNameToProperties.find(ClassPropertyName(clsName, prop));
		if(entry)
			return entry->second;
		return NULL;
	}
	virtual Option<PropertyDescription> findProperty(const NamespacedName& cls, String propName) const
	{
		PropDescImpl* prop(findPropImpl(cls, propName));
		if(prop)
			return *prop;
		return None();
	}

	virtual Option<PropertyDescription> findProperty(int32_t clsId, String propName) const
	{
		ClassDescImpl* cls(getClassImpl(clsId));
		PX_ASSERT(cls);
		if(!cls)
			return None();
		PropDescImpl* prop(findPropImpl(cls->mName, propName));
		if(prop)
			return *prop;
		return None();
	}

	PropDescImpl* getPropertyImpl(int32_t propId) const
	{
		PX_ASSERT(propId >= 0);
		if(propId < 0)
            return NULL;
		uint32_t val = uint32_t(propId);
		if(val >= mProperties.size())
		{
			PX_ASSERT(false);
            return NULL;
		}
		return mProperties[val];
	}

	virtual Option<PropertyDescription> getProperty(int32_t propId) const
	{
		PropDescImpl* impl(getPropertyImpl(propId));
		if(impl)
			return *impl;
		return None();
	}

	virtual void setNamedPropertyValues(DataRef<NamedValue> values, int32_t propId)
	{
		PropDescImpl* impl(getPropertyImpl(propId));
		if(impl)
		{
			impl->mValueNames.resize(values.size());
			PVD_FOREACH(idx, values.size()) impl->mValueNames[idx] = values[idx];
		}
	}

	virtual DataRef<NamedValue> getNamedPropertyValues(int32_t propId) const
	{
		PropDescImpl* impl(getPropertyImpl(propId));
		if(impl)
		{
			return toDataRef(impl->mValueNames);
		}
		return DataRef<NamedValue>();
	}

	virtual uint32_t getNbProperties(int32_t classId) const
	{
		uint32_t retval = 0;
		for(ClassDescImpl* impl(getClassImpl(classId)); impl; impl = getClassImpl(impl->mBaseClass))
		{
			retval += impl->mPropImps.size();
			if(impl->mBaseClass < 0)
				break;
		}
		return retval;
	}

	// Properties need to be returned in base class order, so this requires a recursive function.
	uint32_t getPropertiesImpl(int32_t classId, PropertyDescription*& outBuffer, uint32_t& numItems,
	                           uint32_t& startIdx) const
	{
		ClassDescImpl* impl = getClassImpl(classId);
		if(impl)
		{
			uint32_t retval = 0;
			if(impl->mBaseClass >= 0)
				retval = getPropertiesImpl(impl->mBaseClass, outBuffer, numItems, startIdx);

			uint32_t localStart = PxMin(impl->mPropImps.size(), startIdx);
			uint32_t localNumItems = PxMin(numItems, impl->mPropImps.size() - localStart);
			PVD_FOREACH(idx, localNumItems)
			{
				outBuffer[idx] = *impl->mPropImps[localStart + idx];
			}

			startIdx -= localStart;
			numItems -= localNumItems;
			outBuffer += localNumItems;
			return retval + localNumItems;
		}
		return 0;
	}

	virtual uint32_t getProperties(int32_t classId, PropertyDescription* outBuffer, uint32_t numItems,
	                               uint32_t startIdx) const
	{
		return getPropertiesImpl(classId, outBuffer, numItems, startIdx);
	}

	virtual MarshalQueryResult checkMarshalling(int32_t srcClsId, int32_t dstClsId) const
	{
		Option<ClassDescription> propTypeOpt(getClass(dstClsId));
		if(propTypeOpt.hasValue() == false)
		{
			PX_ASSERT(false);
			return MarshalQueryResult();
		}
		const ClassDescription& propType(propTypeOpt);

		Option<ClassDescription> incomingTypeOpt(getClass(srcClsId));
		if(incomingTypeOpt.hasValue() == false)
		{
			PX_ASSERT(false);
			return MarshalQueryResult();
		}
		const ClassDescription& incomingType(incomingTypeOpt);
		// Can only marshal simple things at this point in time.
		bool needsMarshalling = false;
		bool canMarshal = false;
		TSingleMarshaller single = NULL;
		TBlockMarshaller block = NULL;
		if(incomingType.mClassId != propType.mClassId)
		{
			// Check that marshalling is even possible.
			if((incomingType.mPackedUniformWidth >= 0 && propType.mPackedUniformWidth >= 0) == false)
			{
				PX_ASSERT(false);
				return MarshalQueryResult();
			}

			int32_t srcType = incomingType.mPackedClassType;
			int32_t dstType = propType.mPackedClassType;

			int32_t srcWidth = incomingType.mPackedUniformWidth;
			int32_t dstWidth = propType.mPackedUniformWidth;
			canMarshal = getMarshalOperators(single, block, srcType, dstType);
			if(srcWidth == dstWidth)
				needsMarshalling = canMarshal; // If the types are the same width, we assume we can convert between some
			                                   // of them seamlessly (uint16_t, int16_t)
			else
			{
				needsMarshalling = true;
				// If we can't marshall and we have to then we can't set the property value.
				// This indicates that the src and dest are different properties and we don't
				// know how to convert between them.
				if(!canMarshal)
				{
					PX_ASSERT(false);
					return MarshalQueryResult();
				}
			}
		}
		return MarshalQueryResult(srcClsId, dstClsId, canMarshal, needsMarshalling, block);
	}

	PropertyMessageDescriptionImpl* findPropertyMessageImpl(const NamespacedName& messageName) const
	{
		const TNameToPropertyMessageMap::Entry* entry = mPropertyMessageMap.find(messageName);
		if(entry)
			return entry->second;
		return NULL;
	}

	PropertyMessageDescriptionImpl* getPropertyMessageImpl(int32_t msg) const
	{
		int32_t msgCount = int32_t(mPropertyMessages.size());
		if(msg >= 0 && msg < msgCount)
			return mPropertyMessages[uint32_t(msg)];
		return NULL;
	}

	virtual Option<PropertyMessageDescription> createPropertyMessage(const NamespacedName& clsName,
	                                                                 const NamespacedName& messageName,
	                                                                 DataRef<PropertyMessageArg> entries,
	                                                                 uint32_t messageSize)
	{
		PropertyMessageDescriptionImpl* existing(findPropertyMessageImpl(messageName));
		if(existing)
		{
			PX_ASSERT(false);
			return None();
		}
		ClassDescImpl* cls = findClassImpl(clsName);
		PX_ASSERT(cls);
		if(!cls)
			return None();
		int32_t msgId = int32_t(mPropertyMessages.size());
		PropertyMessageDescriptionImpl* newMessage = PVD_NEW(PropertyMessageDescriptionImpl)(
            PropertyMessageDescription(mStringTable->registerName(clsName), cls->mClassId,
                                       mStringTable->registerName(messageName), msgId, messageSize));
		uint32_t calculatedSize = 0;
		PVD_FOREACH(idx, entries.size())
		{
			PropertyMessageArg entry(entries[idx]);
			ClassDescImpl* dtypeCls = findClassImpl(entry.mDatatypeName);
			if(dtypeCls == NULL)
			{
				PX_ASSERT(false);
				goto DestroyNewMessage;
			}
			ClassDescriptionSizeInfo dtypeInfo(dtypeCls->get32BitSizeInfo());
			uint32_t incomingSize = dtypeInfo.mByteSize;
			if(entry.mByteSize < incomingSize)
			{
				PX_ASSERT(false);
				goto DestroyNewMessage;
			}

			calculatedSize = PxMax(calculatedSize, entry.mMessageOffset + entry.mByteSize);
			if(calculatedSize > messageSize)
			{
				PX_ASSERT(false);
				goto DestroyNewMessage;
			}

			Option<PropertyDescription> propName(findProperty(cls->mClassId, entry.mPropertyName));
			if(propName.hasValue() == false)
			{
				PX_ASSERT(false);
				goto DestroyNewMessage;
			}

			Option<ClassDescription> propCls(getClass(propName.getValue().mDatatype));
			if(propCls.hasValue() == false)
			{
				PX_ASSERT(false);
				goto DestroyNewMessage;
			}

			PropertyMessageEntryImpl newEntry(PropertyMessageEntry(
			    propName, dtypeCls->mName, dtypeCls->mClassId, entry.mMessageOffset, incomingSize, dtypeInfo.mByteSize));
			newMessage->addEntry(newEntry);

			if(newEntry.mDatatypeId == getPvdTypeForType<String>())
				newMessage->mStringOffsetArray.pushBack(entry.mMessageOffset);

			// property messages cannot be marshalled at this time.
			if(newEntry.mDatatypeId != getPvdTypeForType<String>() && newEntry.mDatatypeId != getPvdTypeForType<VoidPtr>())
			{
				MarshalQueryResult marshalInfo = checkMarshalling(newEntry.mDatatypeId, newEntry.mProperty.mDatatype);
				if(marshalInfo.needsMarshalling)
				{
					PX_ASSERT(false);
					goto DestroyNewMessage;
				}
			}
		}

		if(newMessage)
		{
			newMessage->mStringOffsets =
			    DataRef<uint32_t>(newMessage->mStringOffsetArray.begin(), newMessage->mStringOffsetArray.end());
			mPropertyMessages.pushBack(newMessage);
			mPropertyMessageMap.insert(messageName, newMessage);
			return *newMessage;
		}

	DestroyNewMessage:
		if(newMessage)
			PVD_DELETE(newMessage);

		return None();
	}
	virtual Option<PropertyMessageDescription> findPropertyMessage(const NamespacedName& msgName) const
	{
		PropertyMessageDescriptionImpl* desc(findPropertyMessageImpl(msgName));
		if(desc)
			return *desc;
		return None();
	}

	virtual Option<PropertyMessageDescription> getPropertyMessage(int32_t msgId) const
	{
		PropertyMessageDescriptionImpl* desc(getPropertyMessageImpl(msgId));
		if(desc)
			return *desc;
		return None();
	}

	virtual uint32_t getNbPropertyMessages() const
	{
		return mPropertyMessages.size();
	}

	virtual uint32_t getPropertyMessages(PropertyMessageDescription* msgBuf, uint32_t bufLen, uint32_t startIdx = 0) const
	{
		startIdx = PxMin(startIdx, getNbPropertyMessages());
		bufLen = PxMin(bufLen, getNbPropertyMessages() - startIdx);
		PVD_FOREACH(idx, bufLen) msgBuf[idx] = *mPropertyMessages[idx + startIdx];
		return bufLen;
	}

	struct MetaDataWriter
	{
		const PvdObjectModelMetaDataImpl& mMetaData;
		PvdOutputStream& mStream;
		MetaDataWriter(const PvdObjectModelMetaDataImpl& meta, PvdOutputStream& stream)
		: mMetaData(meta), mStream(stream)
		{
		}

		void streamify(NamespacedName& type)
		{
            mStream << mMetaData.mStringTable->strToHandle(type.mNamespace);
            mStream << mMetaData.mStringTable->strToHandle(type.mName);
		}
		void streamify(String& type)
		{
            mStream << mMetaData.mStringTable->strToHandle(type);
		}
		void streamify(int32_t& type)
		{
			mStream << type;
		}
		void streamify(uint32_t& type)
		{
			mStream << type;
		}
		void streamify(uint8_t type)
		{
			mStream << type;
		}
		void streamify(bool type)
		{
			streamify( uint8_t(type));
		}
		void streamify(PropertyType::Enum type)
		{
			uint32_t val = static_cast<uint32_t>(type);
			mStream << val;
		}
		void streamify(NamedValue& type)
		{
			streamify(type.mValue);
			streamify(type.mName);
		}
		void streamifyLinks(PropDescImpl* prop)
		{
			streamify(prop->mPropertyId);
		}
		void streamify(PropertyDescription& prop)
		{
			streamify(prop.mPropertyId);
		}
		void streamify(PropertyMessageEntryImpl& prop)
		{
			prop.serialize(*this);
		}
		void streamify(PtrOffset& off)
		{
			uint32_t type = off.mOffsetType;
			mStream << type;
			mStream << off.mOffset;
		}
		template <typename TDataType>
		void streamify(TDataType* type)
		{
			int32_t existMarker = type ? 1 : 0;
			mStream << existMarker;
			if(type)
				type->serialize(*this);
		}
		template <typename TArrayType>
		void streamify(const Array<TArrayType>& type)
		{
			mStream << static_cast<uint32_t>(type.size());
			PVD_FOREACH(idx, type.size()) streamify(const_cast<TArrayType&>(type[idx]));
		}
		template <typename TArrayType>
		void streamifyLinks(const Array<TArrayType>& type)
		{
			mStream << static_cast<uint32_t>(type.size());
			PVD_FOREACH(idx, type.size()) streamifyLinks(const_cast<TArrayType&>(type[idx]));
		}

	  private:
		MetaDataWriter& operator=(const MetaDataWriter&);
	};

	template <typename TStreamType>
	struct MetaDataReader
	{
		PvdObjectModelMetaDataImpl& mMetaData;
		TStreamType& mStream;
		MetaDataReader(PvdObjectModelMetaDataImpl& meta, TStreamType& stream) : mMetaData(meta), mStream(stream)
		{
		}

		void streamify(NamespacedName& type)
		{
			streamify(type.mNamespace);
			streamify(type.mName);
		}

		void streamify(String& type)
		{
			uint32_t handle;
			mStream >> handle;
            type = mMetaData.mStringTable->handleToStr(handle);
		}
		void streamify(int32_t& type)
		{
			mStream >> type;
		}
		void streamify(uint32_t& type)
		{
			mStream >> type;
		}
		void streamify(bool& type)
		{
			uint8_t data;
			mStream >> data;
			type = data ? true : false;
		}

		void streamify(PropertyType::Enum& type)
		{
			uint32_t val;
			mStream >> val;
			type = static_cast<PropertyType::Enum>(val);
		}
		void streamify(NamedValue& type)
		{
			streamify(type.mValue);
			streamify(type.mName);
		}
		void streamify(PropertyMessageEntryImpl& type)
		{
			type.serialize(*this);
		}
		void streamify(PtrOffset& off)
		{
			uint32_t type;
			mStream >> type;
			mStream >> off.mOffset;
			off.mOffsetType = static_cast<PtrOffsetType::Enum>(type);
		}
		void streamifyLinks(PropDescImpl*& prop)
		{
			int32_t propId;
			streamify(propId);
			prop = mMetaData.getPropertyImpl(propId);
		}
		void streamify(PropertyDescription& prop)
		{
			streamify(prop.mPropertyId);
			prop = mMetaData.getProperty(prop.mPropertyId);
		}
		template <typename TDataType>
		void streamify(TDataType*& type)
		{
			uint32_t existMarker;
			mStream >> existMarker;
			if(existMarker)
			{
				TDataType* newType = PVD_NEW(TDataType)();
				newType->serialize(*this);
				type = newType;
			}
			else
				type = NULL;
		}
		template <typename TArrayType>
		void streamify(Array<TArrayType>& type)
		{
			uint32_t typeSize;
			mStream >> typeSize;
			type.resize(typeSize);
			PVD_FOREACH(idx, type.size()) streamify(type[idx]);
		}
		template <typename TArrayType>
		void streamifyLinks(Array<TArrayType>& type)
		{
			uint32_t typeSize;
			mStream >> typeSize;
			type.resize(typeSize);
			PVD_FOREACH(idx, type.size()) streamifyLinks(type[idx]);
		}

	  private:
		MetaDataReader& operator=(const MetaDataReader&);
	};

	virtual void write(PvdOutputStream& stream) const
	{
		stream << getCurrentPvdObjectModelVersion();
		stream << mNextClassId;
        mStringTable->write(stream);
		MetaDataWriter writer(*this, stream);
		writer.streamify(mProperties);
		writer.streamify(mClasses);
		writer.streamify(mPropertyMessages);
	}

	template <typename TReaderType>
	void read(TReaderType& stream)
	{
		uint32_t version;
		stream >> version;
		stream >> mNextClassId;
        mStringTable->read(stream);
		MetaDataReader<TReaderType> reader(*this, stream);
		reader.streamify(mProperties);
		reader.streamify(mClasses);
		reader.streamify(mPropertyMessages);

		mNameToClasses.clear();
		mNameToProperties.clear();
		mPropertyMessageMap.clear();
		PVD_FOREACH(i, mClasses.size())
		{
			ClassDescImpl* cls(mClasses[i]);
			if(cls == NULL)
				continue;
			mNameToClasses.insert(cls->mName, mClasses[i]);
			uint32_t propCount = getNbProperties(cls->mClassId);
			PropertyDescription descs[16];
			uint32_t offset = 0;
			for(uint32_t idx = 0; idx < propCount; idx = offset)
			{
				uint32_t numProps = getProperties(cls->mClassId, descs, 16, offset);
				offset += numProps;
				for(uint32_t propIdx = 0; propIdx < numProps; ++propIdx)
				{
					PropDescImpl* prop = getPropertyImpl(descs[propIdx].mPropertyId);
					if(prop)
						mNameToProperties.insert(ClassPropertyName(cls->mName, prop->mName), prop);
				}
			}
		}
		PVD_FOREACH(idx, mPropertyMessages.size())
		mPropertyMessageMap.insert(mPropertyMessages[idx]->mMessageName, mPropertyMessages[idx]);
	}

	virtual StringTable& getStringTable() const
	{
        return *mStringTable;
	}
	virtual void addRef()
	{
		++mRefCount;
	}
	virtual void release()
	{
		if(mRefCount)
			--mRefCount;
		if(!mRefCount)
			PVD_DELETE(this);
	}
};
}

uint32_t PvdObjectModelMetaData::getCurrentPvdObjectModelVersion()
{
	return 1;
}

PvdObjectModelMetaData& PvdObjectModelMetaData::create()
{
	PvdObjectModelMetaDataImpl& retval(*PVD_NEW(PvdObjectModelMetaDataImpl)());
	retval.initialize();
	return retval;
}

PvdObjectModelMetaData& PvdObjectModelMetaData::create(PvdInputStream& stream)
{
	PvdObjectModelMetaDataImpl& retval(*PVD_NEW(PvdObjectModelMetaDataImpl)());
	retval.read(stream);
	return retval;
}

StringTable& StringTable::create()
{
	return *PVD_NEW(StringTableImpl)();
}
