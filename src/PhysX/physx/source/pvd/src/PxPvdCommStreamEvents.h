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

#ifndef PXPVDSDK_PXPVDCOMMSTREAMEVENTS_H
#define PXPVDSDK_PXPVDCOMMSTREAMEVENTS_H

#include "foundation/PxVec3.h"
#include "foundation/PxFlags.h"

#include "PxPvdObjectModelBaseTypes.h"
#include "PsTime.h"

namespace physx
{
namespace pvdsdk
{

struct CommStreamFlagTypes
{
	enum Enum
	{
		Is64BitPtr = 1
	};
};

typedef PxFlags<CommStreamFlagTypes::Enum, uint32_t> CommStreamFlags;

template <typename TDataType>
struct PvdCommVariableSizedEventCheck
{
	bool variable_size_check;
};

// Pick out the events that are possibly very large.
// This helps us keep our buffers close to the size the user requested.
#define DECLARE_TYPE_VARIABLE_SIZED(type)                                                                              \
	template <>                                                                                                        \
	struct PvdCommVariableSizedEventCheck<type>                                                                        \
	{                                                                                                                  \
		uint32_t variable_size_check;                                                                                  \
	};

struct NameHandleValue;
struct StreamPropMessageArg;
struct StringHandleEvent;
struct CreateClass;
struct DeriveClass;
struct CreateProperty;
struct CreatePropertyMessage;
struct CreateInstance;
struct SetPropertyValue;
struct BeginSetPropertyValue;
struct AppendPropertyValueData;
struct EndSetPropertyValue;
struct SetPropertyMessage;
struct BeginPropertyMessageGroup;
struct SendPropertyMessageFromGroup;
struct EndPropertyMessageGroup;
struct CreateDestroyInstanceProperty;
struct PushBackObjectRef;
struct RemoveObjectRef;
struct BeginSection;
struct EndSection;
struct SetPickable;
struct SetColor;
struct SetIsTopLevel;
struct SetCamera;
struct AddProfileZone;
struct AddProfileZoneEvent;
struct StreamEndEvent;
struct ErrorMessage;
struct OriginShift;
struct DestroyInstance;

#define DECLARE_COMM_STREAM_EVENTS                                                                                     \
	\
DECLARE_PVD_COMM_STREAM_EVENT(StringHandleEvent) \
DECLARE_PVD_COMM_STREAM_EVENT(CreateClass) \
DECLARE_PVD_COMM_STREAM_EVENT(DeriveClass) \
DECLARE_PVD_COMM_STREAM_EVENT(CreateProperty) \
DECLARE_PVD_COMM_STREAM_EVENT(CreatePropertyMessage) \
DECLARE_PVD_COMM_STREAM_EVENT(CreateInstance) \
DECLARE_PVD_COMM_STREAM_EVENT(SetPropertyValue) \
DECLARE_PVD_COMM_STREAM_EVENT(BeginSetPropertyValue) \
DECLARE_PVD_COMM_STREAM_EVENT(AppendPropertyValueData) \
DECLARE_PVD_COMM_STREAM_EVENT(EndSetPropertyValue) \
DECLARE_PVD_COMM_STREAM_EVENT(SetPropertyMessage) \
DECLARE_PVD_COMM_STREAM_EVENT(BeginPropertyMessageGroup) \
DECLARE_PVD_COMM_STREAM_EVENT(SendPropertyMessageFromGroup) \
DECLARE_PVD_COMM_STREAM_EVENT(EndPropertyMessageGroup) \
DECLARE_PVD_COMM_STREAM_EVENT(DestroyInstance) \
DECLARE_PVD_COMM_STREAM_EVENT(PushBackObjectRef) \
DECLARE_PVD_COMM_STREAM_EVENT(RemoveObjectRef) \
DECLARE_PVD_COMM_STREAM_EVENT(BeginSection) \
DECLARE_PVD_COMM_STREAM_EVENT(EndSection) \
DECLARE_PVD_COMM_STREAM_EVENT(SetPickable) \
DECLARE_PVD_COMM_STREAM_EVENT(SetColor) \
DECLARE_PVD_COMM_STREAM_EVENT(SetIsTopLevel) \
DECLARE_PVD_COMM_STREAM_EVENT(SetCamera) \
DECLARE_PVD_COMM_STREAM_EVENT(AddProfileZone) \
DECLARE_PVD_COMM_STREAM_EVENT(AddProfileZoneEvent) \
DECLARE_PVD_COMM_STREAM_EVENT(StreamEndEvent) \
DECLARE_PVD_COMM_STREAM_EVENT(ErrorMessage) \
DECLARE_PVD_COMM_STREAM_EVENT_NO_COMMA(OriginShift)

struct PvdCommStreamEventTypes
{
	enum Enum
	{
		Unknown = 0,
#define DECLARE_PVD_COMM_STREAM_EVENT(x) x,
#define DECLARE_PVD_COMM_STREAM_EVENT_NO_COMMA(x) x
		DECLARE_COMM_STREAM_EVENTS
#undef DECLARE_PVD_COMM_STREAM_EVENT_NO_COMMA
#undef DECLARE_PVD_COMM_STREAM_EVENT
        , Last
	};
};

template <typename TDataType>
struct DatatypeToCommEventType
{
	bool compile_error;
};
template <PvdCommStreamEventTypes::Enum TEnumType>
struct CommEventTypeToDatatype
{
	bool compile_error;
};

#define DECLARE_PVD_COMM_STREAM_EVENT(x)                                                                               \
	template <>                                                                                                        \
	struct DatatypeToCommEventType<x>                                                                                  \
	{                                                                                                                  \
		enum Enum                                                                                                      \
		{                                                                                                              \
			EEventTypeMap = PvdCommStreamEventTypes::x                                                                 \
		};                                                                                                             \
	};                                                                                                                 \
	template <>                                                                                                        \
	struct CommEventTypeToDatatype<PvdCommStreamEventTypes::x>                                                         \
	{                                                                                                                  \
		typedef x TEventType;                                                                                          \
	};
#define DECLARE_PVD_COMM_STREAM_EVENT_NO_COMMA(x)                                                                      \
	\
template<> struct DatatypeToCommEventType<x>                                                                           \
	{                                                                                                                  \
		enum Enum                                                                                                      \
		{                                                                                                              \
			EEventTypeMap = PvdCommStreamEventTypes::x                                                                 \
		};                                                                                                             \
	};                                                                                                                 \
	\
template<> struct CommEventTypeToDatatype<PvdCommStreamEventTypes::x>                                                  \
	{                                                                                                                  \
		typedef x TEventType;                                                                                          \
	};

DECLARE_COMM_STREAM_EVENTS
#undef DECLARE_PVD_COMM_STREAM_EVENT_NO_COMMA
#undef DECLARE_PVD_COMM_STREAM_EVENT

template <typename TDataType>
PvdCommStreamEventTypes::Enum getCommStreamEventType()
{
	return static_cast<PvdCommStreamEventTypes::Enum>(DatatypeToCommEventType<TDataType>::EEventTypeMap);
}

struct StreamNamespacedName
{
	StringHandle mNamespace; // StringHandle handles
	StringHandle mName;
	StreamNamespacedName(StringHandle ns = 0, StringHandle nm = 0) : mNamespace(ns), mName(nm)
	{
	}
};

class EventSerializeable;

class PvdEventSerializer
{
  protected:
	virtual ~PvdEventSerializer()
	{
	}

  public:
	virtual void streamify(uint8_t& val) = 0;
	virtual void streamify(uint16_t& val) = 0;
	virtual void streamify(uint32_t& val) = 0;
	virtual void streamify(float& val) = 0;
	virtual void streamify(uint64_t& val) = 0;
	virtual void streamify(String& val) = 0;
	virtual void streamify(DataRef<const uint8_t>& data) = 0;
	virtual void streamify(DataRef<NameHandleValue>& data) = 0;
	virtual void streamify(DataRef<StreamPropMessageArg>& data) = 0;
	virtual void streamify(DataRef<StringHandle>& data) = 0;

	void streamify(StringHandle& hdl)
	{
		streamify(hdl.mHandle);
	}
	void streamify(CommStreamFlags& flags)
	{
		uint32_t val(flags);
		streamify(val);
		flags = CommStreamFlags(val);
	}

	void streamify(PvdCommStreamEventTypes::Enum& val)
	{
		uint8_t detyped = static_cast<uint8_t>(val);
		streamify(detyped);
		val = static_cast<PvdCommStreamEventTypes::Enum>(detyped);
	}
	void streamify(PropertyType::Enum& val)
	{
		uint8_t detyped = static_cast<uint8_t>(val);
		streamify(detyped);
		val = static_cast<PropertyType::Enum>(detyped);
	}

	void streamify(bool& val)
	{
		uint8_t detyped = uint8_t(val ? 1 : 0);
		streamify(detyped);
		val = detyped ? true : false;
	}

	void streamify(StreamNamespacedName& name)
	{
		streamify(name.mNamespace);
		streamify(name.mName);
	}

	void streamify(PvdColor& color)
	{
		streamify(color.r);
		streamify(color.g);
		streamify(color.b);
		streamify(color.a);
	}

	void streamify(PxVec3& vec)
	{
		streamify(vec.x);
		streamify(vec.y);
		streamify(vec.z);
	}

	static uint32_t measure(const EventSerializeable& evt);
};

class EventSerializeable
{
  protected:
	virtual ~EventSerializeable()
	{
	}

  public:
	virtual void serialize(PvdEventSerializer& serializer) = 0;
};

/** Numbers generated from random.org
129919156	17973702	401496246	144984007	336950759
907025328	837150850	679717896	601529147	269478202
*/
struct StreamInitialization : public EventSerializeable
{
	static uint32_t getStreamId()
	{
		return 837150850;
	}
	static uint32_t getStreamVersion()
	{
		return 1;
	}

	uint32_t mStreamId;
	uint32_t mStreamVersion;
	uint64_t mTimestampNumerator;
	uint64_t mTimestampDenominator;
	CommStreamFlags mStreamFlags;
	StreamInitialization()
	: mStreamId(getStreamId())
	, mStreamVersion(getStreamVersion())
	, mTimestampNumerator(physx::shdfnd::Time::getCounterFrequency().mNumerator * 10)
	, mTimestampDenominator(physx::shdfnd::Time::getCounterFrequency().mDenominator)
	, mStreamFlags(sizeof(void*) == 4 ? 0 : 1)
	{
	}

	void serialize(PvdEventSerializer& s)
	{
		s.streamify(mStreamId);
		s.streamify(mStreamVersion);
		s.streamify(mTimestampNumerator);
		s.streamify(mTimestampDenominator);
		s.streamify(mStreamFlags);
	}
};

struct EventGroup : public EventSerializeable
{
	uint32_t mDataSize; // in bytes, data directly follows this header
	uint32_t mNumEvents;
	uint64_t mStreamId;
	uint64_t mTimestamp;

	EventGroup(uint32_t dataSize = 0, uint32_t numEvents = 0, uint64_t streamId = 0, uint64_t ts = 0)
	: mDataSize(dataSize), mNumEvents(numEvents), mStreamId(streamId), mTimestamp(ts)
	{
	}

	void serialize(PvdEventSerializer& s)
	{
		s.streamify(mDataSize);
		s.streamify(mNumEvents);
		s.streamify(mStreamId);
		s.streamify(mTimestamp);
	}
};

struct StringHandleEvent : public EventSerializeable
{
	String mString;
	uint32_t mHandle;
	StringHandleEvent(String str, uint32_t hdl) : mString(str), mHandle(hdl)
	{
	}
	StringHandleEvent()
	{
	}

	void serialize(PvdEventSerializer& s)
	{
		s.streamify(mString);
		s.streamify(mHandle);
	}
};

DECLARE_TYPE_VARIABLE_SIZED(StringHandleEvent)

typedef uint64_t Timestamp;

struct CreateClass : public EventSerializeable
{
	StreamNamespacedName mName;
	CreateClass(StreamNamespacedName nm) : mName(nm)
	{
	}
	CreateClass()
	{
	}

	void serialize(PvdEventSerializer& s)
	{
		s.streamify(mName);
	}
};

struct DeriveClass : public EventSerializeable
{
	StreamNamespacedName mParent;
	StreamNamespacedName mChild;

	DeriveClass(StreamNamespacedName p, StreamNamespacedName c) : mParent(p), mChild(c)
	{
	}
	DeriveClass()
	{
	}

	void serialize(PvdEventSerializer& s)
	{
		s.streamify(mParent);
		s.streamify(mChild);
	}
};

struct NameHandleValue : public EventSerializeable
{
	StringHandle mName;
	uint32_t mValue;
	NameHandleValue(StringHandle name, uint32_t val) : mName(name), mValue(val)
	{
	}
	NameHandleValue()
	{
	}

	void serialize(PvdEventSerializer& s)
	{
		s.streamify(mName);
		s.streamify(mValue);
	}
};
/*virtual PvdError createProperty( StreamNamespacedName clsName, StringHandle name, StringHandle semantic
                                    , StreamNamespacedName dtypeName, PropertyType::Enum propertyType
                                    , DataRef<NamedValue> values = DataRef<NamedValue>() ) = 0; */
struct CreateProperty : public EventSerializeable
{
	StreamNamespacedName mClass;
	StringHandle mName;
	StringHandle mSemantic;
	StreamNamespacedName mDatatypeName;
	PropertyType::Enum mPropertyType;
	DataRef<NameHandleValue> mValues;

	CreateProperty(StreamNamespacedName cls, StringHandle name, StringHandle semantic, StreamNamespacedName dtypeName,
	               PropertyType::Enum ptype, DataRef<NameHandleValue> values)
	: mClass(cls), mName(name), mSemantic(semantic), mDatatypeName(dtypeName), mPropertyType(ptype), mValues(values)
	{
	}
	CreateProperty()
	{
	}

	void serialize(PvdEventSerializer& s)
	{
		s.streamify(mClass);
		s.streamify(mName);
		s.streamify(mSemantic);
		s.streamify(mDatatypeName);
		s.streamify(mPropertyType);
		s.streamify(mValues);
	}
};

struct StreamPropMessageArg : public EventSerializeable
{
	StringHandle mPropertyName;
	StreamNamespacedName mDatatypeName;
	uint32_t mMessageOffset;
	uint32_t mByteSize;
	StreamPropMessageArg(StringHandle pname, StreamNamespacedName dtypeName, uint32_t offset, uint32_t byteSize)
	: mPropertyName(pname), mDatatypeName(dtypeName), mMessageOffset(offset), mByteSize(byteSize)
	{
	}

	StreamPropMessageArg()
	{
	}

	void serialize(PvdEventSerializer& s)
	{
		s.streamify(mPropertyName);
		s.streamify(mDatatypeName);
		s.streamify(mMessageOffset);
		s.streamify(mByteSize);
	}
};

/*
    virtual PvdError createPropertyMessage( StreamNamespacedName cls, StreamNamespacedName msgName
                                                , DataRef<PropertyMessageArg> entries, uint32_t messageSizeInBytes ) =
   0;*/
struct CreatePropertyMessage : public EventSerializeable
{
	StreamNamespacedName mClass;
	StreamNamespacedName mMessageName;
	DataRef<StreamPropMessageArg> mMessageEntries;
	uint32_t mMessageByteSize;

	CreatePropertyMessage(StreamNamespacedName cls, StreamNamespacedName msgName, DataRef<StreamPropMessageArg> propArg,
	                      uint32_t messageByteSize)
	: mClass(cls), mMessageName(msgName), mMessageEntries(propArg), mMessageByteSize(messageByteSize)
	{
	}
	CreatePropertyMessage()
	{
	}

	void serialize(PvdEventSerializer& s)
	{
		s.streamify(mClass);
		s.streamify(mMessageName);
		s.streamify(mMessageEntries);
		s.streamify(mMessageByteSize);
	}
};

/**Changing immediate data on instances*/

// virtual PvdError createInstance( StreamNamespacedName cls, uint64_t instance ) = 0;
struct CreateInstance : public EventSerializeable
{
	StreamNamespacedName mClass;
	uint64_t mInstanceId;

	CreateInstance(StreamNamespacedName cls, uint64_t streamId) : mClass(cls), mInstanceId(streamId)
	{
	}
	CreateInstance()
	{
	}

	void serialize(PvdEventSerializer& s)
	{
		s.streamify(mClass);
		s.streamify(mInstanceId);
	}
};

// virtual PvdError setPropertyValue( uint64_t instance, StringHandle name, DataRef<const uint8_t> data,
// StreamNamespacedName incomingTypeName ) = 0;
struct SetPropertyValue : public EventSerializeable
{
	uint64_t mInstanceId;
	StringHandle mPropertyName;
	DataRef<const uint8_t> mData;
	StreamNamespacedName mIncomingTypeName;
	uint32_t mNumItems;

	SetPropertyValue(uint64_t instance, StringHandle name, DataRef<const uint8_t> data,
	                 StreamNamespacedName incomingTypeName, uint32_t numItems)
	: mInstanceId(instance), mPropertyName(name), mData(data), mIncomingTypeName(incomingTypeName), mNumItems(numItems)
	{
	}

	SetPropertyValue()
	{
	}

	void serializeBeginning(PvdEventSerializer& s)
	{
		s.streamify(mInstanceId);
		s.streamify(mPropertyName);
		s.streamify(mIncomingTypeName);
		s.streamify(mNumItems);
	}

	void serialize(PvdEventSerializer& s)
	{
		serializeBeginning(s);
		s.streamify(mData);
	}
};

DECLARE_TYPE_VARIABLE_SIZED(SetPropertyValue)

struct BeginSetPropertyValue : public EventSerializeable
{
	uint64_t mInstanceId;
	StringHandle mPropertyName;
	StreamNamespacedName mIncomingTypeName;

	BeginSetPropertyValue(uint64_t instance, StringHandle name, StreamNamespacedName incomingTypeName)
	: mInstanceId(instance), mPropertyName(name), mIncomingTypeName(incomingTypeName)
	{
	}
	BeginSetPropertyValue()
	{
	}

	void serialize(PvdEventSerializer& s)
	{
		s.streamify(mInstanceId);
		s.streamify(mPropertyName);
		s.streamify(mIncomingTypeName);
	}
};

// virtual PvdError appendPropertyValueData( DataRef<const uint8_t> data ) = 0;
struct AppendPropertyValueData : public EventSerializeable
{
	DataRef<const uint8_t> mData;
	uint32_t mNumItems;
	AppendPropertyValueData(DataRef<const uint8_t> data, uint32_t numItems) : mData(data), mNumItems(numItems)
	{
	}
	AppendPropertyValueData()
	{
	}

	void serialize(PvdEventSerializer& s)
	{
		s.streamify(mData);
		s.streamify(mNumItems);
	}
};

DECLARE_TYPE_VARIABLE_SIZED(AppendPropertyValueData)

// virtual PvdError endSetPropertyValue() = 0;
struct EndSetPropertyValue : public EventSerializeable
{
	EndSetPropertyValue()
	{
	}

	void serialize(PvdEventSerializer&)
	{
	}
};

// virtual PvdError setPropertyMessage( uint64_t instance, StreamNamespacedName msgName, DataRef<const uint8_t> data ) =
// 0;
struct SetPropertyMessage : public EventSerializeable
{
	uint64_t mInstanceId;
	StreamNamespacedName mMessageName;
	DataRef<const uint8_t> mData;

	SetPropertyMessage(uint64_t instance, StreamNamespacedName msgName, DataRef<const uint8_t> data)
	: mInstanceId(instance), mMessageName(msgName), mData(data)
	{
	}

	SetPropertyMessage()
	{
	}

	void serialize(PvdEventSerializer& s)
	{
		s.streamify(mInstanceId);
		s.streamify(mMessageName);
		s.streamify(mData);
	}
};

DECLARE_TYPE_VARIABLE_SIZED(SetPropertyMessage)

// virtual PvdError beginPropertyMessageGroup( StreamNamespacedName msgName ) = 0;
struct BeginPropertyMessageGroup : public EventSerializeable
{
	StreamNamespacedName mMsgName;
	BeginPropertyMessageGroup(StreamNamespacedName msgName) : mMsgName(msgName)
	{
	}
	BeginPropertyMessageGroup()
	{
	}

	void serialize(PvdEventSerializer& s)
	{
		s.streamify(mMsgName);
	}
};

// virtual PvdError sendPropertyMessageFromGroup( uint64_t instance, DataRef<const uint8_t*> data ) = 0;
struct SendPropertyMessageFromGroup : public EventSerializeable
{
	uint64_t mInstance;
	DataRef<const uint8_t> mData;

	SendPropertyMessageFromGroup(uint64_t instance, DataRef<const uint8_t> data) : mInstance(instance), mData(data)
	{
	}
	SendPropertyMessageFromGroup()
	{
	}

	void serialize(PvdEventSerializer& s)
	{
		s.streamify(mInstance);
		s.streamify(mData);
	}
};

DECLARE_TYPE_VARIABLE_SIZED(SendPropertyMessageFromGroup)

// virtual PvdError endPropertyMessageGroup() = 0;
struct EndPropertyMessageGroup : public EventSerializeable
{
	EndPropertyMessageGroup()
	{
	}

	void serialize(PvdEventSerializer&)
	{
	}
};

struct PushBackObjectRef : public EventSerializeable
{
	uint64_t mInstanceId;
	StringHandle mProperty;
	uint64_t mObjectRef;

	PushBackObjectRef(uint64_t instId, StringHandle prop, uint64_t objRef)
	: mInstanceId(instId), mProperty(prop), mObjectRef(objRef)
	{
	}

	PushBackObjectRef()
	{
	}

	void serialize(PvdEventSerializer& s)
	{
		s.streamify(mInstanceId);
		s.streamify(mProperty);
		s.streamify(mObjectRef);
	}
};

struct RemoveObjectRef : public EventSerializeable
{
	uint64_t mInstanceId;
	StringHandle mProperty;
	uint64_t mObjectRef;

	RemoveObjectRef(uint64_t instId, StringHandle prop, uint64_t objRef)
	: mInstanceId(instId), mProperty(prop), mObjectRef(objRef)
	{
	}

	RemoveObjectRef()
	{
	}

	void serialize(PvdEventSerializer& s)
	{
		s.streamify(mInstanceId);
		s.streamify(mProperty);
		s.streamify(mObjectRef);
	}
};

// virtual PvdError destroyInstance( uint64_t key ) = 0;
struct DestroyInstance : public EventSerializeable
{
	uint64_t mInstanceId;
	DestroyInstance(uint64_t instance) : mInstanceId(instance)
	{
	}
	DestroyInstance()
	{
	}

	void serialize(PvdEventSerializer& s)
	{
		s.streamify(mInstanceId);
	}
};

// virtual PvdError beginSection( uint64_t sectionId, StringHandle name ) = 0;
struct BeginSection : public EventSerializeable
{
	uint64_t mSectionId;
	StringHandle mName;
	Timestamp mTimestamp;
	BeginSection(uint64_t sectionId, StringHandle name, uint64_t timestamp)
	: mSectionId(sectionId), mName(name), mTimestamp(timestamp)
	{
	}
	BeginSection()
	{
	}

	void serialize(PvdEventSerializer& s)
	{
		s.streamify(mSectionId);
		s.streamify(mName);
		s.streamify(mTimestamp);
	}
};
// virtual PvdError endSection( uint64_t sectionId, StringHandle name ) = 0;
struct EndSection : public EventSerializeable
{
	uint64_t mSectionId;
	StringHandle mName;
	Timestamp mTimestamp;
	EndSection(uint64_t sectionId, StringHandle name, uint64_t timestamp)
	: mSectionId(sectionId), mName(name), mTimestamp(timestamp)
	{
	}
	EndSection()
	{
	}

	void serialize(PvdEventSerializer& s)
	{
		s.streamify(mSectionId);
		s.streamify(mName);
		s.streamify(mTimestamp);
	}
};

// virtual void setPickable( void* instance, bool pickable ) = 0;
struct SetPickable : public EventSerializeable
{
	uint64_t mInstanceId;
	bool mPickable;
	SetPickable(uint64_t instId, bool pick) : mInstanceId(instId), mPickable(pick)
	{
	}
	SetPickable()
	{
	}

	void serialize(PvdEventSerializer& s)
	{
		s.streamify(mInstanceId);
		s.streamify(mPickable);
	}
};
// virtual void setColor( void* instance, const PvdColor& color ) = 0;
struct SetColor : public EventSerializeable
{
	uint64_t mInstanceId;
	PvdColor mColor;
	SetColor(uint64_t instId, PvdColor color) : mInstanceId(instId), mColor(color)
	{
	}
	SetColor()
	{
	}

	void serialize(PvdEventSerializer& s)
	{
		s.streamify(mInstanceId);
		s.streamify(mColor);
	}
};

// virtual void setColor( void* instance, const PvdColor& color ) = 0;
struct SetIsTopLevel : public EventSerializeable
{
	uint64_t mInstanceId;
	bool mIsTopLevel;

	SetIsTopLevel(uint64_t instId, bool topLevel) : mInstanceId(instId), mIsTopLevel(topLevel)
	{
	}
	SetIsTopLevel() : mIsTopLevel(false)
	{
	}

	void serialize(PvdEventSerializer& s)
	{
		s.streamify(mInstanceId);
		s.streamify(mIsTopLevel);
	}
};

struct SetCamera : public EventSerializeable
{
	String mName;
	PxVec3 mPosition;
	PxVec3 mUp;
	PxVec3 mTarget;
	SetCamera(String name, const PxVec3& pos, const PxVec3& up, const PxVec3& target)
	: mName(name), mPosition(pos), mUp(up), mTarget(target)
	{
	}
	SetCamera() : mName(NULL)
	{
	}

	void serialize(PvdEventSerializer& s)
	{
		s.streamify(mName);
		s.streamify(mPosition);
		s.streamify(mUp);
		s.streamify(mTarget);
	}
};

struct ErrorMessage : public EventSerializeable
{
	uint32_t mCode;
	String mMessage;
	String mFile;
	uint32_t mLine;

	ErrorMessage(uint32_t code, String message, String file, uint32_t line)
	: mCode(code), mMessage(message), mFile(file), mLine(line)
	{
	}

	ErrorMessage() : mMessage(NULL), mFile(NULL)
	{
	}

	void serialize(PvdEventSerializer& s)
	{
		s.streamify(mCode);
		s.streamify(mMessage);
		s.streamify(mFile);
		s.streamify(mLine);
	}
};

struct AddProfileZone : public EventSerializeable
{
	uint64_t mInstanceId;
	String mName;
	AddProfileZone(uint64_t iid, String nm) : mInstanceId(iid), mName(nm)
	{
	}
	AddProfileZone() : mName(NULL)
	{
	}

	void serialize(PvdEventSerializer& s)
	{
		s.streamify(mInstanceId);
		s.streamify(mName);
	}
};

struct AddProfileZoneEvent : public EventSerializeable
{
	uint64_t mInstanceId;
	String mName;
	uint16_t mEventId;
	bool mCompileTimeEnabled;
	AddProfileZoneEvent(uint64_t iid, String nm, uint16_t eid, bool cte)
	: mInstanceId(iid), mName(nm), mEventId(eid), mCompileTimeEnabled(cte)
	{
	}
	AddProfileZoneEvent()
	{
	}

	void serialize(PvdEventSerializer& s)
	{
		s.streamify(mInstanceId);
		s.streamify(mName);
		s.streamify(mEventId);
		s.streamify(mCompileTimeEnabled);
	}
};

struct StreamEndEvent : public EventSerializeable
{
	String mName;
	StreamEndEvent() : mName("StreamEnd")
	{
	}

	void serialize(PvdEventSerializer& s)
	{
		s.streamify(mName);
	}
};

struct OriginShift : public EventSerializeable
{
	uint64_t mInstanceId;
	PxVec3 mShift;

	OriginShift(uint64_t iid, const PxVec3& shift) : mInstanceId(iid), mShift(shift)
	{
	}
	OriginShift()
	{
	}

	void serialize(PvdEventSerializer& s)
	{
		s.streamify(mInstanceId);
		s.streamify(mShift);
	}
};
} // pvdsdk
} // physx

#endif // PXPVDSDK_PXPVDCOMMSTREAMEVENTS_H
