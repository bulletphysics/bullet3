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

#ifndef PXPVDSDK_PXPVDCOMMSTREAMTYPES_H
#define PXPVDSDK_PXPVDCOMMSTREAMTYPES_H

#include "foundation/PxErrorCallback.h"
#include "pvd/PxPvdTransport.h"

#include "PxPvdRenderBuffer.h"
#include "PxPvdObjectModelBaseTypes.h"
#include "PxPvdCommStreamEvents.h"
#include "PxPvdDataStream.h"
#include "PsMutex.h"

namespace physx
{
namespace profile
{
class PxProfileZone;
class PxProfileMemoryEventBuffer;
}
namespace pvdsdk
{
struct PvdErrorMessage;
class PvdObjectModelMetaData;

DEFINE_PVD_TYPE_NAME_MAP(profile::PxProfileZone, "_debugger_", "PxProfileZone")
DEFINE_PVD_TYPE_NAME_MAP(profile::PxProfileMemoryEventBuffer, "_debugger_", "PxProfileMemoryEventBuffer")
DEFINE_PVD_TYPE_NAME_MAP(PvdErrorMessage, "_debugger_", "PvdErrorMessage")
// All event streams are on the 'events' property of objects of these types
static inline NamespacedName getMemoryEventTotalsClassName()
{
	return NamespacedName("_debugger", "MemoryEventTotals");
}

class PvdOMMetaDataProvider
{
  protected:
	virtual ~PvdOMMetaDataProvider()
	{
	}

  public:
	virtual void addRef() = 0;
	virtual void release() = 0;
	virtual PvdObjectModelMetaData& lock() = 0;
	virtual void unlock() = 0;
	virtual bool createInstance(const NamespacedName& clsName, const void* instance) = 0;
	virtual bool isInstanceValid(const void* instance) = 0;
	virtual void destroyInstance(const void* instance) = 0;
	virtual int32_t getInstanceClassType(const void* instance) = 0;
};

class PvdCommStreamEmbeddedTypes
{
  public:
	static const char* getProfileEventStreamSemantic()
	{
		return "profile event stream";
	}
	static const char* getMemoryEventStreamSemantic()
	{
		return "memory event stream";
	}
	static const char* getRendererEventStreamSemantic()
	{
		return "render event stream";
	}
};

class PvdCommStreamEventBufferClient;

template <typename TStreamType>
struct EventStreamifier : public PvdEventSerializer
{
	TStreamType& mBuffer;
	EventStreamifier(TStreamType& buf) : mBuffer(buf)
	{
	}

	template <typename TDataType>
	void write(const TDataType& type)
	{
		mBuffer.write(reinterpret_cast<const uint8_t*>(&type), sizeof(TDataType));
	}
	template <typename TDataType>
	void write(const TDataType* type, uint32_t count)
	{
		mBuffer.write(reinterpret_cast<const uint8_t*>(type), count * sizeof(TDataType));
	}

	void writeRef(DataRef<const uint8_t> data)
	{
		uint32_t amount = static_cast<uint32_t>(data.size());
		write(amount);
		write(data.begin(), amount);
	}
	void writeRef(DataRef<StringHandle> data)
	{
		uint32_t amount = static_cast<uint32_t>(data.size());
		write(amount);
		write(data.begin(), amount);
	}
	template <typename TDataType>
	void writeRef(DataRef<TDataType> data)
	{
		uint32_t amount = static_cast<uint32_t>(data.size());
		write(amount);
		for(uint32_t idx = 0; idx < amount; ++idx)
		{
			TDataType& dtype(const_cast<TDataType&>(data[idx]));
			dtype.serialize(*this);
		}
	}

	virtual void streamify(uint16_t& val)
	{
		write(val);
	}
	virtual void streamify(uint8_t& val)
	{
		write(val);
	}
	virtual void streamify(uint32_t& val)
	{
		write(val);
	}
	virtual void streamify(float& val)
	{
		write(val);
	}
	virtual void streamify(uint64_t& val)
	{
		write(val);
	}
	virtual void streamify(PvdDebugText& val)
	{
		write(val.color);
		write(val.position);
		write(val.size);
		streamify(val.string);
	}

	virtual void streamify(String& val)
	{
		uint32_t len = 0;
		String temp = nonNull(val);
		if(*temp)
			len = static_cast<uint32_t>(strlen(temp) + 1);
		write(len);
		write(val, len);
	}
	virtual void streamify(DataRef<const uint8_t>& val)
	{
		writeRef(val);
	}
	virtual void streamify(DataRef<NameHandleValue>& val)
	{
		writeRef(val);
	}
	virtual void streamify(DataRef<StreamPropMessageArg>& val)
	{
		writeRef(val);
	}
	virtual void streamify(DataRef<StringHandle>& val)
	{
		writeRef(val);
	}

  private:
	EventStreamifier& operator=(const EventStreamifier&);
};

struct MeasureStream
{
	uint32_t mSize;
	MeasureStream() : mSize(0)
	{
	}
	template <typename TDataType>
	void write(const TDataType& val)
	{
		mSize += sizeof(val);
	}
	template <typename TDataType>
	void write(const TDataType*, uint32_t count)
	{
		mSize += sizeof(TDataType) * count;
	}
};

struct DataStreamState
{
	enum Enum
	{
		Open,
		SetPropertyValue,
		PropertyMessageGroup
	};
};

} // pvdsdk
} // physx
#endif // PXPVDSDK_PXPVDCOMMSTREAMTYPES_H
