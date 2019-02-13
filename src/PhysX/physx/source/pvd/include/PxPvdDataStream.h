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
#ifndef PXPVDSDK_PXPVDDATASTREAM_H
#define PXPVDSDK_PXPVDDATASTREAM_H

/** \addtogroup pvd
@{
*/
#include "pvd/PxPvd.h"
#include "PxPvdErrorCodes.h"
#include "PxPvdObjectModelBaseTypes.h"

#if !PX_DOXYGEN
namespace physx
{
namespace pvdsdk
{
#endif

class PvdPropertyDefinitionHelper;

class PvdMetaDataStream
{
  protected:
	virtual ~PvdMetaDataStream()
	{
	}

  public:
	virtual PvdError createClass(const NamespacedName& nm) = 0;
	template <typename TDataType>
	PvdError createClass()
	{
		return createClass(getPvdNamespacedNameForType<TDataType>());
	}

	virtual PvdError deriveClass(const NamespacedName& parent, const NamespacedName& child) = 0;
	template <typename TParentType, typename TChildType>
	PvdError deriveClass()
	{
		return deriveClass(getPvdNamespacedNameForType<TParentType>(), getPvdNamespacedNameForType<TChildType>());
	}

	virtual bool isClassExist(const NamespacedName& nm) = 0;
	template <typename TDataType>
	bool isClassExist()
	{
		return isClassExist(getPvdNamespacedNameForType<TDataType>());
	}

	virtual PvdError createProperty(const NamespacedName& clsName, const char* name, const char* semantic,
	                                const NamespacedName& dtypeName, PropertyType::Enum propertyType,
	                                DataRef<NamedValue> values = DataRef<NamedValue>()) = 0;
	template <typename TClsType, typename TDataType>
	PvdError createProperty(String name, String semantic = "", PropertyType::Enum propertyType = PropertyType::Scalar,
	                        DataRef<NamedValue> values = DataRef<NamedValue>())
	{
		return createProperty(getPvdNamespacedNameForType<TClsType>(), name, semantic,
		                      getPvdNamespacedNameForType<TDataType>(), propertyType, values);
	}

	virtual PvdError createPropertyMessage(const NamespacedName& cls, const NamespacedName& msgName,
	                                       DataRef<PropertyMessageArg> entries, uint32_t messageSizeInBytes) = 0;

	template <typename TClsType, typename TMsgType>
	PvdError createPropertyMessage(DataRef<PropertyMessageArg> entries)
	{
		return createPropertyMessage(getPvdNamespacedNameForType<TClsType>(), getPvdNamespacedNameForType<TMsgType>(),
		                             entries, sizeof(TMsgType));
	}
};

class PvdInstanceDataStream
{
  protected:
	virtual ~PvdInstanceDataStream()
	{
	}

  public:
	virtual PvdError createInstance(const NamespacedName& cls, const void* instance) = 0;

	template <typename TDataType>
	PvdError createInstance(const TDataType* inst)
	{
		return createInstance(getPvdNamespacedNameForType<TDataType>(), inst);
	}
	virtual bool isInstanceValid(const void* instance) = 0;

	// If the property will fit or is already completely in memory
	virtual PvdError setPropertyValue(const void* instance, String name, DataRef<const uint8_t> data,
	                                  const NamespacedName& incomingTypeName) = 0;
	template <typename TDataType>
	PvdError setPropertyValue(const void* instance, String name, const TDataType& value)
	{
		const uint8_t* dataStart = reinterpret_cast<const uint8_t*>(&value);
		return setPropertyValue(instance, name, DataRef<const uint8_t>(dataStart, dataStart + sizeof(TDataType)),
		                        getPvdNamespacedNameForType<TDataType>());
	}

	template <typename TDataType>
	PvdError setPropertyValue(const void* instance, String name, const TDataType* value, uint32_t numItems)
	{
		const uint8_t* dataStart = reinterpret_cast<const uint8_t*>(value);
		return setPropertyValue(instance, name,
		                        DataRef<const uint8_t>(dataStart, dataStart + sizeof(TDataType) * numItems),
		                        getPvdNamespacedNameForType<TDataType>());
	}

	// Else if the property is very large (contact reports) you can send it in chunks.
	virtual PvdError beginSetPropertyValue(const void* instance, String name, const NamespacedName& incomingTypeName) = 0;

	template <typename TDataType>
	PvdError beginSetPropertyValue(const void* instance, String name)
	{
		return beginSetPropertyValue(instance, name, getPvdNamespacedNameForType<TDataType>());
	}
	virtual PvdError appendPropertyValueData(DataRef<const uint8_t> data) = 0;

	template <typename TDataType>
	PvdError appendPropertyValueData(const TDataType* value, uint32_t numItems)
	{
		const uint8_t* dataStart = reinterpret_cast<const uint8_t*>(value);
		return appendPropertyValueData(DataRef<const uint8_t>(dataStart, dataStart + numItems * sizeof(TDataType)));
	}

	virtual PvdError endSetPropertyValue() = 0;

	// Set a set of properties to various values on an object.

	virtual PvdError setPropertyMessage(const void* instance, const NamespacedName& msgName,
	                                    DataRef<const uint8_t> data) = 0;

	template <typename TDataType>
	PvdError setPropertyMessage(const void* instance, const TDataType& value)
	{
		const uint8_t* dataStart = reinterpret_cast<const uint8_t*>(&value);
		return setPropertyMessage(instance, getPvdNamespacedNameForType<TDataType>(),
		                          DataRef<const uint8_t>(dataStart, sizeof(TDataType)));
	}
	// If you need to send of lot of identical messages, this avoids a hashtable lookup per message.
	virtual PvdError beginPropertyMessageGroup(const NamespacedName& msgName) = 0;

	template <typename TDataType>
	PvdError beginPropertyMessageGroup()
	{
		return beginPropertyMessageGroup(getPvdNamespacedNameForType<TDataType>());
	}
	virtual PvdError sendPropertyMessageFromGroup(const void* instance, DataRef<const uint8_t> data) = 0;

	template <typename TDataType>
	PvdError sendPropertyMessageFromGroup(const void* instance, const TDataType& value)
	{
		const uint8_t* dataStart = reinterpret_cast<const uint8_t*>(&value);
		return sendPropertyMessageFromGroup(instance, DataRef<const uint8_t>(dataStart, sizeof(TDataType)));
	}

	virtual PvdError endPropertyMessageGroup() = 0;

	// These functions ensure the target array doesn't contain duplicates
	virtual PvdError pushBackObjectRef(const void* instId, String propName, const void* objRef) = 0;
	virtual PvdError removeObjectRef(const void* instId, String propName, const void* objRef) = 0;

	// Instance elimination.
	virtual PvdError destroyInstance(const void* key) = 0;

	// Profiling hooks
	virtual PvdError beginSection(const void* instance, String name) = 0;
	virtual PvdError endSection(const void* instance, String name) = 0;

	// Origin Shift
	virtual PvdError originShift(const void* scene, PxVec3 shift) = 0;

  public:
	/*For some cases, pvd command cannot be run immediately. For example, when create joints, while the actors may still
	*pending for insert, the joints update commands can be run deffered.
	*/
	class PvdCommand
	{
	  public:
		// Assigned is needed for copying
		PvdCommand(const PvdCommand&)
		{
		}
		PvdCommand& operator=(const PvdCommand&)
		{
			return *this;
		}

	  public:
		PvdCommand()
		{
		}
		virtual ~PvdCommand()
		{
		}

		// Not pure virtual so can have default PvdCommand obj
		virtual bool canRun(PvdInstanceDataStream&)
		{
			return false;
		}
		virtual void run(PvdInstanceDataStream&)
		{
		}
	};

	// PVD SDK provide this helper function to allocate cmd's memory and release them at after flush the command queue
	virtual void* allocateMemForCmd(uint32_t length) = 0;

	// PVD will call the destructor of PvdCommand object at the end fo flushPvdCommand
	virtual void pushPvdCommand(PvdCommand& cmd) = 0;
	virtual void flushPvdCommand() = 0;
};

class PvdDataStream : public PvdInstanceDataStream, public PvdMetaDataStream
{
  protected:
	virtual ~PvdDataStream()
	{
	}

  public:
	virtual void release() = 0;
	virtual bool isConnected() = 0;

	virtual void addProfileZone(void* zone, const char* name) = 0;
	virtual void addProfileZoneEvent(void* zone, const char* name, uint16_t eventId, bool compileTimeEnabled) = 0;

	virtual PvdPropertyDefinitionHelper& getPropertyDefinitionHelper() = 0;

	virtual void setIsTopLevelUIElement(const void* instance, bool topLevel) = 0;
	virtual void sendErrorMessage(uint32_t code, const char* message, const char* file, uint32_t line) = 0;
	virtual void updateCamera(const char* name, const PxVec3& origin, const PxVec3& up, const PxVec3& target) = 0;

/**
	\brief Create a new PvdDataStream.
	\param pvd A pointer to a valid PxPvd instance.  This must be non-null.	
*/
	static PvdDataStream* create(PxPvd* pvd);
};
#if !PX_DOXYGEN
} // pvdsdk
} // physx
#endif

/** @} */
#endif // PXPVDSDK_PXPVDDATASTREAM_H
