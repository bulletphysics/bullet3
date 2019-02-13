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

#ifndef PXPVDSDK_PXPVDOBJECTMODELINTERNALTYPES_H
#define PXPVDSDK_PXPVDOBJECTMODELINTERNALTYPES_H

#include "foundation/PxMemory.h"
#include "PxPvdObjectModelBaseTypes.h"
#include "PsArray.h"
#include "PxPvdFoundation.h"

namespace physx
{
namespace pvdsdk
{

struct PvdInternalType
{
	enum Enum
	{
		None = 0,
#define DECLARE_INTERNAL_PVD_TYPE(type) type,
#include "PxPvdObjectModelInternalTypeDefs.h"
		Last
#undef DECLARE_INTERNAL_PVD_TYPE
	};
};

PX_COMPILE_TIME_ASSERT(uint32_t(PvdInternalType::Last) <= uint32_t(PvdBaseType::InternalStop));

template <typename T>
struct DataTypeToPvdTypeMap
{
	bool compile_error;
};
template <PvdInternalType::Enum>
struct PvdTypeToDataTypeMap
{
	bool compile_error;
};

#define DECLARE_INTERNAL_PVD_TYPE(type)                                                                                \
	template <>                                                                                                        \
	struct DataTypeToPvdTypeMap<type>                                                                                  \
	{                                                                                                                  \
		enum Enum                                                                                                      \
		{                                                                                                              \
			BaseTypeEnum = PvdInternalType::type                                                                       \
		};                                                                                                             \
	};                                                                                                                 \
	template <>                                                                                                        \
	struct PvdTypeToDataTypeMap<PvdInternalType::type>                                                                 \
	{                                                                                                                  \
		typedef type TDataType;                                                                                        \
	};                                                                                                                 \
	template <>                                                                                                        \
	struct PvdDataTypeToNamespacedNameMap<type>                                                                        \
	{                                                                                                                  \
		NamespacedName Name;                                                                                           \
		PvdDataTypeToNamespacedNameMap<type>() : Name("physx3_debugger_internal", #type)                               \
		{                                                                                                              \
		}                                                                                                              \
	};
#include "PxPvdObjectModelInternalTypeDefs.h"
#undef DECLARE_INTERNAL_PVD_TYPE

template <typename TDataType, typename TAlloc>
DataRef<TDataType> toDataRef(const shdfnd::Array<TDataType, TAlloc>& data)
{
	return DataRef<TDataType>(data.begin(), data.end());
}

static inline bool safeStrEq(const DataRef<String>& lhs, const DataRef<String>& rhs)
{
	uint32_t count = lhs.size();
	if(count != rhs.size())
		return false;
	for(uint32_t idx = 0; idx < count; ++idx)
		if(!safeStrEq(lhs[idx], rhs[idx]))
			return false;
	return true;
}

static inline char* copyStr(const char* str)
{
	str = nonNull(str);
	uint32_t len = static_cast<uint32_t>(strlen(str));
	char* newData = reinterpret_cast<char*>(PX_ALLOC(len + 1, "string"));
	PxMemCopy(newData, str, len);
	newData[len] = 0;
	return newData;
}

// Used for predictable bit fields.
template <typename TDataType, uint8_t TNumBits, uint8_t TOffset, typename TInputType>
struct BitMaskSetter
{
	// Create a mask that masks out the orginal value shift into place
	static TDataType createOffsetMask()
	{
		return createMask() << TOffset;
	}
	// Create a mask of TNumBits number of tis
	static TDataType createMask()
	{
		return static_cast<TDataType>((1 << TNumBits) - 1);
	}
	void setValue(TDataType& inCurrent, TInputType inData)
	{
		PX_ASSERT(inData < (1 << TNumBits));

		// Create a mask to remove the current value.
		TDataType theMask = ~(createOffsetMask());
		// Clear out current value.
		inCurrent = inCurrent & theMask;
		// Create the new value.
		TDataType theAddition = reinterpret_cast<TDataType>(inData << TOffset);
		// or it into the existing value.
		inCurrent = inCurrent | theAddition;
	}

	TInputType getValue(TDataType inCurrent)
	{
		return static_cast<TInputType>((inCurrent >> TOffset) & createMask());
	}
};

}
}
#endif // PXPVDSDK_PXPVDOBJECTMODELINTERNALTYPES_H
