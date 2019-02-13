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

#ifndef PXPVDSDK_PXPVDMARSHALLING_H
#define PXPVDSDK_PXPVDMARSHALLING_H

#include "foundation/PxIntrinsics.h"

#include "PxPvdObjectModelBaseTypes.h"
#include "PxPvdBits.h"

namespace physx
{
namespace pvdsdk
{

// Define marshalling

template <typename TSmallerType, typename TLargerType>
struct PvdMarshalling
{
	bool canMarshal;
	PvdMarshalling() : canMarshal(false)
	{
	}
};

template <typename smtype, typename lgtype>
static inline void marshalSingleT(const uint8_t* srcData, uint8_t* destData)
{
	smtype incoming;

	physx::intrinsics::memCopy(&incoming, srcData, sizeof(smtype));
	lgtype outgoing = static_cast<lgtype>(incoming);
	physx::intrinsics::memCopy(destData, &outgoing, sizeof(lgtype));
}

template <typename smtype, typename lgtype>
static inline void marshalBlockT(const uint8_t* srcData, uint8_t* destData, uint32_t numBytes)
{
	for(const uint8_t* item = srcData, *end = srcData + numBytes; item < end;
	    item += sizeof(smtype), destData += sizeof(lgtype))
		marshalSingleT<smtype, lgtype>(item, destData);
}

#define PVD_TYPE_MARSHALLER(smtype, lgtype)                                                                            \
	template <>                                                                                                        \
	struct PvdMarshalling<smtype, lgtype>                                                                              \
	{                                                                                                                  \
		uint32_t canMarshal;                                                                                           \
		static void marshalSingle(const uint8_t* srcData, uint8_t* destData)                                           \
		{                                                                                                              \
			marshalSingleT<smtype, lgtype>(srcData, destData);                                                         \
		}                                                                                                              \
		static void marshalBlock(const uint8_t* srcData, uint8_t* destData, uint32_t numBytes)                         \
		{                                                                                                              \
			marshalBlockT<smtype, lgtype>(srcData, destData, numBytes);                                                \
		}                                                                                                              \
	};

// define marshalling tables.
PVD_TYPE_MARSHALLER(int8_t, int16_t)
PVD_TYPE_MARSHALLER(int8_t, uint16_t)
PVD_TYPE_MARSHALLER(int8_t, int32_t)
PVD_TYPE_MARSHALLER(int8_t, uint32_t)
PVD_TYPE_MARSHALLER(int8_t, int64_t)
PVD_TYPE_MARSHALLER(int8_t, uint64_t)
PVD_TYPE_MARSHALLER(int8_t, PvdF32)
PVD_TYPE_MARSHALLER(int8_t, PvdF64)

PVD_TYPE_MARSHALLER(uint8_t, int16_t)
PVD_TYPE_MARSHALLER(uint8_t, uint16_t)
PVD_TYPE_MARSHALLER(uint8_t, int32_t)
PVD_TYPE_MARSHALLER(uint8_t, uint32_t)
PVD_TYPE_MARSHALLER(uint8_t, int64_t)
PVD_TYPE_MARSHALLER(uint8_t, uint64_t)
PVD_TYPE_MARSHALLER(uint8_t, PvdF32)
PVD_TYPE_MARSHALLER(uint8_t, PvdF64)

PVD_TYPE_MARSHALLER(int16_t, int32_t)
PVD_TYPE_MARSHALLER(int16_t, uint32_t)
PVD_TYPE_MARSHALLER(int16_t, int64_t)
PVD_TYPE_MARSHALLER(int16_t, uint64_t)
PVD_TYPE_MARSHALLER(int16_t, PvdF32)
PVD_TYPE_MARSHALLER(int16_t, PvdF64)

PVD_TYPE_MARSHALLER(uint16_t, int32_t)
PVD_TYPE_MARSHALLER(uint16_t, uint32_t)
PVD_TYPE_MARSHALLER(uint16_t, int64_t)
PVD_TYPE_MARSHALLER(uint16_t, uint64_t)
PVD_TYPE_MARSHALLER(uint16_t, PvdF32)
PVD_TYPE_MARSHALLER(uint16_t, PvdF64)

PVD_TYPE_MARSHALLER(int32_t, int64_t)
PVD_TYPE_MARSHALLER(int32_t, uint64_t)
PVD_TYPE_MARSHALLER(int32_t, PvdF64)
PVD_TYPE_MARSHALLER(int32_t, PvdF32)

PVD_TYPE_MARSHALLER(uint32_t, int64_t)
PVD_TYPE_MARSHALLER(uint32_t, uint64_t)
PVD_TYPE_MARSHALLER(uint32_t, PvdF64)
PVD_TYPE_MARSHALLER(uint32_t, PvdF32)

PVD_TYPE_MARSHALLER(PvdF32, PvdF64)
PVD_TYPE_MARSHALLER(PvdF32, uint32_t)
PVD_TYPE_MARSHALLER(PvdF32, int32_t)

PVD_TYPE_MARSHALLER(uint64_t, PvdF64)
PVD_TYPE_MARSHALLER(int64_t, PvdF64)
PVD_TYPE_MARSHALLER(PvdF64, uint64_t)
PVD_TYPE_MARSHALLER(PvdF64, int64_t)

template <typename TMarshaller>
static inline bool getMarshalOperators(TSingleMarshaller&, TBlockMarshaller&, TMarshaller&, bool)
{
	return false;
}

template <typename TMarshaller>
static inline bool getMarshalOperators(TSingleMarshaller& single, TBlockMarshaller& block, TMarshaller&, uint32_t)
{
	single = TMarshaller::marshalSingle;
	block = TMarshaller::marshalBlock;
	return true;
}

template <typename smtype, typename lgtype>
static inline bool getMarshalOperators(TSingleMarshaller& single, TBlockMarshaller& block)
{
	single = NULL;
	block = NULL;
	PvdMarshalling<smtype, lgtype> marshaller = PvdMarshalling<smtype, lgtype>();
	return getMarshalOperators(single, block, marshaller, marshaller.canMarshal);
}

template <typename smtype>
static inline bool getMarshalOperators(TSingleMarshaller& single, TBlockMarshaller& block, int32_t lgtypeId)
{
	switch(lgtypeId)
	{
	case PvdBaseType::PvdI8: // int8_t:
		return getMarshalOperators<smtype, int8_t>(single, block);
	case PvdBaseType::PvdU8: // uint8_t:
		return getMarshalOperators<smtype, uint8_t>(single, block);
	case PvdBaseType::PvdI16: // int16_t:
		return getMarshalOperators<smtype, int16_t>(single, block);
	case PvdBaseType::PvdU16: // uint16_t:
		return getMarshalOperators<smtype, uint16_t>(single, block);
	case PvdBaseType::PvdI32: // int32_t:
		return getMarshalOperators<smtype, int32_t>(single, block);
	case PvdBaseType::PvdU32: // uint32_t:
		return getMarshalOperators<smtype, uint32_t>(single, block);
	case PvdBaseType::PvdI64: // int64_t:
		return getMarshalOperators<smtype, int64_t>(single, block);
	case PvdBaseType::PvdU64: // uint64_t:
		return getMarshalOperators<smtype, uint64_t>(single, block);
	case PvdBaseType::PvdF32:
		return getMarshalOperators<smtype, PvdF32>(single, block);
	case PvdBaseType::PvdF64:
		return getMarshalOperators<smtype, PvdF64>(single, block);
	}
	return false;
}

static inline bool getMarshalOperators(TSingleMarshaller& single, TBlockMarshaller& block, int32_t smtypeId,
                                       int32_t lgtypeId)
{
	switch(smtypeId)
	{
	case PvdBaseType::PvdI8: // int8_t:
		return getMarshalOperators<int8_t>(single, block, lgtypeId);
	case PvdBaseType::PvdU8: // uint8_t:
		return getMarshalOperators<uint8_t>(single, block, lgtypeId);
	case PvdBaseType::PvdI16: // int16_t:
		return getMarshalOperators<int16_t>(single, block, lgtypeId);
	case PvdBaseType::PvdU16: // uint16_t:
		return getMarshalOperators<uint16_t>(single, block, lgtypeId);
	case PvdBaseType::PvdI32: // int32_t:
		return getMarshalOperators<int32_t>(single, block, lgtypeId);
	case PvdBaseType::PvdU32: // uint32_t:
		return getMarshalOperators<uint32_t>(single, block, lgtypeId);
	case PvdBaseType::PvdI64: // int64_t:
		return getMarshalOperators<int64_t>(single, block, lgtypeId);
	case PvdBaseType::PvdU64: // uint64_t:
		return getMarshalOperators<uint64_t>(single, block, lgtypeId);
	case PvdBaseType::PvdF32:
		return getMarshalOperators<PvdF32>(single, block, lgtypeId);
	case PvdBaseType::PvdF64:
		return getMarshalOperators<PvdF64>(single, block, lgtypeId);
	}
	return false;
}
}
}

#endif // PXPVDSDK_PXPVDMARSHALLING_H
