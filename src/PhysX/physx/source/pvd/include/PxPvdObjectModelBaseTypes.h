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

#ifndef PXPVDSDK_PXPVDOBJECTMODELBASETYPES_H
#define PXPVDSDK_PXPVDOBJECTMODELBASETYPES_H

/** \addtogroup pvd
@{
*/
#include "foundation/PxAssert.h"

#if !PX_DOXYGEN
namespace physx
{
namespace pvdsdk
{
#endif

using namespace physx;

inline const char* nonNull(const char* str)
{
	return str ? str : "";
}
// strcmp will crash if passed a null string, however,
// so we need to make sure that doesn't happen.  We do that
// by equating NULL and the empty string, "".
inline bool safeStrEq(const char* lhs, const char* rhs)
{
	return ::strcmp(nonNull(lhs), nonNull(rhs)) == 0;
}

// Does this string have useful information in it.
inline bool isMeaningful(const char* str)
{
	return *(nonNull(str)) > 0;
}

inline uint32_t safeStrLen(const char* str)
{
	str = nonNull(str);
	return static_cast<uint32_t>(strlen(str));
}

struct ObjectRef
{
	int32_t mInstanceId;

	ObjectRef(int32_t iid = -1) : mInstanceId(iid)
	{
	}
	operator int32_t() const
	{
		return mInstanceId;
	}
	bool hasValue() const
	{
		return mInstanceId > 0;
	}
};

struct U32Array4
{
	uint32_t mD0;
	uint32_t mD1;
	uint32_t mD2;
	uint32_t mD3;
	U32Array4(uint32_t d0, uint32_t d1, uint32_t d2, uint32_t d3) : mD0(d0), mD1(d1), mD2(d2), mD3(d3)
	{
	}
	U32Array4() : mD0(0), mD1(0), mD2(0), mD3(0)
	{
	}
};

typedef bool				PvdBool;
typedef const char*			String;
typedef void*				VoidPtr;
typedef double				PvdF64;
typedef float				PvdF32;
typedef int64_t				PvdI64;
typedef uint64_t			PvdU64;
typedef int32_t				PvdI32;
typedef uint32_t			PvdU32;
typedef int16_t				PvdI16;
typedef uint16_t			PvdU16;
typedef int8_t				PvdI8;
typedef uint8_t				PvdU8;

struct PvdColor
{
	uint8_t r;
	uint8_t g;
	uint8_t b;
	uint8_t a;
	PvdColor(uint8_t _r, uint8_t _g, uint8_t _b, uint8_t _a = 255) : r(_r), g(_g), b(_b), a(_a)
	{
	}
	PvdColor() : r(0), g(0), b(0), a(255)
	{
	}
	PvdColor(uint32_t abgr)
	{
		uint8_t* valPtr = reinterpret_cast<uint8_t*>(&abgr);
		r = valPtr[0];
		g = valPtr[1];
		b = valPtr[2];
		a = valPtr[3];
	}
};

struct StringHandle
{
	uint32_t mHandle;
	StringHandle(uint32_t val = 0) : mHandle(val)
	{
	}
	operator uint32_t() const
	{
		return mHandle;
	}
};

#define DECLARE_TYPES					\
DECLARE_BASE_PVD_TYPE(PvdI8)			\
DECLARE_BASE_PVD_TYPE(PvdU8)			\
DECLARE_BASE_PVD_TYPE(PvdI16)			\
DECLARE_BASE_PVD_TYPE(PvdU16)			\
DECLARE_BASE_PVD_TYPE(PvdI32)			\
DECLARE_BASE_PVD_TYPE(PvdU32)			\
DECLARE_BASE_PVD_TYPE(PvdI64)			\
DECLARE_BASE_PVD_TYPE(PvdU64)			\
DECLARE_BASE_PVD_TYPE(PvdF32)			\
DECLARE_BASE_PVD_TYPE(PvdF64)			\
DECLARE_BASE_PVD_TYPE(PvdBool)			\
DECLARE_BASE_PVD_TYPE(PvdColor)			\
DECLARE_BASE_PVD_TYPE(String)			\
DECLARE_BASE_PVD_TYPE(StringHandle)		\
DECLARE_BASE_PVD_TYPE(ObjectRef)		\
DECLARE_BASE_PVD_TYPE(VoidPtr)			\
DECLARE_BASE_PVD_TYPE(PxVec2)			\
DECLARE_BASE_PVD_TYPE(PxVec3)			\
DECLARE_BASE_PVD_TYPE(PxVec4)			\
DECLARE_BASE_PVD_TYPE(PxBounds3)		\
DECLARE_BASE_PVD_TYPE(PxQuat)			\
DECLARE_BASE_PVD_TYPE(PxTransform)		\
DECLARE_BASE_PVD_TYPE(PxMat33)			\
DECLARE_BASE_PVD_TYPE(PxMat44)			\
DECLARE_BASE_PVD_TYPE(U32Array4)		

struct PvdBaseType
{
	enum Enum
	{
		None          = 0,
		InternalStart = 1,
		InternalStop  = 64,
#define DECLARE_BASE_PVD_TYPE(type) type,
		DECLARE_TYPES
		Last
#undef DECLARE_BASE_PVD_TYPE
	};
};
struct NamespacedName
{
	String mNamespace;
	String mName;
	NamespacedName(String ns, String nm) : mNamespace(ns), mName(nm)
	{
	}
	NamespacedName(String nm = "") : mNamespace(""), mName(nm)
	{
	}
	bool operator==(const NamespacedName& other) const
	{
		return safeStrEq(mNamespace, other.mNamespace) && safeStrEq(mName, other.mName);
	}
};

struct NamedValue
{
	String mName;
	uint32_t mValue;
	NamedValue(String nm = "", uint32_t val = 0) : mName(nm), mValue(val)
	{
	}
};

template <typename T>
struct BaseDataTypeToTypeMap
{
	bool compile_error;
};
template <PvdBaseType::Enum>
struct BaseTypeToDataTypeMap
{
	bool compile_error;
};

// Users can extend this mapping with new datatypes.
template <typename T>
struct PvdDataTypeToNamespacedNameMap
{
	bool Name;
};
// This mapping tells you the what class id to use for the base datatypes
//
#define DECLARE_BASE_PVD_TYPE(type)                                                                                    \
	template <>                                                                                                        \
	struct BaseDataTypeToTypeMap<type>                                                                                 \
	{                                                                                                                  \
		enum Enum                                                                                                      \
		{                                                                                                              \
			BaseTypeEnum = PvdBaseType::type                                                                           \
		};                                                                                                             \
	};                                                                                                                 \
	template <>                                                                                                        \
	struct BaseDataTypeToTypeMap<const type&>                                                                          \
	{                                                                                                                  \
		enum Enum                                                                                                      \
		{                                                                                                              \
			BaseTypeEnum = PvdBaseType::type                                                                           \
		};                                                                                                             \
	};                                                                                                                 \
	template <>                                                                                                        \
	struct BaseTypeToDataTypeMap<PvdBaseType::type>                                                                    \
	{                                                                                                                  \
		typedef type TDataType;                                                                                        \
	};                                                                                                                 \
	template <>                                                                                                        \
	struct PvdDataTypeToNamespacedNameMap<type>                                                                        \
	{                                                                                                                  \
		NamespacedName Name;                                                                                           \
		PvdDataTypeToNamespacedNameMap<type>() : Name("physx3", #type)                                                 \
		{                                                                                                              \
		}                                                                                                              \
	};                                                                                                                 \
	template <>                                                                                                        \
	struct PvdDataTypeToNamespacedNameMap<const type&>                                                                 \
	{                                                                                                                  \
		NamespacedName Name;                                                                                           \
		PvdDataTypeToNamespacedNameMap<const type&>() : Name("physx3", #type)                                          \
		{                                                                                                              \
		}                                                                                                              \
	};

DECLARE_TYPES
#undef DECLARE_BASE_PVD_TYPE

template <typename TDataType>
inline int32_t getPvdTypeForType()
{
	return static_cast<PvdBaseType::Enum>(BaseDataTypeToTypeMap<TDataType>::BaseTypeEnum);
}
template <typename TDataType>
inline NamespacedName getPvdNamespacedNameForType()
{
	return PvdDataTypeToNamespacedNameMap<TDataType>().Name;
}

#define DEFINE_PVD_TYPE_NAME_MAP(type, ns, name)                                                                       \
	template <>                                                                                                        \
	struct PvdDataTypeToNamespacedNameMap<type>                                                                        \
	{                                                                                                                  \
		NamespacedName Name;                                                                                           \
		PvdDataTypeToNamespacedNameMap<type>() : Name(ns, name)                                                        \
		{                                                                                                              \
		}                                                                                                              \
	};

#define DEFINE_PVD_TYPE_ALIAS(newType, oldType)                                                                        \
	template <>                                                                                                        \
	struct PvdDataTypeToNamespacedNameMap<newType>                                                                     \
	{                                                                                                                  \
		NamespacedName Name;                                                                                           \
		PvdDataTypeToNamespacedNameMap<newType>() : Name(PvdDataTypeToNamespacedNameMap<oldType>().Name)               \
		{                                                                                                              \
		}                                                                                                              \
	};

DEFINE_PVD_TYPE_ALIAS(const void*, void*)

struct ArrayData
{
	uint8_t* mBegin;
	uint8_t* mEnd;
	uint8_t* mCapacity; //>= stop
	ArrayData(uint8_t* beg = NULL, uint8_t* end = NULL, uint8_t* cap = NULL) : mBegin(beg), mEnd(end), mCapacity(cap)
	{
	}
	uint8_t* begin()
	{
		return mBegin;
	}
	uint8_t* end()
	{
		return mEnd;
	}
	uint32_t byteCapacity()
	{
		return static_cast<uint32_t>(mCapacity - mBegin);
	}
	uint32_t byteSize() const
	{
		return static_cast<uint32_t>(mEnd - mBegin);
	} // in bytes
	uint32_t numberOfItems(uint32_t objectByteSize)
	{
		if(objectByteSize)
			return byteSize() / objectByteSize;
		return 0;
	}

	void forgetData()
	{
		mBegin = mEnd = mCapacity = 0;
	}
};

template <typename T>
class DataRef
{
	const T* mBegin;
	const T* mEnd;

  public:
	DataRef(const T* b, uint32_t count) : mBegin(b), mEnd(b + count)
	{
	}
	DataRef(const T* b = NULL, const T* e = NULL) : mBegin(b), mEnd(e)
	{
	}
	DataRef(const DataRef& o) : mBegin(o.mBegin), mEnd(o.mEnd)
	{
	}
	DataRef& operator=(const DataRef& o)
	{
		mBegin = o.mBegin;
		mEnd = o.mEnd;
		return *this;
	}
	uint32_t size() const
	{
		return static_cast<uint32_t>(mEnd - mBegin);
	}
	const T* begin() const
	{
		return mBegin;
	}
	const T* end() const
	{
		return mEnd;
	}
	const T& operator[](uint32_t idx) const
	{
		PX_ASSERT(idx < size());
		return mBegin[idx];
	}
	const T& back() const
	{
		PX_ASSERT(mEnd > mBegin);
		return *(mEnd - 1);
	}
};

struct PropertyType
{
	enum Enum
	{
		Unknown = 0,
		Scalar,
		Array
	};
};

// argument to the create property message function
struct PropertyMessageArg
{
	String mPropertyName;
	NamespacedName mDatatypeName;
	// where in the message this property starts.
	uint32_t mMessageOffset;
	// size of this entry object
	uint32_t mByteSize;

	PropertyMessageArg(String propName, NamespacedName dtype, uint32_t msgOffset, uint32_t byteSize)
	: mPropertyName(propName), mDatatypeName(dtype), mMessageOffset(msgOffset), mByteSize(byteSize)
	{
	}
	PropertyMessageArg() : mPropertyName(""), mMessageOffset(0), mByteSize(0)
	{
	}
};

class PvdUserRenderer;
DEFINE_PVD_TYPE_NAME_MAP(PvdUserRenderer, "_debugger_", "PvdUserRenderer")

#if !PX_DOXYGEN
}
}
#endif

/** @} */
#endif // PXPVDSDK_PXPVDOBJECTMODELBASETYPES_H
