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

#ifndef PXFOUNDATION_PXFLAGS_H
#define PXFOUNDATION_PXFLAGS_H

/** \addtogroup foundation
  @{
*/

#include "foundation/Px.h"

#if !PX_DOXYGEN
namespace physx
{
#endif
/**
\brief Container for bitfield flag variables associated with a specific enum type.

This allows for type safe manipulation for bitfields.

<h3>Example</h3>
    // enum that defines each bit...
    struct MyEnum
    {
        enum Enum
        {
            eMAN  = 1,
            eBEAR = 2,
            ePIG  = 4,
        };
    };

    // implements some convenient global operators.
    PX_FLAGS_OPERATORS(MyEnum::Enum, uint8_t);

    PxFlags<MyEnum::Enum, uint8_t> myFlags;
    myFlags |= MyEnum::eMAN;
    myFlags |= MyEnum::eBEAR | MyEnum::ePIG;
    if(myFlags & MyEnum::eBEAR)
    {
        doSomething();
    }
*/

template <typename enumtype, typename storagetype = uint32_t>
class PxFlags
{
  public:
	typedef storagetype InternalType;

	PX_CUDA_CALLABLE PX_INLINE explicit PxFlags(const PxEMPTY)
	{
	}
	PX_CUDA_CALLABLE PX_INLINE PxFlags(void);
	PX_CUDA_CALLABLE PX_INLINE PxFlags(enumtype e);
	PX_CUDA_CALLABLE PX_INLINE PxFlags(const PxFlags<enumtype, storagetype>& f);
	PX_CUDA_CALLABLE PX_INLINE explicit PxFlags(storagetype b);

	PX_CUDA_CALLABLE PX_INLINE bool isSet(enumtype e) const;
	PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype>& set(enumtype e);
	PX_CUDA_CALLABLE PX_INLINE bool operator==(enumtype e) const;
	PX_CUDA_CALLABLE PX_INLINE bool operator==(const PxFlags<enumtype, storagetype>& f) const;
	PX_CUDA_CALLABLE PX_INLINE bool operator==(bool b) const;
	PX_CUDA_CALLABLE PX_INLINE bool operator!=(enumtype e) const;
	PX_CUDA_CALLABLE PX_INLINE bool operator!=(const PxFlags<enumtype, storagetype>& f) const;

	PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype>& operator=(const PxFlags<enumtype, storagetype>& f);
	PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype>& operator=(enumtype e);

	PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype>& operator|=(enumtype e);
	PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype>& operator|=(const PxFlags<enumtype, storagetype>& f);
	PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype> operator|(enumtype e) const;
	PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype> operator|(const PxFlags<enumtype, storagetype>& f) const;

	PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype>& operator&=(enumtype e);
	PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype>& operator&=(const PxFlags<enumtype, storagetype>& f);
	PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype> operator&(enumtype e) const;
	PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype> operator&(const PxFlags<enumtype, storagetype>& f) const;

	PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype>& operator^=(enumtype e);
	PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype>& operator^=(const PxFlags<enumtype, storagetype>& f);
	PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype> operator^(enumtype e) const;
	PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype> operator^(const PxFlags<enumtype, storagetype>& f) const;

	PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype> operator~(void) const;

	PX_CUDA_CALLABLE PX_INLINE operator bool(void) const;
	PX_CUDA_CALLABLE PX_INLINE operator uint8_t(void) const;
	PX_CUDA_CALLABLE PX_INLINE operator uint16_t(void) const;
	PX_CUDA_CALLABLE PX_INLINE operator uint32_t(void) const;

	PX_CUDA_CALLABLE PX_INLINE void clear(enumtype e);

  public:
	friend PX_INLINE PxFlags<enumtype, storagetype> operator&(enumtype a, PxFlags<enumtype, storagetype>& b)
	{
		PxFlags<enumtype, storagetype> out;
		out.mBits = a & b.mBits;
		return out;
	}

  private:
	storagetype mBits;
};

#if !PX_DOXYGEN

#define PX_FLAGS_OPERATORS(enumtype, storagetype)                                                                      \
	PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype> operator|(enumtype a, enumtype b)                                         \
	{                                                                                                                  \
		PxFlags<enumtype, storagetype> r(a);                                                                           \
		r |= b;                                                                                                        \
		return r;                                                                                                      \
	}                                                                                                                  \
	PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype> operator&(enumtype a, enumtype b)                                         \
	{                                                                                                                  \
		PxFlags<enumtype, storagetype> r(a);                                                                           \
		r &= b;                                                                                                        \
		return r;                                                                                                      \
	}                                                                                                                  \
	PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype> operator~(enumtype a)                                                     \
	{                                                                                                                  \
		return ~PxFlags<enumtype, storagetype>(a);                                                                     \
	}

#define PX_FLAGS_TYPEDEF(x, y)                                                                                         \
	typedef PxFlags<x::Enum, y> x##s;                                                                                  \
	PX_FLAGS_OPERATORS(x::Enum, y)

template <typename enumtype, typename storagetype>
PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype>::PxFlags(void)
{
	mBits = 0;
}

template <typename enumtype, typename storagetype>
PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype>::PxFlags(enumtype e)
{
	mBits = static_cast<storagetype>(e);
}

template <typename enumtype, typename storagetype>
PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype>::PxFlags(const PxFlags<enumtype, storagetype>& f)
{
	mBits = f.mBits;
}

template <typename enumtype, typename storagetype>
PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype>::PxFlags(storagetype b)
{
	mBits = b;
}

template <typename enumtype, typename storagetype>
PX_CUDA_CALLABLE PX_INLINE bool PxFlags<enumtype, storagetype>::isSet(enumtype e) const
{
	return (mBits & static_cast<storagetype>(e)) == static_cast<storagetype>(e);
}

template <typename enumtype, typename storagetype>
PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype>& PxFlags<enumtype, storagetype>::set(enumtype e)
{
	mBits = static_cast<storagetype>(e);
	return *this;
}

template <typename enumtype, typename storagetype>
PX_CUDA_CALLABLE PX_INLINE bool PxFlags<enumtype, storagetype>::operator==(enumtype e) const
{
	return mBits == static_cast<storagetype>(e);
}

template <typename enumtype, typename storagetype>
PX_CUDA_CALLABLE PX_INLINE bool PxFlags<enumtype, storagetype>::operator==(const PxFlags<enumtype, storagetype>& f) const
{
	return mBits == f.mBits;
}

template <typename enumtype, typename storagetype>
PX_CUDA_CALLABLE PX_INLINE bool PxFlags<enumtype, storagetype>::operator==(bool b) const
{
	return bool(*this) == b;
}

template <typename enumtype, typename storagetype>
PX_CUDA_CALLABLE PX_INLINE bool PxFlags<enumtype, storagetype>::operator!=(enumtype e) const
{
	return mBits != static_cast<storagetype>(e);
}

template <typename enumtype, typename storagetype>
PX_CUDA_CALLABLE PX_INLINE bool PxFlags<enumtype, storagetype>::operator!=(const PxFlags<enumtype, storagetype>& f) const
{
	return mBits != f.mBits;
}

template <typename enumtype, typename storagetype>
PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype>& PxFlags<enumtype, storagetype>::operator=(enumtype e)
{
	mBits = static_cast<storagetype>(e);
	return *this;
}

template <typename enumtype, typename storagetype>
PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype>& PxFlags<enumtype, storagetype>::operator=(const PxFlags<enumtype, storagetype>& f)
{
	mBits = f.mBits;
	return *this;
}

template <typename enumtype, typename storagetype>
PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype>& PxFlags<enumtype, storagetype>::operator|=(enumtype e)
{
	mBits |= static_cast<storagetype>(e);
	return *this;
}

template <typename enumtype, typename storagetype>
PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype>& PxFlags<enumtype, storagetype>::
operator|=(const PxFlags<enumtype, storagetype>& f)
{
	mBits |= f.mBits;
	return *this;
}

template <typename enumtype, typename storagetype>
PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype> PxFlags<enumtype, storagetype>::operator|(enumtype e) const
{
	PxFlags<enumtype, storagetype> out(*this);
	out |= e;
	return out;
}

template <typename enumtype, typename storagetype>
PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype> PxFlags<enumtype, storagetype>::
operator|(const PxFlags<enumtype, storagetype>& f) const
{
	PxFlags<enumtype, storagetype> out(*this);
	out |= f;
	return out;
}

template <typename enumtype, typename storagetype>
PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype>& PxFlags<enumtype, storagetype>::operator&=(enumtype e)
{
	mBits &= static_cast<storagetype>(e);
	return *this;
}

template <typename enumtype, typename storagetype>
PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype>& PxFlags<enumtype, storagetype>::
operator&=(const PxFlags<enumtype, storagetype>& f)
{
	mBits &= f.mBits;
	return *this;
}

template <typename enumtype, typename storagetype>
PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype> PxFlags<enumtype, storagetype>::operator&(enumtype e) const
{
	PxFlags<enumtype, storagetype> out = *this;
	out.mBits &= static_cast<storagetype>(e);
	return out;
}

template <typename enumtype, typename storagetype>
PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype> PxFlags<enumtype, storagetype>::
operator&(const PxFlags<enumtype, storagetype>& f) const
{
	PxFlags<enumtype, storagetype> out = *this;
	out.mBits &= f.mBits;
	return out;
}

template <typename enumtype, typename storagetype>
PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype>& PxFlags<enumtype, storagetype>::operator^=(enumtype e)
{
	mBits ^= static_cast<storagetype>(e);
	return *this;
}

template <typename enumtype, typename storagetype>
PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype>& PxFlags<enumtype, storagetype>::
operator^=(const PxFlags<enumtype, storagetype>& f)
{
	mBits ^= f.mBits;
	return *this;
}

template <typename enumtype, typename storagetype>
PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype> PxFlags<enumtype, storagetype>::operator^(enumtype e) const
{
	PxFlags<enumtype, storagetype> out = *this;
	out.mBits ^= static_cast<storagetype>(e);
	return out;
}

template <typename enumtype, typename storagetype>
PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype> PxFlags<enumtype, storagetype>::
operator^(const PxFlags<enumtype, storagetype>& f) const
{
	PxFlags<enumtype, storagetype> out = *this;
	out.mBits ^= f.mBits;
	return out;
}

template <typename enumtype, typename storagetype>
PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype> PxFlags<enumtype, storagetype>::operator~(void) const
{
	PxFlags<enumtype, storagetype> out;
	out.mBits = storagetype(~mBits);
	return out;
}

template <typename enumtype, typename storagetype>
PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype>::operator bool(void) const
{
	return mBits ? true : false;
}

template <typename enumtype, typename storagetype>
PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype>::operator uint8_t(void) const
{
	return static_cast<uint8_t>(mBits);
}

template <typename enumtype, typename storagetype>
PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype>::operator uint16_t(void) const
{
	return static_cast<uint16_t>(mBits);
}

template <typename enumtype, typename storagetype>
PX_CUDA_CALLABLE PX_INLINE PxFlags<enumtype, storagetype>::operator uint32_t(void) const
{
	return static_cast<uint32_t>(mBits);
}

template <typename enumtype, typename storagetype>
PX_CUDA_CALLABLE PX_INLINE void PxFlags<enumtype, storagetype>::clear(enumtype e)
{
	mBits &= ~static_cast<storagetype>(e);
}

} // namespace physx
#endif //!PX_DOXYGEN

/** @} */
#endif // #ifndef PXFOUNDATION_PXFLAGS_H
