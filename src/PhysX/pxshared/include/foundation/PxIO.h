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
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#ifndef PXFOUNDATION_PXIO_H
#define PXFOUNDATION_PXIO_H

/** \addtogroup common
  @{
*/

#include "foundation/PxSimpleTypes.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/**
\brief Input stream class for I/O.

The user needs to supply a PxInputStream implementation to a number of methods to allow the SDK to read data.
*/

class PxInputStream
{
  public:
	/**
	\brief read from the stream. The number of bytes read may be less than the number requested.

	\param[in] dest the destination address to which the data will be read
	\param[in] count the number of bytes requested

	\return the number of bytes read from the stream.
	*/

	virtual uint32_t read(void* dest, uint32_t count) = 0;

	virtual ~PxInputStream()
	{
	}
};

/**
\brief Input data class for I/O which provides random read access.

The user needs to supply a PxInputData implementation to a number of methods to allow the SDK to read data.
*/

class PxInputData : public PxInputStream
{
  public:
	/**
	\brief return the length of the input data

	\return size in bytes of the input data
	*/

	virtual uint32_t getLength() const = 0;

	/**
	\brief seek to the given offset from the start of the data.

	\param[in] offset the offset to seek to. 	If greater than the length of the data, this call is equivalent to
	seek(length);
	*/

	virtual void seek(uint32_t offset) = 0;

	/**
	\brief return the current offset from the start of the data

	\return the offset to seek to.
	*/

	virtual uint32_t tell() const = 0;

	virtual ~PxInputData()
	{
	}
};

/**
\brief Output stream class for I/O.

The user needs to supply a PxOutputStream implementation to a number of methods to allow the SDK to write data.
*/

class PxOutputStream
{
  public:
	/**
	\brief write to the stream. The number of bytes written may be less than the number sent.

	\param[in] src the destination address from which the data will be written
	\param[in] count the number of bytes to be written

	\return the number of bytes written to the stream by this call.
	*/

	virtual uint32_t write(const void* src, uint32_t count) = 0;

	virtual ~PxOutputStream()
	{
	}
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif // #ifndef PXFOUNDATION_PXIO_H
