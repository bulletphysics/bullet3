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

#ifndef PSFILEBUFFER_PSIOSTREAM_H
#define PSFILEBUFFER_PSIOSTREAM_H

/*!
\file
\brief PsIOStream class
*/
#include "filebuf/PxFileBuf.h"

#include "Ps.h"
#include "PsString.h"
#include <string.h>
#include <stdlib.h>
#include "PsAsciiConversion.h"

#define safePrintf physx::shdfnd::snprintf

PX_PUSH_PACK_DEFAULT

namespace physx
{
	namespace general_PxIOStream2
	{

/**
\brief A wrapper class for physx::PxFileBuf that provides both binary and ASCII streaming capabilities
*/
class PsIOStream
{
	static const uint32_t MAX_STREAM_STRING = 1024;
public:
	/**
	\param [in] stream the physx::PxFileBuf through which all reads and writes will be performed
	\param [in] streamLen the length of the input data stream when de-serializing
	*/
	PsIOStream(physx::PxFileBuf &stream,uint32_t streamLen) : mBinary(true), mStreamLen(streamLen), mStream(stream) { }
	~PsIOStream(void) { }

	/**
	\brief Set the stream to binary or ASCII

	\param [in] state if true, stream is binary, if false, stream is ASCII

	If the stream is binary, stream access is passed straight through to the respecitve 
	physx::PxFileBuf methods.  If the stream is ASCII, all stream reads and writes are converted to
	human readable ASCII.
	*/
	PX_INLINE void setBinary(bool state) { mBinary = state; }
	PX_INLINE bool getBinary() { return mBinary; }

	PX_INLINE PsIOStream& operator<<(bool v);
	PX_INLINE PsIOStream& operator<<(char c);
	PX_INLINE PsIOStream& operator<<(uint8_t v);
	PX_INLINE PsIOStream& operator<<(int8_t v);

	PX_INLINE PsIOStream& operator<<(const char *c);
	PX_INLINE PsIOStream& operator<<(int64_t v);
	PX_INLINE PsIOStream& operator<<(uint64_t v);
	PX_INLINE PsIOStream& operator<<(double v);
	PX_INLINE PsIOStream& operator<<(float v);
	PX_INLINE PsIOStream& operator<<(uint32_t v);
	PX_INLINE PsIOStream& operator<<(int32_t v);
	PX_INLINE PsIOStream& operator<<(uint16_t v);
	PX_INLINE PsIOStream& operator<<(int16_t v);
	PX_INLINE PsIOStream& operator<<(const physx::PxVec3 &v);
	PX_INLINE PsIOStream& operator<<(const physx::PxQuat &v);
	PX_INLINE PsIOStream& operator<<(const physx::PxBounds3 &v);

	PX_INLINE PsIOStream& operator>>(const char *&c);
	PX_INLINE PsIOStream& operator>>(bool &v);
	PX_INLINE PsIOStream& operator>>(char &c);
	PX_INLINE PsIOStream& operator>>(uint8_t &v);
	PX_INLINE PsIOStream& operator>>(int8_t &v);
	PX_INLINE PsIOStream& operator>>(int64_t &v);
	PX_INLINE PsIOStream& operator>>(uint64_t &v);
	PX_INLINE PsIOStream& operator>>(double &v);
	PX_INLINE PsIOStream& operator>>(float &v);
	PX_INLINE PsIOStream& operator>>(uint32_t &v);
	PX_INLINE PsIOStream& operator>>(int32_t &v);
	PX_INLINE PsIOStream& operator>>(uint16_t &v);
	PX_INLINE PsIOStream& operator>>(int16_t &v);
	PX_INLINE PsIOStream& operator>>(physx::PxVec3 &v);
	PX_INLINE PsIOStream& operator>>(physx::PxQuat &v);
	PX_INLINE PsIOStream& operator>>(physx::PxBounds3 &v);

	uint32_t getStreamLen(void) const { return mStreamLen; }

	physx::PxFileBuf& getStream(void) { return mStream; }

	PX_INLINE void storeString(const char *c,bool zeroTerminate=false);

private:
	PsIOStream& operator=( const PsIOStream& );


	bool      mBinary; // true if we are serializing binary data.  Otherwise, everything is assumed converted to ASCII
	uint32_t     mStreamLen; // the length of the input data stream when de-serializing.
	physx::PxFileBuf &mStream;
	char			mReadString[MAX_STREAM_STRING]; // a temp buffer for streaming strings on input.
};

#include "PsIOStream.inl" // inline methods...

	} // end of namespace
	using namespace general_PxIOStream2;
} // end of physx namespace

PX_POP_PACK

#endif // PSFILEBUFFER_PSIOSTREAM_H
