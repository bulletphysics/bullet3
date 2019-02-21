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
#ifndef PX_PXSTREAMOPERATORS_H
#define PX_PXSTREAMOPERATORS_H

#include "foundation/PxVec3.h"
#include "foundation/PxTransform.h"
#include "foundation/PxBounds3.h"
#include "PxFiltering.h"

#include "PsString.h"

namespace physx
{
	static inline PxU32 strLenght( const char* inStr )
	{
		return inStr ? PxU32(strlen(inStr)) : 0;
	}
}

namespace physx // ADL requires we put the operators in the same namespace as the underlying type of PxOutputStream
{
	inline PxOutputStream& operator << ( PxOutputStream& ioStream, const char* inString )
	{
		if ( inString && *inString )
		{
			ioStream.write( inString, PxU32(strlen(inString)) );
		}
		return ioStream;
	}

	template<typename TDataType>
	inline PxOutputStream& toStream( PxOutputStream& ioStream, const char* inFormat, const TDataType inData )
	{
		char buffer[128] = { 0 };
		Ps::snprintf( buffer, 128, inFormat, inData );
		ioStream << buffer;
		return ioStream;
	}

	struct endl_obj {};
	//static endl_obj endl;

	inline PxOutputStream& operator << ( PxOutputStream& ioStream, bool inData ) { ioStream << (inData ? "true" : "false"); return ioStream; }
	inline PxOutputStream& operator << ( PxOutputStream& ioStream, PxI32 inData ) { return toStream( ioStream, "%d",  inData ); }
	inline PxOutputStream& operator << ( PxOutputStream& ioStream, PxU16 inData ) {	return toStream( ioStream, "%u", PxU32(inData) ); }
	inline PxOutputStream& operator << ( PxOutputStream& ioStream, PxU8 inData ) {	return toStream( ioStream, "%u", PxU32(inData) ); }
	inline PxOutputStream& operator << ( PxOutputStream& ioStream, char inData ) {	return toStream( ioStream, "%c", inData ); }
	inline PxOutputStream& operator << ( PxOutputStream& ioStream, PxU32 inData ) {	return toStream( ioStream, "%u", inData ); }
	inline PxOutputStream& operator << ( PxOutputStream& ioStream, PxU64 inData ) {	return toStream( ioStream, "%" PX_PRIu64, inData ); }
	inline PxOutputStream& operator << ( PxOutputStream& ioStream, const void* inData ) { return ioStream << static_cast<uint64_t>(reinterpret_cast<size_t>(inData)); }
	inline PxOutputStream& operator << ( PxOutputStream& ioStream, PxF32 inData ) { return toStream( ioStream, "%g", PxF64(inData) ); }
	inline PxOutputStream& operator << ( PxOutputStream& ioStream, PxF64 inData ) { return toStream( ioStream, "%g", inData ); }
	inline PxOutputStream& operator << ( PxOutputStream& ioStream, endl_obj) { return ioStream << "\n"; }
	inline PxOutputStream& operator << ( PxOutputStream& ioStream, const PxVec3& inData ) 
	{ 
		ioStream << inData[0];
		ioStream << " ";
		ioStream << inData[1];
		ioStream << " ";
		ioStream << inData[2];
		return ioStream;
	}

	inline PxOutputStream& operator << ( PxOutputStream& ioStream, const PxQuat& inData ) 
	{
		ioStream << inData.x;
		ioStream << " ";
		ioStream << inData.y;
		ioStream << " ";
		ioStream << inData.z;
		ioStream << " ";
		ioStream << inData.w;
		return ioStream;
	}

	inline PxOutputStream& operator << ( PxOutputStream& ioStream, const PxTransform& inData ) 
	{
		ioStream << inData.q;
		ioStream << " ";
		ioStream << inData.p;
		return ioStream;
	}

	inline PxOutputStream& operator << ( PxOutputStream& ioStream, const PxBounds3& inData )
	{
		ioStream << inData.minimum;
		ioStream << " ";
		ioStream << inData.maximum;
		return ioStream;
	}

	inline PxOutputStream& operator << ( PxOutputStream& ioStream, const PxFilterData& inData )
	{
		ioStream << inData.word0 << " " << inData.word1 << " " << inData.word2 << " " << inData.word3;
		return ioStream;
	}

	inline PxOutputStream& operator << ( PxOutputStream& ioStream, struct PxMetaDataPlane& inData )
	{
		ioStream << inData.normal;
		ioStream << " ";
		ioStream << inData.distance;
		return ioStream;
	}
}

#endif
