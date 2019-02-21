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


#ifndef PX_BINARY_CONVERTER_H
#define PX_BINARY_CONVERTER_H
/** \addtogroup extensions
@{
*/

#include "common/PxPhysXCommonConfig.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

struct PxConverterReportMode
{
	enum Enum
	{
		eNONE,		//!< Silent mode. If enabled, no information is sent to the error stream.
		eNORMAL,	//!< Normal mode. If enabled, only important information is sent to the error stream.
		eVERBOSE	//!< Verbose mode. If enabled, detailed information is sent to the error stream.
	};
};


/**
\brief Binary converter for serialized streams.

The binary converter class is targeted at converting binary streams from authoring platforms, 
such as windows, osx or linux to any game runtime platform supported by PhysX. Particularly 
it is currently not supported to run the converter on a platforms that has an endian mismatch 
with the platform corresponding to the source binary file and source meta data. 

If you want to use multiple threads for batch conversions, please create one instance
of this class for each thread.

@see PxSerialization.createBinaryConverter
*/
class PxBinaryConverter
{
public:

	/**
	\brief Releases binary converter
	*/
	virtual		void	release()																			= 0;

	/**
	\brief Sets desired report mode.

	\param[in] mode	Report mode
	*/
	virtual		void	setReportMode(PxConverterReportMode::Enum mode)										= 0;

	/**
	\brief Setups source and target meta-data streams

	The source meta data provided needs to have the same endianness as the platform the converter is run on.
	The meta data needs to be set before calling the conversion method.

	\param[in] srcMetaData	Source platform's meta-data stream
	\param[in] dstMetaData	Target platform's meta-data stream

	\return True if success
	*/
	virtual		bool	setMetaData(PxInputStream& srcMetaData, PxInputStream& dstMetaData)					= 0;

	/**
	\brief Test utility function to compare two sets of meta data.

	The meta data needs to be set before calling the compareMetaData method.
	This method will issue PxErrorCode::eDEBUG_INFO messages if mismatches are encountered.

	\return True if meta data is equivalend
	*/
	virtual		bool	compareMetaData() const = 0;

	/**
	\brief Converts binary stream from source platform to target platform

	The converter needs to be configured with source and destination meta data before calling the conversion method. 
	The source meta data needs to correspond to the same platform as the source binary data.

	\param[in] srcStream	Source stream
	\param[in] srcSize		Number of bytes to convert
	\param[in] targetStream	Target stream

	\return True if success
	*/
	virtual		bool	convert(PxInputStream& srcStream, PxU32 srcSize, PxOutputStream& targetStream)		= 0;


protected:
						PxBinaryConverter()		{}
	virtual				~PxBinaryConverter()	{}
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
