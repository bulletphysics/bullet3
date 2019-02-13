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


#ifndef PX_PHYSICS_METADATA_FLAGS
#define PX_PHYSICS_METADATA_FLAGS

#include "foundation/Px.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

	/**
	\brief Flags used to configure binary meta data entries, typically set through PX_DEF_BIN_METADATA defines.

	@see PxMetaDataEntry
	*/
    struct PxMetaDataFlag
	{
		enum Enum
		{
			eCLASS					= (1<<0),		//!< declares a class
			eVIRTUAL				= (1<<1),		//!< declares class to be virtual
			eTYPEDEF				= (1<<2),		//!< declares a typedef
			ePTR					= (1<<3),		//!< declares a pointer
			eEXTRA_DATA				= (1<<4),		//!< declares extra data exported with PxSerializer::exportExtraData
			eEXTRA_ITEM				= (1<<5),		//!< specifies one element of extra data
			eEXTRA_ITEMS			= (1<<6),		//!< specifies an array of extra data
			eEXTRA_NAME             = (1<<7),       //!< specifies a name of extra data
			eUNION					= (1<<8),		//!< declares a union
			ePADDING				= (1<<9),		//!< declares explicit padding data
			eALIGNMENT				= (1<<10),		//!< declares aligned data
			eCOUNT_MASK_MSB			= (1<<11),		//!< specifies that the count value's most significant bit needs to be masked out
			eCOUNT_SKIP_IF_ONE		= (1<<12),		//!< specifies that the count value is treated as zero for a variable value of one - special case for single triangle meshes
			eCONTROL_FLIP			= (1<<13),		//!< specifies that the control value is the negate of the variable value
			eCONTROL_MASK			= (1<<14),		//!< specifies that the control value is masked - mask bits are assumed to be within eCONTROL_MASK_RANGE
			eCONTROL_MASK_RANGE		= 0x000000FF,	//!< mask range allowed for eCONTROL_MASK 
			eFORCE_DWORD			= 0x7fffffff
		};
	};

#if !PX_DOXYGEN
}
#endif

#endif
