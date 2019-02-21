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


#ifndef PX_PHYSICS_NX_LOCKED_DATA
#define PX_PHYSICS_NX_LOCKED_DATA
/** \addtogroup physics
@{
*/

#include "PxPhysXConfig.h"
#include "foundation/PxFlags.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

struct PxDataAccessFlag
{
	enum Enum
	{
		eREADABLE = (1 << 0),
		eWRITABLE = (1 << 1),
		eDEVICE	  = (1 << 2)
	};
};

/**
\brief collection of set bits defined in PxDataAccessFlag.

@see PxDataAccessFlag
*/
typedef PxFlags<PxDataAccessFlag::Enum,PxU8> PxDataAccessFlags;
PX_FLAGS_OPERATORS(PxDataAccessFlag::Enum,PxU8)


/**
\brief Parent class for bulk data that is shared between the SDK and the application.
*/
class PxLockedData
{ 
public:

	/**
	\brief Any combination of PxDataAccessFlag::eREADABLE and PxDataAccessFlag::eWRITABLE
	@see PxDataAccessFlag
	*/
    virtual PxDataAccessFlags getDataAccessFlags() = 0;

	/**
	\brief Unlocks the bulk data.
	*/
    virtual void unlock() = 0;

	/**
	\brief virtual destructor
	*/
	virtual ~PxLockedData() {}
}; 

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
