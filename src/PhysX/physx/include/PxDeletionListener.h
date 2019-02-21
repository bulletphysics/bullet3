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


#ifndef PX_PHYSICS_NX_DELETIONLISTENER
#define PX_PHYSICS_NX_DELETIONLISTENER
/** \addtogroup physics
@{
*/

#include "PxPhysXConfig.h"
#include "common/PxBase.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

	
/**
\brief Flags specifying deletion event types.

@see PxDeletionListener::onRelease PxPhysics.registerDeletionListener()
*/
struct PxDeletionEventFlag
{
	enum Enum
	{
		eUSER_RELEASE					= (1<<0),	//!< The user has called release on an object.
		eMEMORY_RELEASE					= (1<<1)	//!< The destructor of an object has been called and the memory has been released.
	};
};

/**
\brief Collection of set bits defined in PxDeletionEventFlag.

@see PxDeletionEventFlag
*/
typedef PxFlags<PxDeletionEventFlag::Enum,PxU8> PxDeletionEventFlags;
PX_FLAGS_OPERATORS(PxDeletionEventFlag::Enum,PxU8)


/**
\brief interface to get notification on object deletion

*/
class PxDeletionListener
{
public:
	/**
	\brief Notification if an object or its memory gets released

	If release() gets called on a PxBase object, an eUSER_RELEASE event will get fired immediately. The object state can be queried in the callback but
	it is not allowed to change the state. Furthermore, when reading from the object it is the user's responsibility to make sure that no other thread 
	is writing at the same time to the object (this includes the simulation itself, i.e., #PxScene::fetchResults() must not get called at the same time).

	Calling release() on a PxBase object does not necessarily trigger its destructor immediately. For example, the object can be shared and might still
	be referenced by other objects or the simulation might still be running and accessing the object state. In such cases the destructor will be called
	as soon as it is safe to do so. After the destruction of the object and its memory, an eMEMORY_RELEASE event will get fired. In this case it is not
	allowed to dereference the object pointer in the callback.

	\param[in] observed The object for which the deletion event gets fired.
	\param[in] userData The user data pointer of the object for which the deletion event gets fired. Not available for all object types in which case it will be set to 0.
	\param[in] deletionEvent The type of deletion event. Do not dereference the object pointer argument if the event is eMEMORY_RELEASE.

	*/
	virtual void onRelease(const PxBase* observed, void* userData, PxDeletionEventFlag::Enum deletionEvent) = 0;

protected:
	PxDeletionListener() {}
	virtual ~PxDeletionListener() {}
};


#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
