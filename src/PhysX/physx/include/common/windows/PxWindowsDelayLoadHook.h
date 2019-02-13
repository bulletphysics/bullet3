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


#ifndef PX_PHYSICS_DELAY_LOAD_HOOK
#define PX_PHYSICS_DELAY_LOAD_HOOK

#include "foundation/PxPreprocessor.h"
#include "common/PxPhysXCommonConfig.h"

/** \addtogroup foundation
@{
*/

#if !PX_DOXYGEN
namespace physx
{
#endif
	/**
 	\brief PxDelayLoadHook

	This is a helper class for delay loading the PhysXCommon dll and PhysXFoundation dll. 
	If a PhysXCommon dll or PhysXFoundation dll with a non-default file name needs to be loaded, 
	PxDelayLoadHook can be sub-classed to provide the custom filenames.

	Once the names are set, the instance must be set for use by PhysX.dll using PxSetPhysXDelayLoadHook(), 
	PhysXCooking.dll using PxSetPhysXCookingDelayLoadHook()	or by PhysXCommon.dll using PxSetPhysXCommonDelayLoadHook().

	@see PxSetPhysXDelayLoadHook(), PxSetPhysXCookingDelayLoadHook(), PxSetPhysXCommonDelayLoadHook()
 	*/
	class PxDelayLoadHook
	{
	public:
		PxDelayLoadHook() {}
		virtual ~PxDelayLoadHook() {}

		virtual const char* getPhysXFoundationDllName() const = 0;
		
		virtual const char* getPhysXCommonDllName() const = 0;

	protected:
	private:
	};

	/**
	\brief Sets delay load hook instance for PhysX dll.

	\param[in] hook Delay load hook.

	@see PxDelayLoadHook
	*/
	PX_C_EXPORT PX_PHYSX_CORE_API void PX_CALL_CONV PxSetPhysXDelayLoadHook(const physx::PxDelayLoadHook* hook);

	/**
	\brief Sets delay load hook instance for PhysXCooking dll.

	\param[in] hook Delay load hook.

	@see PxDelayLoadHook
	*/
	PX_C_EXPORT PX_PHYSX_CORE_API void PX_CALL_CONV PxSetPhysXCookingDelayLoadHook(const physx::PxDelayLoadHook* hook);

	/**
	\brief Sets delay load hook instance for PhysXCommon dll.

	\param[in] hook Delay load hook.

	@see PxDelayLoadHook
	*/
	PX_C_EXPORT PX_PHYSX_COMMON_API void PX_CALL_CONV PxSetPhysXCommonDelayLoadHook(const physx::PxDelayLoadHook* hook);

#if !PX_DOXYGEN
} // namespace physx
#endif
/** @} */
#endif
