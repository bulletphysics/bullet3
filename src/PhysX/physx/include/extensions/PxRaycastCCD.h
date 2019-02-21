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

#ifndef PX_RAYCAST_CCD_H
#define PX_RAYCAST_CCD_H
/** \addtogroup extensions
@{
*/

#include "common/PxPhysXCommonConfig.h"
#include "foundation/PxVec3.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

	class PxScene;
	class PxShape;
	class PxRigidDynamic;
	class RaycastCCDManagerInternal;

	/**
	\brief Raycast-CCD manager.

	Raycast-CCD is a simple and potentially cheaper alternative to the SDK's built-in continuous collision detection algorithm.

	This implementation has some limitations:
	- it is only implemented for PxRigidDynamic objects (not for PxArticulationLink)
	- it is only implemented for simple actors with 1 shape (not for "compounds")

	Also, since it is raycast-based, the solution is not perfect. In particular:
	- small dynamic objects can still go through the static world if the ray goes through a crack between edges, or a small
	hole in the world (like the keyhole from a door).
	- dynamic-vs-dynamic CCD is very approximate. It only works well for fast-moving dynamic objects colliding against
	slow-moving dynamic objects.

	Finally, since it is using the SDK's scene queries under the hood, it only works provided the simulation shapes also have
	scene-query shapes associated with them. That is, if the objects in the scene only use PxShapeFlag::eSIMULATION_SHAPE
	(and no PxShapeFlag::eSCENE_QUERY_SHAPE), then the raycast-CCD system will not work.
	*/
	class RaycastCCDManager
	{
		public:
					RaycastCCDManager(PxScene* scene);
					~RaycastCCDManager();

			/**
			\brief Register dynamic object for raycast CCD.

			\param[in] actor	object's actor
			\param[in] shape	object's shape

			\return True if success
			*/
			bool	registerRaycastCCDObject(PxRigidDynamic* actor, PxShape* shape);

			/**
			\brief Perform raycast CCD. Call this after your simulate/fetchResults calls.

			\param[in] doDynamicDynamicCCD	True to enable dynamic-vs-dynamic CCD (more expensive, not always needed)
			*/
			void	doRaycastCCD(bool doDynamicDynamicCCD);

		private:
			RaycastCCDManagerInternal*	mImpl;
	};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
