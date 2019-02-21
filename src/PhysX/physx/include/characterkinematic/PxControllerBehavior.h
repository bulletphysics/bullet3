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

#ifndef PX_PHYSICS_CCT_BEHAVIOR
#define PX_PHYSICS_CCT_BEHAVIOR
/** \addtogroup character
  @{
*/

#include "PxFiltering.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

	class PxShape;
	class PxObstacle;
	class PxController;

	/**
	\brief specifies controller behavior
	*/
	struct PxControllerBehaviorFlag
	{
		enum Enum
		{
			eCCT_CAN_RIDE_ON_OBJECT		= (1<<0),	//!< Controller can ride on touched object (i.e. when this touched object is moving horizontally). \note The CCT vs. CCT case is not supported.
			eCCT_SLIDE					= (1<<1),	//!< Controller should slide on touched object
			eCCT_USER_DEFINED_RIDE		= (1<<2)	//!< Disable all code dealing with controllers riding on objects, let users define it outside of the SDK.
		};
	};

	/**
	\brief Bitfield that contains a set of raised flags defined in PxControllerBehaviorFlag.

	@see PxControllerBehaviorFlag
	*/
	typedef PxFlags<PxControllerBehaviorFlag::Enum, PxU8> PxControllerBehaviorFlags;
	PX_FLAGS_OPERATORS(PxControllerBehaviorFlag::Enum, PxU8)

	/**
	\brief User behavior callback.

	This behavior callback is called to customize the controller's behavior w.r.t. touched shapes.
	*/
	class PxControllerBehaviorCallback
	{
	public:
		/**
		\brief Retrieve behavior flags for a shape.

		When the CCT touches a shape, the CCT's behavior w.r.t. this shape can be customized by users.
		This function retrieves the desired PxControllerBehaviorFlag flags capturing the desired behavior.

		\note See comments about deprecated functions at the start of this class

		\param[in] shape	The shape the CCT is currently touching
		\param[in] actor	The actor owning the shape

		\return Desired behavior flags for the given shape

		@see PxControllerBehaviorFlag
		*/
		virtual PxControllerBehaviorFlags getBehaviorFlags(const PxShape& shape, const PxActor& actor) = 0;

		/**
		\brief Retrieve behavior flags for a controller.

		When the CCT touches a controller, the CCT's behavior w.r.t. this controller can be customized by users.
		This function retrieves the desired PxControllerBehaviorFlag flags capturing the desired behavior.

		\note The flag PxControllerBehaviorFlag::eCCT_CAN_RIDE_ON_OBJECT is not supported.
		\note See comments about deprecated functions at the start of this class

		\param[in] controller	The controller the CCT is currently touching

		\return Desired behavior flags for the given controller

		@see PxControllerBehaviorFlag
		*/
		virtual PxControllerBehaviorFlags getBehaviorFlags(const PxController& controller) = 0;

		/**
		\brief Retrieve behavior flags for an obstacle.

		When the CCT touches an obstacle, the CCT's behavior w.r.t. this obstacle can be customized by users.
		This function retrieves the desired PxControllerBehaviorFlag flags capturing the desired behavior.

		\note See comments about deprecated functions at the start of this class

		\param[in] obstacle		The obstacle the CCT is currently touching

		\return Desired behavior flags for the given obstacle

		@see PxControllerBehaviorFlag
		*/
		virtual PxControllerBehaviorFlags getBehaviorFlags(const PxObstacle& obstacle) = 0;

	protected:
		virtual ~PxControllerBehaviorCallback(){}
	};

#if !PX_DOXYGEN
}
#endif

/** @} */
#endif
