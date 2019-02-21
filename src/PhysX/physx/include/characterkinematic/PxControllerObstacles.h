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

#ifndef PX_PHYSICS_CCT_OBSTACLES
#define PX_PHYSICS_CCT_OBSTACLES
/** \addtogroup character
  @{
*/

#include "characterkinematic/PxExtended.h"
#include "geometry/PxGeometry.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

	class PxControllerManager;

	#define INVALID_OBSTACLE_HANDLE	0xffffffff

	/**
	\brief Base class for obstacles.

	@see PxBoxObstacle PxCapsuleObstacle PxObstacleContext
	*/
	class PxObstacle
	{
		protected:
												PxObstacle() :
													mType		(PxGeometryType::eINVALID),
													mUserData	(NULL),
													mPos		(0.0, 0.0, 0.0),
													mRot		(PxQuat(PxIdentity))
												{}

						PxGeometryType::Enum	mType; 
		public:

		PX_FORCE_INLINE	PxGeometryType::Enum	getType()	const	{ return mType;	}

						void*					mUserData;
						PxExtendedVec3			mPos;
						PxQuat					mRot;
	};

	/**
	\brief A box obstacle.

	@see PxObstacle PxCapsuleObstacle PxObstacleContext
	*/
	class PxBoxObstacle : public PxObstacle
	{
		public:
												PxBoxObstacle() :
													mHalfExtents(0.0f)
												{ mType = PxGeometryType::eBOX;		 }

						PxVec3					mHalfExtents;
	};

	/**
	\brief A capsule obstacle.

	@see PxBoxObstacle PxObstacle PxObstacleContext
	*/
	class PxCapsuleObstacle : public PxObstacle
	{
		public:
												PxCapsuleObstacle() :
													mHalfHeight	(0.0f),
													mRadius		(0.0f)
												{ mType = PxGeometryType::eCAPSULE;	 }

						PxReal					mHalfHeight;
						PxReal					mRadius;
	};

	typedef PxU32	ObstacleHandle;

	/**
	\brief Context class for obstacles.

	An obstacle context class contains and manages a set of user-defined obstacles.

	@see PxBoxObstacle PxCapsuleObstacle PxObstacle
	*/
	class PxObstacleContext
	{
		public:
									PxObstacleContext()		{}
		virtual						~PxObstacleContext()	{}

		/**
		\brief Releases the context.
		*/
		virtual	void				release()															= 0;

		/**
		\brief Retrieves the controller manager associated with this context.

		\return The associated controller manager
		*/
		virtual PxControllerManager&	getControllerManager() const									= 0;

		/**
		\brief Adds an obstacle to the context.

		\param	[in]	obstacle	Obstacle data for the new obstacle. The data gets copied.

		\return Handle for newly-added obstacle
		*/
		virtual	ObstacleHandle		addObstacle(const PxObstacle& obstacle)								= 0;

		/**
		\brief Removes an obstacle from the context.

		\param	[in]	handle	Handle for the obstacle object that needs to be removed.

		\return True if success
		*/
		virtual	bool				removeObstacle(ObstacleHandle handle)								= 0;

		/**
		\brief Updates data for an existing obstacle.

		\param	[in]	handle		Handle for the obstacle object that needs to be updated.
		\param	[in]	obstacle	New obstacle data

		\return True if success
		*/
		virtual	bool				updateObstacle(ObstacleHandle handle, const PxObstacle& obstacle)	= 0;

		/**
		\brief Retrieves number of obstacles in the context.

		\return Number of obstacles in the context
		*/
		virtual	PxU32				getNbObstacles()											const	= 0;

		/**
		\brief Retrieves desired obstacle.

		\param	[in]	i			Obstacle index

		\return Desired obstacle
		*/
		virtual	const PxObstacle*	getObstacle(PxU32 i)										const	= 0;

		/**
		\brief Retrieves desired obstacle by given handle.

		\param	[in]	handle			Obstacle handle

		\return Desired obstacle
		*/
		virtual	const PxObstacle*	getObstacleByHandle(ObstacleHandle handle)					const	= 0;
	};

#if !PX_DOXYGEN
}
#endif

/** @} */
#endif
