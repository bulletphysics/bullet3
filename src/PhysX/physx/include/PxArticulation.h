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


#ifndef PX_PHYSICS_NX_ARTICULATION
#define PX_PHYSICS_NX_ARTICULATION
/** \addtogroup physics 
@{ */

#include "PxArticulationBase.h"


#if !PX_DOXYGEN
namespace physx
{
#endif

class PxArticulationLink;

/**
\brief Articulation drive cache

This cache is used for making one or more impulse applications to the articulation.

@see PxArticulation PxArticulation.createDriveCache
*/
class PxArticulationDriveCache
{
protected:
	PxArticulationDriveCache();
};


/**
\brief a tree structure of bodies connected by joints that is treated as a unit by the dynamics solver

Articulations are more expensive to simulate than the equivalent collection of
PxRigidDynamic and PxJoint structures, but because the dynamics solver treats
each articulation as a single object, they are much less prone to separation and
have better support for actuation. An articulation may have at most 64 links.

@see PxArticulationJoint PxArticulationLink PxPhysics.createArticulation
*/

#if PX_VC
#pragma warning(push)
#pragma warning(disable : 4435)
#endif

class PxArticulation : public PxArticulationBase
{
public:

	virtual void release() = 0;

	/**
	\brief sets maxProjectionIterations.

	This is the maximum number of iterations to run projection on the articulation to bring
	the links back together if the separation  tolerance is exceeded.


	\param[in] iterations the maximum number of projection iterations 
	<b>Default:</b> 4

	@see getMaxProjectionIterations()
	*/
	virtual		void			setMaxProjectionIterations(PxU32 iterations) = 0;

	/**
	\brief gets maxProjectionIterations.

	\return the maximum number of projection iterations

	@see setMaxProjectionIterations()
	*/

	virtual		PxU32			getMaxProjectionIterations() const = 0;

	/**
	\brief sets separationTolerance.

	This is the maximum allowed separation of any joint in the articulation before projection is used

	<b>Default: 0.1f, scaled by the tolerance scale </b>

	\param[in] tolerance the separation tolerance for the articulation

	@see getSeparationTolerance()
	*/
	virtual		void			setSeparationTolerance(PxReal tolerance) = 0;

	/**
	\brief gets separationTolerance.

	\return the separation tolerance

	@see setSeparationTolerance()
	*/

	virtual		PxReal			getSeparationTolerance() const = 0;


	/**
	\brief sets the number of iterations used to compute the drive response to internal forces

	The drive model uses an iterative algorithm to determine the load on each joint of the articulation. 
	This is the number of iterations to use when computing response of the drive to internal forces.

	\param[in] iterations the number of iterations used to compute the drive response to internal forces.
	
	<b>Default:</b> 4

	@see getInternalDriveIterations()
	*/
	virtual		void			setInternalDriveIterations(PxU32 iterations) = 0;

	/**
	\brief gets internal driveIterations.

	\return the number of iterations used to compute the drive response to internal forces

	@see setInternalDriveIterations()
	*/

	virtual		PxU32			getInternalDriveIterations() const = 0;


	/**
	\brief sets the number of iterations for drive response to external forces.

	The drive model uses an iterative algorithm to determine the load on each joint of the articulation. 
	This is the number of iterations to use when computing response of the drive to external forces.

	\param[in] iterations the number of iterations used to compute the drive response to external forces.

	<b>Default:</b> 4

	@see getExternalDriveIterations()
	*/

	virtual		void			setExternalDriveIterations(PxU32 iterations) = 0;

	/**
	\brief gets externalDriveIterations.

	\return the number of iterations used to compute the drive response to external forces

	@see setExternalDriveIterations()
	*/

	virtual		PxU32			getExternalDriveIterations() const = 0;

	/** 
	\brief create a drive cache for applying impulses which are propagated to the entire articulation

	\param[in] compliance the compliance value to use at all joints of the articulation. This is equivalent to the external compliance
	parameter for articulation joints, as the impulse is treated as an external force
	\param[in] driveIterations the number of iterations to use to evaluate the drive strengths

	\return a drive cache

	@see PxArticulationDriveCache updateDriveCache releaseDriveCache applyImpulse computeImpulseResponse
	
	\note this call may only be made on articulations that are in a scene, and may not be made during simulation

	*/
	virtual		PxArticulationDriveCache* 
								createDriveCache(PxReal compliance, PxU32 driveIterations) const = 0;


	/** 
	\brief update a drive cache

	\param[in] driveCache the drive cache to update
	\param[in] compliance the compliance value to use at all joints of the articulation. 
	\param[in] driveIterations the number of iterations to use to evaluate the drive strengths

	@see releaseDriveCache createDriveCache applyImpulse computeImpulseResponse
	
	\note this call may only be made on articulations that are in a scene, and may not be made during simulation

	*/
	virtual		void			updateDriveCache(PxArticulationDriveCache& driveCache,
												 PxReal compliance, 
												 PxU32 driveIterations) const = 0;

	/** 
	\brief release a drive cache
	
	\param[in] driveCache the drive cache to release

	@see createDriveCache updateDriveCache
	*/
	virtual		void			releaseDriveCache(PxArticulationDriveCache& driveCache) const = 0;

	/** 
	\brief apply an impulse to an entire articulation
	
	\param[in] link the link to which to apply the impulse
	\param[in] driveCache the drive cache
	\param[in] linearImpulse the linear impulse to apply
	\param[in] angularImpulse the angular impulse to apply

	@see computeImpulseResponse

	\note this call may only be made on articulations that are in a scene, and may not be made during simulation

	*/
	virtual		void			applyImpulse(PxArticulationLink* link,
											 const PxArticulationDriveCache& driveCache,
											 const PxVec3& linearImpulse,
											 const PxVec3& angularImpulse) = 0;

	/** 
	\brief determine the effect of applying an impulse to an entire articulation, without applying the impulse
	
	\param[in] link the link to which to apply the impulse
	\param[out] linearResponse the change in linear velocity of the articulation link
	\param[out] angularResponse the change in angular velocity of the articulation link
	\param[in] driveCache the drive cache
	\param[in] linearImpulse the linear impulse to apply
	\param[in] angularImpulse the angular impulse to apply

	@see applyImpulse

	This call will wake up the articulation if it is asleep.

	\note this call may only be made on articulations that are in a scene, and may not be made during simulation
	*/

	virtual		void			computeImpulseResponse(PxArticulationLink*link,
													   PxVec3& linearResponse, 
													   PxVec3& angularResponse,
													   const PxArticulationDriveCache& driveCache,
													   const PxVec3& linearImpulse,
													   const PxVec3& angularImpulse) const = 0;

protected:
	PX_INLINE					PxArticulation(PxType concreteType, PxBaseFlags baseFlags) : PxArticulationBase(concreteType, baseFlags){}
	PX_INLINE					PxArticulation(PxBaseFlags baseFlags) : PxArticulationBase(baseFlags){}
	virtual						~PxArticulation() {}

};

#if PX_VC
#pragma warning(pop)
#endif

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
