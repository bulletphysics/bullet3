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


#ifndef PX_SCALE_H
#define PX_SCALE_H

/** \addtogroup common
  @{
*/

#include "common/PxPhysXCommonConfig.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxPhysics;

/**
\brief Class to define the scale at which simulation runs. Most simulation tolerances are
calculated in terms of the values here. 

\note if you change the simulation scale, you will probablly also wish to change the scene's
default value of gravity, and stable simulation will probably require changes to the scene's 
bounceThreshold also.
*/

class PxTolerancesScale
{
public: 

	/** brief
	The approximate size of objects in the simulation. 
	
	For simulating roughly human-sized in metric units, 1 is a good choice.
	If simulation is done in centimetres, use 100 instead. This is used to
	estimate certain length-related tolerances.
	*/
	PxReal	length;

	/** brief
	The typical magnitude of velocities of objects in simulation. This is used to estimate 
	whether a contact should be treated as bouncing or resting based on its impact velocity,
	and a kinetic energy threshold below which the simulation may put objects to sleep.

	For normal physical environments, a good choice is the approximate speed of an object falling
	under gravity for one second.
	*/
	PxReal	speed;

	/**
	\brief constructor sets to default 
	*/
	PX_INLINE PxTolerancesScale();

	/**
	\brief Returns true if the descriptor is valid.
	\return true if the current settings are valid (returns always true).
	*/
	PX_INLINE bool isValid() const;

};

PX_INLINE PxTolerancesScale::PxTolerancesScale():
	length(1.0f),
	speed(10.0f)
	{
	}

PX_INLINE bool PxTolerancesScale::isValid() const
{
	return length>0.0f;
}

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
