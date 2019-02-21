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

#ifndef PX_VEHICLE_SHADERS_H
#define PX_VEHICLE_SHADERS_H
/** \addtogroup vehicle
  @{
*/

#include "foundation/PxSimpleTypes.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/**
\brief Prototype of shader function that is used to compute wheel torque and tire forces.
\param[in]  shaderData is the shader data for the tire being processed.  The shader data describes the tire data in the format required by the tire model that is implemented by the shader function.
\param[in]  tireFriction is the value of friction for the contact between the tire and the ground.
\param[in]  longSlip is the value of longitudinal slip experienced by the tire.
\param[in]  latSlip is the value of lateral slip experienced by the tire.
\param[in]  camber is the camber angle of the tire in radians.
\param[in]  wheelOmega is the rotational speed of the wheel.
\param[in]  wheelRadius is the distance from the tire surface to the center of the wheel.
\param[in]  recipWheelRadius is the reciprocal of wheelRadius.
\param[in]  restTireLoad is the load force experienced by the tire when the vehicle is at rest.
\param[in]  normalisedTireLoad is a pre-computed value equal to the load force on the tire divided by restTireLoad.
\param[in]  tireLoad is the load force currently experienced by the tire (= restTireLoad*normalisedTireLoad)
\param[in]  gravity is the magnitude of gravitational acceleration.
\param[in]  recipGravity is the reciprocal of the magnitude of gravitational acceleration.
\param[out] wheelTorque is the torque that is to be applied to the wheel around the wheel's axle.
\param[out] tireLongForceMag is the magnitude of the longitudinal tire force to be applied to the vehicle's rigid body.
\param[out] tireLatForceMag is the magnitude of the lateral tire force to be applied to the vehicle's rigid body.
\param[out] tireAlignMoment is the aligning moment of the tire that is to be applied to the vehicle's rigid body (not currently used).
@see PxVehicleWheelsDynData::setTireForceShaderFunction,  PxVehicleWheelsDynData::setTireForceShaderData
*/
typedef void (*PxVehicleComputeTireForce)
(const void* shaderData, 
 const PxF32 tireFriction,
 const PxF32 longSlip, const PxF32 latSlip, const PxF32 camber,
 const PxF32 wheelOmega, const PxF32 wheelRadius, const PxF32 recipWheelRadius,
 const PxF32 restTireLoad, const PxF32 normalisedTireLoad, const PxF32 tireLoad,
 const PxF32 gravity, const PxF32 recipGravity,
 PxF32& wheelTorque, PxF32& tireLongForceMag, PxF32& tireLatForceMag, PxF32& tireAlignMoment);


#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif //PX_VEHICLE_SHADERS_H
