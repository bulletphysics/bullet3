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

#ifndef PX_VEHICLE_UTILSSETUP_H
#define PX_VEHICLE_UTILSSETUP_H
/** \addtogroup vehicle
  @{
*/
#include "foundation/PxSimpleTypes.h"
#include "vehicle/PxVehicleSDK.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxVehicleWheelsSimData;
class PxVehicleWheelsDynData;
class PxVehicleDriveSimData4W;
class PxVehicleWheels;


/**
\brief Reconfigure a PxVehicle4W instance as a three-wheeled car with tadpole config (2 front wheels, 1 rear wheel)

\note The rear-left wheel is removed and the rear-right wheel is positioned at the centre of the rear axle.
The suspension of the rear-right wheel is modified to support the entire mass of the front car while preserving its natural frequency and damping ratio.

\param[in,out] wheelsSimData is the data describing the wheels/suspensions/tires of the vehicle.
\param[in,out] wheelsDynData is the data describing the dynamic state of the wheels of the vehicle.
\param[in,out] driveSimData is the data describing the drive model of the vehicle.
*/
void PxVehicle4WEnable3WTadpoleMode(PxVehicleWheelsSimData& wheelsSimData, PxVehicleWheelsDynData& wheelsDynData, PxVehicleDriveSimData4W& driveSimData);

/**
\brief Reconfigure a PxVehicle4W instance as a three-wheeled car with delta config (1 front wheel, 2 rear wheels)

\note The front-left wheel is removed and the front-right wheel is positioned at the centre of the front axle.
The suspension of the front-right wheel is modified to support the entire mass of the front car while preserving its natural frequency and damping ratio.

\param[in,out] wheelsSimData is the data describing the wheels/suspensions/tires of the vehicle.
\param[in,out] wheelsDynData is the data describing the dynamic state of the wheels of the vehicle.
\param[in,out] driveSimData is the data describing the drive model of the vehicle.
*/
void PxVehicle4WEnable3WDeltaMode(PxVehicleWheelsSimData& wheelsSimData, PxVehicleWheelsDynData& wheelsDynData, PxVehicleDriveSimData4W& driveSimData);

/**
\brief Compute the sprung masses of the suspension springs given (i) the number of sprung masses, 
(ii) coordinates of the sprung masses, (iii) the center of mass offset of the rigid body, (iv) the 
total mass of the rigid body, and (v) the direction of gravity (0 for x-axis, 1 for y-axis, 2 for z-axis).

\param[in] nbSprungMasses is the number of sprung masses of the vehicle.  This value corresponds to the number of wheels on the vehicle.
\param[in] sprungMassCoordinates are the coordinates of the sprung masses relative to the actor. The array sprungMassCoordinates must be of 
length nbSprungMasses or greater.
\param[in] centreOfMass is the coordinate of the center of mass of the rigid body relative to the actor.  This value corresponds to 
the value set by PxRigidBody::setCMassLocalPose.
\param[in] totalMass is the total mass of all the sprung masses.  This value corresponds to the value set by PxRigidBody::setMass.
\param[in] gravityDirection is an integer describing the direction of gravitational acceleration. A value of 0 corresponds to (-1,0,0), 
a value of 1 corresponds to (0,-1,0) and a value of 2 corresponds to (0,0,-1).
\param[out] sprungMasses are the masses to set in the associated suspension data with PxVehicleSuspensionData::mSprungMass.  The sprungMasses array must be of length 
nbSprungMasses or greater. Each element in the sprungMasses array corresponds to the suspension located at the same array element in sprungMassCoordinates.
The center of mass of the masses in sprungMasses with the coordinates in sprungMassCoordinates satisfy the specified centerOfMass.
*/
void PxVehicleComputeSprungMasses(const PxU32 nbSprungMasses, const PxVec3* sprungMassCoordinates, const PxVec3& centreOfMass, const PxReal totalMass, const PxU32 gravityDirection, PxReal* sprungMasses);


/**
\brief Reconfigure the vehicle to reflect a new center of mass local pose that has been applied to the actor.  The function requires
(i) the center of mass local pose that was last used to configure the vehicle and the vehicle's actor, (ii) the new center of mass local pose that 
has been applied to the vehicle's actor and will now be applied to the vehicle, and (iii) the direction of gravity (0 for x-axis, 1 for y-axis, 2 for z-axis)

\param[in] oldCMassLocalPose is the center of mass local pose that was last used to configure the vehicle.
\param[in] newCMassLocalPose is the center of mass local pose that will be used to configure the vehicle so that it matches the vehicle's actor.
\param[in] gravityDirection is an integer describing the direction of gravitational acceleration. A value of 0 corresponds to (0,0,-1), 
a value of 1 corresponds to (0,-1,0) and a value of 2 corresponds to (0,0,-1).
\param[in,out] vehicle is the vehicle to be updated with a new center of mass local pose.

\note This function does not update the center of mass of the vehicle actor.  That needs to updated separately with PxRigidBody::setCMassLocalPose

\note The suspension sprung masses are updated so that the natural frequency and damping ratio of the springs are preserved.  This involves altering the
stiffness and damping rate of the suspension springs.
*/
void PxVehicleUpdateCMassLocalPose(const PxTransform& oldCMassLocalPose, const PxTransform& newCMassLocalPose, const PxU32 gravityDirection, PxVehicleWheels* vehicle);

/**
\brief Used by PxVehicleCopyDynamicsData
@see PxVehicleCopyDynamicsData
*/
class PxVehicleCopyDynamicsMap
{
public:

	PxVehicleCopyDynamicsMap()
	{
		for(PxU32 i = 0; i < PX_MAX_NB_WHEELS; i++)
		{
			sourceWheelIds[i] = PX_MAX_U8;
			targetWheelIds[i] = PX_MAX_U8;
		}
	}

	PxU8 sourceWheelIds[PX_MAX_NB_WHEELS];
	PxU8 targetWheelIds[PX_MAX_NB_WHEELS];
};

/**
\brief Copy dynamics data from src to trg, including wheel rotation speed, wheel rotation angle, engine rotation speed etc.

\param[in] wheelMap -  describes the mapping between the wheels in src and the wheels in trg.  

\param[in] src - according to the wheel mapping stored in wheelMap, the dynamics data in src wheels are copied to the corresponding wheels in trg.

\param[out] trg - according to wheel mapping stored in wheelMap, the wheels in trg are given the dynamics data of the corresponding wheels in src.

\note wheelMap must specify a unique mapping between the wheels in src and the wheels in trg.

\note In the event that src has fewer wheels than trg, wheelMap must specify a unique mapping between each src wheel to a trg wheel.

\note In the event that src has more wheels than trg, wheelMap must specify a unique mapping to each trg wheel from a src wheel.

\note In the event that src has fewer wheels than trg, the trg wheels that are not mapped to a src wheel are given the average wheel rotation 
speed of all enabled src wheels.

\note src and trg must be the same vehicle type.
*/
void PxVehicleCopyDynamicsData(const PxVehicleCopyDynamicsMap& wheelMap, const PxVehicleWheels& src, PxVehicleWheels* trg);


#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif //PX_VEHICLE_UTILSSETUP_H
