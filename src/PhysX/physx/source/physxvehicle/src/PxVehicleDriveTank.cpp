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

#include "PxVehicleDriveTank.h"
#include "PxVehicleWheels.h"
#include "PxVehicleSDK.h"
#include "PxVehicleDefaults.h"
#include "PxRigidDynamic.h"
#include "CmPhysXCommon.h"
#include "CmUtils.h"
#include "PsFoundation.h"

namespace physx
{

extern PxF32 gToleranceScaleLength;

bool PxVehicleDriveTank::isValid() const
{
	PX_CHECK_AND_RETURN_VAL(PxVehicleDrive::isValid(), "invalid PxVehicleDrive", false);
	PX_CHECK_AND_RETURN_VAL(mDriveSimData.isValid(), "Invalid PxVehicleDriveTank.mCoreSimData", false);
	return true;
}

PxVehicleDriveTank* PxVehicleDriveTank::allocate(const PxU32 numWheels)
{
	PX_CHECK_AND_RETURN_NULL(numWheels>0, "Cars with zero wheels are illegal");
	PX_CHECK_AND_RETURN_NULL(0 == (numWheels % 2), "PxVehicleDriveTank::allocate - needs to have even number of wheels");
	PX_CHECK_AND_RETURN_NULL(gToleranceScaleLength > 0, "PxVehicleDriveTank::allocate - need to call PxInitVehicleSDK");

	//Compute the bytes needed.
	const PxU32 byteSize = sizeof(PxVehicleDriveTank) + PxVehicleDrive::computeByteSize(numWheels);

	//Allocate the memory.
	PxVehicleDriveTank* veh = static_cast<PxVehicleDriveTank*>(PX_ALLOC(byteSize, "PxVehicleDriveTank"));
	Cm::markSerializedMem(veh, byteSize);
	new(veh) PxVehicleDriveTank();

	//Patch up the pointers.
	PxU8* ptr = reinterpret_cast<PxU8*>(veh) + sizeof(PxVehicleDriveTank);
	PxVehicleDrive::patchupPointers(numWheels, veh, ptr);

	//Initialise.
	veh->init(numWheels);

	//Set the vehicle type.
	veh->mType = PxVehicleTypes::eDRIVETANK;

	//Set the default drive model.
	veh->mDriveModel = PxVehicleDriveTankControlModel::eSTANDARD;

	return veh;
}

void PxVehicleDriveTank::free()
{
	PxVehicleDrive::free();
}

void PxVehicleDriveTank::setup
(PxPhysics* physics, PxRigidDynamic* vehActor, 
 const PxVehicleWheelsSimData& wheelsData, const PxVehicleDriveSimData& driveData,
 const PxU32 numDrivenWheels)
{
	PX_CHECK_AND_RETURN(driveData.isValid(), "PxVehicleDriveTank::setup - illegal drive data");

	//Set up the wheels.
	PxVehicleDrive::setup(physics,vehActor,wheelsData,numDrivenWheels,0);

	//Start setting up the drive.
	PX_CHECK_MSG(driveData.isValid(), "PxVehicle4WDrive - invalid driveData");

	//Copy the simulation data.
	mDriveSimData = driveData;
}

PxVehicleDriveTank* PxVehicleDriveTank::create
(PxPhysics* physics, PxRigidDynamic* vehActor, 
 const PxVehicleWheelsSimData& wheelsData, const PxVehicleDriveSimData& driveData,
 const PxU32 numDrivenWheels)
{
	PxVehicleDriveTank* tank=PxVehicleDriveTank::allocate(numDrivenWheels);
	tank->setup(physics,vehActor,wheelsData,driveData,numDrivenWheels);
	return tank;
}


void PxVehicleDriveTank::setToRestState()
{
	//Set core to rest state.
	PxVehicleDrive::setToRestState();
}
} //namespace physx

