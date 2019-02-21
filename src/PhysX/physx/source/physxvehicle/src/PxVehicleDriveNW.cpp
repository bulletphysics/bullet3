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

#include "PxVehicleDriveNW.h"
#include "PxVehicleDrive.h"
#include "PxVehicleSDK.h"
#include "PxVehicleSuspWheelTire4.h"
#include "PxVehicleSuspLimitConstraintShader.h"
#include "PxVehicleDefaults.h"
#include "PxRigidDynamic.h"
#include "PxShape.h"
#include "PsFoundation.h"
#include "PsUtilities.h"
#include "CmPhysXCommon.h"
#include "PxScene.h"
#include "CmUtils.h"


namespace physx
{

extern PxF32 gToleranceScaleLength;

void PxVehicleDriveSimDataNW::setDiffData(const PxVehicleDifferentialNWData& diff)
{
	PX_CHECK_AND_RETURN(diff.isValid(), "Invalid PxVehicleCoreSimulationData.mDiff");
	mDiff=diff;
}

bool PxVehicleDriveSimDataNW::isValid() const
{
	PX_CHECK_AND_RETURN_VAL(mDiff.isValid(), "Invalid PxVehicleDifferentialNWData", false);
	PX_CHECK_AND_RETURN_VAL(PxVehicleDriveSimData::isValid(), "Invalid PxVehicleDriveSimDataNW", false);
	return true;
}

///////////////////////////////////

bool PxVehicleDriveNW::isValid() const
{
	PX_CHECK_AND_RETURN_VAL(PxVehicleDrive::isValid(), "invalid PxVehicleDrive", false);
	PX_CHECK_AND_RETURN_VAL(mDriveSimData.isValid(), "Invalid PxVehicleNW.mCoreSimData", false);
	return true;
}

PxVehicleDriveNW* PxVehicleDriveNW::allocate(const PxU32 numWheels)
{
	PX_CHECK_AND_RETURN_NULL(numWheels>0, "Cars with zero wheels are illegal");
	PX_CHECK_AND_RETURN_NULL(gToleranceScaleLength > 0, "PxVehicleDriveNW::allocate - need to call PxInitVehicleSDK");

	//Compute the bytes needed.
	const PxU32 byteSize = sizeof(PxVehicleDriveNW) + PxVehicleDrive::computeByteSize(numWheels);

	//Allocate the memory.
	PxVehicleDriveNW* veh = static_cast<PxVehicleDriveNW*>(PX_ALLOC(byteSize, "PxVehicleDriveNW"));
	Cm::markSerializedMem(veh, byteSize);
	new(veh) PxVehicleDriveNW();

	//Patch up the pointers.
	PxU8* ptr = reinterpret_cast<PxU8*>(veh) + sizeof(PxVehicleDriveNW);
	ptr=PxVehicleDrive::patchupPointers(numWheels, veh, ptr);

	//Initialise
	veh->init(numWheels);

	//Set the vehicle type.
	veh->mType = PxVehicleTypes::eDRIVENW;

	return veh;
}

void PxVehicleDriveNW::free()
{
	PxVehicleDrive::free();
}

void PxVehicleDriveNW::setup
(PxPhysics* physics, PxRigidDynamic* vehActor,
 const PxVehicleWheelsSimData& wheelsData, const PxVehicleDriveSimDataNW& driveData,
 const PxU32 numWheels)
{
	PX_CHECK_AND_RETURN(driveData.isValid(), "PxVehicleDriveNW::setup - invalid driveData");

	//Set up the wheels.
	PxVehicleDrive::setup(physics,vehActor,wheelsData,numWheels,0);

	//Start setting up the drive.
	PX_CHECK_MSG(driveData.isValid(), "PxVehicleNWDrive - invalid driveData");

	//Copy the simulation data.
	mDriveSimData = driveData;
}

PxVehicleDriveNW* PxVehicleDriveNW::create
(PxPhysics* physics, PxRigidDynamic* vehActor,
 const PxVehicleWheelsSimData& wheelsData, const PxVehicleDriveSimDataNW& driveData,
 const PxU32 numWheels)
{
	PxVehicleDriveNW* vehNW=PxVehicleDriveNW::allocate(numWheels);
	vehNW->setup(physics,vehActor,wheelsData,driveData,numWheels);
	return vehNW;
}


void PxVehicleDriveNW::setToRestState()
{
	//Set core to rest state.
	PxVehicleDrive::setToRestState();
}












} //namespace physx

