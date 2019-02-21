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

#include "foundation/PxMemory.h"
#include "PxVehicleNoDrive.h"
#include "PxVehicleWheels.h"
#include "PxVehicleDefaults.h"
#include "PxRigidDynamic.h"
#include "CmPhysXCommon.h"
#include "CmUtils.h"
#include "PsFoundation.h"

namespace physx
{

extern PxF32 gToleranceScaleLength;

bool PxVehicleNoDrive::isValid() const
{
	PX_CHECK_AND_RETURN_VAL(PxVehicleWheels::isValid(), "invalid PxVehicleDrive", false);
	return true;
}

PxVehicleNoDrive* PxVehicleNoDrive::allocate(const PxU32 numWheels)
{
	PX_CHECK_AND_RETURN_NULL(numWheels>0, "Cars with zero wheels are illegal");
	PX_CHECK_AND_RETURN_NULL(gToleranceScaleLength > 0, "PxVehicleNoDrive::allocate - need to call PxInitVehicleSDK");

	//Compute the bytes needed.
	const PxU32 numWheels4 = (((numWheels + 3) & ~3) >> 2);
	const PxU32 inputByteSize16 = sizeof(PxReal)*numWheels4*4;
	const PxU32 byteSize = sizeof(PxVehicleNoDrive) + 3*inputByteSize16 + PxVehicleWheels::computeByteSize(numWheels);

	//Allocate the memory.
	PxVehicleNoDrive* veh = static_cast<PxVehicleNoDrive*>(PX_ALLOC(byteSize, "PxVehicleNoDrive"));
	Cm::markSerializedMem(veh, byteSize);
	new(veh) PxVehicleNoDrive();

	//Patch up the pointers.
	PxU8* ptr = reinterpret_cast<PxU8*>(veh) + sizeof(PxVehicleNoDrive);
	veh->mSteerAngles = reinterpret_cast<PxReal*>(ptr);
	ptr += inputByteSize16;
	veh->mDriveTorques = reinterpret_cast<PxReal*>(ptr);
	ptr += inputByteSize16;
	veh->mBrakeTorques = reinterpret_cast<PxReal*>(ptr);
	ptr += inputByteSize16;
	ptr = PxVehicleWheels::patchupPointers(numWheels, veh, ptr);

	//Initialise.
	PxMemZero(veh->mSteerAngles, inputByteSize16);
	PxMemZero(veh->mDriveTorques, inputByteSize16);
	PxMemZero(veh->mBrakeTorques, inputByteSize16);
	veh->init(numWheels);

	//Set the vehicle type.
	veh->mType = PxVehicleTypes::eNODRIVE;

	return veh;
}

void PxVehicleNoDrive::free()
{
	PxVehicleWheels::free();
}

void PxVehicleNoDrive::setup
(PxPhysics* physics, PxRigidDynamic* vehActor, const PxVehicleWheelsSimData& wheelsData)
{
	//Set up the wheels.
	PxVehicleWheels::setup(physics,vehActor,wheelsData,0,wheelsData.getNbWheels());
}

PxVehicleNoDrive* PxVehicleNoDrive::create
(PxPhysics* physics, PxRigidDynamic* vehActor, 
 const PxVehicleWheelsSimData& wheelsData)
{
	PxVehicleNoDrive* veh=PxVehicleNoDrive::allocate(wheelsData.getNbWheels());
	veh->setup(physics,vehActor,wheelsData);
	return veh;
}

void PxVehicleNoDrive::setToRestState()
{
	const PxU32 numWheels4 = (((mWheelsSimData.getNbWheels() + 3) & ~3) >> 2);
	const PxU32 inputByteSize = sizeof(PxReal)*numWheels4*4;
	const PxU32 inputByteSize16 = (inputByteSize + 15) & ~15;
	PxMemZero(mSteerAngles, 3*inputByteSize16);

	//Set core to rest state.
	PxVehicleWheels::setToRestState();
}

void PxVehicleNoDrive::setBrakeTorque(const PxU32 id, const PxReal brakeTorque)
{
	PX_CHECK_AND_RETURN(id < mWheelsSimData.getNbWheels(), "PxVehicleNoDrive::setBrakeTorque - Illegal wheel");
	PX_CHECK_AND_RETURN(brakeTorque>=0, "PxVehicleNoDrive::setBrakeTorque - negative brake torques are illegal");
	mBrakeTorques[id] = brakeTorque;
}

void PxVehicleNoDrive::setDriveTorque(const PxU32 id, const PxReal driveTorque)
{
	PX_CHECK_AND_RETURN(id < mWheelsSimData.getNbWheels(), "PxVehicleNoDrive::setDriveTorque - Illegal wheel");
	mDriveTorques[id] = driveTorque;
}

void PxVehicleNoDrive::setSteerAngle(const PxU32 id, const PxReal steerAngle)
{
	PX_CHECK_AND_RETURN(id < mWheelsSimData.getNbWheels(), "PxVehicleNoDrive::setSteerAngle - Illegal wheel");
	mSteerAngles[id] = steerAngle;
}

PxReal PxVehicleNoDrive::getBrakeTorque(const PxU32 id) const
{
	PX_CHECK_AND_RETURN_VAL(id < mWheelsSimData.getNbWheels(), "PxVehicleNoDrive::getBrakeTorque - Illegal wheel", 0);
	return mBrakeTorques[id];
}

PxReal PxVehicleNoDrive::getDriveTorque(const PxU32 id) const
{
	PX_CHECK_AND_RETURN_VAL(id < mWheelsSimData.getNbWheels(), "PxVehicleNoDrive::getDriveTorque - Illegal wheel",0);
	return mDriveTorques[id];
}

PxReal PxVehicleNoDrive::getSteerAngle(const PxU32 id) const
{
	PX_CHECK_AND_RETURN_VAL(id < mWheelsSimData.getNbWheels(), "PxVehicleNoDrive::getSteerAngle - Illegal wheel",0);
	return mSteerAngles[id];
}

} //namespace physx

