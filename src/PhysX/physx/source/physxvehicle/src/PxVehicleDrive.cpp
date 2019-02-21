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

#include "PxVehicleDrive.h"
#include "PxVehicleSDK.h"
#include "PxVehicleDefaults.h"
#include "PxRigidDynamic.h"
#include "PsFoundation.h"
#include "PsUtilities.h"
#include "CmPhysXCommon.h"

namespace physx
{

bool PxVehicleDriveSimData::isValid() const
{
	PX_CHECK_AND_RETURN_VAL(mEngine.isValid(), "Invalid PxVehicleCoreSimulationData.mEngine", false);
	PX_CHECK_AND_RETURN_VAL(mGears.isValid(), "Invalid PxVehicleCoreSimulationData.mGears", false);
	PX_CHECK_AND_RETURN_VAL(mClutch.isValid(), "Invalid PxVehicleCoreSimulationData.mClutch", false);
	PX_CHECK_AND_RETURN_VAL(mAutoBox.isValid(), "Invalid PxVehicleCoreSimulationData.mAutoBox", false);
	return true;
}

void PxVehicleDriveSimData::setEngineData(const PxVehicleEngineData& engine)
{
	PX_CHECK_AND_RETURN(engine.mTorqueCurve.getNbDataPairs()>0, "Engine torque curve must specify at least one entry");
	PX_CHECK_AND_RETURN(engine.mPeakTorque>0, "Engine peak torque  must be greater than zero");
	PX_CHECK_AND_RETURN(engine.mMaxOmega>0, "Engine max omega must be greater than zero");
	PX_CHECK_AND_RETURN(engine.mDampingRateFullThrottle>=0, "Full throttle damping rate must be greater than or equal to zero");
	PX_CHECK_AND_RETURN(engine.mDampingRateZeroThrottleClutchEngaged>=0, "Zero throttle clutch engaged damping rate must be greater than or equal to zero");
	PX_CHECK_AND_RETURN(engine.mDampingRateZeroThrottleClutchDisengaged>=0, "Zero throttle clutch disengaged damping rate must be greater than or equal to zero");

	mEngine=engine;
	mEngine.mRecipMOI=1.0f/engine.mMOI;
	mEngine.mRecipMaxOmega=1.0f/engine.mMaxOmega;
}

void PxVehicleDriveSimData::setGearsData(const PxVehicleGearsData& gears)
{
	PX_CHECK_AND_RETURN(gears.mRatios[PxVehicleGearsData::eREVERSE]<0, "Reverse gear ratio must be negative");
	PX_CHECK_AND_RETURN(gears.mRatios[PxVehicleGearsData::eNEUTRAL]==0, "Neutral gear ratio must be zero");
	PX_CHECK_AND_RETURN(gears.mRatios[PxVehicleGearsData::eFIRST]>0, "First gear ratio must be positive");
	PX_CHECK_AND_RETURN(PxVehicleGearsData::eSECOND>=gears.mNbRatios || (gears.mRatios[PxVehicleGearsData::eSECOND]>0 && gears.mRatios[PxVehicleGearsData::eSECOND] < gears.mRatios[PxVehicleGearsData::eFIRST]), "Second gear ratio must be positive and less than first gear ratio");
	PX_CHECK_AND_RETURN(PxVehicleGearsData::eTHIRD>=gears.mNbRatios || (gears.mRatios[PxVehicleGearsData::eTHIRD]>0 && gears.mRatios[PxVehicleGearsData::eTHIRD] < gears.mRatios[PxVehicleGearsData::eSECOND]), "Third gear ratio must be positive and less than second gear ratio");
	PX_CHECK_AND_RETURN(PxVehicleGearsData::eFOURTH>=gears.mNbRatios || (gears.mRatios[PxVehicleGearsData::eFOURTH]>0 && gears.mRatios[PxVehicleGearsData::eFOURTH] < gears.mRatios[PxVehicleGearsData::eTHIRD]), "Fourth gear ratio must be positive and less than third gear ratio");
	PX_CHECK_AND_RETURN(PxVehicleGearsData::eFIFTH>=gears.mNbRatios || (gears.mRatios[PxVehicleGearsData::eFIFTH]>0 && gears.mRatios[PxVehicleGearsData::eFIFTH] < gears.mRatios[PxVehicleGearsData::eFOURTH]), "Fifth gear ratio must be positive and less than fourth gear ratio");
	PX_CHECK_AND_RETURN(PxVehicleGearsData::eSIXTH>=gears.mNbRatios || (gears.mRatios[PxVehicleGearsData::eSIXTH]>0 && gears.mRatios[PxVehicleGearsData::eSIXTH] < gears.mRatios[PxVehicleGearsData::eFIFTH]), "Sixth gear ratio must be positive and less than fifth gear ratio");
	PX_CHECK_AND_RETURN(gears.mFinalRatio>0, "Final gear ratio must be greater than zero");
	PX_CHECK_AND_RETURN(gears.mNbRatios>=3, "Number of gear ratios must be at least 3 - we need at least reverse, neutral, and a forward gear");

	mGears=gears;
}

void PxVehicleDriveSimData::setClutchData(const PxVehicleClutchData& clutch)
{
	PX_CHECK_AND_RETURN(clutch.mStrength>0, "Clutch strength must be greater than zero");
	PX_CHECK_AND_RETURN(PxVehicleClutchAccuracyMode::eBEST_POSSIBLE==clutch.mAccuracyMode || clutch.mEstimateIterations > 0, "Clutch mEstimateIterations must be greater than zero in eESTIMATE mode.");

	mClutch=clutch;
}

void PxVehicleDriveSimData::setAutoBoxData(const PxVehicleAutoBoxData& autobox)
{
	PX_CHECK_AND_RETURN(autobox.mUpRatios[PxVehicleGearsData::eREVERSE]>=0, "Autobox gearup ratio in reverse must be greater than or equal to zero");
	PX_CHECK_AND_RETURN(autobox.mUpRatios[PxVehicleGearsData::eNEUTRAL]>=0, "Autobox gearup ratio in neutral must be greater than zero");
	PX_CHECK_AND_RETURN(autobox.mUpRatios[PxVehicleGearsData::eFIRST]>=0, "Autobox gearup ratio in first must be greater than or equal to zero");
	PX_CHECK_AND_RETURN(autobox.mUpRatios[PxVehicleGearsData::eSECOND]>=0, "Autobox gearup ratio in second must be greater than zero");
	PX_CHECK_AND_RETURN(autobox.mUpRatios[PxVehicleGearsData::eTHIRD]>=0, "Autobox gearup ratio in third must be greater than zero");
	PX_CHECK_AND_RETURN(autobox.mUpRatios[PxVehicleGearsData::eFOURTH]>=0, "Autobox gearup ratio in fourth must be greater than zero");
	PX_CHECK_AND_RETURN(autobox.mUpRatios[PxVehicleGearsData::eFIFTH]>=0, "Autobox gearup ratio in fifth must be greater than zero");
	PX_CHECK_AND_RETURN(autobox.mDownRatios[PxVehicleGearsData::eREVERSE]>=0, "Autobox geardown ratio in reverse must be greater than or equal to zero");
	PX_CHECK_AND_RETURN(autobox.mDownRatios[PxVehicleGearsData::eNEUTRAL]>=0, "Autobox geardown ratio in neutral must be greater than zero");
	PX_CHECK_AND_RETURN(autobox.mDownRatios[PxVehicleGearsData::eFIRST]>=0, "Autobox geardown ratio in first must be greater than or equal to zero");
	PX_CHECK_AND_RETURN(autobox.mDownRatios[PxVehicleGearsData::eSECOND]>=0, "Autobox geardown ratio in second must be greater than zero");
	PX_CHECK_AND_RETURN(autobox.mDownRatios[PxVehicleGearsData::eTHIRD]>=0, "Autobox geardown ratio in third must be greater than zero");
	PX_CHECK_AND_RETURN(autobox.mDownRatios[PxVehicleGearsData::eFOURTH]>=0, "Autobox geardown ratio in fourth must be greater than zero");
	PX_CHECK_AND_RETURN(autobox.mDownRatios[PxVehicleGearsData::eFIFTH]>=0, "Autobox geardown ratio in fifth must be greater than zero");
	PX_CHECK_AND_RETURN(autobox.mDownRatios[PxVehicleGearsData::eSIXTH]>=0, "Autobox geardown ratio in fifth must be greater than zero");

	mAutoBox=autobox;
}

///////////////////////////////////

PxVehicleDriveDynData::PxVehicleDriveDynData()
	:	mUseAutoGears(false),
		mGearUpPressed(false),
		mGearDownPressed(false),
		mCurrentGear(PxVehicleGearsData::eNEUTRAL),
		mTargetGear(PxVehicleGearsData::eNEUTRAL),
		mEnginespeed(0.0f),
		mGearSwitchTime(0.0f),
		mAutoBoxSwitchTime(0.0f)
{
	for(PxU32 i=0;i<eMAX_NB_ANALOG_INPUTS;i++)
	{
		mControlAnalogVals[i]=0.0f;		
	}
}

void PxVehicleDriveDynData::setToRestState()
{
	//Set analog inputs to zero so the vehicle starts completely at rest.
	for(PxU32 i=0;i<eMAX_NB_ANALOG_INPUTS;i++)
	{
		mControlAnalogVals[i]=0.0f;
	}
	mGearUpPressed=false;
	mGearDownPressed=false;

	//Set the vehicle to neutral gear.
	mCurrentGear=PxVehicleGearsData::eNEUTRAL;
	mTargetGear=PxVehicleGearsData::eNEUTRAL;
	mGearSwitchTime=0.0f;
	mAutoBoxSwitchTime=0.0f;

	//Set internal dynamics to zero so the vehicle starts completely at rest.
	mEnginespeed=0.0f;
}

bool PxVehicleDriveDynData::isValid() const
{
	return true;
}

void PxVehicleDriveDynData::setAnalogInput(const PxU32 type, const PxReal analogVal)
{
	PX_CHECK_AND_RETURN(analogVal>=-1.01f && analogVal<=1.01f, "PxVehicleDriveDynData::setAnalogInput - analogVal must be in range (-1,1)");
	PX_CHECK_AND_RETURN(type<eMAX_NB_ANALOG_INPUTS, "PxVehicleDriveDynData::setAnalogInput - illegal type");
	mControlAnalogVals[type]=analogVal;
}

PxReal PxVehicleDriveDynData::getAnalogInput(const PxU32 type) const
{
	PX_CHECK_AND_RETURN_VAL(type<eMAX_NB_ANALOG_INPUTS, "PxVehicleDriveDynData::getAnalogInput - illegal type", 0.0f);
	return mControlAnalogVals[type];
}

///////////////////////////////////

bool PxVehicleDrive::isValid() const
{
	PX_CHECK_AND_RETURN_VAL(PxVehicleWheels::isValid(), "invalid PxVehicleWheels", false);
	PX_CHECK_AND_RETURN_VAL(mDriveDynData.isValid(), "Invalid PxVehicleDrive.mCoreSimData", false);
	return true;
}

PxU32 PxVehicleDrive::computeByteSize(const PxU32 numWheels)
{
	return PxVehicleWheels::computeByteSize(numWheels);
}

PxU8* PxVehicleDrive::patchupPointers( const PxU32 numWheels, PxVehicleDrive* veh, PxU8* ptr)
{
	return PxVehicleWheels::patchupPointers(numWheels, veh, ptr);
}

void PxVehicleDrive::init(const PxU32 numWheels)
{
	PxVehicleWheels::init(numWheels);
}

void PxVehicleDrive::free()
{
	PxVehicleWheels::free();
}

void PxVehicleDrive::setup
(PxPhysics* physics, PxRigidDynamic* vehActor, 
 const PxVehicleWheelsSimData& wheelsData,  
 const PxU32 numDrivenWheels, const PxU32 numNonDrivenWheels)
{
	//Set up the wheels.
	PxVehicleWheels::setup(physics,vehActor,wheelsData,numDrivenWheels,numNonDrivenWheels);
}

void PxVehicleDrive::setToRestState()
{
	//Set core to rest state.
	PxVehicleWheels::setToRestState();

	//Set dynamics data to rest state.
	mDriveDynData.setToRestState();
}

} //namespace physx

