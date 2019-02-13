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

#include "PxVehicleComponents.h"
#include "PxVehicleDefaults.h"
#include "CmPhysXCommon.h"
#include "CmBitMap.h"
#include "PsFoundation.h"

namespace physx
{

bool PxVehicleChassisData::isValid() const
{
	PX_CHECK_AND_RETURN_VAL(mMOI.x>0.0f && mMOI.y>0.0f && mMOI.z>0.0f, "Illegal PxVehicleChassisData.mMOI - each element of the chassis moi needs to non-zero", false);
	PX_CHECK_AND_RETURN_VAL(mMass>0.0f, "Ilegal PxVehicleChassisData.mMass -  chassis mass needs to be non-zero", false);
	return true;
}


bool PxVehicleEngineData::isValid() const
{
	PX_CHECK_AND_RETURN_VAL(mPeakTorque>0.0f, "PxVehicleEngineData.mPeakTorque must be greater than zero", false);
	PX_CHECK_AND_RETURN_VAL(mMaxOmega>0.0f, "PxVehicleEngineData.mMaxOmega must be greater than zero", false);
	PX_CHECK_AND_RETURN_VAL(mDampingRateFullThrottle>0.0f, "PxVehicleEngineData.mDampingRateFullThrottle must be greater than zero", false);
	PX_CHECK_AND_RETURN_VAL(mDampingRateZeroThrottleClutchEngaged>0.0f, "PxVehicleEngineData.mDampingRateZeroThrottleClutchEngaged must be greater than zero", false);
	PX_CHECK_AND_RETURN_VAL(mDampingRateZeroThrottleClutchDisengaged>0.0f, "PxVehicleEngineData.mDampingRateZeroThrottleClutchDisengaged must be greater than zero", false);
	PX_CHECK_AND_RETURN_VAL(mRecipMOI>0.0f, "PxVehicleEngineData.mRecipMOI must be greater than zero", false);
	PX_CHECK_AND_RETURN_VAL(mRecipMaxOmega>0.0f, "PxVehicleEngineData.mRecipMaxOmega must be greater than zero", false);
	PX_CHECK_AND_RETURN_VAL(PxAbs((1.0f/mMaxOmega)-mRecipMaxOmega) <= 0.001f, "PxVehicleEngineData.mMaxOmega and PxVehicleEngineData.mRecipMaxOmega don't match", false);
	return true;
}

bool PxVehicleGearsData::isValid() const
{
	PX_CHECK_AND_RETURN_VAL(mFinalRatio>0, "PxVehicleGearsData.mFinalRatio must be greater than zero", false);
	PX_CHECK_AND_RETURN_VAL(mNbRatios>=1, "PxVehicleGearsData.mNbRatios must be greater than zero", false);
	PX_CHECK_AND_RETURN_VAL(mSwitchTime>=0.0f, "PxVehicleGearsData.mSwitchTime must be greater than or equal to zero", false);

	PX_CHECK_AND_RETURN_VAL(mRatios[PxVehicleGearsData::eREVERSE]<0.0f, "PxVehicleGearsData.mRatios[PxVehicleGearsData::eREVERSE] must be less than zero", false);
	PX_CHECK_AND_RETURN_VAL(mRatios[PxVehicleGearsData::eNEUTRAL]==0.0f, "PxVehicleGearsData.mRatios[PxVehicleGearsData::eNEUTRAL] must be zero", false);
	for(PxU32 i=PxVehicleGearsData::eFIRST;i<mNbRatios;i++)
	{
		PX_CHECK_AND_RETURN_VAL(mRatios[i]>0.0f, "Forward gear ratios must be greater than zero", false);
	}
	for(PxU32 i=PxVehicleGearsData::eSECOND;i<mNbRatios;i++)
	{
		PX_CHECK_AND_RETURN_VAL(mRatios[i]<mRatios[i-1], "Forward gear ratios must be a descending sequence of gear ratios", false);
	}
	return true;
}

bool PxVehicleAutoBoxData::isValid() const
{
	for(PxU32 i=0;i<PxVehicleGearsData::eGEARSRATIO_COUNT;i++)
	{
		PX_CHECK_AND_RETURN_VAL(mUpRatios[i]>=0.0f, "PxVehicleAutoBoxData.mUpRatios must be greater than or equal to zero", false);
		PX_CHECK_AND_RETURN_VAL(mDownRatios[i]>=0.0f, "PxVehicleAutoBoxData.mDownRatios must be greater than or equal to zero", false);
	}
	return true;
}

bool PxVehicleDifferential4WData::isValid() const
{
	PX_CHECK_AND_RETURN_VAL(mFrontRearSplit<=1.0f && mFrontRearSplit>=0.0f, "PxVehicleDifferential4WData.mFrontRearSplit must be in range (0,1)", false);
	PX_CHECK_AND_RETURN_VAL(mFrontLeftRightSplit<=1.0f && mFrontLeftRightSplit>=0.0f, "PxVehicleDifferential4WData.mFrontLeftRightSplit must be in range (0,1)", false);
	PX_CHECK_AND_RETURN_VAL(mRearLeftRightSplit<=1.0f || mRearLeftRightSplit>=0.0f, "PxVehicleDifferential4WData.mRearLeftRightSplit must be in range (0,1)", false);
	PX_CHECK_AND_RETURN_VAL(mCentreBias>=1.0f, "PxVehicleDifferential4WData.mCentreBias must be greater than or equal to 1.0f", false);
	PX_CHECK_AND_RETURN_VAL(mFrontBias>=1.0f, "PxVehicleDifferential4WData.mFrontBias must be greater than or equal to 1.0f", false);
	PX_CHECK_AND_RETURN_VAL(mRearBias>=1.0f, "PxVehicleDifferential4WData.mRearBias must be greater than or equal to 1.0f", false);
	PX_CHECK_AND_RETURN_VAL(mType<PxVehicleDifferential4WData::eMAX_NB_DIFF_TYPES, "PxVehicleDifferential4WData.mType has illegal value", false);
	return true;
}

void PxVehicleDifferentialNWData::setDrivenWheel(const PxU32 wheelId, const bool drivenState)
{
	Cm::BitMap bitmap;
	bitmap.setWords(mBitmapBuffer,((PX_MAX_NB_WHEELS + 31) & ~31) >> 5);
	PxU32 numDrivenWheels=mNbDrivenWheels;
	if(drivenState)
	{
		if(!bitmap.test(wheelId))
		{
			numDrivenWheels++;
			bitmap.set(wheelId);
			mInvNbDrivenWheels=1.0f/(1.0f*numDrivenWheels);
		}
	}
	else if(bitmap.test(wheelId))
	{
		numDrivenWheels--;
		bitmap.reset(wheelId);
		mInvNbDrivenWheels =  numDrivenWheels>0.0f ? 1.0f/(1.0f*numDrivenWheels) : 0.0f;
	}
	mNbDrivenWheels=numDrivenWheels;
}

bool PxVehicleDifferentialNWData::getIsDrivenWheel(const PxU32 wheelId) const
{
	Cm::BitMap bitmap;
	bitmap.setWords(const_cast<PxU32*>(mBitmapBuffer),((PX_MAX_NB_WHEELS + 31) & ~31) >> 5);
	return (bitmap.test(wheelId) ? true : false);
}

PxU32 PxVehicleDifferentialNWData::getDrivenWheelStatus() const
{
	PX_ASSERT(((PX_MAX_NB_WHEELS + 31) & ~31) >> 5 == 1);
	return mBitmapBuffer[0];
}

void PxVehicleDifferentialNWData::setDrivenWheelStatus(PxU32 status)
{
	PX_ASSERT(((PX_MAX_NB_WHEELS + 31) & ~31) >> 5 == 1);

	Cm::BitMap bitmap;
	bitmap.setWords(&status, 1);
	
	for(PxU32 i = 0; i < PX_MAX_NB_WHEELS; ++i)
	{
		setDrivenWheel(i, !!bitmap.test(i));
	}
}

bool PxVehicleDifferentialNWData::isValid() const
{
	PX_CHECK_AND_RETURN_VAL(mNbDrivenWheels<=PX_MAX_NB_WHEELS,  "PxVehicleDifferentialNWData.mNbDrivenWheels must be in range (0,20)", false);
	return true;
}

bool PxVehicleAckermannGeometryData::isValid() const
{
	PX_CHECK_AND_RETURN_VAL(mAccuracy>=0.0f && mAccuracy<=1.0f, "PxVehicleAckermannGeometryData.mAccuracy must be in range (0,1)", false);
	PX_CHECK_AND_RETURN_VAL(mFrontWidth>0.0f, "PxVehicleAckermannGeometryData.mFrontWidth must be greater than zero", false);
	PX_CHECK_AND_RETURN_VAL(mRearWidth>0.0f, "PxVehicleAckermannGeometryData.mRearWidth must be greater than zero", false);
	PX_CHECK_AND_RETURN_VAL(mAxleSeparation>0.0f, "PxVehicleAckermannGeometryData.mAxleSeparation must be greater than zero", false);
	return true;
}

bool PxVehicleClutchData::isValid() const
{
	PX_CHECK_AND_RETURN_VAL(mStrength>0, "PxVehicleClutchData.mStrength must be greater than zero", false);
	return true;
}

bool PxVehicleTireLoadFilterData::isValid() const
{
	PX_CHECK_AND_RETURN_VAL(mMaxNormalisedLoad>=mMinNormalisedLoad, "PxVehicleTireLoadFilterData.mMaxNormalisedLoad must be greater than or equal to PxVehicleTireLoadFilterData.mMinNormalisedLoad", false);
	PX_CHECK_AND_RETURN_VAL(mMaxFilteredNormalisedLoad>0, "PxVehicleTireLoadFilterData.mMaxFilteredNormalisedLoad must be greater than zero", false);
	PX_CHECK_AND_RETURN_VAL(PxAbs((1.0f/(mMaxNormalisedLoad - mMinNormalisedLoad)) - mDenominator) < 0.001f, "PxVehicleTireLoadFilterData.mMaxFilteredNormalisedLoad, PxVehicleTireLoadFilterData.mMinNormalisedLoad, and PxVehicleTireLoadFilterData.mDenominator don't match", false);
	return true;
}

bool PxVehicleWheelData::isValid() const
{
	PX_CHECK_AND_RETURN_VAL(mRadius>0.0f, "PxVehicleWheelData.mRadius must be greater than zero", false);
	PX_CHECK_AND_RETURN_VAL(mWidth>0.0f, "PxVehicleWheelData.mWidth must be greater than zero", false);
	PX_CHECK_AND_RETURN_VAL(mMass>0.0f, "PxVehicleWheelData.mMass must be greater than zero", false);
	PX_CHECK_AND_RETURN_VAL(mMOI>0.0f, "PxVehicleWheelData.mMOI must be greater than zero", false);
	PX_CHECK_AND_RETURN_VAL(mDampingRate>0.0f, "PxVehicleWheelData.mDampingRate must be greater than zero", false);
	PX_CHECK_AND_RETURN_VAL(mMaxBrakeTorque>=0.0f, "PxVehicleWheelData.mMaxBrakeTorque must be greater than or equal to zero", false);
	PX_CHECK_AND_RETURN_VAL(mMaxHandBrakeTorque>=0.0f, "PxVehicleWheelData.mMaxHandBrakeTorque must be greater than or equal to zero", false);
	PX_CHECK_AND_RETURN_VAL(mToeAngle<=PxPi, "PxVehicleWheelData.mToeAngle must be less than Pi", false);
	PX_CHECK_AND_RETURN_VAL(PxAbs((1.0f/mRadius) - mRecipRadius) < 0.001f, "PxVehicleWheelData.mRadius and PxVehicleWheelData.mRecipRadius don't match", false);
	PX_CHECK_AND_RETURN_VAL(PxAbs((1.0f/mMOI) - mRecipMOI) < 0.001f, "PxVehicleWheelData.mMOI and PxVehicleWheelData.mRecipMOI don't match", false);
	return true;
}

bool PxVehicleSuspensionData::isValid() const
{
	PX_CHECK_AND_RETURN_VAL(mSpringStrength>=0.0f, "PxVehicleSuspensionData.mSpringStrength must be greater than or equal to zero", false);
	PX_CHECK_AND_RETURN_VAL(mSpringDamperRate>=0.0f, "PxVehicleSuspensionData.mSpringDamperRate must be greater than or equal to zero", false);
	PX_CHECK_AND_RETURN_VAL(mMaxCompression>=0.0f, "PxVehicleSuspensionData.mMaxCompression must be greater than or equal to zero", false);
	PX_CHECK_AND_RETURN_VAL(mMaxDroop>=0.0f, "PxVehicleSuspensionData.mMaxDroop must be greater than or equal to zero", false);
	PX_CHECK_AND_RETURN_VAL(mSprungMass>=0.0f, "PxVehicleSuspensionData.mSprungMass must be greater than or equal to zero", false);
	return true;
}

bool PxVehicleAntiRollBarData::isValid() const
{
	PX_CHECK_AND_RETURN_VAL(((mWheel0< PX_MAX_NB_WHEELS) && (mWheel1 < PX_MAX_NB_WHEELS)), "PxVehicleAntiRoll.mWheel0 and PxVehicleAntiRoll.mWheel1 are an illegal pair", false);
	PX_CHECK_AND_RETURN_VAL((mWheel0 != mWheel1), "PxVehicleAntiRoll.mWheel0 == PxVehicleAntiRoll.mWheel1", false);
	PX_CHECK_AND_RETURN_VAL(mStiffness>=0, "PxVehicleAntiRoll::mStiffness must be greater than or equal to zero", false);
	return true;
}

bool PxVehicleTireData::isValid() const 
{
	PX_CHECK_AND_RETURN_VAL(mFrictionVsSlipGraph[0][0]>=0.0f && mFrictionVsSlipGraph[0][1]>=0.0f, "Illegal values for mFrictionVsSlipGraph[0]", false);
	PX_CHECK_AND_RETURN_VAL(mFrictionVsSlipGraph[1][0]>=0.0f && mFrictionVsSlipGraph[1][1]>=0.0f, "Illegal values for mFrictionVsSlipGraph[1]", false);
	PX_CHECK_AND_RETURN_VAL(mFrictionVsSlipGraph[2][0]>=0.0f && mFrictionVsSlipGraph[2][1]>=0.0f, "Illegal values for mFrictionVsSlipGraph[2]", false);
	PX_CHECK_AND_RETURN_VAL(PxAbs((1.0f/(mFrictionVsSlipGraph[1][0]-mFrictionVsSlipGraph[0][0])) - mFrictionVsSlipGraphRecipx1Minusx0) < 0.001f, "PxVehicleTireData.mFrictionVsSlipGraphRecipx1Minusx0 not set up", false);
	PX_CHECK_AND_RETURN_VAL(PxAbs((1.0f/(mFrictionVsSlipGraph[2][0]-mFrictionVsSlipGraph[1][0])) - mFrictionVsSlipGraphRecipx2Minusx1) < 0.001f, "PxVehicleTireData.mFrictionVsSlipGraphRecipx2Minusx1 not set up", false);
	return true;
}
} //namespace physx

