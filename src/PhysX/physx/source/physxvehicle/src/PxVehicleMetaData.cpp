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

#include "PxVehicleComponents.h"
#include "PxVehicleDrive.h"
#include "PxVehicleNoDrive.h"
#include "PxVehicleDrive4W.h"
#include "PxVehicleDriveNW.h"
#include "PxVehicleDriveTank.h"
#include "PxVehicleSuspWheelTire4.h"
#include "PxVehicleSuspLimitConstraintShader.h"

#include "PxMetaData.h"

using namespace physx;

namespace
{
	typedef PxFixedSizeLookupTable<PxVehicleEngineData::eMAX_NB_ENGINE_TORQUE_CURVE_ENTRIES> PxVehicleEngineTable;
	class ShadowLookupTable : public PxVehicleEngineTable
	{
	public:
		static void getBinaryMetaData(PxOutputStream& stream_)
		{
			PX_DEF_BIN_METADATA_CLASS(stream_,		ShadowLookupTable)
			PX_DEF_BIN_METADATA_ITEMS_AUTO(stream_,	PxVehicleEngineTable,	PxReal,	mDataPairs,		0)	
			PX_DEF_BIN_METADATA_ITEM(stream_,		PxVehicleEngineTable,	PxU32,	mNbDataPairs,	0)	
			PX_DEF_BIN_METADATA_ITEMS_AUTO(stream_,	PxVehicleEngineTable,	PxU32,	mPad,			PxMetaDataFlag::ePADDING)
		}
	};
}

static void getBinaryMetaData_PxFixedSizeLookupTable(PxOutputStream& stream)
{    
	ShadowLookupTable::getBinaryMetaData(stream);
    PX_DEF_BIN_METADATA_TYPEDEF(stream, PxFixedSizeLookupTable<PxVehicleEngineData::eMAX_NB_ENGINE_TORQUE_CURVE_ENTRIES>, ShadowLookupTable)	
}

void PxVehicleDriveSimData::getBinaryMetaData(PxOutputStream& stream)
{
	getBinaryMetaData_PxFixedSizeLookupTable(stream);
	PX_DEF_BIN_METADATA_TYPEDEF(stream,		PxVehicleClutchAccuracyMode::Enum, PxU32)

	//PxVehicleEngineData
	PX_DEF_BIN_METADATA_CLASS(stream,		PxVehicleEngineData)
	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleEngineData,	ShadowLookupTable,	mTorqueCurve,								0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleEngineData, 	PxReal,				mMOI,										0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleEngineData, 	PxReal,				mPeakTorque,								0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleEngineData, 	PxReal,				mMaxOmega,									0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleEngineData, 	PxReal,				mDampingRateFullThrottle,					0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleEngineData, 	PxReal,				mDampingRateZeroThrottleClutchEngaged,		0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleEngineData, 	PxReal,				mDampingRateZeroThrottleClutchDisengaged,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleEngineData, 	PxReal,				mRecipMOI,									0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleEngineData, 	PxReal,				mRecipMaxOmega,								0)


	//PxVehicleGearsData
	PX_DEF_BIN_METADATA_CLASS(stream,		PxVehicleGearsData)
	
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleGearsData,		PxReal,				mRatios,									0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleGearsData,		PxReal,				mFinalRatio,								0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleGearsData,		PxU32,				mNbRatios,									0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleGearsData,		PxReal,				mSwitchTime,								0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleGearsData,		PxReal,				mPad,										PxMetaDataFlag::ePADDING)	

	//PxVehicleClutchData
	PX_DEF_BIN_METADATA_CLASS(stream,		PxVehicleClutchData)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleClutchData,	PxReal,                            mStrength,		   			0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleClutchData,	PxVehicleClutchAccuracyMode::Enum, mAccuracyMode,	   			0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleClutchData,	PxU32,	                           mEstimateIterations,			0)	
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleClutchData,	PxU8,	                           mPad,		       			PxMetaDataFlag::ePADDING)

	//PxVehicleAutoBoxData
	PX_DEF_BIN_METADATA_CLASS(stream,		PxVehicleAutoBoxData)	
	
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleAutoBoxData,	PxReal, 			mUpRatios,	    							0)	
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleAutoBoxData,	PxReal, 			mDownRatios,								0)	

	//PxVehicleDriveSimData
	PX_DEF_BIN_METADATA_CLASS(stream,		PxVehicleDriveSimData)
	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleDriveSimData,	PxVehicleEngineData,	mEngine,								0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleDriveSimData,	PxVehicleGearsData,		mGears,									0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleDriveSimData,	PxVehicleClutchData,	mClutch,								0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleDriveSimData,	PxVehicleAutoBoxData,	mAutoBox,								0)	
}

void PxVehicleDrive::getBinaryMetaData(PxOutputStream& stream)
{
	//PxVehicleDriveDynData
	PX_DEF_BIN_METADATA_CLASS(stream,		PxVehicleDriveDynData)	

	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleDriveDynData,	PxReal,					mControlAnalogVals,		0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleDriveDynData, 	bool,					mUseAutoGears,			0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleDriveDynData,	bool,					mGearUpPressed,			0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleDriveDynData,	bool,					mGearDownPressed,		0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleDriveDynData,	PxU32,					mCurrentGear,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleDriveDynData,	PxU32,					mTargetGear,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleDriveDynData,	PxReal, 				mEnginespeed,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleDriveDynData,	PxReal, 				mGearSwitchTime,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleDriveDynData,	PxReal, 				mAutoBoxSwitchTime,		0)

	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleDriveDynData,	PxU32,					mPad,					PxMetaDataFlag::ePADDING)	

	//PxVehicleDrive
	PX_DEF_BIN_METADATA_VCLASS(stream,		PxVehicleDrive)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	PxVehicleDrive,			PxVehicleWheels)	

	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleDrive,			PxVehicleDriveDynData,	mDriveDynData,			0)	
}

void PxVehicleDriveSimData4W::getBinaryMetaData(PxOutputStream& stream)
{
	//PxVehicleDifferential4WData
	PX_DEF_BIN_METADATA_CLASS(stream,		PxVehicleDifferential4WData)	

	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleDifferential4WData,		PxReal,		mFrontRearSplit,		0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleDifferential4WData,		PxReal,		mFrontLeftRightSplit,	0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleDifferential4WData,		PxReal,		mRearLeftRightSplit,	0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleDifferential4WData,		PxReal,		mCentreBias,			0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleDifferential4WData,		PxReal,		mFrontBias,				0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleDifferential4WData,		PxReal,		mRearBias,				0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleDifferential4WData,		PxU32,		mType,					0)
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleDifferential4WData,		PxReal,		mPad,					PxMetaDataFlag::ePADDING)
	
	//PxVehicleAckermannGeometryData
	PX_DEF_BIN_METADATA_CLASS(stream,		PxVehicleAckermannGeometryData)	

	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleAckermannGeometryData, 	PxReal,		mAccuracy,				0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleAckermannGeometryData, 	PxReal,		mFrontWidth,			0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleAckermannGeometryData, 	PxReal,		mRearWidth,				0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleAckermannGeometryData, 	PxReal,		mAxleSeparation,		0)	
	
	//PxVehicleDriveSimData4W	
	PX_DEF_BIN_METADATA_CLASS(stream,		PxVehicleDriveSimData4W)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	PxVehicleDriveSimData4W,			PxVehicleDriveSimData)	

	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleDriveSimData4W,			PxVehicleDifferential4WData,	mDiff,				0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleDriveSimData4W,			PxVehicleAckermannGeometryData,	mAckermannGeometry, 0)
}

void PxVehicleDriveSimDataNW::getBinaryMetaData(PxOutputStream& stream)
{
	//PxVehicleDifferentialWData
	PX_DEF_BIN_METADATA_CLASS(stream,		PxVehicleDifferentialNWData)	
	
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleDifferentialNWData, 		PxU32,						mBitmapBuffer,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleDifferentialNWData, 		PxU32,						mNbDrivenWheels,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleDifferentialNWData, 		PxReal,						mInvNbDrivenWheels, 	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleDifferentialNWData, 		PxU32,						mPad,					0)

	//PxVehicleDriveSimDataNW
	PX_DEF_BIN_METADATA_CLASS(stream,		PxVehicleDriveSimDataNW)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	PxVehicleDriveSimDataNW,			PxVehicleDriveSimData)	

	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleDriveSimDataNW,			PxVehicleDifferentialNWData, mDiff,					0)	
}

void PxVehicleNoDrive::getBinaryMetaData(PxOutputStream& stream)
{
	PxVehicleDrive::getBinaryMetaData(stream);
	PxVehicleWheels::getBinaryMetaData(stream);
	PxVehicleDriveSimData::getBinaryMetaData(stream);

	PX_DEF_BIN_METADATA_VCLASS(stream,		PxVehicleNoDrive)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	PxVehicleNoDrive,	PxVehicleWheels)	

	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleNoDrive,	PxReal,			mSteerAngles,	PxMetaDataFlag::ePTR)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleNoDrive,	PxReal,			mDriveTorques,	PxMetaDataFlag::ePTR)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleNoDrive,	PxReal,			mBrakeTorques,	PxMetaDataFlag::ePTR)	
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleNoDrive,	PxU32,			mPad,			PxMetaDataFlag::ePADDING)

	PX_DEF_BIN_METADATA_EXTRA_ITEMS(stream,	PxVehicleNoDrive,	PxReal, 		mSteerAngles,	mWheelsSimData.mNbWheels4, 0, PX_SERIAL_ALIGN)
	PX_DEF_BIN_METADATA_EXTRA_ITEMS(stream,	PxVehicleNoDrive,	PxReal, 		mSteerAngles,	mWheelsSimData.mNbWheels4, 0, 0)
	PX_DEF_BIN_METADATA_EXTRA_ITEMS(stream,	PxVehicleNoDrive,	PxReal, 		mSteerAngles,	mWheelsSimData.mNbWheels4, 0, 0)
	PX_DEF_BIN_METADATA_EXTRA_ITEMS(stream,	PxVehicleNoDrive,	PxReal, 		mSteerAngles,	mWheelsSimData.mNbWheels4, 0, 0)

	PX_DEF_BIN_METADATA_EXTRA_ITEMS(stream,	PxVehicleNoDrive,	PxReal, 		mDriveTorques,  mWheelsSimData.mNbWheels4, 0, 0)
	PX_DEF_BIN_METADATA_EXTRA_ITEMS(stream,	PxVehicleNoDrive,	PxReal, 		mDriveTorques,  mWheelsSimData.mNbWheels4, 0, 0)
	PX_DEF_BIN_METADATA_EXTRA_ITEMS(stream,	PxVehicleNoDrive,	PxReal, 		mDriveTorques,  mWheelsSimData.mNbWheels4, 0, 0)
	PX_DEF_BIN_METADATA_EXTRA_ITEMS(stream,	PxVehicleNoDrive,	PxReal, 		mDriveTorques,  mWheelsSimData.mNbWheels4, 0, 0)

	PX_DEF_BIN_METADATA_EXTRA_ITEMS(stream,	PxVehicleNoDrive,	PxReal, 		mBrakeTorques,  mWheelsSimData.mNbWheels4, 0, 0)
	PX_DEF_BIN_METADATA_EXTRA_ITEMS(stream,	PxVehicleNoDrive,	PxReal, 		mBrakeTorques,  mWheelsSimData.mNbWheels4, 0, 0)
	PX_DEF_BIN_METADATA_EXTRA_ITEMS(stream,	PxVehicleNoDrive,	PxReal, 		mBrakeTorques,  mWheelsSimData.mNbWheels4, 0, 0)
	PX_DEF_BIN_METADATA_EXTRA_ITEMS(stream,	PxVehicleNoDrive,	PxReal, 		mBrakeTorques,  mWheelsSimData.mNbWheels4, 0, 0)
}

void PxVehicleNoDrive::exportExtraData(PxSerializationContext& stream)
{
	PxVehicleWheels::exportExtraData(stream);

	PxU32 size = sizeof(PxReal)*4*mWheelsSimData.mNbWheels4;
	stream.alignData();
	stream.writeData(mSteerAngles, size);
	stream.writeData(mDriveTorques, size);
	stream.writeData(mBrakeTorques, size);
}

void PxVehicleNoDrive::importExtraData(PxDeserializationContext& context)
{
	PxVehicleWheels::importExtraData(context);

	context.alignExtraData();
	mSteerAngles = context.readExtraData<PxReal>(4*mWheelsSimData.mNbWheels4);
	mDriveTorques = context.readExtraData<PxReal>(4*mWheelsSimData.mNbWheels4);
	mBrakeTorques = context.readExtraData<PxReal>(4*mWheelsSimData.mNbWheels4);
}

PxVehicleNoDrive* PxVehicleNoDrive::createObject(PxU8*& address, PxDeserializationContext& context)
{
	PxVehicleNoDrive* obj = new (address) PxVehicleNoDrive(PxBaseFlag::eIS_RELEASABLE);
	address += sizeof(PxVehicleNoDrive);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}

void PxVehicleDrive4W::getBinaryMetaData(PxOutputStream& stream)
{
	PxVehicleDrive::getBinaryMetaData(stream);
	PxVehicleWheels::getBinaryMetaData(stream);
	PxVehicleDriveSimData::getBinaryMetaData(stream);
	PxVehicleDriveSimData4W::getBinaryMetaData(stream);

	PX_DEF_BIN_METADATA_VCLASS(stream,		PxVehicleDrive4W)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	PxVehicleDrive4W,	PxVehicleDrive)	

	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleDrive4W,	PxVehicleDriveSimData4W, mDriveSimData, 0)	
}

PxVehicleDrive4W* PxVehicleDrive4W::createObject(PxU8*& address, PxDeserializationContext& context)
{
	PxVehicleDrive4W* obj = new (address) PxVehicleDrive4W(PxBaseFlag::eIS_RELEASABLE);
	address += sizeof(PxVehicleDrive4W);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}

void PxVehicleDriveNW::getBinaryMetaData(PxOutputStream& stream)
{
	PxVehicleDriveSimDataNW::getBinaryMetaData(stream);

	PX_DEF_BIN_METADATA_VCLASS(stream,		PxVehicleDriveNW)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	PxVehicleDriveNW,	PxVehicleDrive)	

	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleDriveNW,	PxVehicleDriveSimDataNW, mDriveSimData, 0)	
}

PxVehicleDriveNW* PxVehicleDriveNW::createObject(PxU8*& address, PxDeserializationContext& context)
{
	PxVehicleDriveNW* obj = new (address) PxVehicleDriveNW(PxBaseFlag::eIS_RELEASABLE);
	address += sizeof(PxVehicleDriveNW);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}

void PxVehicleDriveTank::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_VCLASS(stream,		PxVehicleDriveTank)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	PxVehicleDriveTank, PxVehicleDrive)	

	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleDriveTank,	PxVehicleDriveSimData,	mDriveSimData,	0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleDriveTank,	PxU32,					mDriveModel,	0)		
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleDriveTank, PxU32,					mPad,			PxMetaDataFlag::ePADDING)	
}

PxVehicleDriveTank* PxVehicleDriveTank::createObject(PxU8*& address, PxDeserializationContext& context)
{
	PxVehicleDriveTank* obj = new (address) PxVehicleDriveTank(PxBaseFlag::eIS_RELEASABLE);
	address += sizeof(PxVehicleDriveTank);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}

void PxVehicleWheelsSimData::getBinaryMetaData(PxOutputStream& stream)
{
	PxVehicleWheels4SimData::getBinaryMetaData(stream);
	//PxVehicleTireLoadFilterData
	PX_DEF_BIN_METADATA_CLASS(stream,		PxVehicleTireLoadFilterData)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleTireLoadFilterData,	PxReal, mMinNormalisedLoad,			0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleTireLoadFilterData,	PxReal, mMinFilteredNormalisedLoad,	0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleTireLoadFilterData,	PxReal, mMaxNormalisedLoad,			0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleTireLoadFilterData,	PxReal, mMaxFilteredNormalisedLoad,	0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleTireLoadFilterData,	PxReal, mDenominator,				0)	
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleTireLoadFilterData,	PxU32,  mPad,						PxMetaDataFlag::ePADDING)

	//Add anti-roll here to save us having to add an extra function.
	PX_DEF_BIN_METADATA_CLASS(stream,		PxVehicleAntiRollBarData)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleAntiRollBarData,		PxU32,		mWheel0,		0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleAntiRollBarData,		PxU32,		mWheel1,		0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleAntiRollBarData,		PxReal,		mStiffness,		0)	
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleAntiRollBarData,		PxU32,		mPad,			PxMetaDataFlag::ePADDING)	

	//PxVehicleWheelsSimData
	PX_DEF_BIN_METADATA_CLASS(stream,		PxVehicleWheelsSimData)	

	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheelsSimData, 	PxVehicleTireLoadFilterData,	mNormalisedLoadFilter,			0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheelsSimData, 	PxVehicleWheels4SimData,		mWheels4SimData,				PxMetaDataFlag::ePTR)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheelsSimData, 	PxU32, 							mNbWheels4,						0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheelsSimData, 	PxU32, 							mNbActiveWheels,				0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheelsSimData, 	PxVehicleAntiRollBarData,			mAntiRollBars,					PxMetaDataFlag::ePTR)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheelsSimData, 	PxU32, 							mNbAntiRollBars4,				0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheelsSimData, 	PxU32, 							mNbActiveAntiRollBars,			0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheelsSimData, 	PxU32, 							mActiveWheelsBitmapBuffer,		0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheelsSimData, 	PxReal, 						mThresholdLongitudinalSpeed,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheelsSimData, 	PxU32, 							mLowForwardSpeedSubStepCount,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheelsSimData, 	PxU32, 							mHighForwardSpeedSubStepCount,	0)
    PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheelsSimData, 	PxU32, 							mMinLongSlipDenominator,		0)

#if PX_P64_FAMILY
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream, PxVehicleWheelsSimData, PxU32, mPad, PxMetaDataFlag::ePADDING)
#else
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream, PxVehicleWheelsSimData, PxU32, mPad, PxMetaDataFlag::ePADDING)
#endif

	PX_DEF_BIN_METADATA_EXTRA_ITEMS(stream, PxVehicleWheelsSimData, PxVehicleWheels4SimData, mWheels4SimData, mNbWheels4, 0, 0)
	PX_DEF_BIN_METADATA_EXTRA_ITEMS(stream, PxVehicleWheelsSimData, PxVehicleAntiRollBarData, mAntiRollBars, mNbAntiRollBars4, 0, 0)
}

void PxVehicleWheelsDynData::getBinaryMetaData(PxOutputStream& stream)
{
	PxVehicleWheels4DynData::getBinaryMetaData(stream);
	PxVehicleConstraintShader::getBinaryMetaData(stream);

	//PxVehicleTireForceCalculator
	PX_DEF_BIN_METADATA_CLASS(stream,		PxVehicleTireForceCalculator)

	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleTireForceCalculator, 	void*,	mShaderData,	PxMetaDataFlag::ePTR)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleTireForceCalculator, 	PxU32,	mShader,		PxMetaDataFlag::ePTR)		

#if !PX_P64_FAMILY
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleTireForceCalculator,	PxU32,	mPad,			PxMetaDataFlag::ePADDING)
#endif

	//PxVehicleWheelsDynData
	PX_DEF_BIN_METADATA_CLASS(stream,		PxVehicleWheelsDynData)	

	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheelsDynData,			PxVehicleWheels4DynData,	    mWheels4DynData,		PxMetaDataFlag::ePTR)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheelsDynData,			PxVehicleTireForceCalculator,	mTireForceCalculators,	PxMetaDataFlag::ePTR)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheelsDynData,			PxU32,							mUserDatas,				PxMetaDataFlag::ePTR)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheelsDynData,			PxU32,							mNbWheels4,				0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheelsDynData,			PxU32,							mNbActiveWheels,		0)	
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleWheelsDynData,			PxU32,							mPad,					PxMetaDataFlag::ePADDING)

	PX_DEF_BIN_METADATA_EXTRA_ITEMS(stream,	PxVehicleWheelsDynData,			PxVehicleWheels4DynData,		mWheels4DynData,		mNbWheels4, 0, 0)
	PX_DEF_BIN_METADATA_EXTRA_ITEM(stream,	PxVehicleWheelsDynData,			PxVehicleTireForceCalculator,	mTireForceCalculators,	0)

	PX_DEF_BIN_METADATA_EXTRA_ITEMS(stream,	PxVehicleWheelsDynData, 		void, 							mTireForceCalculators,	mNbWheels4, PxMetaDataFlag::ePTR, 0)
	PX_DEF_BIN_METADATA_EXTRA_ITEMS(stream,	PxVehicleWheelsDynData, 		void, 							mTireForceCalculators,	mNbWheels4, PxMetaDataFlag::ePTR, 0)
	PX_DEF_BIN_METADATA_EXTRA_ITEMS(stream,	PxVehicleWheelsDynData, 		void, 							mTireForceCalculators,	mNbWheels4, PxMetaDataFlag::ePTR, 0)
	PX_DEF_BIN_METADATA_EXTRA_ITEMS(stream,	PxVehicleWheelsDynData, 		void, 							mTireForceCalculators,	mNbWheels4, PxMetaDataFlag::ePTR, 0)

	PX_DEF_BIN_METADATA_EXTRA_ITEMS(stream,	PxVehicleWheelsDynData, 		void, 							mUserDatas,				mNbWheels4,	PxMetaDataFlag::ePTR, 0)
	PX_DEF_BIN_METADATA_EXTRA_ITEMS(stream,	PxVehicleWheelsDynData, 		void, 							mUserDatas,				mNbWheels4,	PxMetaDataFlag::ePTR, 0)
	PX_DEF_BIN_METADATA_EXTRA_ITEMS(stream,	PxVehicleWheelsDynData, 		void, 							mUserDatas,				mNbWheels4, PxMetaDataFlag::ePTR, 0)
	PX_DEF_BIN_METADATA_EXTRA_ITEMS(stream,	PxVehicleWheelsDynData, 		void, 							mUserDatas,				mNbWheels4, PxMetaDataFlag::ePTR, 0)

	PX_DEF_BIN_METADATA_EXTRA_ITEMS(stream, PxVehicleWheelsDynData,			PxVehicleConstraintShader,		mWheels4DynData,		mNbWheels4, 0, 0)
}

void PxVehicleWheels::exportExtraData(PxSerializationContext& stream)
{	
	PxU32 size = computeByteSize(mWheelsSimData.mNbActiveWheels);
	stream.alignData();
	stream.writeData(mWheelsSimData.mWheels4SimData, size);
}

void PxVehicleWheels::importExtraData(PxDeserializationContext& context)
{
	PxU32 size = computeByteSize(mWheelsSimData.mNbActiveWheels);
	PxU8* ptr = context.readExtraData<PxU8, PX_SERIAL_ALIGN>(size);
	patchupPointers(mWheelsSimData.mNbActiveWheels, this, ptr);
}

void PxVehicleWheels::getBinaryMetaData(PxOutputStream& stream)
{
	PxVehicleWheelsSimData::getBinaryMetaData(stream);
	PxVehicleWheelsDynData::getBinaryMetaData(stream);

	//PxVehicleWheels
	PX_DEF_BIN_METADATA_VCLASS(stream,		PxVehicleWheels)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	PxVehicleWheels,	PxBase)	

	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheels,	PxVehicleWheelsSimData, mWheelsSimData,					0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheels,	PxVehicleWheelsDynData, mWheelsDynData,					0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheels,	PxRigidDynamic,			mActor,							PxMetaDataFlag::ePTR)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheels,	PxU32,					mNbNonDrivenWheels,				0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheels,	PxU8,					mOnConstraintReleaseCounter,	0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheels,	PxU8,					mType,							0)	
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleWheels,	PxU8,					mPad0,							PxMetaDataFlag::ePADDING)
}

void PxVehicleConstraintShader::getBinaryMetaData(PxOutputStream& stream)
{
	//SuspLimitConstraintData
	PX_DEF_BIN_METADATA_CLASS(stream,		SuspLimitConstraintData)
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	SuspLimitConstraintData,	PxVec3,						mCMOffsets, 			0)	
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	SuspLimitConstraintData,	PxVec3,						mDirs,					0)	
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	SuspLimitConstraintData,	PxReal,						mErrors,				0)
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	SuspLimitConstraintData,	bool,						mActiveFlags,			0)

	//StickyTireConstraintData
	PX_DEF_BIN_METADATA_CLASS(stream,		StickyTireConstraintData)
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	StickyTireConstraintData,	PxVec3, 					mCMOffsets,				0)	
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	StickyTireConstraintData,	PxVec3, 					mDirs,					0)	
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	StickyTireConstraintData,	PxReal, 					mTargetSpeeds,			0)
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	StickyTireConstraintData,	bool,						mActiveFlags,			0)

	//VehicleConstraintData
	PX_DEF_BIN_METADATA_CLASS(stream,		VehicleConstraintData)
	PX_DEF_BIN_METADATA_ITEM(stream,		VehicleConstraintData,		SuspLimitConstraintData,	mSuspLimitData,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,		VehicleConstraintData,		StickyTireConstraintData,	mStickyTireForwardData, 0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		VehicleConstraintData,		StickyTireConstraintData,	mStickyTireSideData,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		VehicleConstraintData,		PxQuat,						mCMassRotation,			0)

	//PxVehicleConstraintShader
	PX_DEF_BIN_METADATA_VCLASS(stream,		PxVehicleConstraintShader)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	PxVehicleConstraintShader,	PxConstraintConnector)
	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleConstraintShader,	VehicleConstraintData,	mData,			0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleConstraintShader,	PxConstraint,			mConstraint,	PxMetaDataFlag::ePTR)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleConstraintShader,	PxVehicleWheels,		mVehicle,		PxMetaDataFlag::ePTR)	
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleConstraintShader,	PxU32,				    mPad,			PxMetaDataFlag::ePADDING)
}

void PxVehicleWheels4SimData::getBinaryMetaData(PxOutputStream& stream)
{
	//PxVehicleSuspensionData
	PX_DEF_BIN_METADATA_CLASS(stream,		PxVehicleSuspensionData)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleSuspensionData,	PxReal, 	mSpringStrength,							0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleSuspensionData,	PxReal, 	mSpringDamperRate,							0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleSuspensionData,	PxReal, 	mMaxCompression,							0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleSuspensionData,	PxReal, 	mMaxDroop,									0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleSuspensionData,	PxReal, 	mSprungMass,								0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleSuspensionData,	PxReal, 	mCamberAtRest,		    					0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleSuspensionData,	PxReal, 	mCamberAtMaxCompression,					0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleSuspensionData,	PxReal, 	mCamberAtMaxDroop,							0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleSuspensionData,	PxReal, 	mRecipMaxCompression,						0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleSuspensionData,	PxReal, 	mRecipMaxDroop,								0)
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleSuspensionData,	PxU32,		mPad,										PxMetaDataFlag::ePADDING)	

	//PxVehicleWheelData
	PX_DEF_BIN_METADATA_CLASS(stream,		PxVehicleWheelData)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheelData,			PxReal,		mRadius,									0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheelData,			PxReal, 	mWidth,										0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheelData,			PxReal, 	mMass,										0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheelData,			PxReal, 	mMOI,										0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheelData,			PxReal, 	mDampingRate,								0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheelData,			PxReal, 	mMaxBrakeTorque,							0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheelData,			PxReal, 	mMaxHandBrakeTorque,						0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheelData,			PxReal, 	mMaxSteer,									0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheelData,			PxReal, 	mToeAngle,									0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheelData,			PxReal, 	mRecipRadius,								0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheelData,			PxReal, 	mRecipMOI,									0)	
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleWheelData,			PxReal, 	mPad,										PxMetaDataFlag::ePADDING)	

	//PxVehicleTireData
	PX_DEF_BIN_METADATA_CLASS(stream,		PxVehicleTireData)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleTireData,			PxReal,		mLatStiffX,						            0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleTireData,			PxReal,		mLatStiffY,									0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleTireData,			PxReal,		mLongitudinalStiffnessPerUnitGravity,		0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleTireData,			PxReal,		mCamberStiffnessPerUnitGravity,				0)	
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleTireData,			PxReal,		mFrictionVsSlipGraph,						0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleTireData,			PxU32,		mType,										0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleTireData,			PxReal,		mRecipLongitudinalStiffnessPerUnitGravity,	0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleTireData,			PxReal,		mFrictionVsSlipGraphRecipx1Minusx0, 		0)	
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleTireData,			PxReal,		mFrictionVsSlipGraphRecipx2Minusx1, 		0)	
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleTireData,			PxReal,		mPad,										PxMetaDataFlag::ePADDING)	

	//PxVehicleWheels4SimData
	PX_DEF_BIN_METADATA_CLASS(stream,		PxVehicleWheels4SimData)
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleWheels4SimData, 	PxVehicleSuspensionData,	mSuspensions,					0)	
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleWheels4SimData, 	PxVehicleWheelData,			mWheels,						0)	
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleWheels4SimData, 	PxVehicleTireData,			mTires,							0)	
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleWheels4SimData, 	PxVec3, 					mSuspDownwardTravelDirections,	0)	
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleWheels4SimData, 	PxVec3, 					mSuspForceAppPointOffsets,		0)	
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleWheels4SimData, 	PxVec3, 					mTireForceAppPointOffsets,		0)	
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleWheels4SimData, 	PxVec3, 					mWheelCentreOffsets,			0)	
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleWheels4SimData, 	PxReal, 					mTireRestLoads,					0)
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleWheels4SimData, 	PxReal, 					mRecipTireRestLoads,			0)
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleWheels4SimData, 	PxFilterData, 				mSqFilterData,					0)
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleWheels4SimData, 	PxU8, 						mWheelShapeMap,					0)
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleWheels4SimData, 	PxU32, 						mPad,							PxMetaDataFlag::ePADDING)
}

/*
void PxVehicleAntiRollBar::getBinaryMetaData(PxOutputStream& stream)
{
}
*/

void PxVehicleWheels4DynData::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,		PxVehicleWheels4DynData)
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleWheels4DynData,	PxReal,						mWheelSpeeds,					0)	
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleWheels4DynData,	PxReal, 					mCorrectedWheelSpeeds,			0)
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleWheels4DynData,	PxReal, 					mWheelRotationAngles,			0)	
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleWheels4DynData,	PxReal, 					mTireLowForwardSpeedTimers, 	0)
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleWheels4DynData,	PxReal, 					mTireLowSideSpeedTimers,		0)
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleWheels4DynData,	PxU8, 						mQueryOrCachedHitResults,		0)	
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleWheels4DynData,	PxReal, 					mJounces,						0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheels4DynData,	PxVehicleConstraintShader,	mVehicleConstraints,			PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheels4DynData,	PxRaycastQueryResult,		mRaycastResults,				PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheels4DynData,	PxSweepQueryResult,			mSweepResults,					PxMetaDataFlag::ePTR)

	PX_DEF_BIN_METADATA_ITEM(stream,		PxVehicleWheels4DynData,	bool,						mHasCachedRaycastHitPlane,		0)
#if PX_P64_FAMILY
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PxVehicleWheels4DynData,	PxU32,						mPad,							PxMetaDataFlag::ePADDING)
#endif
}
