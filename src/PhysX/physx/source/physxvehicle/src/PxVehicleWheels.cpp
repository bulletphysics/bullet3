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

#include "PxVehicleWheels.h"
#include "PxVehicleSuspWheelTire4.h"
#include "PxVehicleSuspLimitConstraintShader.h"
#include "PxVehicleDefaults.h"
#include "PxRigidDynamic.h"
#include "PxShape.h"
#include "PsUtilities.h"
#include "CmPhysXCommon.h"
#include "CmBitMap.h"
#include "PxPhysics.h"
#include "PsIntrinsics.h"
#include "PsFoundation.h"

namespace physx
{

extern PxVec3 gRight;
extern PxVec3 gUp;
extern PxVec3 gForward;

PxF32 gThresholdLongSpeed=5.0f;
PxU32 gLowLongSpeedSubstepCount=3;
PxU32 gHighLongSpeedSubstepCount=1;
PxF32 gMinLongSlipDenominator=4.0f;

extern PxF32 gToleranceScaleLength;


PxU32 PxVehicleWheelsSimData::computeByteSize(const PxU32 numWheels)
{
	const PxU32 numWheels4 =(((numWheels + 3) & ~3) >> 2);
	const PxU32 byteSize = sizeof(PxVehicleWheels4SimData)*numWheels4 + sizeof(PxVehicleAntiRollBarData)*2*numWheels4;
	return byteSize;
}

PxU8* PxVehicleWheelsSimData::patchUpPointers(const PxU32 numWheels, PxVehicleWheelsSimData* simData, PxU8* ptrIn)
{
	const PxU32 numWheels4 =(((numWheels + 3) & ~3) >> 2);
	PxU8* ptr = ptrIn;
	simData->mWheels4SimData = reinterpret_cast<PxVehicleWheels4SimData*>(ptr);
	ptr += sizeof(PxVehicleWheels4SimData)*numWheels4;
	simData->mAntiRollBars = reinterpret_cast<PxVehicleAntiRollBarData*>(ptr);
	ptr += sizeof(PxVehicleAntiRollBarData)*numWheels4*2;
	PX_ASSERT((ptrIn + computeByteSize(numWheels)) == ptr);
	return ptr;
}

PxVehicleWheelsSimData::PxVehicleWheelsSimData(const PxU32 numWheels)
{
	const PxU32 numWheels4 =(((numWheels + 3) & ~3) >> 2);

	//Set numWheels
	mNbWheels4 = numWheels4;
	mNbActiveWheels = numWheels;

	//Set numAntiRollBars to zero.
	mNbAntiRollBars4 = 2*numWheels4;
	mNbActiveAntiRollBars = 0;

	//Placement new for wheels4
	for(PxU32 i=0;i<numWheels4;i++)
	{
		new(&mWheels4SimData[i]) PxVehicleWheels4SimData();
	}

	//Placement new for anti-roll bars
	for(PxU32 i=0;i<numWheels4*2;i++)
	{
		new(&mAntiRollBars[i]) PxVehicleAntiRollBarData();
	}

	//Placement new for tire load filter data.
	new(&mNormalisedLoadFilter) PxVehicleTireLoadFilterData();

	//Enable used wheels, disabled unused wheels.
	PxMemZero(mActiveWheelsBitmapBuffer, sizeof(PxU32) * (((PX_MAX_NB_WHEELS + 31) & ~31) >> 5));
	for(PxU32 i=0;i<numWheels;i++)
	{
		//Enabled used wheel.
		enableWheel(i);
		setWheelShapeMapping(i,PxI32(i));
		setSceneQueryFilterData(i, PxFilterData());
	}
	for(PxU32 i=numWheels;i<numWheels4*4;i++)
	{
		//Disable unused wheel.
		disableWheel(i);
		setWheelShapeMapping(i,-1);
		setSceneQueryFilterData(i, PxFilterData());
	}

	//Default values for substep count computation.
	mThresholdLongitudinalSpeed = gThresholdLongSpeed*gToleranceScaleLength;
	mLowForwardSpeedSubStepCount = gLowLongSpeedSubstepCount;
	mHighForwardSpeedSubStepCount = gHighLongSpeedSubstepCount;
	mMinLongSlipDenominator = gMinLongSlipDenominator*gToleranceScaleLength;
}

PxVehicleWheelsSimData* PxVehicleWheelsSimData::allocate(const PxU32 numWheels)
{
	//Byte size
	const PxU32 byteSize = sizeof(PxVehicleWheelsSimData) + computeByteSize(numWheels);

	//Allocate
	PxU8* ptrStart = static_cast<PxU8*>(PX_ALLOC(byteSize, "PxVehicleWheelsSimData"));

	//Patchup pointers
	PxU8* ptr = ptrStart;
	PxVehicleWheelsSimData* simData = reinterpret_cast<PxVehicleWheelsSimData*>(ptr);
	ptr += sizeof(PxVehicleWheelsSimData);
	ptr = patchUpPointers(numWheels, simData, ptr);
	PX_ASSERT((ptrStart+ byteSize) == ptr);

	//Constructor.
	new(simData) PxVehicleWheelsSimData(numWheels);

	//Finished.
	return simData;
}

void PxVehicleWheelsSimData::setChassisMass(const PxF32 chassisMass)
{

	//Target spring natural frequency = 9.66
	//Target spring damping ratio = 0.62
	const PxF32 mult=1.0f/(1.0f*mNbActiveWheels);
	const PxF32 sprungMass=chassisMass*mult;
	const PxF32 w0=9.66f;
	const PxF32 r=0.62f;
	for(PxU32 i=0;i<mNbActiveWheels;i++)
	{
		PxVehicleSuspensionData susp=getSuspensionData(i);
		susp.mSprungMass=sprungMass;
		susp.mSpringStrength=w0*w0*sprungMass;
		susp.mSpringDamperRate=r*2*sprungMass*w0;
		setSuspensionData(i,susp);
	}
}

void PxVehicleWheelsSimData::free()
{
	for(PxU32 i=0;i<mNbWheels4;i++)
	{
		mWheels4SimData[i].~PxVehicleWheels4SimData();
	}

	PX_FREE(this);
}

PxVehicleWheelsSimData& PxVehicleWheelsSimData::operator=(const PxVehicleWheelsSimData& src)
{
	PX_CHECK_MSG(mNbActiveWheels == src.mNbActiveWheels, "target PxVehicleSuspWheelTireNSimData must match the number of wheels in src");

	for(PxU32 i=0;i<src.mNbWheels4;i++)
	{
		mWheels4SimData[i] = src.mWheels4SimData[i];
	}

	mNbActiveAntiRollBars = src.mNbActiveAntiRollBars;
	for(PxU32 i=0; i<src.mNbActiveAntiRollBars; i++)
	{
		mAntiRollBars[i] = src.mAntiRollBars[i];
	}

	mNormalisedLoadFilter = src.mNormalisedLoadFilter;

	mThresholdLongitudinalSpeed = src.mThresholdLongitudinalSpeed;
	mLowForwardSpeedSubStepCount = src.mLowForwardSpeedSubStepCount;
	mHighForwardSpeedSubStepCount = src.mHighForwardSpeedSubStepCount;
	mMinLongSlipDenominator = src.mMinLongSlipDenominator;

	PxMemCopy(mActiveWheelsBitmapBuffer, src.mActiveWheelsBitmapBuffer, sizeof(PxU32)* (((PX_MAX_NB_WHEELS + 31) & ~31) >> 5));

	return *this;
}

void PxVehicleWheelsSimData::copy(const PxVehicleWheelsSimData& src, const PxU32 srcWheel, const PxU32 wheel)
{
	PX_CHECK_AND_RETURN(srcWheel < src.mNbActiveWheels, "Illegal src wheel");
	PX_CHECK_AND_RETURN(wheel < mNbActiveWheels, "Illegal target wheel");

	setSuspensionData(wheel,src.getSuspensionData(srcWheel));
	setWheelData(wheel,src.getWheelData(srcWheel));
	setTireData(wheel,src.getTireData(srcWheel));
	setSuspTravelDirection(wheel,src.getSuspTravelDirection(srcWheel));
	setSuspForceAppPointOffset(wheel,src.getSuspForceAppPointOffset(srcWheel));
	setTireForceAppPointOffset(wheel,src.getTireForceAppPointOffset(srcWheel));
	setWheelCentreOffset(wheel,src.getWheelCentreOffset(srcWheel));
	setWheelShapeMapping(wheel, src.getWheelShapeMapping(srcWheel));
	setSceneQueryFilterData(wheel, src.getSceneQueryFilterData(srcWheel));
	if(src.getIsWheelDisabled(srcWheel))
		disableWheel(wheel);
	else
		enableWheel(wheel);
}

bool PxVehicleWheelsSimData::isValid() const
{
	for(PxU32 i=0;i<mNbWheels4-1;i++)
	{
		mWheels4SimData[i].isValid(0);
		mWheels4SimData[i].isValid(1);
		mWheels4SimData[i].isValid(2);
		mWheels4SimData[i].isValid(3);
	}
	const PxU32 numInLastBlock = 4 - (4*mNbWheels4 - mNbActiveWheels);
	for(PxU32 i=0;i<numInLastBlock;i++)
	{
		mWheels4SimData[mNbWheels4-1].isValid(i);
	}

	PX_CHECK_AND_RETURN_VAL(mNormalisedLoadFilter.isValid(), "Invalid PxVehicleWheelsSimData.mNormalisedLoadFilter", false);
	return true;
}

void PxVehicleWheelsSimData::disableWheel(const PxU32 wheel)
{
	PX_CHECK_AND_RETURN(wheel < 4*mNbWheels4, "PxVehicleWheelsSimData::disableWheel - Illegal wheel");

	Cm::BitMap bm;
	bm.setWords(mActiveWheelsBitmapBuffer, ((PX_MAX_NB_WHEELS + 31) & ~31) >> 5);
	bm.reset(wheel);
}

void PxVehicleWheelsSimData::enableWheel(const PxU32 wheel)
{
	PX_CHECK_AND_RETURN(wheel < 4*mNbWheels4, "PxVehicleWheelsSimData::disableWheel - Illegal wheel");

	Cm::BitMap bm;
	bm.setWords(mActiveWheelsBitmapBuffer, ((PX_MAX_NB_WHEELS + 31) & ~31) >> 5);
	bm.set(wheel);
}

bool PxVehicleWheelsSimData::getIsWheelDisabled(const PxU32 wheel) const
{
	PX_CHECK_AND_RETURN_VAL(wheel < 4*mNbWheels4, "PxVehicleWheelsSimData::getIsWheelDisabled - Illegal wheel", false);
	Cm::BitMap bm;
	bm.setWords(const_cast<PxU32*>(mActiveWheelsBitmapBuffer), ((PX_MAX_NB_WHEELS + 31) & ~31) >> 5);
	return (bm.test(wheel) ? false : true);
}

const PxVehicleSuspensionData& PxVehicleWheelsSimData::getSuspensionData(const PxU32 id) const	
{
	PX_CHECK_MSG(id < 4*mNbWheels4, "PxVehicleWheelsSimData::getSuspensionData - Illegal wheel");
	return mWheels4SimData[id>>2].getSuspensionData(id & 3);
}

const PxVehicleWheelData& PxVehicleWheelsSimData::getWheelData(const PxU32 id) const	
{
	PX_CHECK_MSG(id < 4*mNbWheels4, "PxVehicleWheelsSimData::getWheelData - Illegal wheel");
	return mWheels4SimData[id>>2].getWheelData(id & 3);
}

const PxVehicleTireData& PxVehicleWheelsSimData::getTireData(const PxU32 id) const	
{
	PX_CHECK_MSG(id < 4*mNbWheels4, "PxVehicleWheelsSimData::getTireData - Illegal wheel");
	return mWheels4SimData[id>>2].getTireData(id & 3);
}

const PxVec3& PxVehicleWheelsSimData::getSuspTravelDirection(const PxU32 id) const	
{
	PX_CHECK_MSG(id < 4*mNbWheels4, "PxVehicleWheelsSimData::getSuspTravelDirection - Illegal wheel");
	return mWheels4SimData[id>>2].getSuspTravelDirection(id & 3);
}

const PxVec3& PxVehicleWheelsSimData::getSuspForceAppPointOffset(const PxU32 id) const	
{
	PX_CHECK_MSG(id < 4*mNbWheels4, "PxVehicleWheelsSimData::getSuspForceAppPointOffset - Illegal wheel");
	return mWheels4SimData[id>>2].getSuspForceAppPointOffset(id & 3);
}

const PxVec3& PxVehicleWheelsSimData::getTireForceAppPointOffset(const PxU32 id) const	
{
	PX_CHECK_MSG(id < 4*mNbWheels4, "PxVehicleWheelsSimData::getTireForceAppPointOffset - Illegal wheel");
	return mWheels4SimData[id>>2].getTireForceAppPointOffset(id & 3);
}

const PxVec3& PxVehicleWheelsSimData::getWheelCentreOffset(const PxU32 id) const	
{
	PX_CHECK_MSG(id < 4*mNbWheels4, "PxVehicleWheelsSimData::getWheelCentreOffset - Illegal wheel");
	return mWheels4SimData[id>>2].getWheelCentreOffset(id & 3);
}

PxI32 PxVehicleWheelsSimData::getWheelShapeMapping(const PxU32 id) const
{
	PX_CHECK_MSG(id < 4*mNbWheels4, "PxVehicleWheelsSimData::getWheelShapeMapping - Illegal wheel");
	return mWheels4SimData[id>>2].getWheelShapeMapping(id & 3);
}

const PxFilterData& PxVehicleWheelsSimData::getSceneQueryFilterData(const PxU32 id) const
{
	PX_CHECK_MSG(id < 4*mNbWheels4, "PxVehicleWheelsSimData::getSceneQueryFilterData - Illegal wheel");
	return mWheels4SimData[id>>2].getSceneQueryFilterData(id & 3);
}

const PxVehicleAntiRollBarData& PxVehicleWheelsSimData::getAntiRollBarData(const PxU32 antiRollId) const
{
	PX_CHECK_MSG(antiRollId < mNbActiveAntiRollBars, "PxVehicleWheelsSimData::getAntiRollBarData - Illegal anti-roll bar");
	return mAntiRollBars[antiRollId];
}

void PxVehicleWheelsSimData::setSuspensionData(const PxU32 id, const PxVehicleSuspensionData& susp)
{
	PX_CHECK_AND_RETURN(id < 4*mNbWheels4, "PxVehicleWheelsSimData::setSuspensionData - Illegal wheel");
	mWheels4SimData[id>>2].setSuspensionData(id & 3, susp);
}

void PxVehicleWheelsSimData::setWheelData(const PxU32 id, const PxVehicleWheelData& wheel)
{
	PX_CHECK_AND_RETURN(id < 4*mNbWheels4, "PxVehicleWheelsSimData::setWheelData - Illegal wheel");
	mWheels4SimData[id>>2].setWheelData(id & 3, wheel);
}

void PxVehicleWheelsSimData::setTireData(const PxU32 id, const PxVehicleTireData& tire)
{
	PX_CHECK_AND_RETURN(id < 4*mNbWheels4, "PxVehicleWheelsSimData::setTireData - Illegal wheel");
	mWheels4SimData[id>>2].setTireData(id & 3, tire);
}

void PxVehicleWheelsSimData::setSuspTravelDirection(const PxU32 id, const PxVec3& dir)										
{
	PX_CHECK_AND_RETURN(id < 4*mNbWheels4, "PxVehicleWheelsSimData::setSuspTravelDirection - Illegal wheel");
	mWheels4SimData[id>>2].setSuspTravelDirection(id & 3, dir);
}

void PxVehicleWheelsSimData::setSuspForceAppPointOffset(const PxU32 id, const PxVec3& offset)									
{
	PX_CHECK_AND_RETURN(id < 4*mNbWheels4, "PxVehicleWheelsSimData::setSuspForceAppPointOffset - Illegal wheel");
	mWheels4SimData[id>>2].setSuspForceAppPointOffset(id & 3, offset);
}

void PxVehicleWheelsSimData::setTireForceAppPointOffset(const PxU32 id, const PxVec3& offset)									
{
	PX_CHECK_AND_RETURN(id < 4*mNbWheels4, "PxVehicleWheelsSimData::setTireForceAppPointOffset - Illegal wheel");
	mWheels4SimData[id>>2].setTireForceAppPointOffset(id & 3, offset);
}

void PxVehicleWheelsSimData::setWheelCentreOffset(const PxU32 id, const PxVec3& offset)									
{
	PX_CHECK_AND_RETURN(id < 4*mNbWheels4, "PxVehicleWheelsSimData::setWheelCentreOffset - Illegal wheel");
	mWheels4SimData[id>>2].setWheelCentreOffset(id & 3, offset);
}

void PxVehicleWheelsSimData::setWheelShapeMapping(const PxU32 id, const PxI32 shapeId)
{
	PX_CHECK_AND_RETURN(id < 4*mNbWheels4, "PxVehicleWheelsSimData::setWheelShapeMapping - Illegal wheel");
	mWheels4SimData[id>>2].setWheelShapeMapping(id & 3, shapeId);
}

void PxVehicleWheelsSimData::setSceneQueryFilterData(const PxU32 id, const PxFilterData& sqFilterData)
{
	PX_CHECK_AND_RETURN(id < 4*mNbWheels4, "PxVehicleWheelsSimData::setSceneQueryFilterData - Illegal wheel");
	mWheels4SimData[id>>2].setSceneQueryFilterData(id & 3, sqFilterData);
}

void PxVehicleWheelsSimData::setTireLoadFilterData(const PxVehicleTireLoadFilterData& tireLoadFilter)
{
	PX_CHECK_AND_RETURN(tireLoadFilter.mMaxNormalisedLoad>tireLoadFilter.mMinNormalisedLoad, "Illegal graph points");
	PX_CHECK_AND_RETURN(tireLoadFilter.mMaxFilteredNormalisedLoad>0, "Max filtered load must be greater than zero");
	mNormalisedLoadFilter=tireLoadFilter;
	mNormalisedLoadFilter.mDenominator=1.0f/(mNormalisedLoadFilter.mMaxNormalisedLoad-mNormalisedLoadFilter.mMinNormalisedLoad);
}

PxU32 PxVehicleWheelsSimData::addAntiRollBarData(const PxVehicleAntiRollBarData& antiRollBar)
{
	PX_CHECK_AND_RETURN_VAL(antiRollBar.isValid(), "Illegal antiRollBar", 0xffffffff)
	PX_CHECK_AND_RETURN_VAL(antiRollBar.mWheel0 < mNbActiveWheels, "Illegal wheel0", 0xffffffff);
	PX_CHECK_AND_RETURN_VAL(antiRollBar.mWheel1 < mNbActiveWheels, "Illegal wheel1", 0xffffffff);

	//If the anti-roll pair already exists then modify it.
	for(PxU32 i = 0; i < mNbActiveAntiRollBars; i++)
	{
		if( ((mAntiRollBars[i].mWheel0 == antiRollBar.mWheel0) && (mAntiRollBars[i].mWheel1 == antiRollBar.mWheel1)) || 
			((mAntiRollBars[i].mWheel1 == antiRollBar.mWheel0) && (mAntiRollBars[i].mWheel0 == antiRollBar.mWheel1)) )
		{
			mAntiRollBars[i].mStiffness = antiRollBar.mStiffness;
			return i;
		}
	}

	//Check we have space for an extra anti-roll bar.
	PX_CHECK_AND_RETURN_VAL(mNbActiveAntiRollBars <  2*mNbWheels4, "The buffer of anti-roll bars is full", 0xffffffff);

	//Add a new anti-roll bar.
	const PxU32 id = mNbActiveAntiRollBars;
	mAntiRollBars[mNbActiveAntiRollBars] = antiRollBar;
	mNbActiveAntiRollBars++;

	//Finished.
	return id;
}

//Only used for serialization.
void PxVehicleWheelsSimData::setAntiRollBarData(const PxU32 id, const PxVehicleAntiRollBarData& antiRoll) 
{
	PX_UNUSED(id);
	addAntiRollBarData(antiRoll);
}

void PxVehicleWheelsSimData::setSubStepCount(const PxReal thresholdLongitudinalSpeed, const PxU32 lowForwardSpeedSubStepCount, const PxU32 highForwardSpeedSubStepCount)
{
	PX_CHECK_AND_RETURN(thresholdLongitudinalSpeed>=0, "thresholdLongitudinalSpeed must be greater than or equal to zero.");
	PX_CHECK_AND_RETURN(lowForwardSpeedSubStepCount>0, "lowForwardSpeedSubStepCount must be greater than zero.");
	PX_CHECK_AND_RETURN(highForwardSpeedSubStepCount>0, "highForwardSpeedSubStepCount must be greater than zero.");

	mThresholdLongitudinalSpeed=thresholdLongitudinalSpeed;
	mLowForwardSpeedSubStepCount=lowForwardSpeedSubStepCount;
	mHighForwardSpeedSubStepCount=highForwardSpeedSubStepCount;
}

void PxVehicleWheelsSimData::setMinLongSlipDenominator(const PxReal minLongSlipDenominator)
{
	PX_CHECK_AND_RETURN(minLongSlipDenominator>0, "minLongSlipDenominator must be greater than or equal to zero.");
	mMinLongSlipDenominator=minLongSlipDenominator;
}



/////////////////////////////

PxU32 PxVehicleWheelsDynData::computeByteSize(const PxU32 numWheels)
{
	const PxU32 numWheels4 =(((numWheels + 3) & ~3) >> 2);
	const PxU32 byteSize = 
		sizeof(PxVehicleWheels4DynData)*numWheels4 +
		sizeof(PxVehicleTireForceCalculator) + sizeof(void*)*4*numWheels4 +
		sizeof(void*)*4*numWheels4 +
		sizeof(PxVehicleConstraintShader)*numWheels4;
	return byteSize;
}

PxU8* PxVehicleWheelsDynData::patchUpPointers(const PxU32 numWheels, PxVehicleWheelsDynData* dynData, PxU8* ptrIn)
{
	const PxU32 numWheels4 =(((numWheels + 3) & ~3) >> 2);
	PxU8* ptr = ptrIn;
	dynData->mWheels4DynData = reinterpret_cast<PxVehicleWheels4DynData*>(ptr);
	ptr += sizeof(PxVehicleWheels4DynData)*numWheels4;
	dynData->mTireForceCalculators = reinterpret_cast<PxVehicleTireForceCalculator*>(ptr);
	ptr += sizeof(PxVehicleTireForceCalculator);
	dynData->mTireForceCalculators->mShaderData = reinterpret_cast<const void**>(ptr);
	ptr += sizeof(void*)*4*numWheels4;
	dynData->mUserDatas = reinterpret_cast<void**>(ptr);
	ptr += sizeof(void*)*4*numWheels4;
	for(PxU32 i=0;i<numWheels4;i++)
	{
		PxVehicleConstraintShader* shader = new(ptr) PxVehicleConstraintShader();
		dynData->mWheels4DynData[i].setVehicleConstraintShader(shader);
		ptr += sizeof(PxVehicleConstraintShader);
	}

	PX_ASSERT((ptrIn + computeByteSize(numWheels)) == ptr);
	return ptr;
}

PxVehicleWheelsDynData::PxVehicleWheelsDynData(const PxU32 numWheels)
{
	const PxU32 numWheels4 =(((numWheels + 3) & ~3) >> 2);

	mNbWheels4=numWheels4;
	mNbActiveWheels=numWheels;

	//Placement new for wheels4
	for(PxU32 i=0;i<numWheels4;i++)
	{
		new(&mWheels4DynData[i]) PxVehicleWheels4DynData();
	}

	//Initialise tire calculator
	for(PxU32 i=0;i<4*numWheels4;i++)
	{
		mTireForceCalculators->mShaderData[i]=NULL;
	}
	new(mTireForceCalculators) PxVehicleTireForceCalculator;

	//Initialise user data
	for(PxU32 i=0;i<4*numWheels4;i++)
	{
		mUserDatas[i]=NULL;
	}
}

bool PxVehicleWheelsDynData::isValid() const
{
	for(PxU32 i=0;i<mNbWheels4;i++)
	{
		PX_CHECK_AND_RETURN_VAL(mWheels4DynData[i].isValid(), "Invalid PxVehicleSuspWheelTireNDynData.mSuspWheelTire4DynData[i]", false);
	}
	return true;
}

void PxVehicleWheelsDynData::setToRestState()
{
	//Set susp/wheel/tires to rest state.
	const PxU32 numSuspWheelTire4=mNbWheels4;
	for(PxU32 i=0;i<numSuspWheelTire4;i++)
	{
		mWheels4DynData[i].setToRestState();
	}
}

void PxVehicleWheelsDynData::setTireForceShaderFunction(PxVehicleComputeTireForce tireForceShaderFn)
{
	mTireForceCalculators->mShader=tireForceShaderFn;
}

void PxVehicleWheelsDynData::setTireForceShaderData(const PxU32 tireId, const void* tireForceShaderData)
{
	PX_CHECK_AND_RETURN(tireId < mNbActiveWheels, "PxVehicleWheelsDynData::setTireForceShaderData - Illegal tire");
	mTireForceCalculators->mShaderData[tireId]=tireForceShaderData;
}

const void* PxVehicleWheelsDynData::getTireForceShaderData(const PxU32 tireId) const
{
	PX_CHECK_AND_RETURN_VAL(tireId < mNbActiveWheels, "PxVehicleWheelsDynData::getTireForceShaderData - Illegal tire", NULL);
	return mTireForceCalculators->mShaderData[tireId];
}

void PxVehicleWheelsDynData::setWheelRotationSpeed(const PxU32 wheelIdx, const PxReal speed)
{
	PX_CHECK_AND_RETURN(wheelIdx < mNbActiveWheels, "PxVehicleWheelsDynData::setWheelRotationSpeed - Illegal wheel");
	PxVehicleWheels4DynData& suspWheelTire4=mWheels4DynData[(wheelIdx>>2)];
	suspWheelTire4.mWheelSpeeds[wheelIdx & 3] = speed;
	suspWheelTire4.mCorrectedWheelSpeeds[wheelIdx & 3] = speed;
}

PxReal PxVehicleWheelsDynData::getWheelRotationSpeed(const PxU32 wheelIdx) const
{
	PX_CHECK_AND_RETURN_VAL(wheelIdx < mNbActiveWheels, "PxVehicleWheelsDynData::getWheelRotationSpeed - Illegal wheel", 0.0f);
	const PxVehicleWheels4DynData& suspWheelTire4=mWheels4DynData[(wheelIdx>>2)];
	return suspWheelTire4.mCorrectedWheelSpeeds[wheelIdx & 3];
}

void PxVehicleWheelsDynData::setWheelRotationAngle(const PxU32 wheelIdx, const PxReal angle)
{
	PX_CHECK_AND_RETURN(wheelIdx < mNbActiveWheels, "PxVehicleWheelsDynData::setWheelRotationAngle - Illegal wheel");
	PxVehicleWheels4DynData& suspWheelTire4=mWheels4DynData[(wheelIdx>>2)];
	suspWheelTire4.mWheelRotationAngles[wheelIdx & 3] = angle;
}

PxReal PxVehicleWheelsDynData::getWheelRotationAngle(const PxU32 wheelIdx) const
{
	PX_CHECK_AND_RETURN_VAL(wheelIdx < mNbActiveWheels, "PxVehicleWheelsDynData::getWheelRotationAngle - Illegal wheel", 0.0f);
	const PxVehicleWheels4DynData& suspWheelTire4=mWheels4DynData[(wheelIdx>>2)];
	return suspWheelTire4.mWheelRotationAngles[wheelIdx & 3];
}

void PxVehicleWheels::setToRestState()
{
	//Set the rigid body to rest and clear all the accumulated forces and impulses.
	if(!(mActor->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC))
	{
		mActor->setLinearVelocity(PxVec3(0,0,0));
		mActor->setAngularVelocity(PxVec3(0,0,0));
		mActor->clearForce(PxForceMode::eACCELERATION);
		mActor->clearForce(PxForceMode::eVELOCITY_CHANGE);
		mActor->clearTorque(PxForceMode::eACCELERATION);
		mActor->clearTorque(PxForceMode::eVELOCITY_CHANGE);
	}

	//Set the wheels to rest state.
	mWheelsDynData.setToRestState();
}

bool PxVehicleWheels::isValid() const
{
	PX_CHECK_AND_RETURN_VAL(mWheelsSimData.isValid(), "invalid mWheelsSimData", false);
	PX_CHECK_AND_RETURN_VAL(mWheelsDynData.isValid(), "invalid mWheelsDynData", false);
	return true;
}

PxU32 PxVehicleWheels::computeByteSize(const PxU32 numWheels)
{
	const PxU32 byteSize = 
		PxVehicleWheelsSimData::computeByteSize(numWheels) + 
		PxVehicleWheelsDynData::computeByteSize(numWheels);
	return byteSize;
}

PxU8* PxVehicleWheels::patchupPointers(const PxU32 numWheels, PxVehicleWheels* vehWheels, PxU8* ptrIn)
{
	PxU8* ptr = ptrIn;
	ptr = PxVehicleWheelsSimData::patchUpPointers(numWheels, &vehWheels->mWheelsSimData, ptr);
	ptr = PxVehicleWheelsDynData::patchUpPointers(numWheels, &vehWheels->mWheelsDynData, ptr);
	PX_ASSERT((ptrIn + computeByteSize(numWheels)) == ptr);
	return ptr;
}

void PxVehicleWheels::init(const PxU32 numWheels)
{
	new(&mWheelsSimData) PxVehicleWheelsSimData(numWheels);
	new(&mWheelsDynData) PxVehicleWheelsDynData(numWheels);

	for(PxU32 i = 0; i < mWheelsSimData.mNbWheels4; i++)
	{
		new(&mWheelsDynData.mWheels4DynData[i].getVehicletConstraintShader()) PxVehicleConstraintShader(this);
	}

	mOnConstraintReleaseCounter = Ps::to8(mWheelsSimData.mNbWheels4);
}

void PxVehicleWheels::free()
{
	PX_CHECK_AND_RETURN(mWheelsSimData.mNbWheels4>0, "Cars with zero wheels are illegal");

	const PxU32 numSuspWheelTire4 = mWheelsSimData.mNbWheels4;

	for(PxU32 i=0;i<numSuspWheelTire4;i++)
	{
		mWheelsDynData.mWheels4DynData[i].getVehicletConstraintShader().release();
	}
}

static PxConstraintShaderTable t = 
		{ 
			PxVehicleConstraintShader::vehicleSuspLimitConstraintSolverPrep,
			0,
			PxVehicleConstraintShader::visualiseConstraint,
			PxConstraintFlag::Enum(0)
		};

void PxVehicleWheels::setup
(PxPhysics* physics, PxRigidDynamic* vehActor, 
 const PxVehicleWheelsSimData& wheelsData,
 const PxU32 numDrivenWheels, const PxU32 numNonDrivenWheels)
{
	mNbNonDrivenWheels = numNonDrivenWheels;

	PX_CHECK_AND_RETURN(wheelsData.getNbWheels() == mWheelsSimData.getNbWheels(), "PxVehicleWheels::setup - vehicle must be setup with same number of wheels as wheelsData");
	PX_CHECK_AND_RETURN(vehActor, "PxVehicleWheels::setup - vehActor is null ptr : you need to instantiate an empty PxRigidDynamic for the vehicle");
	PX_CHECK_AND_RETURN(wheelsData.isValid(), "PxVehicleWheels::setup -invalid wheelsData");
	PX_UNUSED(numDrivenWheels);
	
#if PX_CHECKED
	PxF32 totalSprungMass=0.0f;
	for(PxU32 i=0;i<(numDrivenWheels+numNonDrivenWheels);i++)
	{
		totalSprungMass+=wheelsData.getSuspensionData(i).mSprungMass;
	}
	PX_CHECK_MSG(PxAbs((vehActor->getMass()-totalSprungMass)/vehActor->getMass()) < 0.01f, "Sum of suspension sprung masses doesn't match actor mass");
#endif

	//Copy the simulation data.
	mWheelsSimData=wheelsData;

	//Set the actor pointer.
	mActor=vehActor;

	//Set all the sq result ptrs to null.
	const PxU32 numSuspWheelTire4=wheelsData.mNbWheels4;
	for(PxU32 i=0;i<numSuspWheelTire4;i++)
	{
		mWheelsDynData.mWheels4DynData[i].mRaycastResults=NULL;
		mWheelsDynData.mWheels4DynData[i].mSweepResults=NULL;
	}

	//Set up the suspension limits constraints.
	for(PxU32 i=0;i<numSuspWheelTire4;i++)
	{
		PxVehicleConstraintShader& shader=mWheelsDynData.mWheels4DynData[i].getVehicletConstraintShader();
		for(PxU32 j=0;j<4;j++)
		{
			shader.mData.mSuspLimitData.mCMOffsets[j]=wheelsData.mWheels4SimData[i].getSuspForceAppPointOffset(j);
			shader.mData.mSuspLimitData.mDirs[j]=wheelsData.mWheels4SimData[i].getSuspTravelDirection(j);
			shader.mData.mSuspLimitData.mErrors[j]=0.0f;
			shader.mData.mSuspLimitData.mActiveFlags[j]=false;

			shader.mData.mStickyTireForwardData.mCMOffsets[j]=PxVec3(0,0,0);
			shader.mData.mStickyTireForwardData.mDirs[j]=PxVec3(0,0,0);
			shader.mData.mStickyTireForwardData.mTargetSpeeds[j]=0.0f;
			shader.mData.mStickyTireForwardData.mActiveFlags[j]=false;

			shader.mData.mStickyTireSideData.mCMOffsets[j]=PxVec3(0,0,0);
			shader.mData.mStickyTireSideData.mDirs[j]=PxVec3(0,0,0);
			shader.mData.mStickyTireSideData.mTargetSpeeds[j]=0.0f;
			shader.mData.mStickyTireSideData.mActiveFlags[j]=false;
		}


		shader.mConstraint=physics->createConstraint(vehActor, NULL, shader, t, sizeof(PxVehicleConstraintShader::VehicleConstraintData));
		shader.mConstraint->markDirty();
	}

	//Set up the shader data ptrs.
	for(PxU32 i=0;i<wheelsData.mNbActiveWheels;i++)
	{
		mWheelsDynData.setTireForceShaderData(i,&mWheelsSimData.getTireData(i));
	}

	//Disable the unused wheels.
	for(PxU32 i=wheelsData.mNbActiveWheels;i<4*mWheelsSimData.mNbWheels4;i++)
	{
		mWheelsSimData.disableWheel(i);
	}

	//Pose the wheels that are mapped to shapes so that all shapes are at the rest pose.
	for(PxU32 i=0;i<wheelsData.mNbActiveWheels;i++)
	{
		const PxI32 shapeId = mWheelsSimData.getWheelShapeMapping(i);
		if(-1!=shapeId)
		{
			PX_CHECK_AND_RETURN(PxU32(shapeId) < mActor->getNbShapes(), "Illegal wheel shape mapping, shape does not exist on actor");

			//Compute the shape local pose
			const PxTransform chassisCMOffset=mActor->getCMassLocalPose();
			PxTransform wheelOffset=chassisCMOffset;
			wheelOffset.p+=mWheelsSimData.getWheelCentreOffset(i);
			//Pose the shape.
			PxShape* shapeBuffer[1];
			mActor->getShapes(shapeBuffer,1,PxU32(shapeId));
			shapeBuffer[0]->setLocalPose(wheelOffset);
		}
	}
}

void	PxVehicleWheels::requiresObjects(PxProcessPxBaseCallback& c)
{
	c.process(*mActor);

	for(PxU32 i=0;i<mWheelsSimData.mNbWheels4;i++)
	{
		c.process(*mWheelsDynData.mWheels4DynData[i].getVehicletConstraintShader().getPxConstraint());
	}
}

static PxConstraint* resolveConstraintPtr(PxDeserializationContext& context,
										  PxConstraint* old,
										  PxConstraintConnector* connector,
										  PxConstraintShaderTable &shaders)
{
	context.translatePxBase(old);
	PxConstraint* new_nx = static_cast<PxConstraint*>(old);
	new_nx->setConstraintFunctions(*connector, shaders);
	return new_nx;
}

void PxVehicleWheels::resolveReferences(PxDeserializationContext& context)
{	
	context.translatePxBase(mActor);

	for(PxU32 i=0;i<mWheelsSimData.mNbWheels4;i++)
	{
		PxVehicleConstraintShader& shader=mWheelsDynData.mWheels4DynData[i].getVehicletConstraintShader();
		shader.setPxConstraint(resolveConstraintPtr(context,shader.getPxConstraint(), shader.getConnector(), t));
	}

	//Set up the shader data ptrs.
	for(PxU32 i=0;i<mWheelsSimData.mNbActiveWheels;i++)
	{
		mWheelsDynData.setTireForceShaderData(i,&mWheelsSimData.getTireData(i));
	}	
}

PxReal PxVehicleWheels::computeForwardSpeed() const
{
	const PxTransform vehicleChassisTrnsfm=mActor->getGlobalPose().transform(mActor->getCMassLocalPose());
	return mActor->getLinearVelocity().dot(vehicleChassisTrnsfm.q.rotate(gForward));
}

PxReal PxVehicleWheels::computeSidewaysSpeed() const
{
	const PxTransform vehicleChassisTrnsfm=mActor->getGlobalPose().transform(mActor->getCMassLocalPose());
	return mActor->getLinearVelocity().dot(vehicleChassisTrnsfm.q.rotate(gRight));
}

////////////////////////////////////////////////////////////////////////////

void PxVehicleWheelsDynData::setUserData(const PxU32 tireIdx, void* userData)
{
	PX_CHECK_AND_RETURN(tireIdx < mNbActiveWheels, "PxVehicleWheelsDynData::setUserData - Illegal wheel");
	mUserDatas[tireIdx]=userData;
}

void* PxVehicleWheelsDynData::getUserData(const PxU32 tireIdx) const
{
	PX_CHECK_AND_RETURN_VAL(tireIdx < mNbActiveWheels, "PxVehicleWheelsDynData::setUserData - Illegal wheel", NULL);
	return mUserDatas[tireIdx];
}


////////////////////////////////////////////////////////////////////////////

void PxVehicleWheelsDynData::copy(const PxVehicleWheelsDynData& src, const PxU32 srcWheel, const PxU32 trgWheel)
{
	PX_CHECK_AND_RETURN(srcWheel < src.mNbActiveWheels, "PxVehicleWheelsDynData::copy - Illegal src wheel");
	PX_CHECK_AND_RETURN(trgWheel < mNbActiveWheels, "PxVehicleWheelsDynData::copy - Illegal trg wheel");

	const PxVehicleWheels4DynData& src4 = src.mWheels4DynData[(srcWheel>>2)];
	PxVehicleWheels4DynData& trg4 = mWheels4DynData[(trgWheel>>2)];

	trg4.mWheelSpeeds[trgWheel & 3] = src4.mWheelSpeeds[srcWheel & 3];
	trg4.mCorrectedWheelSpeeds[trgWheel & 3] = src4.mCorrectedWheelSpeeds[srcWheel & 3];
	trg4.mTireLowForwardSpeedTimers[trgWheel & 3] = src4.mTireLowForwardSpeedTimers[srcWheel & 3];
	trg4.mTireLowSideSpeedTimers[trgWheel & 3] = src4.mTireLowSideSpeedTimers[srcWheel & 3];
	trg4.mWheelRotationAngles[trgWheel & 3] = src4.mWheelRotationAngles[srcWheel & 3];

	if(src4.mRaycastResults)
	{
		const PxVehicleWheels4DynData::SuspLineRaycast& suspLineRaycastSrc = reinterpret_cast<const PxVehicleWheels4DynData::SuspLineRaycast&>(src4.mQueryOrCachedHitResults);
		PxVehicleWheels4DynData::SuspLineRaycast& suspLineRaycastTrg = reinterpret_cast<PxVehicleWheels4DynData::SuspLineRaycast&>(trg4.mQueryOrCachedHitResults);

		suspLineRaycastTrg.mStarts[trgWheel & 3] = suspLineRaycastSrc.mStarts[srcWheel & 3];
		suspLineRaycastTrg.mDirs[trgWheel & 3] = suspLineRaycastSrc.mDirs[srcWheel & 3];
		suspLineRaycastTrg.mLengths[trgWheel & 3] = suspLineRaycastSrc.mLengths[srcWheel & 3];
	}
	else if(src4.mSweepResults)
	{
		const PxVehicleWheels4DynData::SuspLineSweep& suspLineSweepSrc = reinterpret_cast<const PxVehicleWheels4DynData::SuspLineSweep&>(src4.mQueryOrCachedHitResults);
		PxVehicleWheels4DynData::SuspLineSweep& suspLineSweepTrg = reinterpret_cast<PxVehicleWheels4DynData::SuspLineSweep&>(trg4.mQueryOrCachedHitResults);

		suspLineSweepTrg.mStartPose[trgWheel & 3] = suspLineSweepSrc.mStartPose[srcWheel & 3];
		suspLineSweepTrg.mDirs[trgWheel & 3] = suspLineSweepSrc.mDirs[srcWheel & 3];
		suspLineSweepTrg.mLengths[trgWheel & 3] = suspLineSweepSrc.mLengths[srcWheel & 3];
	}
	else
	{
		const PxVehicleWheels4DynData::CachedSuspLineSceneQuerytHitResult& cachedHitResultSrc = reinterpret_cast<const PxVehicleWheels4DynData::CachedSuspLineSceneQuerytHitResult&>(src4.mQueryOrCachedHitResults);
		PxVehicleWheels4DynData::CachedSuspLineSceneQuerytHitResult& cachedHitResultTrg = reinterpret_cast<PxVehicleWheels4DynData::CachedSuspLineSceneQuerytHitResult&>(trg4.mQueryOrCachedHitResults);

		cachedHitResultTrg.mPlanes[trgWheel & 3] = cachedHitResultSrc.mPlanes[srcWheel & 3];
		cachedHitResultTrg.mFrictionMultipliers[trgWheel & 3] = cachedHitResultSrc.mFrictionMultipliers[srcWheel & 3];
		cachedHitResultTrg.mCounts[trgWheel & 3] = cachedHitResultSrc.mCounts[srcWheel & 3];
		cachedHitResultTrg.mDistances[trgWheel & 3] = cachedHitResultSrc.mDistances[srcWheel & 3];
	}
}


} //namespace physx

