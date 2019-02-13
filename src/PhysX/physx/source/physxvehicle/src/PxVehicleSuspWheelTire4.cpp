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

#include "PxVehicleSuspWheelTire4.h"
#include "PxVehicleDefaults.h"
#include "PsFoundation.h"
#include "CmPhysXCommon.h"
#include "PsUtilities.h"

namespace physx
{

PxVehicleWheels4SimData::PxVehicleWheels4SimData()
{
	for(PxU32 i=0;i<4;i++)
	{
		mSuspDownwardTravelDirections[i]=PxVec3(0,0,0);	//Must be filled out
		mSuspForceAppPointOffsets[i]=PxVec3(0,0,0);		//Must be filled out
		mTireForceAppPointOffsets[i]=PxVec3(0,0,0);		//Must be filled out
		mWheelCentreOffsets[i]=PxVec3(0,0,0);			//Must be filled out

		mTireRestLoads[i]=20.0f + 1500.0f;
		mRecipTireRestLoads[i]=1.0f/mTireRestLoads[i];
	}
}

bool PxVehicleWheels4SimData::isValid(const PxU32 id) const
{
	PX_ASSERT(id<4);
	PX_CHECK_AND_RETURN_VAL(mSuspensions[id].isValid(), "Invalid PxVehicleSuspWheelTire4SimulationData.mSuspensions", false);
	PX_CHECK_AND_RETURN_VAL(mWheels[id].isValid(), "Invalid PxVehicleSuspWheelTire4SimulationData.mWheels", false);
	PX_CHECK_AND_RETURN_VAL(mTires[id].isValid(), "Invalid PxVehicleSuspWheelTire4SimulationData.mTires", false);
	PX_CHECK_AND_RETURN_VAL(mSuspDownwardTravelDirections[id].magnitude()>=0.999f && mSuspDownwardTravelDirections[id].magnitude()<=1.001f, "Invalid PxVehicleSuspWheelTire4SimulationData.mSuspDownwardTravelDirections", false);
	PX_CHECK_AND_RETURN_VAL(mSuspForceAppPointOffsets[id].magnitude()!=0.0f, "Invalid PxVehicleSuspWheelTire4SimulationData.mSuspForceAppPointOffsets.mSuspForceAppPointOffsets", false);
	PX_CHECK_AND_RETURN_VAL(mTireForceAppPointOffsets[id].magnitude()!=0.0f, "Invalid PxVehicleSuspWheelTire4SimulationData.mTireForceAppPointOffsets.mTireForceAppPointOffsets", false);
	PX_CHECK_AND_RETURN_VAL(mWheelCentreOffsets[id].magnitude()!=0.0f, "Invalid PxVehicleSuspWheelTire4SimulationData.mWheelCentreOffsets.mWheelCentreOffsets", false);
	PX_CHECK_AND_RETURN_VAL(mTireRestLoads[id]>0.0f, "Invalid PxVehicleSuspWheelTire4SimulationData.mTireRestLoads", false);
	PX_CHECK_AND_RETURN_VAL(PxAbs((1.0f/mTireRestLoads[id]) - mRecipTireRestLoads[id]) <= 0.001f, "Invalid PxVehicleSuspWheelTire4SimulationData.mRecipTireRestLoads", false);
	PX_UNUSED(id);
	return true;
}

void PxVehicleWheels4SimData::setSuspensionData(const PxU32 id, const PxVehicleSuspensionData& susp)
{
	PX_CHECK_AND_RETURN(id<4, "Illegal suspension id");
	PX_CHECK_AND_RETURN(susp.mSpringStrength>0, "Susp spring strength must be greater than zero");
	PX_CHECK_AND_RETURN(susp.mSpringDamperRate>=0, "Susp spring damper rate must be greater than or equal to zero");
	PX_CHECK_AND_RETURN(susp.mMaxCompression>=0, "Susp max compression must be greater than or equal to zero");
	PX_CHECK_AND_RETURN(susp.mMaxDroop>=0, "Susp max droop must be greater than or equal to zero");
	PX_CHECK_AND_RETURN(susp.mMaxDroop>0 || susp.mMaxCompression>0, "Either one of max droop or max compression must be greater than zero");
	PX_CHECK_AND_RETURN(susp.mSprungMass>0, "Susp spring mass must be greater than zero");

	mSuspensions[id]=susp;
	mSuspensions[id].mRecipMaxCompression = 1.0f/((susp.mMaxCompression > 0.0f) ? susp.mMaxCompression : 1.0f);
	mSuspensions[id].mRecipMaxDroop = 1.0f/((susp.mMaxDroop > 0.0f) ? susp.mMaxDroop : 1.0f);

	mTireRestLoads[id]=mWheels[id].mMass+mSuspensions[id].mSprungMass;
	mRecipTireRestLoads[id]=1.0f/mTireRestLoads[id];
}

/////////////////////////////

void PxVehicleWheels4SimData::setWheelData(const PxU32 id, const PxVehicleWheelData& wheel)
{
	PX_CHECK_AND_RETURN(id<4, "Illegal wheel id");
	PX_CHECK_AND_RETURN(wheel.mRadius>0, "Wheel radius must be greater than zero");
	PX_CHECK_AND_RETURN(wheel.mMaxBrakeTorque>=0, "Wheel brake torque must be zero or be a positive value");
	PX_CHECK_AND_RETURN(wheel.mMaxHandBrakeTorque>=0, "Wheel handbrake torque must be zero or be a positive value");
	PX_CHECK_AND_RETURN(PxAbs(wheel.mMaxSteer)<PxHalfPi, "Wheel max steer must be in range (-Pi/2,Pi/2)");
	PX_CHECK_AND_RETURN(wheel.mMass>0, "Wheel mass must be greater than zero");
	PX_CHECK_AND_RETURN(wheel.mMOI>0, "Wheel moi must be greater than zero");
	PX_CHECK_AND_RETURN(wheel.mToeAngle>-PxHalfPi && wheel.mToeAngle<PxHalfPi, "Wheel toe angle must be in range (-Pi/2,Pi/2)");
	PX_CHECK_AND_RETURN(wheel.mWidth>0, "Wheel width must be greater than zero");
	PX_CHECK_AND_RETURN(wheel.mDampingRate>=0, "Wheel damping rate must be greater than or equal to zero");

	mWheels[id]=wheel;
	mWheels[id].mRecipRadius=1.0f/mWheels[id].mRadius;
	mWheels[id].mRecipMOI=1.0f/mWheels[id].mMOI;

	mTireRestLoads[id]=mWheels[id].mMass+mSuspensions[id].mSprungMass;
	mRecipTireRestLoads[id]=1.0f/mTireRestLoads[id];
}

/////////////////////////////

void PxVehicleWheels4SimData::setTireData(const PxU32 id, const PxVehicleTireData& tire)
{
	PX_CHECK_AND_RETURN(id<4, "Illegal tire id");
	PX_CHECK_AND_RETURN(tire.mLatStiffX>0, "Tire mLatStiffX must greater than zero");
	PX_CHECK_AND_RETURN(tire.mLatStiffY>0, "Tire mLatStiffY must greater than zero");
	PX_CHECK_AND_RETURN(tire.mLongitudinalStiffnessPerUnitGravity>0, "Tire longitudinal stiffness must greater than zero");
	PX_CHECK_AND_RETURN(tire.mCamberStiffnessPerUnitGravity>=0, "Tire camber stiffness must greater than or equal to zero");
	PX_CHECK_AND_RETURN(tire.mFrictionVsSlipGraph[0][0]==0, "mFrictionVsSlipGraph[0][0] must be zero");
	PX_CHECK_AND_RETURN(tire.mFrictionVsSlipGraph[0][1]>0, "mFrictionVsSlipGraph[0][0] must be greater than zero");
	PX_CHECK_AND_RETURN(tire.mFrictionVsSlipGraph[1][0]>0, "mFrictionVsSlipGraph[1][0] must be greater than zero");
	PX_CHECK_AND_RETURN(tire.mFrictionVsSlipGraph[1][1]>=tire.mFrictionVsSlipGraph[0][1], "mFrictionVsSlipGraph[1][1] must be greater than mFrictionVsSlipGraph[0][1]");
	PX_CHECK_AND_RETURN(tire.mFrictionVsSlipGraph[2][0]> tire.mFrictionVsSlipGraph[1][0], "mFrictionVsSlipGraph[2][0] must be greater than mFrictionVsSlipGraph[1][0]");
	PX_CHECK_AND_RETURN(tire.mFrictionVsSlipGraph[2][1]<=tire.mFrictionVsSlipGraph[1][1], "mFrictionVsSlipGraph[2][1] must be less than or equal to mFrictionVsSlipGraph[1][1]");

	mTires[id]=tire;
	mTires[id].mRecipLongitudinalStiffnessPerUnitGravity=1.0f/mTires[id].mLongitudinalStiffnessPerUnitGravity;
	mTires[id].mFrictionVsSlipGraphRecipx1Minusx0=1.0f/(mTires[id].mFrictionVsSlipGraph[1][0]-mTires[id].mFrictionVsSlipGraph[0][0]);
	mTires[id].mFrictionVsSlipGraphRecipx2Minusx1=1.0f/(mTires[id].mFrictionVsSlipGraph[2][0]-mTires[id].mFrictionVsSlipGraph[1][0]);
}

/////////////////////////////

void PxVehicleWheels4SimData::setSuspTravelDirection(const PxU32 id, const PxVec3& dir)
{
	PX_CHECK_AND_RETURN(id<4, "Illegal suspension id");
	PX_CHECK_AND_RETURN(dir.magnitude()>0.999f && dir.magnitude()<1.0001f, "Suspension travel dir must be unit vector");

	mSuspDownwardTravelDirections[id]=dir;
}

/////////////////////////////

void PxVehicleWheels4SimData::setSuspForceAppPointOffset(const PxU32 id, const PxVec3& offset)
{
	PX_CHECK_AND_RETURN(id<4, "Illegal suspension id");
	PX_CHECK_AND_RETURN(offset.magnitude()>0, "Susp force app point must be offset from centre of mass");

	mSuspForceAppPointOffsets[id]=offset;
}

/////////////////////////////

void PxVehicleWheels4SimData::setTireForceAppPointOffset(const PxU32 id, const PxVec3& offset)
{
	PX_CHECK_AND_RETURN(id<4, "Illegal tire id");
	PX_CHECK_AND_RETURN(offset.magnitude()>0, "Tire force app point must be offset from centre of mass");

	mTireForceAppPointOffsets[id]=offset;
}

/////////////////////////////

void PxVehicleWheels4SimData::setWheelCentreOffset(const PxU32 id, const PxVec3& offset)
{
	PX_CHECK_AND_RETURN(id<4, "Illegal wheel id");
	PX_CHECK_AND_RETURN(offset.magnitude()>0, "Tire force app point must be offset from centre of mass");

	mWheelCentreOffsets[id]=offset;
}

/////////////////////////////

void PxVehicleWheels4SimData::setWheelShapeMapping(const PxU32 id, const PxI32 shapeId)
{
	PX_CHECK_AND_RETURN(id<4, "Illegal wheel id");
	PX_CHECK_AND_RETURN((-1==shapeId) || (PxU32(shapeId) < PX_MAX_U8), "Illegal shapeId: must be -1 or less than PX_MAX_U8");
	mWheelShapeMap[id] = Ps::to8(-1!=shapeId ? shapeId : PX_MAX_U8);
}

/////////////////////////////

void PxVehicleWheels4SimData::setSceneQueryFilterData(const PxU32 id, const PxFilterData& sqFilterData)
{
	PX_CHECK_AND_RETURN(id<4, "Illegal wheel id");
	mSqFilterData[id]=sqFilterData;
}


} //namespace physx

