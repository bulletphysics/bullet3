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

#include "PxVehicleUtilTelemetry.h"
#include "PsFoundation.h"
#include "PsUtilities.h"
#include "stdio.h"
#include "CmPhysXCommon.h"

namespace physx
{

#if PX_DEBUG_VEHICLE_ON

PxVehicleGraphDesc::PxVehicleGraphDesc()
:	mPosX(PX_MAX_F32),
	mPosY(PX_MAX_F32),
	mSizeX(PX_MAX_F32),
	mSizeY(PX_MAX_F32),
	mBackgroundColor(PxVec3(PX_MAX_F32,PX_MAX_F32,PX_MAX_F32)),
	mAlpha(PX_MAX_F32)
{
}

bool PxVehicleGraphDesc::isValid() const
{
	PX_CHECK_AND_RETURN_VAL(mPosX != PX_MAX_F32, "PxVehicleGraphDesc.mPosX must be initialised", false);
	PX_CHECK_AND_RETURN_VAL(mPosY != PX_MAX_F32, "PxVehicleGraphDesc.mPosY must be initialised", false);
	PX_CHECK_AND_RETURN_VAL(mSizeX != PX_MAX_F32, "PxVehicleGraphDesc.mSizeX must be initialised", false);
	PX_CHECK_AND_RETURN_VAL(mSizeY != PX_MAX_F32, "PxVehicleGraphDesc.mSizeY must be initialised", false);
	PX_CHECK_AND_RETURN_VAL(mBackgroundColor.x != PX_MAX_F32 && mBackgroundColor.y != PX_MAX_F32 && mBackgroundColor.z != PX_MAX_F32, "PxVehicleGraphDesc.mBackgroundColor must be initialised", false);
	PX_CHECK_AND_RETURN_VAL(mAlpha != PX_MAX_F32, "PxVehicleGraphDesc.mAlpha must be initialised", false);
	return true;
}

PxVehicleGraphChannelDesc::PxVehicleGraphChannelDesc()
:	mMinY(PX_MAX_F32),
	mMaxY(PX_MAX_F32),
	mMidY(PX_MAX_F32),
	mColorLow(PxVec3(PX_MAX_F32,PX_MAX_F32,PX_MAX_F32)),
	mColorHigh(PxVec3(PX_MAX_F32,PX_MAX_F32,PX_MAX_F32)),
	mTitle(NULL)
{
}

bool PxVehicleGraphChannelDesc::isValid() const
{
	PX_CHECK_AND_RETURN_VAL(mMinY != PX_MAX_F32, "PxVehicleGraphChannelDesc.mMinY must be initialised", false);
	PX_CHECK_AND_RETURN_VAL(mMaxY != PX_MAX_F32, "PxVehicleGraphChannelDesc.mMaxY must be initialised", false);
	PX_CHECK_AND_RETURN_VAL(mMidY != PX_MAX_F32, "PxVehicleGraphChannelDesc.mMidY must be initialised", false);
	PX_CHECK_AND_RETURN_VAL(mColorLow.x != PX_MAX_F32 && mColorLow.y != PX_MAX_F32 && mColorLow.z != PX_MAX_F32, "PxVehicleGraphChannelDesc.mColorLow must be initialised", false);
	PX_CHECK_AND_RETURN_VAL(mColorHigh.x != PX_MAX_F32 && mColorHigh.y != PX_MAX_F32 && mColorHigh.z != PX_MAX_F32, "PxVehicleGraphChannelDesc.mColorHigh must be initialised", false);
	PX_CHECK_AND_RETURN_VAL(mTitle, "PxVehicleGraphChannelDesc.mTitle must be initialised", false);
	return true;
}

PxVehicleGraph::PxVehicleGraph()
{
	mBackgroundMinX=0;
	mBackgroundMaxX=0;
	mBackgroundMinY=0;
	mBackgroundMaxY=0;
	mSampleTide=0;
	mBackgroundColor=PxVec3(255.f,255.f,255.f);
	mBackgroundAlpha=1.0f;
	for(PxU32 i=0;i<eMAX_NB_CHANNELS;i++)
	{
		mChannelMinY[i]=0;
		mChannelMaxY[i]=0;
		mChannelMidY[i]=0;
		mChannelColorLow[i]=PxVec3(0,0,255.f);
		mChannelColorHigh[i]=PxVec3(255.f,0,0);
		memset(mChannelSamples[i], 0, sizeof(PxReal)*eMAX_NB_SAMPLES);
	}
	mNbChannels = 0;
	PX_COMPILE_TIME_ASSERT(size_t(PxVehicleGraph::eMAX_NB_CHANNELS) >= size_t(PxVehicleDriveGraphChannel::eMAX_NB_DRIVE_CHANNELS) && size_t(PxVehicleGraph::eMAX_NB_CHANNELS) >= size_t(PxVehicleWheelGraphChannel::eMAX_NB_WHEEL_CHANNELS));
}

PxVehicleGraph::~PxVehicleGraph()
{
}

void PxVehicleGraph::setup(const PxVehicleGraphDesc& desc, const PxVehicleGraphType::Enum graphType)
{
	mBackgroundMinX = (desc.mPosX - 0.5f*desc.mSizeX);
	mBackgroundMaxX = (desc.mPosX + 0.5f*desc.mSizeX);
	mBackgroundMinY = (desc.mPosY - 0.5f*desc.mSizeY);
	mBackgroundMaxY = (desc.mPosY + 0.5f*desc.mSizeY);

	mBackgroundColor=desc.mBackgroundColor;
	mBackgroundAlpha=desc.mAlpha;

	mNbChannels = (PxVehicleGraphType::eWHEEL==graphType) ? PxU32(PxVehicleWheelGraphChannel::eMAX_NB_WHEEL_CHANNELS) : PxU32(PxVehicleDriveGraphChannel::eMAX_NB_DRIVE_CHANNELS);
}

void PxVehicleGraph::setChannel(PxVehicleGraphChannelDesc& desc, const PxU32 channel)
{
	PX_ASSERT(channel<eMAX_NB_CHANNELS);

	mChannelMinY[channel]=desc.mMinY;
	mChannelMaxY[channel]=desc.mMaxY;
	mChannelMidY[channel]=desc.mMidY;
	PX_CHECK_MSG(mChannelMinY[channel]<=mChannelMidY[channel], "mChannelMinY must be less than or equal to mChannelMidY");
	PX_CHECK_MSG(mChannelMidY[channel]<=mChannelMaxY[channel], "mChannelMidY must be less than or equal to mChannelMaxY");

	mChannelColorLow[channel]=desc.mColorLow;
	mChannelColorHigh[channel]=desc.mColorHigh;

	strcpy(mChannelTitle[channel], desc.mTitle);
}

void PxVehicleGraph::clearRecordedChannelData()
{
	mSampleTide=0;
	for(PxU32 i=0;i<eMAX_NB_CHANNELS;i++)
	{
		memset(mChannelSamples[i], 0, sizeof(PxReal)*eMAX_NB_SAMPLES);
	}
}

void PxVehicleGraph::updateTimeSlice(const PxReal* const samples)
{
	mSampleTide++;
	mSampleTide=mSampleTide%eMAX_NB_SAMPLES;

	for(PxU32 i=0;i<mNbChannels;i++)
	{
		mChannelSamples[i][mSampleTide]=samples[i];
	}
}

void PxVehicleGraph::computeGraphChannel(const PxU32 channel, PxReal* xy, PxVec3* colors, char* title) const
{
	PX_ASSERT(channel<mNbChannels);
	const PxReal sizeX=mBackgroundMaxX-mBackgroundMinX;
	const PxReal sizeY=mBackgroundMaxY-mBackgroundMinY;
	const PxF32 minVal=mChannelMinY[channel];
	const PxF32 maxVal=mChannelMaxY[channel];
	const PxF32 midVal=mChannelMidY[channel];
	const PxVec3 colorLow=mChannelColorLow[channel];
	const PxVec3 colorHigh=mChannelColorHigh[channel];
	for(PxU32 i=0;i<PxVehicleGraph::eMAX_NB_SAMPLES;i++)
	{
		const PxU32 index = (mSampleTide+1+i)%PxVehicleGraph::eMAX_NB_SAMPLES;
		xy[2*i+0] = mBackgroundMinX+sizeX*i/(1.0f * PxVehicleGraph::eMAX_NB_SAMPLES);
		const PxF32 sampleVal = PxClamp(mChannelSamples[channel][index],minVal,maxVal);
		const PxReal y = (sampleVal-minVal)/(maxVal-minVal);		
		xy[2*i+1] = mBackgroundMinY+sizeY*y;
		colors[i] = sampleVal < midVal ? colorLow : colorHigh;
	}

	strcpy(title,mChannelTitle[channel]);
}

PxF32 PxVehicleGraph::getLatestValue(const PxU32 channel) const
{
	PX_CHECK_AND_RETURN_VAL(channel < mNbChannels, "PxVehicleGraph::getLatestValue: Illegal channel", 0.0f);
	return mChannelSamples[channel][mSampleTide];
}

void PxVehicleGraph::setupEngineGraph
(const PxF32 sizeX, const PxF32 sizeY, const PxF32 posX, const PxF32 posY, 
 const PxVec3& backgoundColor, const PxVec3& lineColorHigh, const PxVec3& lineColorLow)
{
	PxVehicleGraphDesc desc;
	desc.mSizeX=sizeX;
	desc.mSizeY=sizeY;
	desc.mPosX=posX;
	desc.mPosY=posY;
	desc.mBackgroundColor=backgoundColor;
	desc.mAlpha=0.5f;
	setup(desc,PxVehicleGraphType::eDRIVE);

	//Engine revs
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColorHigh=lineColorHigh;
		desc2.mColorLow=lineColorLow;
		desc2.mMinY=0.0f;
		desc2.mMaxY=800.0f;
		desc2.mMidY=400.0f;
		char title[64];
		sprintf(title, "engineRevs");
		desc2.mTitle=title;
		setChannel(desc2,PxVehicleDriveGraphChannel::eENGINE_REVS);
	}

	//Engine torque
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColorHigh=lineColorHigh;
		desc2.mColorLow=lineColorLow;
		desc2.mMinY=0.0f;
		desc2.mMaxY=1000.0f;
		desc2.mMidY=0.0f;
		char title[64];
		sprintf(title, "engineDriveTorque");
		desc2.mTitle=title;
		setChannel(desc2,PxVehicleDriveGraphChannel::eENGINE_DRIVE_TORQUE);
	}

	//Clutch slip
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColorHigh=lineColorHigh;
		desc2.mColorLow=lineColorLow;
		desc2.mMinY=-200.0f;
		desc2.mMaxY=200.0f;
		desc2.mMidY=0.0f;
		char title[64];
		sprintf(title, "clutchSlip");
		desc2.mTitle=title;
		setChannel(desc2,PxVehicleDriveGraphChannel::eCLUTCH_SLIP);
	}

	//Accel control
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColorHigh=lineColorHigh;
		desc2.mColorLow=lineColorLow;
		desc2.mMinY=0.0f;
		desc2.mMaxY=1.1f;
		desc2.mMidY=0.0f;
		char title[64];
		sprintf(title, "accel");
		desc2.mTitle=title;
		setChannel(desc2,PxVehicleDriveGraphChannel::eACCEL_CONTROL);
	}

	//Brake control
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColorHigh=lineColorHigh;
		desc2.mColorLow=lineColorLow;
		desc2.mMinY=0.0f;
		desc2.mMaxY=1.1f;
		desc2.mMidY=0.0f;
		char title[64];
		sprintf(title, "brake/tank brake left");
		desc2.mTitle=title;
		setChannel(desc2,PxVehicleDriveGraphChannel::eBRAKE_CONTROL);
	}

	//HandBrake control
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColorHigh=lineColorHigh;
		desc2.mColorLow=lineColorLow;
		desc2.mMinY=0.0f;
		desc2.mMaxY=1.1f;
		desc2.mMidY=0.0f;
		char title[64];
		sprintf(title, "handbrake/tank brake right");
		desc2.mTitle=title;
		setChannel(desc2,PxVehicleDriveGraphChannel::eHANDBRAKE_CONTROL);
	}

	//Steer control
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColorHigh=lineColorHigh;
		desc2.mColorLow=lineColorLow;
		desc2.mMinY=-1.1f;
		desc2.mMaxY=1.1f;
		desc2.mMidY=0.0f;
		char title[64];
		sprintf(title, "steerLeft/tank thrust left");
		desc2.mTitle=title;
		setChannel(desc2,PxVehicleDriveGraphChannel::eSTEER_LEFT_CONTROL);
	}

	//Steer control
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColorHigh=lineColorHigh;
		desc2.mColorLow=lineColorLow;
		desc2.mMinY=-1.1f;
		desc2.mMaxY=1.1f;
		desc2.mMidY=0.0f;
		char title[64];
		sprintf(title, "steerRight/tank thrust right");
		desc2.mTitle=title;
		setChannel(desc2,PxVehicleDriveGraphChannel::eSTEER_RIGHT_CONTROL);
	}

	//Gear
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColorHigh=lineColorHigh;
		desc2.mColorLow=lineColorLow;
		desc2.mMinY=-4.f;
		desc2.mMaxY=20.f;
		desc2.mMidY=0.0f;
		char title[64];
		sprintf(title, "gearRatio");
		desc2.mTitle=title;
		setChannel(desc2,PxVehicleDriveGraphChannel::eGEAR_RATIO);
	}
}

void PxVehicleGraph::setupWheelGraph
	(const PxF32 sizeX, const PxF32 sizeY, const PxF32 posX, const PxF32 posY, 
	const PxVec3& backgoundColor, const PxVec3& lineColorHigh, const PxVec3& lineColorLow)
{
	PxVehicleGraphDesc desc;
	desc.mSizeX=sizeX;
	desc.mSizeY=sizeY;
	desc.mPosX=posX;
	desc.mPosY=posY;
	desc.mBackgroundColor=backgoundColor;
	desc.mAlpha=0.5f;
	setup(desc,PxVehicleGraphType::eWHEEL);

	//Jounce data channel
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColorHigh=lineColorHigh;
		desc2.mColorLow=lineColorLow;
		desc2.mMinY=-0.2f;
		desc2.mMaxY=0.4f;
		desc2.mMidY=0.0f;
		char title[64];
		sprintf(title, "suspJounce");
		desc2.mTitle=title;
		setChannel(desc2,PxVehicleWheelGraphChannel::eJOUNCE);
	}

	//Jounce susp force channel
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColorHigh=lineColorHigh;
		desc2.mColorLow=lineColorLow;
		desc2.mMinY=0.0f;
		desc2.mMaxY=20000.0f;
		desc2.mMidY=0.0f;
		char title[64];
		sprintf(title, "suspForce");
		desc2.mTitle=title;
		setChannel(desc2,PxVehicleWheelGraphChannel::eSUSPFORCE);
	}

	//Tire load channel.
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColorHigh=lineColorHigh;
		desc2.mColorLow=lineColorLow;
		desc2.mMinY=0.0f;
		desc2.mMaxY=20000.0f;
		desc2.mMidY=0.0f;
		char title[64];
		sprintf(title, "tireLoad");
		desc2.mTitle=title;
		setChannel(desc2,PxVehicleWheelGraphChannel::eTIRELOAD);
	}

	//Normalised tire load channel.
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColorHigh=lineColorHigh;
		desc2.mColorLow=lineColorLow;
		desc2.mMinY=0.0f;
		desc2.mMaxY=3.0f;
		desc2.mMidY=1.0f;
		char title[64];
		sprintf(title, "normTireLoad");
		desc2.mTitle=title;
		setChannel(desc2,PxVehicleWheelGraphChannel::eNORMALIZED_TIRELOAD);
	}

	//Wheel omega channel
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColorHigh=lineColorHigh;
		desc2.mColorLow=lineColorLow;
		desc2.mMinY=-50.0f;
		desc2.mMaxY=250.0f;
		desc2.mMidY=0.0f;
		char title[64];
		sprintf(title, "wheelOmega");
		desc2.mTitle=title;
		setChannel(desc2,PxVehicleWheelGraphChannel::eWHEEL_OMEGA);
	}

	//Tire friction
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColorHigh=lineColorHigh;
		desc2.mColorLow=lineColorLow;
		desc2.mMinY=0.0f;
		desc2.mMaxY=1.1f;
		desc2.mMidY=1.0f;
		char title[64];
		sprintf(title, "friction");
		desc2.mTitle=title;
		setChannel(desc2,PxVehicleWheelGraphChannel::eTIRE_FRICTION);
	}


	//Tire long slip
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColorHigh=lineColorHigh;
		desc2.mColorLow=lineColorLow;
		desc2.mMinY=-0.2f;
		desc2.mMaxY=0.2f;
		desc2.mMidY=0.0f;
		char title[64];
		sprintf(title, "tireLongSlip");
		desc2.mTitle=title;
		setChannel(desc2,PxVehicleWheelGraphChannel::eTIRE_LONG_SLIP);
	}

	//Normalised tire long force
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColorHigh=lineColorHigh;
		desc2.mColorLow=lineColorLow;
		desc2.mMinY=0.0f;
		desc2.mMaxY=2.0f;
		desc2.mMidY=1.0f;
		char title[64];
		sprintf(title, "normTireLongForce");
		desc2.mTitle=title;
		setChannel(desc2,PxVehicleWheelGraphChannel::eNORM_TIRE_LONG_FORCE);
	}

	//Tire lat slip
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColorHigh=lineColorHigh;
		desc2.mColorLow=lineColorLow;
		desc2.mMinY=-1.0f;
		desc2.mMaxY=1.0f;
		desc2.mMidY=0.0f;
		char title[64];
		sprintf(title, "tireLatSlip");
		desc2.mTitle=title;
		setChannel(desc2,PxVehicleWheelGraphChannel::eTIRE_LAT_SLIP);
	}

	//Normalised tire lat force
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColorHigh=lineColorHigh;
		desc2.mColorLow=lineColorLow;
		desc2.mMinY=0.0f;
		desc2.mMaxY=2.0f;
		desc2.mMidY=1.0f;
		char title[64];
		sprintf(title, "normTireLatForce");
		desc2.mTitle=title;
		setChannel(desc2,PxVehicleWheelGraphChannel::eNORM_TIRE_LAT_FORCE);
	}

	//Normalized aligning moment
	{
		PxVehicleGraphChannelDesc desc2;
		desc2.mColorHigh=lineColorHigh;
		desc2.mColorLow=lineColorLow;
		desc2.mMinY=0.0f;
		desc2.mMaxY=2.0f;
		desc2.mMidY=1.0f;
		char title[64];
		sprintf(title, "normTireAlignMoment");
		desc2.mTitle=title;
		setChannel(desc2,PxVehicleWheelGraphChannel::eNORM_TIRE_ALIGNING_MOMENT);
	}
}

PxVehicleTelemetryData* physx::PxVehicleTelemetryData::allocate(const PxU32 numWheels)
{
	//Work out the byte size required.
	PxU32 size = sizeof(PxVehicleTelemetryData);
	size += sizeof(PxVehicleGraph);					//engine graph
	size += sizeof(PxVehicleGraph)*numWheels;		//wheel graphs
	size += sizeof(PxVec3)*numWheels;				//tire force app points
	size += sizeof(PxVec3)*numWheels;				//susp force app points

	//Allocate the memory.
	PxVehicleTelemetryData* vehTelData=static_cast<PxVehicleTelemetryData*>(PX_ALLOC(size, "PxVehicleNWTelemetryData"));

	//Patch up the pointers.
	PxU8* ptr = reinterpret_cast<PxU8*>(vehTelData) + sizeof(PxVehicleTelemetryData);
	vehTelData->mEngineGraph = reinterpret_cast<PxVehicleGraph*>(ptr);
	new(vehTelData->mEngineGraph) PxVehicleGraph();
	ptr += sizeof(PxVehicleGraph);			
	vehTelData->mWheelGraphs = reinterpret_cast<PxVehicleGraph*>(ptr);
	for(PxU32 i=0;i<numWheels;i++)
	{
		new(&vehTelData->mWheelGraphs[i]) PxVehicleGraph();
	}
	ptr += sizeof(PxVehicleGraph)*numWheels;	
	vehTelData->mSuspforceAppPoints = reinterpret_cast<PxVec3*>(ptr);
	ptr += sizeof(PxVec3)*numWheels;	
	vehTelData->mTireforceAppPoints = reinterpret_cast<PxVec3*>(ptr);
	ptr += sizeof(PxVec3)*numWheels;	

	//Set the number of wheels in each structure that needs it.
	vehTelData->mNbActiveWheels=numWheels;

	//Finished.
	return vehTelData;
}

void PxVehicleTelemetryData::free()
{
	PX_FREE(this);
}

void physx::PxVehicleTelemetryData::setup
(const PxF32 graphSizeX, const PxF32 graphSizeY,
const PxF32 engineGraphPosX, const PxF32 engineGraphPosY,
const PxF32* const wheelGraphPosX, const PxF32* const wheelGraphPosY,
const PxVec3& backgroundColor, const PxVec3& lineColorHigh, const PxVec3& lineColorLow)
{
	mEngineGraph->setupEngineGraph
		(graphSizeX, graphSizeY, engineGraphPosX, engineGraphPosY, 
		backgroundColor, lineColorHigh, lineColorLow);

	const PxU32 numActiveWheels=mNbActiveWheels;
	for(PxU32 k=0;k<numActiveWheels;k++)
	{
		mWheelGraphs[k].setupWheelGraph
			(graphSizeX, graphSizeY, wheelGraphPosX[k], wheelGraphPosY[k], 
			 backgroundColor, lineColorHigh, lineColorLow);

		mTireforceAppPoints[k]=PxVec3(0,0,0);
		mSuspforceAppPoints[k]=PxVec3(0,0,0);
	}
}

void physx::PxVehicleTelemetryData::clear()
{
	mEngineGraph->clearRecordedChannelData();

	const PxU32 numActiveWheels=mNbActiveWheels;
	for(PxU32 k=0;k<numActiveWheels;k++)
	{
		mWheelGraphs[k].clearRecordedChannelData();
		mTireforceAppPoints[k]=PxVec3(0,0,0);
		mSuspforceAppPoints[k]=PxVec3(0,0,0);
	}
}

#endif //PX_DEBUG_VEHICLE_ON

} //physx




