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

#ifndef PX_VEHICLE_CORE_COMPONENTS_H
#define PX_VEHICLE_CORE_COMPONENTS_H
/** \addtogroup vehicle
  @{
*/

#include "foundation/PxMemory.h"
#include "foundation/PxVec3.h"
#include "common/PxCoreUtilityTypes.h"
#include "PxVehicleSDK.h"
#include "common/PxTypeInfo.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxVehicleChassisData
{
public:

	friend class PxVehicleDriveSimData4W;

	PxVehicleChassisData()
		:	mMOI(PxVec3(0,0,0)),
			mMass(1500),
			mCMOffset(PxVec3(0,0,0))
	{
	}

	/**
	\brief Moment of inertia of vehicle rigid body actor.
	
	\note Specified in kilograms metres-squared (kg m^2).
	*/
	PxVec3 mMOI;

	/**
	\brief Mass of vehicle rigid body actor.
	
	\note Specified in kilograms (kg).
	*/
	PxReal mMass;

	/**
	\brief Center of mass offset of vehicle rigid body actor.

	\note Specified in metres (m).
	*/
	PxVec3 mCMOffset;

private:

	PxReal pad;

	bool isValid() const;
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleChassisData)& 0x0f));

class PxVehicleEngineData
{
public:

	friend class PxVehicleDriveSimData;
	
	enum
	{
		eMAX_NB_ENGINE_TORQUE_CURVE_ENTRIES = 8
	};

	PxVehicleEngineData()
		: 	mMOI(1.0f),
			mPeakTorque(500.0f),
			mMaxOmega(600.0f),
			mDampingRateFullThrottle(0.15f),
			mDampingRateZeroThrottleClutchEngaged(2.0f),
			mDampingRateZeroThrottleClutchDisengaged(0.35f)
	{
		mTorqueCurve.addPair(0.0f, 0.8f);
		mTorqueCurve.addPair(0.33f, 1.0f);
		mTorqueCurve.addPair(1.0f, 0.8f);

		mRecipMOI=1.0f/mMOI;
		mRecipMaxOmega=1.0f/mMaxOmega;
	}

	/**
	\brief Graph of normalized torque (torque/mPeakTorque) against normalized engine speed ( engineRotationSpeed / mMaxOmega ).
	
	\note The normalized engine speed is the x-axis of the graph, while the normalized torque is the y-axis of the graph.
	*/
	PxFixedSizeLookupTable<eMAX_NB_ENGINE_TORQUE_CURVE_ENTRIES> mTorqueCurve;

	/**
	\brief Moment of inertia of the engine around the axis of rotation.
	
	\note Specified in kilograms metres-squared (kg m^2)
	*/
	PxReal mMOI;

	/**
	\brief Maximum torque available to apply to the engine when the accelerator pedal is at maximum.
	
	\note The torque available is the value of the accelerator pedal (in range [0, 1]) multiplied by the normalized torque as computed from mTorqueCurve multiplied by mPeakTorque.
	
	\note Specified in kilograms metres-squared per second-squared (kg m^2 s^-2).

	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxReal mPeakTorque;

	/**
	\brief Maximum rotation speed of the engine.

	\note Specified in radians per second (s^-1).

	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxReal mMaxOmega;

	/**
	\brief Damping rate of engine when full throttle is applied.
	
	\note If the clutch is engaged (any gear except neutral) then the damping rate applied at run-time is an interpolation 
	between mDampingRateZeroThrottleClutchEngaged and mDampingRateFullThrottle:
	mDampingRateZeroThrottleClutchEngaged + (mDampingRateFullThrottle-mDampingRateZeroThrottleClutchEngaged)*acceleratorPedal;
	
	\note If the clutch is disengaged (in neutral gear) the damping rate applied at run-time is an interpolation
	between mDampingRateZeroThrottleClutchDisengaged and mDampingRateFullThrottle:
	mDampingRateZeroThrottleClutchDisengaged + (mDampingRateFullThrottle-mDampingRateZeroThrottleClutchDisengaged)*acceleratorPedal;
	
	\note Specified in kilograms metres-squared per second (kg m^2 s^-1).

	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxReal mDampingRateFullThrottle;


	/**
	\brief Damping rate of engine when full throttle is applied.
	
	\note If the clutch is engaged (any gear except neutral) then the damping rate applied at run-time is an interpolation 
	between mDampingRateZeroThrottleClutchEngaged and mDampingRateFullThrottle:
	mDampingRateZeroThrottleClutchEngaged + (mDampingRateFullThrottle-mDampingRateZeroThrottleClutchEngaged)*acceleratorPedal;
	
	\note If the clutch is disengaged (in neutral gear) the damping rate applied at run-time is an interpolation
	between mDampingRateZeroThrottleClutchDisengaged and mDampingRateFullThrottle:
	mDampingRateZeroThrottleClutchDisengaged + (mDampingRateFullThrottle-mDampingRateZeroThrottleClutchDisengaged)*acceleratorPedal;
	
	\note Specified in kilograms metres-squared per second (kg m^2 s^-1).

	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxReal mDampingRateZeroThrottleClutchEngaged;

	/**
	\brief Damping rate of engine when full throttle is applied.
	
	\note If the clutch is engaged (any gear except neutral) then the damping rate applied at run-time is an interpolation 
	between mDampingRateZeroThrottleClutchEngaged and mDampingRateFullThrottle:
	mDampingRateZeroThrottleClutchEngaged + (mDampingRateFullThrottle-mDampingRateZeroThrottleClutchEngaged)*acceleratorPedal;
	
	\note If the clutch is disengaged (in neutral gear) the damping rate applied at run-time is an interpolation
	between mDampingRateZeroThrottleClutchDisengaged and mDampingRateFullThrottle:
	mDampingRateZeroThrottleClutchDisengaged + (mDampingRateFullThrottle-mDampingRateZeroThrottleClutchDisengaged)*acceleratorPedal;
	
	\note Specified in kilograms metres-squared per second (kg m^2 s^-1).

	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxReal mDampingRateZeroThrottleClutchDisengaged;

	/**
	\brief Return value of mRecipMOI(=1.0f/mMOI) that is automatically set by PxVehicleDriveSimData::setEngineData
	*/
	PX_FORCE_INLINE PxReal getRecipMOI() const {return mRecipMOI;}

	/**
	\brief Return value of mRecipMaxOmega( = 1.0f / mMaxOmega ) that is automatically set by PxVehicleDriveSimData::setEngineData
	*/
	PX_FORCE_INLINE PxReal getRecipMaxOmega() const {return mRecipMaxOmega;}

private:

	/**
	\brief Reciprocal of the engine moment of inertia.
	
	\note Not necessary to set this value because it is set by PxVehicleDriveSimData::setEngineData

	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxReal mRecipMOI;

	/**
	\brief Reciprocal of the maximum rotation speed of the engine.
	
	\note Not necessary to set this value because it is set by PxVehicleDriveSimData::setEngineData

	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxReal mRecipMaxOmega;

	bool isValid() const;


//serialization
public:
	PxVehicleEngineData(const PxEMPTY) : mTorqueCurve(PxEmpty) {}
//~serialization
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleEngineData)& 0x0f));

class PxVehicleGearsData
{
public:

	friend class PxVehicleDriveSimData;

	enum Enum
	{
		eREVERSE=0,
		eNEUTRAL,
		eFIRST,
		eSECOND,
		eTHIRD,
		eFOURTH,
		eFIFTH,
		eSIXTH,
		eSEVENTH,
		eEIGHTH,
		eNINTH,
		eTENTH,
		eELEVENTH,
		eTWELFTH,
		eTHIRTEENTH,
		eFOURTEENTH,
		eFIFTEENTH,
		eSIXTEENTH,
		eSEVENTEENTH,
		eEIGHTEENTH,
		eNINETEENTH,
		eTWENTIETH,
		eTWENTYFIRST,
		eTWENTYSECOND,
		eTWENTYTHIRD,
		eTWENTYFOURTH,
		eTWENTYFIFTH,
		eTWENTYSIXTH,
		eTWENTYSEVENTH,
		eTWENTYEIGHTH,
		eTWENTYNINTH,
		eTHIRTIETH,
		eGEARSRATIO_COUNT
	};

	PxVehicleGearsData()
		: 	mFinalRatio(4.0f),
			mNbRatios(7),
			mSwitchTime(0.5f)
	{
		mRatios[PxVehicleGearsData::eREVERSE]=-4.0f;
		mRatios[PxVehicleGearsData::eNEUTRAL]=0.0f;
		mRatios[PxVehicleGearsData::eFIRST]=4.0f;
		mRatios[PxVehicleGearsData::eSECOND]=2.0f;
		mRatios[PxVehicleGearsData::eTHIRD]=1.5f;
		mRatios[PxVehicleGearsData::eFOURTH]=1.1f;
		mRatios[PxVehicleGearsData::eFIFTH]=1.0f;
		
		for(PxU32 i = PxVehicleGearsData::eSIXTH; i < PxVehicleGearsData::eGEARSRATIO_COUNT; ++i)
			mRatios[i]=0.f;
	}
	
	/**
	\brief Gear ratios 

	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxReal mRatios[PxVehicleGearsData::eGEARSRATIO_COUNT];

	/**
	\brief Gear ratio applied is mRatios[currentGear]*finalRatio

	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxReal mFinalRatio;

	/**
	\brief Number of gears (including reverse and neutral).

	<b>Range:</b> (0, MAX_NB_GEAR_RATIOS)<br>
	*/
	PxU32 mNbRatios;
	
	/**
	\brief Time it takes to switch gear.
	
	\note Specified in seconds (s).

	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxReal mSwitchTime;
	
private:

	PxReal mPad;

	bool isValid() const;

//serialization
public:
	PxVehicleGearsData(const PxEMPTY) {}
	PxReal getGearRatio(PxVehicleGearsData::Enum a)  const {return mRatios[a];}
	void setGearRatio(PxVehicleGearsData::Enum a, PxReal ratio)   { mRatios[a] = ratio;}
//~serialization
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleGearsData)& 0x0f));

class PxVehicleAutoBoxData
{
public:

	friend class PxVehicleDriveSimData;

	PxVehicleAutoBoxData()
	{
		for(PxU32 i=0;i<PxVehicleGearsData::eGEARSRATIO_COUNT;i++)
		{
			mUpRatios[i]=0.65f;
			mDownRatios[i]=0.50f;
		}
		//Not sure how important this is but we want to kick out of neutral very quickly.
		mUpRatios[PxVehicleGearsData::eNEUTRAL]=0.15f;
		//Set the latency time in an unused element of one of the arrays.
		mDownRatios[PxVehicleGearsData::eREVERSE]=2.0f;	
	}
	
	/**
	\brief Value of ( engineRotationSpeed / PxVehicleEngineData::mMaxOmega ) that is high enough to increment gear.
	
	\note When ( engineRotationSpeed / PxVehicleEngineData::mMaxOmega ) > mUpRatios[currentGear] the autobox will begin 
	a transition to currentGear+1 unless currentGear is the highest possible gear or neutral or reverse.

	<b>Range:</b> [0, 1]<br>
	*/
	PxReal mUpRatios[PxVehicleGearsData::eGEARSRATIO_COUNT];

	/**
	\brief Value of engineRevs/maxEngineRevs that is low enough to decrement gear.
	
	\note When ( engineRotationSpeed / PxVehicleEngineData::mMaxOmega ) < mDownRatios[currentGear] the autobox will begin 
	a transition to currentGear-1 unless currentGear is first gear or neutral or reverse.

	<b>Range:</b> [0, 1]<br>
	*/
	PxReal mDownRatios[PxVehicleGearsData::eGEARSRATIO_COUNT];

	/**
	\brief Set the latency time of the autobox.
	
	\note Latency time is the minimum time that must pass between each gear change that is initiated by the autobox.
	The auto-box will only attempt to initiate another gear change up or down if the simulation time that has passed since the most recent
	automated gear change is greater than the specified latency.
	
	\note Specified in seconds (s).

	@see getLatency
	*/
	void setLatency(const PxReal latency) 
	{ 
		mDownRatios[PxVehicleGearsData::eREVERSE]=latency;
	}

	/**
	\brief Get the latency time of the autobox.
	
	\note Specified in seconds (s).

	@see setLatency
	*/
	PxReal getLatency() const 
	{ 
		return mDownRatios[PxVehicleGearsData::eREVERSE];
	}

private:
	bool isValid() const;

//serialization
public:
	PxVehicleAutoBoxData(const PxEMPTY) {}
	
	PxReal getUpRatios(PxVehicleGearsData::Enum a)  const {return mUpRatios[a];}
	void setUpRatios(PxVehicleGearsData::Enum a, PxReal ratio)   { mUpRatios[a] = ratio;}

	PxReal getDownRatios(PxVehicleGearsData::Enum a)  const {return mDownRatios[a];}
	void setDownRatios(PxVehicleGearsData::Enum a, PxReal ratio)   { mDownRatios[a] = ratio;}
//~serialization
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleAutoBoxData)& 0x0f));

class PxVehicleDifferential4WData
{
public:

	friend class PxVehicleDriveSimData4W;

	enum Enum
	{
		eDIFF_TYPE_LS_4WD,			//limited slip differential for car with 4 driven wheels
		eDIFF_TYPE_LS_FRONTWD,		//limited slip differential for car with front-wheel drive
		eDIFF_TYPE_LS_REARWD,		//limited slip differential for car with rear-wheel drive
		eDIFF_TYPE_OPEN_4WD,		//open differential for car with 4 driven wheels 
		eDIFF_TYPE_OPEN_FRONTWD,	//open differential for car with front-wheel drive
		eDIFF_TYPE_OPEN_REARWD,		//open differential for car with rear-wheel drive
		eMAX_NB_DIFF_TYPES
	};

	PxVehicleDifferential4WData()
		:	mFrontRearSplit(0.45f),
			mFrontLeftRightSplit(0.5f),
			mRearLeftRightSplit(0.5f),
			mCentreBias(1.3f),
			mFrontBias(1.3f),
			mRearBias(1.3f),
			mType(PxVehicleDifferential4WData::eDIFF_TYPE_LS_4WD)
	{
	}

	/**
	\brief Ratio of torque split between front and rear (>0.5 means more to front, <0.5 means more to rear).
	
	\note Only applied to DIFF_TYPE_LS_4WD and eDIFF_TYPE_OPEN_4WD

	<b>Range:</b> [0, 1]<br>
	*/
	PxReal mFrontRearSplit;

	/**
	\brief Ratio of torque split between front-left and front-right (>0.5 means more to front-left, <0.5 means more to front-right).
	
	\note Only applied to DIFF_TYPE_LS_4WD and eDIFF_TYPE_OPEN_4WD and eDIFF_TYPE_LS_FRONTWD

	<b>Range:</b> [0, 1]<br>
	*/
	PxReal mFrontLeftRightSplit;

	/**
	\brief Ratio of torque split between rear-left and rear-right (>0.5 means more to rear-left, <0.5 means more to rear-right).
	
	\note Only applied to DIFF_TYPE_LS_4WD and eDIFF_TYPE_OPEN_4WD and eDIFF_TYPE_LS_REARWD

	<b>Range:</b> [0, 1]<br>
	*/
	PxReal mRearLeftRightSplit;

	/**
	\brief Maximum allowed ratio of average front wheel rotation speed and rear wheel rotation speeds 
	The differential will divert more torque to the slower wheels when the bias is exceeded.
	
	\note Only applied to DIFF_TYPE_LS_4WD

	<b>Range:</b> [1, PX_MAX_F32)<br>
	*/
	PxReal mCentreBias;

	/**
	\brief Maximum allowed ratio of front-left and front-right wheel rotation speeds.
	The differential will divert more torque to the slower wheel when the bias is exceeded.
	
	\note Only applied to DIFF_TYPE_LS_4WD and DIFF_TYPE_LS_FRONTWD

	<b>Range:</b> [1, PX_MAX_F32)<br>
	*/
	PxReal mFrontBias;

	/**
	\brief Maximum allowed ratio of rear-left and rear-right wheel rotation speeds.
	The differential will divert more torque to the slower wheel when the bias is exceeded.
	
	\note Only applied to DIFF_TYPE_LS_4WD and DIFF_TYPE_LS_REARWD

	<b>Range:</b> [1, PX_MAX_F32)<br>
	*/
	PxReal mRearBias;

	/**
	\brief Type of differential.

	<b>Range:</b> [DIFF_TYPE_LS_4WD, DIFF_TYPE_OPEN_FRONTWD]<br>
	*/
	PxVehicleDifferential4WData::Enum mType;

private:

	PxReal mPad[1];

	bool isValid() const;

//serialization
public:
	PxVehicleDifferential4WData(const PxEMPTY) {}
//~serialization
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleDifferential4WData)& 0x0f));

class PxVehicleDifferentialNWData
{
public:

	friend class PxVehicleDriveSimDataNW;
	friend class PxVehicleUpdate;

	PxVehicleDifferentialNWData()
	{
		PxMemSet(mBitmapBuffer, 0, sizeof(PxU32) * (((PX_MAX_NB_WHEELS + 31) & ~31) >> 5));
		mNbDrivenWheels=0;
		mInvNbDrivenWheels=0.0f;
	}

	/**
	\brief Set a specific wheel to be driven or non-driven by the differential.
	
	\note The available drive torque will be split equally between all driven wheels.
	Zero torque will be applied to non-driven wheels.
	The default state of each wheel is to be uncoupled to the differential.
	*/
	void setDrivenWheel(const PxU32 wheelId, const bool drivenState);

	/**
	\brief Test if a specific wheel has been configured as a driven or non-driven wheel.
	*/
	bool getIsDrivenWheel(const PxU32 wheelId) const;

private:

	PxU32 mBitmapBuffer[((PX_MAX_NB_WHEELS + 31) & ~31) >> 5];
	PxU32 mNbDrivenWheels;
	PxReal mInvNbDrivenWheels;
	PxU32 mPad;

	bool isValid() const;

//serialization
public:
	PxVehicleDifferentialNWData(const PxEMPTY) {}
	PxU32 getDrivenWheelStatus() const;
	void setDrivenWheelStatus(PxU32 status);
//~serialization
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleDifferentialNWData)& 0x0f));


class PxVehicleAckermannGeometryData
{
public:

	friend class PxVehicleDriveSimData4W;

	PxVehicleAckermannGeometryData()
		: 	mAccuracy(1.0f),
			mFrontWidth(0.0f),		//Must be filled out 
			mRearWidth(0.0f),		//Must be filled out
			mAxleSeparation(0.0f)	//Must be filled out
	{
	}

	/**
	\brief Accuracy of Ackermann steer calculation.
	
	\note Accuracy with value 0.0 results in no Ackermann steer-correction, while
	accuracy with value 1.0 results in perfect Ackermann steer-correction.
	
	\note Perfect Ackermann steer correction modifies the steer angles applied to the front-left and 
	front-right wheels so that the perpendiculars to the wheels' longitudinal directions cross the 
	extended vector of the rear axle at the same point.  It is also applied to any steer angle applied 
	to the rear wheels but instead using the extended vector of the front axle.
	
	\note In general, more steer correction produces better cornering behavior.

	<b>Range:</b> [0, 1]<br>
	*/		
	PxReal mAccuracy;

	/**
	\brief Distance between center-point of the two front wheels.
	
	\note Specified in metres (m).

	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxReal mFrontWidth;		

	/**
	\brief Distance between center-point of the two rear wheels.
	
	\note Specified in metres (m).

	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxReal mRearWidth;		

	/**
	\brief Distance between center of front axle and center of rear axle.

	\note Specified in metres (m).

	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxReal mAxleSeparation;	

private:

	bool isValid() const;

//serialization
public:
	PxVehicleAckermannGeometryData(const PxEMPTY) {}
//~serialization
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleAckermannGeometryData)& 0x0f));

/**
\brief Choose between a potentially more expensive but more accurate solution to the clutch model or a potentially cheaper but less accurate solution.
@see PxVehicleClutchData
*/
struct PxVehicleClutchAccuracyMode
{
	enum Enum
	{
		eESTIMATE = 0,
		eBEST_POSSIBLE
	};
};

class PxVehicleClutchData
{
public:

	friend class PxVehicleDriveSimData;

	PxVehicleClutchData()
		: 	mStrength(10.0f),
		    mAccuracyMode(PxVehicleClutchAccuracyMode::eBEST_POSSIBLE),
			mEstimateIterations(5)
	{
	}

	/**
	\brief Strength of clutch.
	
	\note The clutch is the mechanism that couples the engine to the wheels.
	A stronger clutch more strongly couples the engine to the wheels, while a
	clutch of strength zero completely decouples the engine from the wheels.
	Stronger clutches more quickly bring the wheels and engine into equilibrium, while weaker
	clutches take longer, resulting in periods of clutch slip and delays in power transmission
	from the engine to the wheels.
	The torque generated by the clutch is proportional to the clutch strength and 
	the velocity difference between the engine's rotational speed and the rotational speed of the 
	driven wheels after accounting for the gear ratio.  
	The torque at the clutch is applied negatively to the engine and positively to the driven wheels.
	
	\note Specified in kilograms metres-squared per second (kg m^2 s^-1)

	<b>Range:</b> [0,PX_MAX_F32)<br>
	*/
	PxReal mStrength;

	/**
	\brief The engine and wheel rotation speeds that are coupled through the clutch can be updated by choosing
	one of two modes: eESTIMATE and eBEST_POSSIBLE.

	\note If eESTIMATE is chosen the vehicle sdk will update the wheel and engine rotation speeds 
	with estimated values to the implemented clutch model.  

	\note If eBEST_POSSIBLE is chosen the vehicle sdk will compute the best possible 
	solution (within floating point tolerance) to the implemented clutch model. 
	This is the recommended mode.

	\note The clutch model remains the same if either eESTIMATE or eBEST_POSSIBLE is chosen but the accuracy and 
	computational cost of the solution to the model can be tuned as required.
	*/
	PxVehicleClutchAccuracyMode::Enum mAccuracyMode;

	/**
	\brief Tune the mathematical accuracy and computational cost of the computed estimate to the wheel and 
	engine rotation speeds if eESTIMATE is chosen.

	\note As mEstimateIterations increases the computational cost of the clutch also increases and the solution 
	approaches the solution that would be computed if eBEST_POSSIBLE was chosen instead.

	\note This has no effect if eBEST_POSSIBLE is chosen as the accuracy mode.

	\note A value of zero is not allowed if eESTIMATE is chosen as the accuracy mode.
	*/
	PxU32 mEstimateIterations;

private:

	PxU8 mPad[4];

	bool isValid() const;

//serialization
public:
	PxVehicleClutchData(const PxEMPTY) {}
//~serialization
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleClutchData)& 0x0f));


/**
\brief Tire load variation can be strongly dependent on the time-step so it is a good idea to filter it 
to give less jerky handling behavior. 

\note The x-axis of the graph is normalized tire load, while the y-axis is the filtered normalized tire load.

\note The normalized load is the force acting downwards on the tire divided by the force experienced by the tire when the car is at rest on the ground.

\note The rest load is approximately the product of the value of gravitational acceleration and PxVehicleSuspensionData::mSprungMass.

\note The minimum possible normalized load is zero.

\note There are two points on the graph: (mMinNormalisedLoad, mMinNormalisedFilteredLoad) and (mMaxNormalisedLoad, mMaxFilteredNormalisedLoad).

\note Normalized loads less than mMinNormalisedLoad have filtered normalized load = mMinNormalisedFilteredLoad.

\note Normalized loads greater than mMaxNormalisedLoad have filtered normalized load = mMaxFilteredNormalisedLoad.

\note Normalized loads in-between are linearly interpolated between mMinNormalisedFilteredLoad and mMaxFilteredNormalisedLoad.

\note The tire load applied as input to the tire force computation is the filtered normalized load multiplied by the rest load.
*/
class PxVehicleTireLoadFilterData
{
public:

	friend class PxVehicleWheelsSimData;

	PxVehicleTireLoadFilterData()
		: 	mMinNormalisedLoad(0),
			mMinFilteredNormalisedLoad(0.2308f),
			mMaxNormalisedLoad(3.0f),
			mMaxFilteredNormalisedLoad(3.0f)
	{
		mDenominator=1.0f/(mMaxNormalisedLoad - mMinNormalisedLoad);
	}

	/**
	\brief Graph point (mMinNormalisedLoad,mMinFilteredNormalisedLoad)
	*/
	PxReal mMinNormalisedLoad; 

	/**
	\brief Graph point (mMinNormalisedLoad,mMinFilteredNormalisedLoad)
	*/
	PxReal mMinFilteredNormalisedLoad; 

	/**
	\brief Graph point (mMaxNormalisedLoad,mMaxFilteredNormalisedLoad)
	*/
	PxReal mMaxNormalisedLoad;
		
	/**
	\brief Graph point (mMaxNormalisedLoad,mMaxFilteredNormalisedLoad)
	*/
	PxReal mMaxFilteredNormalisedLoad;

	PX_FORCE_INLINE PxReal getDenominator() const {return mDenominator;}

private:

	/**
	\brief Not necessary to set this value.
	*/
	//1.0f/(mMaxNormalisedLoad-mMinNormalisedLoad) for quick calculations
	PxReal mDenominator;

	PxU32 mPad[3];

	bool isValid() const;

//serialization
public:
	PxVehicleTireLoadFilterData(const PxEMPTY) {}
//~serialization
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleTireLoadFilterData)& 0x0f));

class PxVehicleWheelData
{
public:

	friend class PxVehicleWheels4SimData;

	PxVehicleWheelData()
		: 	mRadius(0.0f),				//Must be filled out
			mWidth(0.0f),
			mMass(20.0f),
			mMOI(0.0f),					//Must be filled out
			mDampingRate(0.25f),
			mMaxBrakeTorque(1500.0f),
			mMaxHandBrakeTorque(0.0f),	
			mMaxSteer(0.0f),			
			mToeAngle(0.0f),
			mRecipRadius(0.0f),			//Must be filled out
			mRecipMOI(0.0f)				//Must be filled out
	{
	}

	/**
	\brief Radius of unit that includes metal wheel plus rubber tire.
	
	\note Specified in metres (m).

	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxReal mRadius;

	/**
	\brief Maximum width of unit that includes wheel plus tire.

	\note Specified in metres (m).

	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxReal mWidth;

	/**
	\brief Mass of unit that includes wheel plus tire.
	
	\note Specified in kilograms (kg).

	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxReal mMass;

	/**
	\brief Moment of inertia of unit that includes wheel plus tire about the rolling axis.
	
	\note Specified in kilograms metres-squared (kg m^2).

	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxReal mMOI;

	/**
	\brief Damping rate applied to wheel.
	
	\note Specified in kilograms metres-squared per second (kg m^2 s^-1).

	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxReal mDampingRate;

	/**
	\brief Max brake torque that can be applied to wheel.
	
	\note Specified in kilograms metres-squared per second-squared (kg m^2 s^-2)

	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxReal mMaxBrakeTorque;

	/**
	\brief Max handbrake torque that can be applied to wheel.
	
	\note Specified in kilograms metres-squared per second-squared (kg m^2 s^-2)

	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxReal mMaxHandBrakeTorque;

	/**
	\brief Max steer angle that can be achieved by the wheel.
	
	\note Specified in radians.

	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxReal mMaxSteer;

	/**
	\brief Wheel toe angle.  This value is ignored by PxVehicleDriveTank and PxVehicleNoDrive.
	
	\note Specified in radians.

	<b>Range:</b> [0, Pi/2]<br>
	*/
	PxReal mToeAngle;//in radians

	/**
	\brief Return value equal to 1.0f/mRadius
	
	@see PxVehicleWheelsSimData::setWheelData
	*/
	PX_FORCE_INLINE PxReal getRecipRadius() const {return mRecipRadius;}

	/**
	\brief Return value equal to 1.0f/mRecipMOI
	
	@see PxVehicleWheelsSimData::setWheelData
	*/
	PX_FORCE_INLINE PxReal getRecipMOI() const {return mRecipMOI;}

private:

	/**
	\brief Reciprocal of radius of unit that includes metal wheel plus rubber tire.
	
	\note Not necessary to set this value because it is set by PxVehicleWheelsSimData::setWheelData

	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxReal mRecipRadius;

	/**
	\brief Reciprocal of moment of inertia of unit that includes wheel plus tire about single allowed axis of rotation.
	
	\note Not necessary to set this value because it is set by PxVehicleWheelsSimData::setWheelData

	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxReal mRecipMOI;

	PxReal mPad[1];

	bool isValid() const;
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleWheelData)& 0x0f));

class PxVehicleSuspensionData
{
public:

	friend class PxVehicleWheels4SimData;

	PxVehicleSuspensionData()
		: 	mSpringStrength(0.0f),
			mSpringDamperRate(0.0f),
			mMaxCompression(0.3f),
			mMaxDroop(0.1f),
			mSprungMass(0.0f),
			mCamberAtRest(0.0f),
			mCamberAtMaxCompression(0.0f),
			mCamberAtMaxDroop(0.0f),
			mRecipMaxCompression(1.0f),
			mRecipMaxDroop(1.0f)
	{
	}
	
	/**
	\brief Spring strength of suspension unit.
	
	\note Specified in kilograms per second-squared (kg s^-2).

	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxReal mSpringStrength;

	/**
	\brief Spring damper rate of suspension unit.
	
	\note Specified in kilograms per second (kg s^-1).

	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxReal mSpringDamperRate;

	/**
	\brief Maximum compression allowed by suspension spring.
	
	\note Specified in metres (m).

	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxReal mMaxCompression;

	/**
	\brief Maximum elongation allowed by suspension spring.
	
	\note Specified in metres (m).

	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxReal mMaxDroop;

	/**
	\brief Mass of vehicle that is supported by suspension spring.
	
	\note Specified in kilograms (kg).

	\note Each suspension is guaranteed to generate an upwards force of |gravity|*mSprungMass along the suspension direction when the wheel is perfectly 
	at rest and sitting at the rest pose defined by the wheel centre offset.  

	\note The sum of the sprung masses of all suspensions of a vehicle should match the mass of the PxRigidDynamic associated with the vehicle.  
	When this condition is satisfied for a vehicle on a horizontal plane the wheels of the vehicle are guaranteed to sit at the rest pose 
	defined by the wheel centre offset.  The mass matching condition is not enforced.

	\note As the wheel compresses or elongates along the suspension direction the force generated by the spring is 
	F = |gravity|*mSprungMass + deltaX*mSpringStrength + deltaXDot*mSpringDamperRate
	where deltaX is the deviation from the defined rest pose and deltaXDot is the velocity of the sprung mass along the suspension direction.
	In practice, deltaXDot is computed by comparing the current and previous deviation from the rest pose and dividing the difference 
	by the simulation timestep.

	\note If a single suspension spring is hanging in the air and generates zero force the remaining springs of the vehicle will necessarily 
	sit in a compressed configuration.  In summary, the sum of the remaining suspension forces cannot balance the downwards gravitational force 
	acting on the vehicle without extra force arising from the deltaX*mSpringStrength force term.

	\note Theoretically, a suspension spring should generate zero force at maximum elongation and increase linearly as the suspension approaches the rest pose.
	PxVehicleSuspensionData will only enforce this physical law if the spring is configured so that |gravity|*mSprungMass == mMaxDroop*mSpringStrength.  
	To help decouple vehicle handling from visual wheel positioning this condition is not enforced.  
	In practice, the value of |gravity|*mSprungMass + deltaX*mSpringStrength is clamped at zero to ensure it never falls negative.

	@see PxVehicleComputeSprungMasses, PxVehicleWheelsSimData::setWheelCentreOffset, PxVehicleSuspensionData::mSpringStrength, PxVehicleSuspensionData::mSpringDamperRate, PxVehicleSuspensionData::mMaxDroop

	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxReal mSprungMass;

	/**
	\brief Camber angle (in radians) of wheel when the suspension is at its rest position.
	
	\note Specified in radians.

	<b>Range:</b> [-pi/2, pi/2]<br>

	*/
	PxReal mCamberAtRest;

	/**
	\brief Camber angle (in radians) of wheel when the suspension is at maximum compression.

	\note For compressed suspensions the camber angle is a linear interpolation of 
	mCamberAngleAtRest and mCamberAtMaxCompression

	\note Specified in radians.

	<b>Range:</b> [-pi/2, pi/2]<br>
	*/
	PxReal mCamberAtMaxCompression; 

	/**
	\brief Camber angle (in radians) of wheel when the suspension is at maximum droop.

	\note For extended suspensions the camber angle is linearly interpolation of 
	mCamberAngleAtRest and mCamberAtMaxDroop

	\note Specified in radians.

	<b>Range:</b> [-pi/2, pi/2]<br>
	*/
	PxReal mCamberAtMaxDroop; 

	/**
	\brief Reciprocal of maximum compression.
	
	\note Not necessary to set this value because it is set by PxVehicleWheelsSimData::setSuspensionData

	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PX_FORCE_INLINE PxReal getRecipMaxCompression() const {return mRecipMaxCompression;}

	/**
	\brief Reciprocal of maximum droop.
	
	\note Not necessary to set this value because it is set by PxVehicleWheelsSimData::setSuspensionData

	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PX_FORCE_INLINE PxReal getRecipMaxDroop() const {return mRecipMaxDroop;}

	/**
	\brief Set a new sprung mass for the suspension and modify the spring strength so that the natural frequency
	of the spring is preserved.
	\param[in] newSprungMass is the new mass that the suspension spring will support.
	*/
	void setMassAndPreserveNaturalFrequency(const PxReal newSprungMass)
	{
		const PxF32 oldStrength = mSpringStrength;
		const PxF32 oldSprungMass = mSprungMass;
		const PxF32 newStrength = oldStrength * (newSprungMass / oldSprungMass);
		mSpringStrength = newStrength;
		mSprungMass = newSprungMass;
	}

private:

	/**
	\brief Cached value of 1.0f/mMaxCompression
	
	\note Not necessary to set this value because it is set by PxVehicleWheelsSimData::setSuspensionData
	*/
	PxReal mRecipMaxCompression;

	/**
	\brief Cached value of 1.0f/mMaxDroop
	
	\note Not necessary to set this value because it is set by PxVehicleWheelsSimData::setSuspensionData
	*/
	PxReal mRecipMaxDroop;

	//padding
	PxReal mPad[2];

	bool isValid() const;
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleSuspensionData)& 0x0f));

class PxVehicleAntiRollBarData
{
public:

	friend class PxVehicleWheelsSimData;

	PxVehicleAntiRollBarData()
		: mWheel0(0xffffffff),
		  mWheel1(0xffffffff),
		  mStiffness(0.0f)
	{
	}

	/*
	\brief The anti-roll bar connects two wheels with indices mWheel0 and mWheel1
	*/
	PxU32 mWheel0;

	/*
	\brief The anti-roll bar connects two wheels with indices mWheel0 and mWheel1
	*/
	PxU32 mWheel1;

	/*
	\brief The stiffness of the anti-roll bar.

	\note Specified in kilograms per second-squared (kg s^-2).

	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxF32 mStiffness;

private:

	PxF32 mPad[1];

	bool isValid() const;
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleAntiRollBarData)& 0x0f));

class PxVehicleTireData
{
public:
	friend class PxVehicleWheels4SimData;

	PxVehicleTireData()
		: 	mLatStiffX(2.0f),
			mLatStiffY(0.3125f*(180.0f / PxPi)),
			mLongitudinalStiffnessPerUnitGravity(1000.0f),
			mCamberStiffnessPerUnitGravity(0.1f*(180.0f / PxPi)),
			mType(0)
	{
		mFrictionVsSlipGraph[0][0]=0.0f;
		mFrictionVsSlipGraph[0][1]=1.0f;
		mFrictionVsSlipGraph[1][0]=0.1f;
		mFrictionVsSlipGraph[1][1]=1.0f;
		mFrictionVsSlipGraph[2][0]=1.0f;
		mFrictionVsSlipGraph[2][1]=1.0f;

		mRecipLongitudinalStiffnessPerUnitGravity=1.0f/mLongitudinalStiffnessPerUnitGravity;

		mFrictionVsSlipGraphRecipx1Minusx0=1.0f/(mFrictionVsSlipGraph[1][0]-mFrictionVsSlipGraph[0][0]);
		mFrictionVsSlipGraphRecipx2Minusx1=1.0f/(mFrictionVsSlipGraph[2][0]-mFrictionVsSlipGraph[1][0]);
	}

	/**
	\brief Tire lateral stiffness is a graph of tire load that has linear behavior near zero load and 
	flattens at large loads.  mLatStiffX describes the minimum normalized load (load/restLoad) that gives a 
	flat lateral stiffness response to load.

	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxReal mLatStiffX;

	/**
	\brief Tire lateral stiffness is a graph of tire load that has linear behavior near zero load and 
	flattens at large loads. mLatStiffY describes the maximum possible value of lateralStiffness/restLoad that occurs 
	when (load/restLoad)>= mLatStiffX.
	
	\note If load/restLoad is greater than mLatStiffX then the lateral stiffness is mLatStiffY*restLoad.
	
	\note If load/restLoad is less than mLatStiffX then the lateral stiffness is mLastStiffY*(load/mLatStiffX)
	
	\note Lateral force can be approximated as lateralStiffness * lateralSlip.
	
	\note Specified in per radian.
	
	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxReal mLatStiffY;

	/**
	\brief Tire Longitudinal stiffness per unit gravitational acceleration.

	\note Longitudinal stiffness of the tire is calculated as gravitationalAcceleration*mLongitudinalStiffnessPerUnitGravity.

	\note Longitudinal force can be approximated as gravitationalAcceleration*mLongitudinalStiffnessPerUnitGravity*longitudinalSlip.

	\note Specified in kilograms per radian.
	
	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxReal mLongitudinalStiffnessPerUnitGravity;

	/**
	\brief tire Tire camber stiffness per unity gravitational acceleration.

	\note Camber stiffness of the tire is calculated as gravitationalAcceleration*mCamberStiffnessPerUnitGravity
	
	\note Camber force can be approximated as gravitationalAcceleration*mCamberStiffnessPerUnitGravity*camberAngle.

	\note Specified in kilograms per radian.
	
	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxReal mCamberStiffnessPerUnitGravity;

	/**
	\brief Graph of friction vs longitudinal slip with 3 points. 
	
	\note mFrictionVsSlipGraph[0][0] is always zero.

	\note mFrictionVsSlipGraph[0][1] is the friction available at zero longitudinal slip.
	
	\note mFrictionVsSlipGraph[1][0] is the value of longitudinal slip with maximum friction.
	
	\note mFrictionVsSlipGraph[1][1] is the maximum friction.
	
	\note mFrictionVsSlipGraph[2][0] is the end point of the graph.
	
	\note mFrictionVsSlipGraph[2][1] is the value of friction for slips greater than mFrictionVsSlipGraph[2][0].
	
	\note The friction value computed from the friction vs longitudinal slip graph is used to scale the friction
	value for the combination of material and tire type (PxVehicleDrivableSurfaceToTireFrictionPairs).

	\note mFrictionVsSlipGraph[2][0] > mFrictionVsSlipGraph[1][0] > mFrictionVsSlipGraph[0][0]

	\note mFrictionVsSlipGraph[1][1] is typically greater than  mFrictionVsSlipGraph[0][1]

	\note mFrictionVsSlipGraph[2][1] is typically smaller than mFrictionVsSlipGraph[1][1]

	\note longitudinal slips > mFrictionVsSlipGraph[2][0] use friction multiplier mFrictionVsSlipGraph[2][1]

	\note The final friction value used by the tire model is the value returned by PxVehicleDrivableSurfaceToTireFrictionPairs 
	multiplied by the value computed from mFrictionVsSlipGraph.

	@see PxVehicleDrivableSurfaceToTireFrictionPairs, PxVehicleComputeTireForce
	
	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxReal mFrictionVsSlipGraph[3][2];

	/**
	\brief Tire type denoting slicks, wets, snow, winter, summer, all-terrain, mud etc.
	
	@see PxVehicleDrivableSurfaceToTireFrictionPairs
	
	<b>Range:</b> [0, PX_MAX_F32)<br>
	*/
	PxU32 mType;

	/**
	\brief Return Cached value of 1.0/mLongitudinalStiffnessPerUnitGravity
	
	@see PxVehicleWheelsSimData::setTireData
	*/
	PX_FORCE_INLINE PxReal getRecipLongitudinalStiffnessPerUnitGravity() const {return mRecipLongitudinalStiffnessPerUnitGravity;}

	/**
	\brief Return Cached value of 1.0f/(mFrictionVsSlipGraph[1][0]-mFrictionVsSlipGraph[0][0])
	
	@see PxVehicleWheelsSimData::setTireData
	*/
	PX_FORCE_INLINE PxReal getFrictionVsSlipGraphRecipx1Minusx0() const {return mFrictionVsSlipGraphRecipx1Minusx0;}

	/**
	\brief Return Cached value of 1.0f/(mFrictionVsSlipGraph[2][0]-mFrictionVsSlipGraph[1][0])
	
	@see PxVehicleWheelsSimData::setTireData
	*/
	PX_FORCE_INLINE PxReal getFrictionVsSlipGraphRecipx2Minusx1() const {return mFrictionVsSlipGraphRecipx2Minusx1;}

private:

	/**
	\brief Cached value of 1.0/mLongitudinalStiffnessPerUnitGravity.
	
	\note Not necessary to set this value because it is set by PxVehicleWheelsSimData::setTireData

	@see PxVehicleWheelsSimData::setTireData
	*/
	PxReal mRecipLongitudinalStiffnessPerUnitGravity;

	/**
	\brief Cached value of 1.0f/(mFrictionVsSlipGraph[1][0]-mFrictionVsSlipGraph[0][0])
	
	\note Not necessary to set this value because it is set by PxVehicleWheelsSimData::setTireData
	
	@see PxVehicleWheelsSimData::setTireData
	*/
	PxReal mFrictionVsSlipGraphRecipx1Minusx0;

	/**
	\brief Cached value of 1.0f/(mFrictionVsSlipGraph[2][0]-mFrictionVsSlipGraph[1][0])
	
	\note Not necessary to set this value because it is set by PxVehicleWheelsSimData::setTireData

	@see PxVehicleWheelsSimData::setTireData
	*/
	PxReal mFrictionVsSlipGraphRecipx2Minusx1;

	PxReal mPad[2];

	bool isValid() const;
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleTireData)& 0x0f));
#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif //PX_VEHICLE_CORE_COMPONENTS_H
