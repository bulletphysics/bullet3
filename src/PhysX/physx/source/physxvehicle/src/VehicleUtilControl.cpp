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

#include "PxVehicleUtilControl.h"
#include "PxVehicleDrive4W.h"
#include "PsFoundation.h"
#include "PsUtilities.h"
#include "CmPhysXCommon.h"

namespace physx
{

#if PX_CHECKED
void testValidAnalogValue(const PxF32 actualValue, const PxF32 minVal, const PxF32 maxVal, const char* errorString)
{
	const PxF32 tolerance = 1e-2f;
	PX_CHECK_MSG((actualValue > (minVal - tolerance)) && (actualValue < (maxVal + tolerance)), errorString);
}
#endif


PxF32 processDigitalValue
(const PxU32 inputType, 
 const PxVehicleKeySmoothingData& keySmoothing, const bool digitalValue,
 const PxF32 timestep, 
 const PxF32 analogVal)
{
	PxF32 newAnalogVal=analogVal;
	if(digitalValue)
	{
		newAnalogVal+=keySmoothing.mRiseRates[inputType]*timestep;
	}
	else
	{
		newAnalogVal-=keySmoothing.mFallRates[inputType]*timestep;
	}

	return PxClamp(newAnalogVal,0.0f,1.0f);
}

void PxVehicleDriveSmoothDigitalRawInputsAndSetAnalogInputs
(const PxVehicleKeySmoothingData& keySmoothing, const PxFixedSizeLookupTable<8>& steerVsForwardSpeedTable,
 const PxVehicleDrive4WRawInputData& rawInputData, 
 const PxF32 timestep, 
 const bool isVehicleInAir,
 const PxVehicleWheels& vehicle, PxVehicleDriveDynData& driveDynData)
{
	const bool gearup=rawInputData.getGearUp();
	const bool geardown=rawInputData.getGearDown();
	driveDynData.setGearDown(geardown);
	driveDynData.setGearUp(gearup);

	const PxF32 accel=processDigitalValue(PxVehicleDrive4WControl::eANALOG_INPUT_ACCEL,keySmoothing,rawInputData.getDigitalAccel(),timestep,driveDynData.getAnalogInput(PxVehicleDrive4WControl::eANALOG_INPUT_ACCEL));
	driveDynData.setAnalogInput(PxVehicleDrive4WControl::eANALOG_INPUT_ACCEL,accel);

	const PxF32 brake=processDigitalValue(PxVehicleDrive4WControl::eANALOG_INPUT_BRAKE,keySmoothing,rawInputData.getDigitalBrake(),timestep,driveDynData.getAnalogInput(PxVehicleDrive4WControl::eANALOG_INPUT_BRAKE));
	driveDynData.setAnalogInput(PxVehicleDrive4WControl::eANALOG_INPUT_BRAKE,brake);

	const PxF32 handbrake=processDigitalValue(PxVehicleDrive4WControl::eANALOG_INPUT_HANDBRAKE,keySmoothing,rawInputData.getDigitalHandbrake(),timestep,driveDynData.getAnalogInput(PxVehicleDrive4WControl::eANALOG_INPUT_HANDBRAKE));
	driveDynData.setAnalogInput(PxVehicleDrive4WControl::eANALOG_INPUT_HANDBRAKE,handbrake);

	PxF32 steerLeft=processDigitalValue(PxVehicleDrive4WControl::eANALOG_INPUT_STEER_LEFT,keySmoothing,rawInputData.getDigitalSteerLeft(),timestep,driveDynData.getAnalogInput(PxVehicleDrive4WControl::eANALOG_INPUT_STEER_LEFT));
	PxF32 steerRight=processDigitalValue(PxVehicleDrive4WControl::eANALOG_INPUT_STEER_RIGHT,keySmoothing,rawInputData.getDigitalSteerRight(),timestep,driveDynData.getAnalogInput(PxVehicleDrive4WControl::eANALOG_INPUT_STEER_RIGHT));
	const PxF32 vz=vehicle.computeForwardSpeed();
	const PxF32 vzAbs=PxAbs(vz);
	const PxF32 maxSteer=(isVehicleInAir ? 1.0f :steerVsForwardSpeedTable.getYVal(vzAbs));
	const PxF32 steer=PxAbs(steerRight-steerLeft);
	if(steer>maxSteer)
	{
		const PxF32 k=maxSteer/steer;
		steerLeft*=k;
		steerRight*=k;
	}
	driveDynData.setAnalogInput(PxVehicleDrive4WControl::eANALOG_INPUT_STEER_LEFT, steerLeft);
	driveDynData.setAnalogInput(PxVehicleDrive4WControl::eANALOG_INPUT_STEER_RIGHT, steerRight);
}

//////////////////////////////////

//process value in range(0,1)
PX_FORCE_INLINE PxF32 processPositiveAnalogValue
(const PxF32 riseRate, const PxF32 fallRate,
 const PxF32 currentVal, const PxF32 targetVal,
 const PxF32 timestep)
{
	PX_ASSERT(targetVal>=-0.01f && targetVal<=1.01f);
	PxF32 val;
	if(currentVal<targetVal)
	{
		val=currentVal + riseRate*timestep;
		val=PxMin(val,targetVal);
	}
	else 
	{
		val=currentVal - fallRate*timestep;
		val=PxMax(val,targetVal);
	}
	return val;
}

//process value in range(-1,1)
PX_FORCE_INLINE PxF32 processAnalogValue
(const PxF32 riseRate, const PxF32 fallRate,  
 const PxF32 currentVal, const PxF32 targetVal,
 const PxF32 timestep)
{
	PX_ASSERT(PxAbs(targetVal)<=1.01f);

	PxF32 val=0.0f;	// PT: the following code could leave that variable uninitialized!!!!!
	if(0==targetVal)
	{
		//Drift slowly back to zero 
		if(currentVal>0)
		{
			val=currentVal-fallRate*timestep;
			val=PxMax(val,0.0f);
		}
		else if(currentVal<0)
		{
			val=currentVal+fallRate*timestep;
			val=PxMin(val,0.0f);
		}
	}
	else
	{
		if(currentVal < targetVal)
		{
			if(currentVal<0)
			{
				val=currentVal + fallRate*timestep;
				val=PxMin(val,targetVal);
			}
			else
			{
				val=currentVal + riseRate*timestep;
				val=PxMin(val,targetVal);
			}
		}
		else 
		{
			if(currentVal>0)
			{
				val=currentVal - fallRate*timestep;
				val=PxMax(val,targetVal);
			}
			else
			{
				val=currentVal - riseRate*timestep;
				val=PxMax(val,targetVal);
			}
		}	
	}
	return val;
}

void PxVehicleDriveSmoothAnalogRawInputsAndSetAnalogInputs
(const PxVehiclePadSmoothingData& padSmoothing, const PxFixedSizeLookupTable<8>& steerVsForwardSpeedTable,
 const PxVehicleDrive4WRawInputData& rawInputData, 
 const PxF32 timestep, 
 const bool isVehicleInAir,
 const PxVehicleWheels& vehicle, PxVehicleDriveDynData& driveDynData)
{
	//gearup/geardown
	const bool gearup=rawInputData.getGearUp();
	const bool geardown=rawInputData.getGearDown();
	driveDynData.setGearUp(gearup);
	driveDynData.setGearDown(geardown);

	//Update analog inputs for focus vehicle.

	//Process the accel.
	{
		const PxF32 riseRate=padSmoothing.mRiseRates[PxVehicleDrive4WControl::eANALOG_INPUT_ACCEL];
		const PxF32 fallRate=padSmoothing.mFallRates[PxVehicleDrive4WControl::eANALOG_INPUT_ACCEL];
		const PxF32 currentVal=driveDynData.getAnalogInput(PxVehicleDrive4WControl::eANALOG_INPUT_ACCEL);
		const PxF32 targetVal=rawInputData.getAnalogAccel();
		const PxF32 accel=processPositiveAnalogValue(riseRate,fallRate,currentVal,targetVal,timestep);
		driveDynData.setAnalogInput(PxVehicleDrive4WControl::eANALOG_INPUT_ACCEL, accel);
	}

	//Process the brake
	{
		const PxF32 riseRate=padSmoothing.mRiseRates[PxVehicleDrive4WControl::eANALOG_INPUT_BRAKE];
		const PxF32 fallRate=padSmoothing.mFallRates[PxVehicleDrive4WControl::eANALOG_INPUT_BRAKE];
		const PxF32 currentVal=driveDynData.getAnalogInput(PxVehicleDrive4WControl::eANALOG_INPUT_BRAKE);
		const PxF32 targetVal=rawInputData.getAnalogBrake();
		const PxF32 brake=processPositiveAnalogValue(riseRate,fallRate,currentVal,targetVal,timestep);
		driveDynData.setAnalogInput(PxVehicleDrive4WControl::eANALOG_INPUT_BRAKE, brake);
	}

	//Process the handbrake.
	{
		const PxF32 riseRate=padSmoothing.mRiseRates[PxVehicleDrive4WControl::eANALOG_INPUT_HANDBRAKE];
		const PxF32 fallRate=padSmoothing.mFallRates[PxVehicleDrive4WControl::eANALOG_INPUT_HANDBRAKE];
		const PxF32 currentVal=driveDynData.getAnalogInput(PxVehicleDrive4WControl::eANALOG_INPUT_HANDBRAKE);
		const PxF32 targetVal=rawInputData.getAnalogHandbrake();
		const PxF32 handbrake=processPositiveAnalogValue(riseRate,fallRate,currentVal,targetVal,timestep);
		driveDynData.setAnalogInput(PxVehicleDrive4WControl::eANALOG_INPUT_HANDBRAKE, handbrake);
	}

	//Process the steer
	{
		const PxF32 vz=vehicle.computeForwardSpeed();
		const PxF32 vzAbs=PxAbs(vz);
		const PxF32 riseRate=padSmoothing.mRiseRates[PxVehicleDrive4WControl::eANALOG_INPUT_STEER_RIGHT];
		const PxF32 fallRate=padSmoothing.mFallRates[PxVehicleDrive4WControl::eANALOG_INPUT_STEER_RIGHT];
		const PxF32 currentVal=driveDynData.getAnalogInput(PxVehicleDrive4WControl::eANALOG_INPUT_STEER_RIGHT)-driveDynData.getAnalogInput(PxVehicleDrive4WControl::eANALOG_INPUT_STEER_LEFT);
		const PxF32 targetVal=rawInputData.getAnalogSteer()*(isVehicleInAir ? 1.0f :steerVsForwardSpeedTable.getYVal(vzAbs));
		const PxF32 steer=processAnalogValue(riseRate,fallRate,currentVal,targetVal,timestep);
		driveDynData.setAnalogInput(PxVehicleDrive4WControl::eANALOG_INPUT_STEER_LEFT, 0.0f);
		driveDynData.setAnalogInput(PxVehicleDrive4WControl::eANALOG_INPUT_STEER_RIGHT, steer);
	}
}


////////////////

void PxVehicleDrive4WSmoothDigitalRawInputsAndSetAnalogInputs
(const PxVehicleKeySmoothingData& keySmoothing, const PxFixedSizeLookupTable<8>& steerVsForwardSpeedTable,
 const PxVehicleDrive4WRawInputData& rawInputData, 
 const PxF32 timestep, 
 const bool isVehicleInAir,
 PxVehicleDrive4W& focusVehicle)
{
	PxVehicleDriveSmoothDigitalRawInputsAndSetAnalogInputs
		(keySmoothing, steerVsForwardSpeedTable, rawInputData, timestep, isVehicleInAir, focusVehicle, focusVehicle.mDriveDynData);
}

void PxVehicleDrive4WSmoothAnalogRawInputsAndSetAnalogInputs
(const PxVehiclePadSmoothingData& padSmoothing, const PxFixedSizeLookupTable<8>& steerVsForwardSpeedTable,
 const PxVehicleDrive4WRawInputData& rawInputData, 
 const PxF32 timestep, 
 const bool isVehicleInAir,
 PxVehicleDrive4W& focusVehicle)
{
	PxVehicleDriveSmoothAnalogRawInputsAndSetAnalogInputs
		(padSmoothing,steerVsForwardSpeedTable,rawInputData,timestep,isVehicleInAir,focusVehicle,focusVehicle.mDriveDynData);
}

////////////////

void PxVehicleDriveNWSmoothDigitalRawInputsAndSetAnalogInputs
(const PxVehicleKeySmoothingData& keySmoothing, const PxFixedSizeLookupTable<8>& steerVsForwardSpeedTable,
 const PxVehicleDriveNWRawInputData& rawInputData, 
 const PxReal timestep, 
 const bool isVehicleInAir, 
 PxVehicleDriveNW& focusVehicle)
{
	PxVehicleDriveSmoothDigitalRawInputsAndSetAnalogInputs
		(keySmoothing,steerVsForwardSpeedTable,rawInputData,timestep,isVehicleInAir,focusVehicle,focusVehicle.mDriveDynData);
}

void PxVehicleDriveNWSmoothAnalogRawInputsAndSetAnalogInputs
(const PxVehiclePadSmoothingData& padSmoothing, const PxFixedSizeLookupTable<8>& steerVsForwardSpeedTable,
 const PxVehicleDriveNWRawInputData& rawInputData, 
 const PxReal timestep, 
 const bool isVehicleInAir, 
 PxVehicleDriveNW& focusVehicle)
{
	PxVehicleDriveSmoothAnalogRawInputsAndSetAnalogInputs
		(padSmoothing,steerVsForwardSpeedTable,rawInputData,timestep,isVehicleInAir,focusVehicle,focusVehicle.mDriveDynData);
}

////////////////

void PxVehicleDriveTankSmoothAnalogRawInputsAndSetAnalogInputs
(const PxVehiclePadSmoothingData& padSmoothing, 
 const PxVehicleDriveTankRawInputData& rawInputData, 
 const PxReal timestep, 
 PxVehicleDriveTank& focusVehicle)
{
	//Process the gearup/geardown buttons.
	const bool gearup=rawInputData.getGearUp();
	const bool geardown=rawInputData.getGearDown();
	focusVehicle.mDriveDynData.setGearUp(gearup);
	focusVehicle.mDriveDynData.setGearDown(geardown);

	//Process the accel.
	{
		const PxF32 riseRate=padSmoothing.mRiseRates[PxVehicleDriveTankControl::eANALOG_INPUT_ACCEL];
		const PxF32 fallRate=padSmoothing.mFallRates[PxVehicleDriveTankControl::eANALOG_INPUT_ACCEL];
		const PxF32 currentVal=focusVehicle.mDriveDynData.getAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_ACCEL);
		const PxF32 targetVal=rawInputData.getAnalogAccel();
		const PxF32 accel=processPositiveAnalogValue(riseRate,fallRate,currentVal,targetVal,timestep);
		focusVehicle.mDriveDynData.setAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_ACCEL, accel);
	}

	PX_ASSERT(focusVehicle.getDriveModel()==rawInputData.getDriveModel());
	switch(rawInputData.getDriveModel())
	{
	case PxVehicleDriveTankControlModel::eSPECIAL:
		{
			//Process the left brake.
			{
				const PxF32 riseRate=padSmoothing.mRiseRates[PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_LEFT];
				const PxF32 fallRate=padSmoothing.mFallRates[PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_LEFT];
				const PxF32 currentVal=focusVehicle.mDriveDynData.getAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_LEFT);
				const PxF32 targetVal=rawInputData.getAnalogLeftBrake();
				const PxF32 accel=processPositiveAnalogValue(riseRate,fallRate,currentVal,targetVal,timestep);
				focusVehicle.mDriveDynData.setAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_LEFT, accel);
			}

			//Process the right brake.
			{
				const PxF32 riseRate=padSmoothing.mRiseRates[PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_RIGHT];
				const PxF32 fallRate=padSmoothing.mFallRates[PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_RIGHT];
				const PxF32 currentVal=focusVehicle.mDriveDynData.getAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_RIGHT);
				const PxF32 targetVal=rawInputData.getAnalogRightBrake();
				const PxF32 accel=processPositiveAnalogValue(riseRate,fallRate,currentVal,targetVal,timestep);
				focusVehicle.mDriveDynData.setAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_RIGHT, accel);
			}

			//Left thrust
			{
				const PxF32 riseRate=padSmoothing.mRiseRates[PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_LEFT];
				const PxF32 fallRate=padSmoothing.mFallRates[PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_LEFT];
				const PxF32 currentVal=focusVehicle.mDriveDynData.getAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_LEFT);
				const PxF32 targetVal=rawInputData.getAnalogLeftThrust();
				const PxF32 val=processAnalogValue(riseRate,fallRate,currentVal,targetVal,timestep);
				focusVehicle.mDriveDynData.setAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_LEFT, val);
			}

			//Right thrust
			{
				const PxF32 riseRate=padSmoothing.mRiseRates[PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_RIGHT];
				const PxF32 fallRate=padSmoothing.mFallRates[PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_RIGHT];
				const PxF32 currentVal=focusVehicle.mDriveDynData.getAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_RIGHT);
				const PxF32 targetVal=rawInputData.getAnalogRightThrust();
				const PxF32 val=processAnalogValue(riseRate,fallRate,currentVal,targetVal,timestep);
				focusVehicle.mDriveDynData.setAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_RIGHT, val);
			}
		}
		break;

	case PxVehicleDriveTankControlModel::eSTANDARD:
		{
			//Right thrust
			{
				const PxF32 riseRate=padSmoothing.mRiseRates[PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_RIGHT];
				const PxF32 fallRate=padSmoothing.mFallRates[PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_RIGHT];
				const PxF32 currentVal=focusVehicle.mDriveDynData.getAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_RIGHT)-focusVehicle.mDriveDynData.getAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_RIGHT);
				const PxF32 targetVal=rawInputData.getAnalogRightThrust()-rawInputData.getAnalogRightBrake();
				const PxF32 val=processAnalogValue(riseRate,fallRate,currentVal,targetVal,timestep);
				if(val>0)
				{
					focusVehicle.mDriveDynData.setAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_RIGHT, val);
					focusVehicle.mDriveDynData.setAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_RIGHT, 0.0f);
				}
				else
				{
					focusVehicle.mDriveDynData.setAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_RIGHT, 0.0f);
					focusVehicle.mDriveDynData.setAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_RIGHT, -val);
				}
			}

			//Left thrust
			{
				const PxF32 riseRate=padSmoothing.mRiseRates[PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_LEFT];
				const PxF32 fallRate=padSmoothing.mFallRates[PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_LEFT];
				const PxF32 currentVal=focusVehicle.mDriveDynData.getAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_LEFT)-focusVehicle.mDriveDynData.getAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_LEFT);
				const PxF32 targetVal=rawInputData.getAnalogLeftThrust()-rawInputData.getAnalogLeftBrake();
				const PxF32 val=processAnalogValue(riseRate,fallRate,currentVal,targetVal,timestep);
				if(val>0)
				{
					focusVehicle.mDriveDynData.setAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_LEFT, val);
					focusVehicle.mDriveDynData.setAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_LEFT, 0.0f);
				}
				else
				{
					focusVehicle.mDriveDynData.setAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_LEFT, 0.0f);
					focusVehicle.mDriveDynData.setAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_LEFT, -val);
				}
			}
		}
		break;
	}
}

void PxVehicleDriveTankSmoothDigitalRawInputsAndSetAnalogInputs
(const PxVehicleKeySmoothingData& keySmoothing, 
 const PxVehicleDriveTankRawInputData& rawInputData, 
 const PxF32 timestep, 
 PxVehicleDriveTank& focusVehicle)
{
	PxF32 val;
	val=processDigitalValue(PxVehicleDriveTankControl::eANALOG_INPUT_ACCEL,keySmoothing,rawInputData.getDigitalAccel(),timestep,focusVehicle.mDriveDynData.getAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_ACCEL));
	focusVehicle.mDriveDynData.setAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_ACCEL, val);
	val=processDigitalValue(PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_LEFT,keySmoothing,rawInputData.getDigitalLeftThrust(),timestep,focusVehicle.mDriveDynData.getAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_LEFT));
	focusVehicle.mDriveDynData.setAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_LEFT, val);
	val=processDigitalValue(PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_RIGHT,keySmoothing,rawInputData.getDigitalRightThrust(),timestep,focusVehicle.mDriveDynData.getAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_RIGHT));
	focusVehicle.mDriveDynData.setAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_RIGHT, val);
	val=processDigitalValue(PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_LEFT,keySmoothing,rawInputData.getDigitalLeftBrake(),timestep,focusVehicle.mDriveDynData.getAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_LEFT));
	focusVehicle.mDriveDynData.setAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_LEFT, val);
	val=processDigitalValue(PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_RIGHT,keySmoothing,rawInputData.getDigitalRightBrake(),timestep,focusVehicle.mDriveDynData.getAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_RIGHT));
	focusVehicle.mDriveDynData.setAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_RIGHT, val);

	//Update digital inputs for focus vehicle.
	focusVehicle.mDriveDynData.setGearUp(rawInputData.getGearUp());
	focusVehicle.mDriveDynData.setGearDown(rawInputData.getGearDown());

	switch(rawInputData.getDriveModel())
	{
	case PxVehicleDriveTankControlModel::eSPECIAL:
		{
			const PxF32 thrustL=focusVehicle.mDriveDynData.getAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_LEFT)-focusVehicle.mDriveDynData.getAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_LEFT);
			focusVehicle.mDriveDynData.setAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_LEFT, thrustL);
			focusVehicle.mDriveDynData.setAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_LEFT, 0.0f);

			const PxF32 thrustR=focusVehicle.mDriveDynData.getAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_RIGHT)-focusVehicle.mDriveDynData.getAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_RIGHT);
			focusVehicle.mDriveDynData.setAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_RIGHT, thrustR);
			focusVehicle.mDriveDynData.setAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_RIGHT, 0.0f);
		}
		break;
	case PxVehicleDriveTankControlModel::eSTANDARD:
		{
			const PxF32 thrustL=focusVehicle.mDriveDynData.getAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_LEFT)-focusVehicle.mDriveDynData.getAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_LEFT);
			if(thrustL>0)
			{
				focusVehicle.mDriveDynData.setAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_LEFT, thrustL);
				focusVehicle.mDriveDynData.setAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_LEFT, 0.0f);
			}
			else
			{
				focusVehicle.mDriveDynData.setAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_LEFT, 0.0f);
				focusVehicle.mDriveDynData.setAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_LEFT, -thrustL);
			}

			const PxF32 thrustR=focusVehicle.mDriveDynData.getAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_RIGHT)-focusVehicle.mDriveDynData.getAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_RIGHT);
			if(thrustR>0)
			{
				focusVehicle.mDriveDynData.setAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_RIGHT, thrustR);
				focusVehicle.mDriveDynData.setAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_RIGHT, 0.0f);
			}
			else
			{
				focusVehicle.mDriveDynData.setAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_RIGHT, 0.0f);
				focusVehicle.mDriveDynData.setAnalogInput(PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_RIGHT, -thrustR);
			}
		}
		break;
	}

}



} //physx

