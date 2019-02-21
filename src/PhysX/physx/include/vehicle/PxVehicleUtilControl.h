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

#ifndef PX_VEHICLE_CONTROL_H
#define PX_VEHICLE_CONTROL_H
/** \addtogroup vehicle
  @{
*/
#include "vehicle/PxVehicleSDK.h"
#include "vehicle/PxVehicleDrive4W.h"
#include "vehicle/PxVehicleDriveNW.h"
#include "vehicle/PxVehicleDriveTank.h"


#if !PX_DOXYGEN
namespace physx
{
#endif

#if PX_CHECKED
	void testValidAnalogValue(const PxF32 actualValue, const PxF32 minVal, const PxF32 maxVal, const char* errorString);
#endif

/**
\brief Used to produce smooth vehicle driving control values from key inputs.
@see PxVehicle4WSmoothDigitalRawInputsAndSetAnalogInputs, PxVehicle4WSmoothAnalogRawInputsAndSetAnalogInputs
*/
struct PxVehicleKeySmoothingData
{
public:

	/**
	\brief Rise rate of each analog value if digital value is 1
	*/
	PxReal mRiseRates[PxVehicleDriveDynData::eMAX_NB_ANALOG_INPUTS];

	/**
	\brief Fall rate of each analog value if digital value is 0
	*/
	PxReal mFallRates[PxVehicleDriveDynData::eMAX_NB_ANALOG_INPUTS];
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleKeySmoothingData)& 0x0f));

/**
\brief Used to produce smooth analog vehicle control values from analog inputs.
@see PxVehicleDrive4WSmoothDigitalRawInputsAndSetAnalogInputs, PxVehicleDrive4WSmoothAnalogRawInputsAndSetAnalogInputs
*/
struct PxVehiclePadSmoothingData
{
public:

	/**
	\brief Rise rate of each analog value from previous value towards target if target>previous
	*/
	PxReal mRiseRates[PxVehicleDriveDynData::eMAX_NB_ANALOG_INPUTS];

	/**
	\brief Rise rate of each analog value from previous value towards target if target<previous
	*/
	PxReal mFallRates[PxVehicleDriveDynData::eMAX_NB_ANALOG_INPUTS];
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehiclePadSmoothingData)& 0x0f));

/**
\brief Used to produce smooth vehicle driving control values from analog and digital inputs.
@see PxVehicleDrive4WSmoothDigitalRawInputsAndSetAnalogInputs, PxVehicleDrive4WSmoothAnalogRawInputsAndSetAnalogInputs
*/
class PxVehicleDrive4WRawInputData
{
public:

	PxVehicleDrive4WRawInputData()
	{
		for(PxU32 i=0;i<PxVehicleDrive4WControl::eMAX_NB_DRIVE4W_ANALOG_INPUTS;i++)
		{
			mRawDigitalInputs[i]=false;
			mRawAnalogInputs[i]=0.0f;
		}

		mGearUp = false;
		mGearDown = false;
	}

	virtual ~PxVehicleDrive4WRawInputData()
	{
	}

	/**
	\brief Record if the accel button has been pressed on keyboard.
	\param[in] accelKeyPressed is true if the accelerator key has been pressed and false otherwise.
	*/
	void setDigitalAccel(const bool accelKeyPressed)			{mRawDigitalInputs[PxVehicleDrive4WControl::eANALOG_INPUT_ACCEL]=accelKeyPressed;}

	/**
	\brief Record if the brake button has been pressed on keyboard.
	\param[in] brakeKeyPressed is true if the brake key has been pressed and false otherwise.
	*/
	void setDigitalBrake(const bool brakeKeyPressed)			{mRawDigitalInputs[PxVehicleDrive4WControl::eANALOG_INPUT_BRAKE]=brakeKeyPressed;}

	/**
	\brief Record if the handbrake button has been pressed on keyboard.
	\param[in] handbrakeKeyPressed is true if the handbrake key has been pressed and false otherwise.
	*/
	void setDigitalHandbrake(const bool handbrakeKeyPressed)	{mRawDigitalInputs[PxVehicleDrive4WControl::eANALOG_INPUT_HANDBRAKE]=handbrakeKeyPressed;}

	/**
	\brief Record if the left steer button has been pressed on keyboard.
	\param[in] steerLeftKeyPressed is true if the steer-left key has been pressed and false otherwise.
	*/
	void setDigitalSteerLeft(const bool steerLeftKeyPressed)	{mRawDigitalInputs[PxVehicleDrive4WControl::eANALOG_INPUT_STEER_LEFT]=steerLeftKeyPressed;}

	/**
	\brief Record if the right steer button has been pressed on keyboard.
	\param[in] steerRightKeyPressed is true if the steer-right key has been pressed and false otherwise.
	*/
	void setDigitalSteerRight(const bool steerRightKeyPressed)	{mRawDigitalInputs[PxVehicleDrive4WControl::eANALOG_INPUT_STEER_RIGHT]=steerRightKeyPressed;}


	/**
	\brief Return if the accel button has been pressed on keyboard.
	\return True if the accel button has been pressed, false otherwise.
	*/
	bool getDigitalAccel() const								{return mRawDigitalInputs[PxVehicleDrive4WControl::eANALOG_INPUT_ACCEL];}

	/**
	\brief Return if the brake button has been pressed on keyboard.
	\return True if the brake button has been pressed, false otherwise.
	*/
	bool getDigitalBrake() const								{return mRawDigitalInputs[PxVehicleDrive4WControl::eANALOG_INPUT_BRAKE];}

	/**
	\brief Return if the handbrake button has been pressed on keyboard.
	\return True if the handbrake button has been pressed, false otherwise.
	*/
	bool getDigitalHandbrake() const							{return mRawDigitalInputs[PxVehicleDrive4WControl::eANALOG_INPUT_HANDBRAKE];}

	/**
	\brief Return if the left steer button has been pressed on keyboard.
	\return True if the steer-left button has been pressed, false otherwise.
	*/
	bool getDigitalSteerLeft() const							{return mRawDigitalInputs[PxVehicleDrive4WControl::eANALOG_INPUT_STEER_LEFT];}

	/**
	\brief Return if the right steer button has been pressed on keyboard.
	\return True if the steer-right button has been pressed, false otherwise.
	*/
	bool getDigitalSteerRight() const							{return mRawDigitalInputs[PxVehicleDrive4WControl::eANALOG_INPUT_STEER_RIGHT];}


	/**
	\brief Set the analog accel value from the gamepad
	\param[in] accel is the analog accelerator pedal value in range(0,1) where 1 represents the pedal fully pressed and 0 represents the pedal in its rest state.
	*/
	void setAnalogAccel(const PxReal accel)						
	{
#if PX_CHECKED
		testValidAnalogValue(accel, 0.0f, 1.0f, "Analog accel must be in range (0,1)");
#endif
		mRawAnalogInputs[PxVehicleDrive4WControl::eANALOG_INPUT_ACCEL]=accel;
	}

	/**
	\brief Set the analog brake value from the gamepad
	\param[in] brake is the analog brake pedal value in range(0,1) where 1 represents the pedal fully pressed and 0 represents the pedal in its rest state.
	*/
	void setAnalogBrake(const PxReal brake)						
	{
#if PX_CHECKED
		testValidAnalogValue(brake, 0.0f, 1.0f, "Analog brake must be in range (0,1)");
#endif
		mRawAnalogInputs[PxVehicleDrive4WControl::eANALOG_INPUT_BRAKE]=brake;
	}

	/**
	\brief Set the analog handbrake value from the gamepad
	\param[in] handbrake is the analog handbrake value in range(0,1) where 1 represents the handbrake fully engaged and 0 represents the handbrake in its rest state.
	*/
	void setAnalogHandbrake(const PxReal handbrake)				
	{
#if PX_CHECKED
		testValidAnalogValue(handbrake, 0.0f, 1.0f, "Analog handbrake must be in range (0,1)");
#endif
		mRawAnalogInputs[PxVehicleDrive4WControl::eANALOG_INPUT_HANDBRAKE]=handbrake;
	}

	/**
	\brief Set the analog steer value from the gamepad
	\param[in] steer is the analog steer value in range(-1,1) where -1 represents the steering wheel at left lock and +1 represents the steering wheel at right lock.
	*/
	void setAnalogSteer(const PxReal steer)						
	{
#if PX_CHECKED
		testValidAnalogValue(steer, -1.0f, 1.0f, "Analog steer must be in range (-1,1)");
#endif
		mRawAnalogInputs[PxVehicleDrive4WControl::eANALOG_INPUT_STEER_RIGHT]=steer;
	}

	/**
	\brief Return the analog accel value from the gamepad
	\return The analog accel value.
	*/
	PxReal getAnalogAccel() const								{return mRawAnalogInputs[PxVehicleDrive4WControl::eANALOG_INPUT_ACCEL];}

	/**
	\brief Return the analog brake value from the gamepad
	\return The analog brake value.
	*/
	PxReal getAnalogBrake() const								{return mRawAnalogInputs[PxVehicleDrive4WControl::eANALOG_INPUT_BRAKE];}

	/**
	\brief Return the analog handbrake value from the gamepad
	\return The analog handbrake value.
	*/
	PxReal getAnalogHandbrake() const							{return mRawAnalogInputs[PxVehicleDrive4WControl::eANALOG_INPUT_HANDBRAKE];}

	/**
	\brief Return the analog steer value from the gamepad
	*/
	PxReal getAnalogSteer() const								{return mRawAnalogInputs[PxVehicleDrive4WControl::eANALOG_INPUT_STEER_RIGHT];}

	/**
	\brief Record if the gearup button has been pressed on keyboard or gamepad
	\param[in] gearUpKeyPressed is true if the gear-up button has been pressed, false otherwise.
	*/
	void setGearUp(const bool gearUpKeyPressed)					{mGearUp=gearUpKeyPressed;}

	/**
	\brief Record if the geardown button has been pressed on keyboard or gamepad
	\param[in] gearDownKeyPressed is true if the gear-down button has been pressed, false otherwise.
	*/
	void setGearDown(const bool gearDownKeyPressed)				{mGearDown=gearDownKeyPressed;}

	/**
	\brief Return if the gearup button has been pressed on keyboard or gamepad
	\return The value of the gear-up button.
	*/
	bool getGearUp() const										{return mGearUp;}

	/**
	\brief Record if the geardown button has been pressed on keyboard or gamepad
	\return The value of the gear-down button.
	*/
	bool getGearDown() const									{return mGearDown;}

private:

	bool mRawDigitalInputs[PxVehicleDrive4WControl::eMAX_NB_DRIVE4W_ANALOG_INPUTS];
	PxReal mRawAnalogInputs[PxVehicleDrive4WControl::eMAX_NB_DRIVE4W_ANALOG_INPUTS];

	bool mGearUp;
	bool mGearDown;
};

/**
\brief Used to smooth and set analog vehicle control values (accel,brake,handbrake,steer) from digital inputs (keyboard).
 Also used to set boolean gearup, geardown values.
 \param[in] keySmoothing describes the rise and fall rates of the corresponding analog values when keys are pressed on and off.
 \param[in] steerVsForwardSpeedTable is a table of maximum allowed steer versus forward vehicle speed.
 \param[in] rawInputData is the state of all digital inputs that control the vehicle.
 \param[in] timestep is the time that has passed since the last call to PxVehicleDrive4WSmoothDigitalRawInputsAndSetAnalogInputs
 \param[in] isVehicleInAir describes if the vehicle is in the air or on the ground and is used to decide whether or not to apply steerVsForwardSpeedTable.
 \param[in] focusVehicle is the vehicle that will be given analog and gearup/geardown control values arising from the digital inputs.
 */
void PxVehicleDrive4WSmoothDigitalRawInputsAndSetAnalogInputs
	(const PxVehicleKeySmoothingData& keySmoothing, const PxFixedSizeLookupTable<8>& steerVsForwardSpeedTable,
	 const PxVehicleDrive4WRawInputData& rawInputData, 
	 const PxReal timestep, 
	 const bool isVehicleInAir, 
	 PxVehicleDrive4W& focusVehicle);

/**
\brief Used to smooth and set analog vehicle control values from analog inputs (gamepad).
Also used to set boolean gearup, geardown values.
\param[in] padSmoothing describes how quickly the control values applied to the vehicle blend from the current vehicle values towards the raw analog values from the gamepad.
\param[in] steerVsForwardSpeedTable is a table of maximum allowed steer versus forward vehicle speed.
\param[in] rawInputData is the state of all gamepad analog inputs that will be used control the vehicle.
\param[in] timestep is the time that has passed since the last call to PxVehicleDrive4WSmoothDigitalRawInputsAndSetAnalogInputs
\param[in] isVehicleInAir describes if the vehicle is in the air or on the ground and is used to decide whether or not to apply steerVsForwardSpeedTable.
\param[in] focusVehicle is the vehicle that will be given analog control values arising from the gamepad inputs.
*/
void PxVehicleDrive4WSmoothAnalogRawInputsAndSetAnalogInputs
	(const PxVehiclePadSmoothingData& padSmoothing, const PxFixedSizeLookupTable<8>& steerVsForwardSpeedTable,
	 const PxVehicleDrive4WRawInputData& rawInputData, 
	 const PxReal timestep, 
	 const bool isVehicleInAir, 
	 PxVehicleDrive4W& focusVehicle);


/**
\brief Used to produce smooth vehicle driving control values from analog and digital inputs.
@see PxVehicleDriveNWSmoothDigitalRawInputsAndSetAnalogInputs, PxVehicleDriveNWSmoothAnalogRawInputsAndSetAnalogInputs
*/
class PxVehicleDriveNWRawInputData : public PxVehicleDrive4WRawInputData
{
public:

	PxVehicleDriveNWRawInputData() : PxVehicleDrive4WRawInputData(){}
	~PxVehicleDriveNWRawInputData(){}
};

/**
\brief Used to smooth and set analog vehicle control values (accel,brake,handbrake,steer) from digital inputs (keyboard).
 Also used to set boolean gearup, geardown values.
 \param[in] keySmoothing describes the rise and fall rates of the corresponding analog values when keys are pressed on and off.
 \param[in] steerVsForwardSpeedTable is a table of maximum allowed steer versus forward vehicle speed.
 \param[in] rawInputData is the state of all digital inputs that control the vehicle.
 \param[in] timestep is the time that has passed since the last call to PxVehicleDrive4WSmoothDigitalRawInputsAndSetAnalogInputs
 \param[in] isVehicleInAir describes if the vehicle is in the air or on the ground and is used to decide whether or not to apply steerVsForwardSpeedTable.
 \param[in] focusVehicle is the vehicle that will be given analog and gearup/geardown control values arising from the digital inputs.
*/
void PxVehicleDriveNWSmoothDigitalRawInputsAndSetAnalogInputs
	(const PxVehicleKeySmoothingData& keySmoothing, const PxFixedSizeLookupTable<8>& steerVsForwardSpeedTable,
	 const PxVehicleDriveNWRawInputData& rawInputData, 
	 const PxReal timestep, 
	 const bool isVehicleInAir, 
	 PxVehicleDriveNW& focusVehicle);

/**
\brief Used to smooth and set analog vehicle control values from analog inputs (gamepad).
Also used to set boolean gearup, geardown values.
\param[in] padSmoothing describes how quickly the control values applied to the vehicle blend from the current vehicle values towards the raw analog values from the gamepad.
\param[in] steerVsForwardSpeedTable is a table of maximum allowed steer versus forward vehicle speed.
\param[in] rawInputData is the state of all gamepad analog inputs that will be used control the vehicle.
\param[in] timestep is the time that has passed since the last call to PxVehicleDrive4WSmoothDigitalRawInputsAndSetAnalogInputs
\param[in] isVehicleInAir describes if the vehicle is in the air or on the ground and is used to decide whether or not to apply steerVsForwardSpeedTable.
\param[in] focusVehicle is the vehicle that will be given analog control values arising from the gamepad inputs.
*/
void PxVehicleDriveNWSmoothAnalogRawInputsAndSetAnalogInputs
	(const PxVehiclePadSmoothingData& padSmoothing, const PxFixedSizeLookupTable<8>& steerVsForwardSpeedTable,
	 const PxVehicleDriveNWRawInputData& rawInputData, 
	 const PxReal timestep, 
	 const bool isVehicleInAir, 
	 PxVehicleDriveNW& focusVehicle);


/**
\brief Used to produce smooth analog tank control values from analog and digital inputs.
@see PxVehicleDriveTankSmoothDigitalRawInputsAndSetAnalogInputs, PxVehicleDriveTankSmoothAnalogRawInputsAndSetAnalogInputs
*/
class PxVehicleDriveTankRawInputData
{
public:

	PxVehicleDriveTankRawInputData(const PxVehicleDriveTankControlModel::Enum mode)
		: mMode(mode)
	{
		for(PxU32 i=0;i<PxVehicleDriveTankControl::eMAX_NB_DRIVETANK_ANALOG_INPUTS;i++)
		{
			mRawAnalogInputs[i]=0.0f;
			mRawDigitalInputs[i]=false;
		}

		mGearUp=false;
		mGearDown=false;
	}

	~PxVehicleDriveTankRawInputData()
	{
	}

	/**
	\brief Return the drive model (eDRIVE_MODEL_SPECIAL or eDRIVE_MODEL_STANDARD)
	\return The chosen tank drive model.
	*/
	PxVehicleDriveTankControlModel::Enum getDriveModel() const
	{
		return mMode;
	}

	/**
	\brief Set if the accel button has been pressed on the keyboard
	\param[in] b is true if the digital accel button has been pressed, false otherwise.
	*/
	void setDigitalAccel(const bool b)					{mRawDigitalInputs[PxVehicleDriveTankControl::eANALOG_INPUT_ACCEL]=b;}

	/**
	\brief Set if the left thrust button has been pressed on the keyboard
	\param[in] b is true if the digital left thrust button has been pressed, false otherwise.
	*/
	void setDigitalLeftThrust(const bool b)				{mRawDigitalInputs[PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_LEFT]=b;}

	/**
	\brief Set if the right thrust button has been pressed on the keyboard
	\param[in] b is true if the digital right thrust button has been pressed, false otherwise.
	*/
	void setDigitalRightThrust(const bool b)			{mRawDigitalInputs[PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_RIGHT]=b;}

	/**
	\brief Set if the left brake button has been pressed on the keyboard
	\param[in] b is true if the digital left brake button has been pressed, false otherwise.
	*/
	void setDigitalLeftBrake(const bool b)				{mRawDigitalInputs[PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_LEFT]=b;}

	/**
	\brief Set if the right brake button has been pressed on the keyboard
	\param[in] b is true if the digital right brake button has been pressed, false otherwise.
	*/
	void setDigitalRightBrake(const bool b)				{mRawDigitalInputs[PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_RIGHT]=b;}

	/**
	\brief Return if the accel button has been pressed on the keyboard
	\return True if the accel button has been pressed, false otherwise.
	*/
	bool getDigitalAccel() const						{return mRawDigitalInputs[PxVehicleDriveTankControl::eANALOG_INPUT_ACCEL];}

	/**
	\brief Return if the left thrust button has been pressed on the keyboard
	\return True if the left thrust button has been pressed, false otherwise.
	*/
	bool getDigitalLeftThrust() const					{return mRawDigitalInputs[PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_LEFT];}

	/**
	\brief Return if the right thrust button has been pressed on the keyboard
	\return True if the right thrust button has been pressed, false otherwise.
	*/
	bool getDigitalRightThrust() const					{return mRawDigitalInputs[PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_RIGHT];}

	/**
	\brief Return if the left brake button has been pressed on the keyboard
	\return True if the left brake button has been pressed, false otherwise.
	*/
	bool getDigitalLeftBrake() const					{return mRawDigitalInputs[PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_LEFT];}

	/**
	\brief Return if the right brake button has been pressed on the keyboard
	\return True if the right brake button has been pressed, false otherwise.
	*/
	bool getDigitalRightBrake() const					{return mRawDigitalInputs[PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_RIGHT];}


	/**
	\brief Set the analog accel value from the gamepad	
	\param[in] accel is a value in range (0,1) where 1 represents the accelerator pedal fully pressed and 0 represents the pedal in its rest state.
	In range (0,1).
	*/
	void setAnalogAccel(const PxF32 accel)					
	{
#if PX_CHECKED
		testValidAnalogValue(accel, 0.0f, 1.0f, "Tank analog accel must be in range (-1,1)");
#endif
		mRawAnalogInputs[PxVehicleDriveTankControl::eANALOG_INPUT_ACCEL]=accel;
	}

	/**
	\brief Set the analog left thrust value from the gamepad
	\param[in] leftThrust represents the state of the left stick.
	\note In range (0,1) for standard mode (eSTANDARD), in range (-1,1) for special mode (eSPECIAL)
	*/
	void setAnalogLeftThrust(const PxF32 leftThrust)			
	{
#if PX_CHECKED
		if(mMode == PxVehicleDriveTankControlModel::eSPECIAL)
		{
			testValidAnalogValue(leftThrust, -1.0f, 1.0f, "Tank left thrust must be in range (-1,1) in eSPECIAL mode.");
		}
		else
		{
			testValidAnalogValue(leftThrust, 0.0f, 1.0f, "Tank left thrust must be in range (0,1) in eSTANDARD mode.");
		}
#endif
		mRawAnalogInputs[PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_LEFT]=leftThrust;
	}

	/**
	\brief Set the analog right thrust value from the gamepad
	\param[in] rightThrust represents the state of the right stick.	
	\note In range (0,1) for standard mode (eSTANDARD), in range (-1,1) for special mode (eSPECIAL)
	*/
	void setAnalogRightThrust(const PxF32 rightThrust)			
	{
#if PX_CHECKED
		if(mMode == PxVehicleDriveTankControlModel::eSPECIAL)
		{
			testValidAnalogValue(rightThrust, -1.0f, 1.0f, "Tank right thrust must be in range (-1,1) in eSPECIAL mode.");
		}
		else
		{
			testValidAnalogValue(rightThrust, 0.0f, 1.0f, "Tank right thrust must be in range (0,1) in eSTANDARD mode.");
		}
#endif
		mRawAnalogInputs[PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_RIGHT]=rightThrust;
	}

	/**
	\brief Set the analog left brake value from the gamepad	
	\param[in] leftBrake is a value in range (0,1) where 1 represents the left brake pedal fully pressed and 0 represents the left brake pedal in its rest state.
	\note In range (0,1).
	*/
	void setAnalogLeftBrake(const PxF32 leftBrake)			
	{
#if PX_CHECKED
		testValidAnalogValue(leftBrake, 0.0f, 1.0f, "Tank left brake must be in range (0,1).");
#endif
		mRawAnalogInputs[PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_LEFT]=leftBrake;
	}

	/**
	\brief Set the analog right brake value from the gamepad	
	\param[in] rightBrake is a value in range (0,1) where 1 represents the right brake pedal fully pressed and 0 represents the right brake pedal in its rest state.
	\note In range (0,1).
	*/
	void setAnalogRightBrake(const PxF32 rightBrake)			
	{
#if PX_CHECKED
		testValidAnalogValue(rightBrake, 0.0f, 1.0f, "Tank right brake must be in range (0,1).");
#endif
		mRawAnalogInputs[PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_RIGHT]=rightBrake;
	}

	/**
	\brief Return the analog accel value from the gamepad
	\return The analog accel value.
	*/
	PxF32 getAnalogAccel() const								
	{
		return mRawAnalogInputs[PxVehicleDriveTankControl::eANALOG_INPUT_ACCEL];
	}

	/**
	\brief Return the analog left thrust value from the gamepad
	\return The analog left thrust value.
	*/
	PxF32 getAnalogLeftThrust() const							
	{
		return mRawAnalogInputs[PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_LEFT];
	}

	/**
	\brief Return the analog right thrust value from the gamepad
	\return The analog right thrust value.
	*/
	PxF32 getAnalogRightThrust() const						
	{
		return mRawAnalogInputs[PxVehicleDriveTankControl::eANALOG_INPUT_THRUST_RIGHT];
	}

	/**
	\brief Return the analog left brake value from the gamepad
	\return The analog left brake value.
	*/
	PxF32 getAnalogLeftBrake() const							
	{
		return mRawAnalogInputs[PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_LEFT];
	}

	/**
	\brief Return the analog right brake value from the gamepad
	\return The analog right brake value.
	*/
	PxF32 getAnalogRightBrake() const							
	{
		return mRawAnalogInputs[PxVehicleDriveTankControl::eANALOG_INPUT_BRAKE_RIGHT];
	}

	/**
	\brief Record if the gear-up button has been pressed on keyboard or gamepad
	\param[in] gearUp is true if the gear-up button has been pressed, false otherwise.
	*/
	void setGearUp(const bool gearUp)					{mGearUp=gearUp;}

	/**
	\brief Record if the gear-down button has been pressed on keyboard or gamepad
	\param[in] gearDown is true if the gear-down button has been pressed, false otherwise.
	*/
	void setGearDown(const bool gearDown)				{mGearDown=gearDown;}

	/**
	\brief Return if the gear-up button has been pressed on keyboard or gamepad
	\return True if the gear-up button has been pressed, false otherwise.
	*/
	bool getGearUp() const								{return mGearUp;}

	/**
	\brief Return if the gear-down button has been pressed on keyboard or gamepad
	\return True if the gear-down button has been pressed, false otherwise.
	*/
	bool getGearDown() const							{return mGearDown;}

private:

	PxVehicleDriveTankControlModel::Enum mMode;

	PxReal mRawAnalogInputs[PxVehicleDriveTankControl::eMAX_NB_DRIVETANK_ANALOG_INPUTS];
	bool mRawDigitalInputs[PxVehicleDriveTankControl::eMAX_NB_DRIVETANK_ANALOG_INPUTS];

	bool mGearUp;
	bool mGearDown;
};

/**
\brief Used to smooth and set analog tank control values from digital inputs (keyboard).
Also used to set boolean gearup, geardown values.
\param[in] keySmoothing describes the rise and fall rates of the corresponding analog values when keys are pressed on and off.
\param[in] rawInputData is the state of all digital inputs that control the vehicle.
\param[in] timestep is the time that has passed since the last call to PxVehicleDrive4WSmoothDigitalRawInputsAndSetAnalogInputs
\param[in] focusVehicle is the vehicle that will be given analog and gearup/geardown control values arising from the digital inputs.
*/
void PxVehicleDriveTankSmoothDigitalRawInputsAndSetAnalogInputs
(const PxVehicleKeySmoothingData& keySmoothing, 
 const PxVehicleDriveTankRawInputData& rawInputData, 
 const PxReal timestep, 
 PxVehicleDriveTank& focusVehicle);


/**
\brief Used to smooth and set analog tank control values from analog inputs (gamepad).
Also used to set boolean gearup, geardown values.
\param[in] padSmoothing describes how quickly the control values applied to the vehicle blend from the current vehicle values towards the raw analog values from the gamepad.
\param[in] rawInputData is the state of all gamepad analog inputs that will be used control the vehicle.
\param[in] timestep is the time that has passed since the last call to PxVehicleDrive4WSmoothDigitalRawInputsAndSetAnalogInputs
\param[in] focusVehicle is the vehicle that will be given analog control values arising from the gamepad inputs.
*/
void PxVehicleDriveTankSmoothAnalogRawInputsAndSetAnalogInputs
(const PxVehiclePadSmoothingData& padSmoothing, 
 const PxVehicleDriveTankRawInputData& rawInputData, 
 const PxReal timestep, 
 PxVehicleDriveTank& focusVehicle);


#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif //PX_VEHICLE_CONTROL_H
