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

#ifndef PX_VEHICLE_DRIVE_H
#define PX_VEHICLE_DRIVE_H
/** \addtogroup vehicle
  @{
*/

#include "vehicle/PxVehicleWheels.h"
#include "vehicle/PxVehicleComponents.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

struct PxFilterData;
class PxGeometry;
class PxPhysics;
class PxBatchQuery;
class PxVehicleDrivableSurfaceToTireFrictionPairs;
class PxShape;
class PxMaterial;
class PxRigidDynamic;

/**
\brief Data structure describing non-wheel configuration data of a vehicle that has engine, gears, clutch, and auto-box.
@see PxVehicleWheelsSimData for wheels configuration data.
*/
class PxVehicleDriveSimData
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================
public:

	friend class PxVehicleDriveTank;

	/**
	\brief Return the engine data
	*/
	PX_FORCE_INLINE const PxVehicleEngineData& getEngineData() const 
	{
		return mEngine;
	}
	
	/**
	\brief Set the engine data
	\param[in] engine - the data stored in engine is copied to the vehicle's engine.
	*/
	void setEngineData(const PxVehicleEngineData& engine);

	/**
	\brief Return the gears data
	*/
	PX_FORCE_INLINE const PxVehicleGearsData& getGearsData() const 
	{
		return mGears;
	}
	
	/**
	\brief Set the gears data
	\param[in] gears - the data stored in gears is copied to the vehicle's gears.
	*/
	void setGearsData(const PxVehicleGearsData& gears);

	/**
	\brief Return the clutch data
	*/
	PX_FORCE_INLINE const PxVehicleClutchData& getClutchData() const 
	{
		return mClutch;
	}
	
	/**
	\brief Set the clutch data
	\param[in] clutch - the data stored in clutch is copied to the vehicle's clutch.
	*/
	void setClutchData(const PxVehicleClutchData& clutch);

	/**
	\brief Return the autobox data
	*/
	PX_FORCE_INLINE const PxVehicleAutoBoxData& getAutoBoxData() const 
	{
		return mAutoBox;
	}

	/**
	\brief Set the autobox data
	\param[in] autobox - the data stored in autobox is copied to the vehicle's autobox.
	*/
	void setAutoBoxData(const PxVehicleAutoBoxData& autobox);

protected:
	/*
	\brief Engine simulation data
	@see setEngineData, getEngineData
	*/
	PxVehicleEngineData				mEngine;

	/*
	\brief Gear simulation data
	@see setGearsData, getGearsData
	*/
	PxVehicleGearsData				mGears;

	/*
	\brief Clutch simulation data
	@see setClutchData, getClutchData
	*/
	PxVehicleClutchData				mClutch;

	/*
	\brief Autobox simulation data
	@see setAutoboxData, getAutoboxData
	*/
	PxVehicleAutoBoxData			mAutoBox;

	/**
	\brief Test that a PxVehicleDriveSimData instance has been configured with legal data.
	Call only after setting all components with setEngineData,setGearsData,setClutchData,setAutoBoxData
	@see PxVehicleDrive4W::setup, PxVehicleDriveTank::setup
	*/
	bool isValid() const;


//serialization
public:
	PxVehicleDriveSimData() {}
	PxVehicleDriveSimData(const PxEMPTY) :  mEngine(PxEmpty), mGears(PxEmpty), mClutch(PxEmpty), mAutoBox(PxEmpty) {}
	static void getBinaryMetaData(PxOutputStream& stream);
//~serialization
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleDriveSimData) & 15));


/**
\brief Data structure with instanced dynamics data for vehicle with engine, clutch, gears, autobox
@see PxVehicleWheelsDynData for wheels dynamics data.
*/
class PxVehicleDriveDynData
{
public:
	
	enum
	{
		eMAX_NB_ANALOG_INPUTS=16
	};

	friend class PxVehicleDrive;

	/**
	\brief Set all dynamics data to zero to bring the vehicle to rest.
	*/
	void setToRestState();

	/**
	\brief Set an analog control value to drive the vehicle.
	\param[in] type describes the type of analog control being modified
	\param[in] analogVal is the new value of the specific analog control.
	@see PxVehicleDrive4WControl, PxVehicleDriveNWControl, PxVehicleDriveTankControl
	*/
	void setAnalogInput(const PxU32 type, const PxReal analogVal);

	/**
	\brief Get the analog control value that has been applied to the vehicle.
	\return The value of the specified analog control value.
	@see PxVehicleDrive4WControl, PxVehicleDriveNWControl, PxVehicleDriveTankControl
	*/
	PxReal getAnalogInput(const PxU32 type) const;

	/**
	\brief Inform the vehicle that the gear-up button has been pressed.

	\param[in] digitalVal is the state of the gear-up button.
	
	\note If digitalVal is true the vehicle will attempt to initiate a gear change at the next call to PxVehicleUpdates.

	\note The value of mGearUpPressed is not reset by PxVehicleUpdates
	*/
	void setGearUp(const bool digitalVal) 
	{
		mGearUpPressed = digitalVal;
	}

	/**
	\brief Set that the gear-down button has been pressed.

	\param[in] digitalVal is the state of the gear-down button.

	\note If digitalVal is true the vehicle will attempt to initiate a gear change at the next call to PxVehicleUpdates.

	\note The value of mGearDownPressed is not reset by PxVehicleUpdates
	*/
	void setGearDown(const bool digitalVal) 
	{
		mGearDownPressed = digitalVal;
	}

	/**
	\brief Check if the gear-up button has been pressed
	\return The state of the gear-up button.
	*/
	bool getGearUp() const 
	{
		return mGearUpPressed;
	}

	/**
	\brief Check if the gear-down button has been pressed
	\return The state of the gear-down button.
	*/
	bool getGearDown() const 
	{
		return mGearDownPressed;
	}

	/**
	\brief Set the flag that will be used to select auto-gears
	If useAutoGears is true the auto-box will be active.
	\param[in] useAutoGears is the active state of the auto-box.
	*/
	PX_FORCE_INLINE void setUseAutoGears(const bool useAutoGears)
	{
		mUseAutoGears=useAutoGears;
	}

	/**
	\brief Get the flag status that is used to select auto-gears
	\return The active status of the auto-box.
	*/
	PX_FORCE_INLINE bool getUseAutoGears() const
	{
		return mUseAutoGears;
	}

	/**
	\brief Toggle the auto-gears flag
	If useAutoGears is true the auto-box will be active.
	*/
	PX_FORCE_INLINE void toggleAutoGears() 
	{
		mUseAutoGears = !mUseAutoGears;
	}

	/**
	\brief Set the current gear.

	\param[in] currentGear is the vehicle's gear.

	\note If the target gear is different from the current gear the vehicle will 
	attempt to start a gear change from the current gear that has just been set 
	towards the target gear at the next call to PxVehicleUpdates.

	@see setTargetGear, PxVehicleGearsData
	*/
	PX_FORCE_INLINE void setCurrentGear(PxU32 currentGear) 
	{
		mCurrentGear = currentGear;
	}

	/**
	\brief Get the current gear.

	\return The vehicle's current gear.

	@see getTargetGear, PxVehicleGearsData
	*/
	PX_FORCE_INLINE PxU32 getCurrentGear() const
	{
		return mCurrentGear;
	}

	/**
	\brief Set the target gear.

	\param[in] targetGear is the vehicle's target gear.

	\note If the target gear is different from the current gear the vehicle will 
	attempt to start a gear change towards the target gear at the next call to 
	PxVehicleUpdates.

	@see PxVehicleGearsData
	*/
	PX_FORCE_INLINE void setTargetGear(PxU32 targetGear) 
	{
		mTargetGear = targetGear;
	}

	/**
	\brief Get the target gear.

	\return The vehicle's target gear.

	@see setTargetGear, PxVehicleGearsData
	*/
	PX_FORCE_INLINE PxU32 getTargetGear() const
	{
		return mTargetGear;
	}
	
	/**
	\brief Start a gear change to a target gear. 
	
	\param[in] targetGear is the gear the vehicle will begin a transition towards.

	\note The gear change will begin at the next call to PxVehicleUpadates.

	@see PxVehicleGearsData
	*/
	PX_FORCE_INLINE void startGearChange(const PxU32 targetGear)
	{
		mTargetGear=targetGear;
	}

	/**
	\brief Force an immediate gear change to a target gear

	\param[in] targetGear is the gear the vehicle will be given immediately.

	@see PxVehicleGearsData
	*/
	PX_FORCE_INLINE void forceGearChange(const PxU32 targetGear)
	{
		mTargetGear=targetGear;
		mCurrentGear=targetGear;
	}

	/**
	\brief Set the rotation speed of the engine (radians per second)

	\param[in] speed is the rotational speed (radians per second) to apply to the engine.
	*/
	PX_FORCE_INLINE void setEngineRotationSpeed(const PxF32 speed)
	{
		mEnginespeed = speed;
	}

	/**
	\brief Return the rotation speed of the engine (radians per second)

	\return The rotational speed (radians per second) of the engine.
	*/
	PX_FORCE_INLINE PxReal getEngineRotationSpeed() const
	{
		return mEnginespeed;
	}

	/**
	\brief Return the time that has passed since the current gear change was initiated.
	
	\return The time that has passed since the current gear change was initiated. 

	\note If no gear change is in process the gear switch time will be zero.

	@see PxVehicleGearsData.mSwitchTime
	*/
	PX_FORCE_INLINE PxReal getGearSwitchTime() const
	{
		return mGearSwitchTime;
	}

	/**
	\brief Return the time that has passed since the autobox last initiated a gear change.

	\return  The time that has passed since the autobox last initiated a gear change.

	@see PxVehicleAutoBoxData::setLatency, PxVehicleAutoBoxData::getLatency
	*/
	PX_FORCE_INLINE PxReal getAutoBoxSwitchTime() const
	{
		return mAutoBoxSwitchTime;
	}

	/**
	\brief All dynamic data values are public for fast access.
	*/


	/**
	\brief Analog control values used by vehicle simulation. 
	@see setAnalogInput, getAnalogInput, PxVehicleDrive4WControl, PxVehicleDriveNWControl, PxVehicleDriveTankControl
	*/
	PxReal mControlAnalogVals[eMAX_NB_ANALOG_INPUTS];

	/**
	\brief Auto-gear flag used by vehicle simulation.  Set true to enable the autobox, false to disable the autobox.
	@see setUseAutoGears, setUseAutoGears, toggleAutoGears, PxVehicleAutoBoxData
	*/
	bool mUseAutoGears;

	/**
	\brief Gear-up digital control value used by vehicle simulation.  
	
	\note If true a gear change will be initiated towards currentGear+1 (or to first gear if in reverse).

	@see setDigitalInput, getDigitalInput
	*/
	bool mGearUpPressed;

	/**
	\brief Gear-down digital control value used by vehicle simulation.  
	
	\note If true a gear change will be initiated towards currentGear-1 (or to reverse if in first).

	@see setDigitalInput, getDigitalInput
	*/
	bool mGearDownPressed;

	/**
	\brief Current gear 
	@see startGearChange, forceGearChange, getCurrentGear, PxVehicleGearsData
	*/
	PxU32 mCurrentGear;

	/**
	\brief Target gear (different from current gear if a gear change is underway) 
	@see startGearChange, forceGearChange, getTargetGear, PxVehicleGearsData
	*/
	PxU32 mTargetGear;

	/**
	\brief Rotation speed of engine
	@see setToRestState, getEngineRotationSpeed
	*/	
	PxReal mEnginespeed;

	/**
	\brief Reported time that has passed since gear change started.
	@see setToRestState, startGearChange, PxVehicleGearsData::mSwitchTime
	*/
	PxReal mGearSwitchTime;

	/**
	\brief Reported time that has passed since last autobox gearup/geardown decision.
	@see setToRestState, PxVehicleAutoBoxData::setLatency
	*/
	PxReal mAutoBoxSwitchTime;

private:
	PxU32 mPad[2];

	/**
	\brief Test that a PxVehicleDriveDynData instance has legal values.
	@see setToRestState
	*/
	bool isValid() const;

//serialization
public:
	PxVehicleDriveDynData();
	PxVehicleDriveDynData(const PxEMPTY)  {}
	PxU32 getNbAnalogInput() const { return eMAX_NB_ANALOG_INPUTS; }
	PX_FORCE_INLINE void setGearChange(const PxU32 gearChange) { mTargetGear= gearChange; }
	PX_FORCE_INLINE PxU32 getGearChange() const { return mTargetGear; }
	PX_FORCE_INLINE void setGearSwitchTime(const PxReal switchTime) { mGearSwitchTime = switchTime; }
	PX_FORCE_INLINE void setAutoBoxSwitchTime(const PxReal autoBoxSwitchTime) { mAutoBoxSwitchTime = autoBoxSwitchTime; }
//~serialization
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleDriveDynData) & 15));

/**
\brief A complete vehicle with instance dynamics data and configuration data for wheels and engine,clutch,gears,autobox.
@see PxVehicleDrive4W, PxVehicleDriveTank
*/
class PxVehicleDrive : public PxVehicleWheels
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================
public:

	friend class PxVehicleUpdate;

	/**
	\brief Dynamics data of vehicle instance.
	@see setup
	*/
	PxVehicleDriveDynData mDriveDynData;

protected:
	
	/**
	\brief Test that all instanced dynamics data and configuration data have legal values.
	*/
	bool isValid() const;

	/**
	\brief Set vehicle to rest.
	*/
	void setToRestState();

	/**
	@see PxVehicleDrive4W::allocate, PxVehicleDriveTank::allocate
	*/
	static PxU32 computeByteSize(const PxU32 numWheels);
	static PxU8* patchupPointers(const PxU32 nbWheels, PxVehicleDrive* vehDrive, PxU8* ptr);
	virtual void init(const PxU32 numWheels);

	/**
	\brief Deallocate a PxVehicle4WDrive instance.
	@see PxVehicleDrive4W::free, PxVehicleDriveTank::free
	*/
	void free();

	/**
	@see PxVehicleDrive4W::setup, PxVehicleDriveTank::setup
	*/
	void setup
		(PxPhysics* physics, PxRigidDynamic* vehActor, 
		 const PxVehicleWheelsSimData& wheelsData,
		 const PxU32 nbDrivenWheels, const PxU32 nbNonDrivenWheels);

//serialization
public:
	static void getBinaryMetaData(PxOutputStream& stream);
	PxVehicleDrive(PxBaseFlags baseFlags) : PxVehicleWheels(baseFlags), mDriveDynData(PxEmpty) {}
	virtual const char* getConcreteTypeName() const { return "PxVehicleDrive"; }
protected:
	PxVehicleDrive(PxType concreteType, PxBaseFlags baseFlags) : PxVehicleWheels(concreteType, baseFlags) {}	
	~PxVehicleDrive() {}
	virtual bool isKindOf(const char* name)	const { return !::strcmp("PxVehicleDrive", name) || PxBase::isKindOf(name); }
//~serialization
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleDrive) & 15));

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif //PX_VEHICLE_DRIVE_H
