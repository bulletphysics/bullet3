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

#ifndef PX_VEHICLE_DRIVE_TANK_H
#define PX_VEHICLE_DRIVE_TANK_H
/** \addtogroup vehicle
  @{
*/

#include "vehicle/PxVehicleDrive.h"
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
\brief The ordering of the wheels of a PxVehicleDriveTank.

@see PxVehicleWheelsSimData, PxVehicleWheelsDynData
*/
struct PxVehicleDriveTankWheelOrder
{
	enum Enum
	{
		eFRONT_LEFT=0,
		eFRONT_RIGHT,
		e1ST_FROM_FRONT_LEFT,
		e1ST_FROM_FRONT_RIGHT,
		e2ND_FROM_FRONT_LEFT,
		e2ND_FROM_FRONT_RIGHT,
		e3RD_FROM_FRONT_LEFT,
		e3RD_FROM_FRONT_RIGHT,
		e4TH_FROM_FRONT_LEFT,
		e4TH_FROM_FRONT_RIGHT,
		e5TH_FROM_FRONT_LEFT,
		e5TH_FROM_FRONT_RIGHT,
		e6TH_FROM_FRONT_LEFT,
		e6TH_FROM_FRONT_RIGHT,
		e7TH_FROM_FRONT_LEFT,
		e7TH_FROM_FRONT_RIGHT,
		e8TH_FROM_FRONT_LEFT,
		e8TH_FROM_FRONT_RIGHT,
		e9TH_FROM_FRONT_LEFT,
		e9TH_FROM_FRONT_RIGHT
	};
};


/**
\brief The control inputs for a PxVehicleDriveTank.

\note The values of eANALOG_INPUT_THRUST_LEFT and eANALOG_INPUT_THRUST_RIGHT determine how much 
of the total available drive torque is diverted to the left and right wheels.  These entries in the 
enumerated list represent the state of the left and right control sticks of a tank. The total available 
drive torque available is controlled by eANALOG_INPUT_ACCEL, which represents the state of the acceleration
pedal and controls how much torque will be applied to the engine.  

\note To accelerate forwards eANALOG_INPUT_ACCEL must be greater than zero so that torque is applied to drive the 
engine, while eANALOG_INPUT_THRUST_LEFT and eANALOG_INPUT_THRUST_RIGHT must also be greater than zero
to divert the available drive torque to the left and wheels. If eANALOG_INPUT_THRUST_LEFT > eANALOG_INPUT_THRUST_RIGHT
the tank will turn to the right.  If eANALOG_INPUT_THRUST_RIGHT > eANALOG_INPUT_THRUST_LEFT
the tank will turn to the left.

@see PxVehicleDriveDynData::setAnalogInput, PxVehicleDriveDynData::getAnalogInput
*/

struct PxVehicleDriveTankControl
{
	enum Enum
	{
		eANALOG_INPUT_ACCEL=0,
		eANALOG_INPUT_BRAKE_LEFT,	
		eANALOG_INPUT_BRAKE_RIGHT,	
		eANALOG_INPUT_THRUST_LEFT,	
		eANALOG_INPUT_THRUST_RIGHT,	
		eMAX_NB_DRIVETANK_ANALOG_INPUTS
	};
};

/**
\brief Two driving models are supported.

\note If eSTANDARD is chosen the left and right wheels are always driven in the same direction.  If the tank is in 
a forward gear the left and right wheels will all be driven forwards, while in reverse gear the left and right wheels
will all be driven backwards. With eSTANDARD the legal range of left and right thrust is (0,1).

\note If eSPECIAL is chosen it is possible to drive the left and right wheels in different directions. 
With eSPECIAL the legal range of left and right thrust is (-1,1).  In forward(reverse) gear negative thrust values drive the wheels 
backwards(forwards), while positive thrust values drives the wheels forwards(backwards).  

\note A sharp left turn can be achieved in eSTANDARD mode by braking with the left wheels and thrusting forward with the 
right wheels. A smaller turning circle can theoretically be achieved in eSPECIAL mode by applying negative thrust to the left wheels and positive
thrust to the right wheels.

\note In both modes the legal ranges of acceleration and left/right brake are all (0,1).

@see PxVehicleDriveTank::setDriveModel
*/
struct PxVehicleDriveTankControlModel
{
	enum Enum
	{
		eSTANDARD=0,
		eSPECIAL
	};
};


/**
\brief Data structure with instanced dynamics data and configuration data of a tank.
*/
class PxVehicleDriveTank : public PxVehicleDrive
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
	\brief Allocate a PxVehicleTankDrive instance for a tank with nbWheels

	\param[in] nbWheels is the number of wheels on the vehicle.

	\note It is assumed that all wheels are driven wheels.

	\return The instantiated vehicle.

	@see free, setup
	*/
	static PxVehicleDriveTank* allocate(const PxU32 nbWheels);

	/**
	\brief Deallocate a PxVehicleDriveTank instance.
	@see allocate
	*/
	void free();

	/**
	\brief Set up a tank using simulation data for the wheels and drive model.
	\param[in] physics is a PxPhysics instance that is needed to create special vehicle constraints that are maintained by the vehicle.
	\param[in] vehActor is a PxRigidDynamic instance that is used to represent the tank in the PhysX SDK.
	\param[in] wheelsData describes the configuration of all suspension/tires/wheels of the tank. The tank instance takes a copy of this data.
	\param[in] driveData describes the properties of the tank's drive model (gears/engine/clutch/autobox).  The tank instance takes a copy of this data.
	\param[in] nbDrivenWheels is the number of wheels on the tank.
	\note It is assumed that the first shapes of the actor are the wheel shapes, followed by the chassis shapes.  To break this assumption use PxVehicleWheelsSimData::setWheelShapeMapping.
	@see allocate, free, setToRestState, PxVehicleWheelsSimData::setWheelShapeMapping
	\note nbDrivenWheels must be an even number
	\note The wheels must be arranged according to PxVehicleDriveTankWheelOrder; that is, 
	the even wheels are on the left side of the tank and the odd wheels are on the right side of the tank. 
	*/
	void setup
		(PxPhysics* physics, PxRigidDynamic* vehActor, 
		 const PxVehicleWheelsSimData& wheelsData, const PxVehicleDriveSimData& driveData,
		 const PxU32 nbDrivenWheels);

	/**
	\brief Allocate and set up a tank using simulation data for the wheels and drive model.
	\param[in] physics is a PxPhysics instance that is needed to create special vehicle constraints that are maintained by the tank.
	\param[in] vehActor is a PxRigidDynamic instance that is used to represent the tank in the PhysX SDK.
	\param[in] wheelsData describes the configuration of all suspension/tires/wheels of the tank. The tank instance takes a copy of this data.
	\param[in] driveData describes the properties of the tank's drive model (gears/engine/clutch/differential/autobox).  The tank instance takes a copy of this data.
	\param[in] nbDrivenWheels is the number of wheels on the tank.
	\note It is assumed that the first shapes of the actor are the wheel shapes, followed by the chassis shapes.  To break this assumption use PxVehicleWheelsSimData::setWheelShapeMapping.
	\return The instantiated vehicle.
	@see allocate, free, setToRestState, PxVehicleWheelsSimData::setWheelShapeMapping
	*/
	static PxVehicleDriveTank* create
		(PxPhysics* physics, PxRigidDynamic* vehActor, 
		 const PxVehicleWheelsSimData& wheelsData, const PxVehicleDriveSimData& driveData,
		 const PxU32 nbDrivenWheels);

	/**
	\brief Set the control model used by the tank.
	\note eDRIVE_MODEL_STANDARD: turning achieved by braking on one side, accelerating on the other side.
	\note eDRIVE_MODEL_SPECIAL: turning achieved by accelerating forwards on one side, accelerating backwards on the other side.
	\note The default value is eDRIVE_MODEL_STANDARD
	*/
	void setDriveModel(const PxVehicleDriveTankControlModel::Enum driveModel)
	{
		mDriveModel=driveModel;
	}

	/**
	\brief Return the control model used by the tank.
	*/
	PxVehicleDriveTankControlModel::Enum getDriveModel() const {return mDriveModel;}

	/**
	\brief Set a vehicle to its rest state.  Aside from the rigid body transform, this will set the vehicle and rigid body 
	to the state they were in immediately after setup or create.
	\note Calling setToRestState invalidates the cached raycast hit planes under each wheel meaning that suspension line
	raycasts need to be performed at least once with PxVehicleSuspensionRaycasts before calling PxVehicleUpdates. 
	@see setup, create, PxVehicleSuspensionRaycasts, PxVehicleUpdates
	*/
	void setToRestState();

	/**
	\brief Simulation data that models vehicle components
	@see setup, create
	*/
	PxVehicleDriveSimData mDriveSimData;

private:
	/**
	\brief Test if the instanced dynamics and configuration data has legal values.
	*/
	bool isValid() const;

	/**
	\brief Drive model
	@see setDriveModel, getDriveModel, PxVehicleDriveTankControlModel
	*/
	PxVehicleDriveTankControlModel::Enum mDriveModel;

	PxU32 mPad[3];

//serialization
public:
											PxVehicleDriveTank(PxBaseFlags baseFlags) : PxVehicleDrive(baseFlags)			{}
	static		PxVehicleDriveTank*			createObject(PxU8*& address, PxDeserializationContext& context);
	static		void						getBinaryMetaData(PxOutputStream& stream);
	virtual		const char*					getConcreteTypeName()		const	{	return "PxVehicleDriveTank";	}
	virtual		bool						isKindOf(const char* name)	const	{	return !::strcmp("PxVehicleDriveTank", name) || PxBase::isKindOf(name); }
protected:
											PxVehicleDriveTank();
											~PxVehicleDriveTank(){}
//~serialization
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleDriveTank) & 15));

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif //PX_VEHICLE_DRIVE_TANK_H
