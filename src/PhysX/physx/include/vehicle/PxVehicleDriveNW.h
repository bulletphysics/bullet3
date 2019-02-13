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

#ifndef PX_VEHICLE_NWDRIVE_H
#define PX_VEHICLE_NWDRIVE_H
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
\brief Data structure describing configuration data of a vehicle with up to PX_MAX_NB_WHEELS driven equally through the differential. The vehicle has an
engine, clutch, gears, autobox, differential.
@see PxVehicleDriveSimData
*/
class PxVehicleDriveSimDataNW : public PxVehicleDriveSimData
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================
public:

	friend class PxVehicleDriveNW;

	PxVehicleDriveSimDataNW() 
		: PxVehicleDriveSimData()
	{
	}

	/**
	\brief Return the data describing the differential of a vehicle with up to PX_MAX_NB_WHEELS driven wheels.
	*/
	const PxVehicleDifferentialNWData& getDiffData()  const
	{
		return mDiff;
	}

	/**
	\brief Set the data describing the differential of a vehicle with up to PX_MAX_NB_WHEELS driven wheels.
	The differential data describes the set of wheels that are driven by the differential.
	*/
	void setDiffData(const PxVehicleDifferentialNWData& diff);

private:

	/**
	\brief Differential simulation data
	@see setDiffData, getDiffData
	*/
	PxVehicleDifferentialNWData mDiff;

	/**
	\brief Test if the NW-drive simulation data has been setup with legal data.
	Call only after setting all components.
	@see setEngineData, setClutchData, setGearsData, setAutoboxData, setDiffData, setAckermannGeometryData 
	*/
	bool isValid() const;

//serialization
public:
	PxVehicleDriveSimDataNW(const PxEMPTY) : PxVehicleDriveSimData(PxEmpty), mDiff(PxEmpty) {}
	static void getBinaryMetaData(PxOutputStream& stream);
//~serialization
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleDriveSimDataNW) & 15));


/**
\brief The control inputs for a PxVehicleDriveNW.

@see PxVehicleDriveDynData::setAnalogInput, PxVehicleDriveDynData::getAnalogInput
*/
struct PxVehicleDriveNWControl
{
	enum Enum
	{
		eANALOG_INPUT_ACCEL=0,
		eANALOG_INPUT_BRAKE,		
		eANALOG_INPUT_HANDBRAKE,	
		eANALOG_INPUT_STEER_LEFT,	
		eANALOG_INPUT_STEER_RIGHT,	
		eMAX_NB_DRIVENW_ANALOG_INPUTS
	};
};

/**
\brief Data structure with instanced dynamics data and configuration data of a vehicle with up to PX_MAX_NB_WHEELS driven wheels.
*/
class PxVehicleDriveNW : public PxVehicleDrive
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
	\brief Allocate a PxVehicleDriveNW instance for a NWDrive vehicle with nbWheels 

	\param[in] nbWheels is the number of wheels on the vehicle.

	\return The instantiated vehicle.

	@see free, setup
	*/
	static PxVehicleDriveNW* allocate(const PxU32 nbWheels);

	/**
	\brief Deallocate a PxVehicleDriveNW instance.
	@see allocate
	*/
	void free();

	/**
	\brief Set up a vehicle using simulation data for the wheels and drive model.
	\param[in] physics is a PxPhysics instance that is needed to create special vehicle constraints that are maintained by the vehicle.
	\param[in] vehActor is a PxRigidDynamic instance that is used to represent the vehicle in the PhysX SDK.
	\param[in] wheelsData describes the configuration of all suspension/tires/wheels of the vehicle. The vehicle instance takes a copy of this data.
	\param[in] driveData describes the properties of the vehicle's drive model (gears/engine/clutch/differential/autobox).  The vehicle instance takes a copy of this data.
	\param[in] nbWheels is the number of wheels on the vehicle.
	\note It is assumed that the first shapes of the actor are the wheel shapes, followed by the chassis shapes.  To break this assumption use PxVehicleWheelsSimData::setWheelShapeMapping.
	@see allocate, free, setToRestState, PxVehicleWheelsSimData::setWheelShapeMapping
	*/
	void setup
		(PxPhysics* physics, PxRigidDynamic* vehActor,
		 const PxVehicleWheelsSimData& wheelsData, const PxVehicleDriveSimDataNW& driveData,
		 const PxU32 nbWheels);

	/**
	\brief Allocate and set up a vehicle using simulation data for the wheels and drive model.
	\param[in] physics is a PxPhysics instance that is needed to create special vehicle constraints that are maintained by the vehicle.
	\param[in] vehActor is a PxRigidDynamic instance that is used to represent the vehicle in the PhysX SDK.
	\param[in] wheelsData describes the configuration of all suspension/tires/wheels of the vehicle. The vehicle instance takes a copy of this data.
	\param[in] driveData describes the properties of the vehicle's drive model (gears/engine/clutch/differential/autobox).  The vehicle instance takes a copy of this data.
	\param[in] nbWheels is the number of wheels on the vehicle.
	\note It is assumed that the first shapes of the actor are the wheel shapes, followed by the chassis shapes.  To break this assumption use PxVehicleWheelsSimData::setWheelShapeMapping.
	\return The instantiated vehicle.
	@see allocate, free, setToRestState, PxVehicleWheelsSimData::setWheelShapeMapping
	*/
	static PxVehicleDriveNW* create
		(PxPhysics* physics, PxRigidDynamic* vehActor,
		 const PxVehicleWheelsSimData& wheelsData, const PxVehicleDriveSimDataNW& driveData,
		 const PxU32 nbWheels);

	/**
	\brief Set a vehicle to its rest state.  Aside from the rigid body transform, this will set the vehicle and rigid body 
	to the state they were in immediately after setup or create.
	\note Calling setToRestState invalidates the cached raycast hit planes under each wheel meaning that suspension line
	raycasts need to be performed at least once with PxVehicleSuspensionRaycasts before calling PxVehicleUpdates. 
	@see setup, create, PxVehicleSuspensionRaycasts, PxVehicleUpdates
	*/
	void setToRestState();

	/**
	\brief Simulation data that describes the configuration of the vehicle's drive model.
	@see setup, create
	*/
	PxVehicleDriveSimDataNW mDriveSimData;

private:

	/**
	\brief Test if the instanced dynamics and configuration data has legal values.
	*/
	bool isValid() const;

//serialization
public:
								PxVehicleDriveNW(PxBaseFlags baseFlags) : PxVehicleDrive(baseFlags), mDriveSimData(PxEmpty) {}
								PxVehicleDriveNW();
								~PxVehicleDriveNW(){}
	static	PxVehicleDriveNW*	createObject(PxU8*& address, PxDeserializationContext& context);
	static	void				getBinaryMetaData(PxOutputStream& stream);
	virtual	const char*			getConcreteTypeName() const			{ return "PxVehicleDriveNW";	}
	virtual	bool				isKindOf(const char* name)	const	{ return !::strcmp("PxVehicleDriveNW", name) || PxBase::isKindOf(name); }
//~serialization
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleDriveNW) & 15));



#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif //PX_VEHICLE_NWDRIVE_H
