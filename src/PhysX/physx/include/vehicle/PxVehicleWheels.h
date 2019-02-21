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


#ifndef PX_VEHICLE_WHEELS_H
#define PX_VEHICLE_WHEELS_H
/** \addtogroup vehicle
  @{
*/

#include "foundation/PxSimpleTypes.h"
#include "vehicle/PxVehicleShaders.h"
#include "vehicle/PxVehicleComponents.h"
#include "common/PxBase.h"
#include "PxRigidDynamic.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxVehicleWheels4SimData;
class PxVehicleWheels4DynData;
class PxVehicleTireForceCalculator;
class PxShape;
class PxPhysics;
class PxMaterial;

/**
\brief Data structure describing configuration data of a vehicle with up to 20 wheels.
*/

class PxVehicleWheelsSimData
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================
public:

	friend class PxVehicleWheels;
	friend class PxVehicleNoDrive;
	friend class PxVehicleDrive4W;
	friend class PxVehicleDriveTank;
	friend class PxVehicleUpdate;

	/**
	\brief Allocate a PxVehicleWheelsSimData instance for with nbWheels.
	@see free
	*/
	static PxVehicleWheelsSimData* allocate(const PxU32 nbWheels);

	/**
	\brief Setup with mass information that can be applied to the default values of the suspensions, wheels, and tires
	set in their respective constructors.

	\param chassisMass is the mass of the chassis.

	\note This function assumes that the suspensions equally share the load of the chassis mass.  It also
	assumes that the suspension will have a particular natural frequency and damping ratio that is typical
	of a standard car.  If either of these assumptions is broken then each suspension will need to 
	be individually configured with custom strength, damping rate, and sprung mass.

	@see allocate
	*/
	void setChassisMass(const PxF32 chassisMass);

	/**
	\brief Free a PxVehicleWheelsSimData instance 
	@see allocate
	*/
	void free();

	/**
	\brief Copy wheel simulation data.
	\note The number of wheels on both instances of PxVehicleWheelsSimData must match.
	*/
	PxVehicleWheelsSimData& operator=(const PxVehicleWheelsSimData& src);

	/**
	\brief Copy the data of a single wheel unit (wheel, suspension, tire) from srcWheel of src to trgWheel.
	\param[in] src is the data to be copied.
	\param[in] srcWheel is the wheel whose data will be copied from src.
	\param[in] trgWheel is the wheel that will be assigned the copied data.
	*/
	void copy(const PxVehicleWheelsSimData& src, const PxU32 srcWheel, const PxU32 trgWheel);

	/**
	\brief Return the number of wheels 
	@see allocate
	*/
	PxU32 getNbWheels() const {return mNbActiveWheels;}

	/**
	\brief Return the suspension data of the idth wheel
	*/
	const PxVehicleSuspensionData& getSuspensionData(const PxU32 id) const;
		
	/**
	\brief Return the wheel data of the idth wheel
	*/
	const PxVehicleWheelData& getWheelData(const PxU32 id) const;

	/**
	\brief Return the tire data of the idth wheel
	*/
	const PxVehicleTireData& getTireData(const PxU32 id) const;
	
	/**
	\brief Return the direction of travel of the suspension of the idth wheel
	*/
	const PxVec3& getSuspTravelDirection(const PxU32 id) const;

	/**
	\brief Return the application point of the suspension force of the suspension of the idth wheel as an offset from the rigid body center of mass.
	\note Specified relative to the center of mass of the rigid body
	*/
	const PxVec3& getSuspForceAppPointOffset(const PxU32 id) const;

	/**
	\brief Return the application point of the tire force of the tire of the idth wheel as an offset from the rigid body center of mass.
	\note Specified relative to the centre of mass of the rigid body
	*/
	const PxVec3& getTireForceAppPointOffset(const PxU32 id) const;
		
	/**
	\brief Return the offset from the rigid body centre of mass to the centre of the idth wheel.
	*/
	const PxVec3& getWheelCentreOffset(const PxU32 id) const;	

	/**
	\brief Return the wheel mapping for the ith wheel.  
	
	\note The return value is the element in the array of 
	shapes of the vehicle's PxRigidDynamic that corresponds to the ith wheel.  A return value of -1 means
	that the wheel is not mapped to a PxShape.

	@see PxRigidActor.getShapes
	*/
	PxI32 getWheelShapeMapping(const PxU32 wheelId) const;

	/**
	\brief Return the scene query filter data used by the specified suspension line
	*/
	const PxFilterData& getSceneQueryFilterData(const PxU32 suspId) const;

	/**
	\brief Return the number of unique anti-roll bars that have been added with addAntiRollBarData
	@see PxVehicleWheelsSimData::addAntiRollBarData
	*/
	PxU32 getNbAntiRollBars() const 
	{
		return mNbActiveAntiRollBars;
	}

	/**
	\brief Return a specific anti-roll bar.
	\param antiRollId is the unique id of the anti-roll bar
	\note The return value of addAntiRollBarData is a unique id for that specific anti-roll bar 
	and can be used as input parameter for getAntiRollBarData in order to query the same anti-roll bar.  
	Alternatively, it is possible to iterate over all anti-roll bars by choosing antiRollId 
	in range (0, getNbAntiRollBars()).
	*/
	const PxVehicleAntiRollBarData& getAntiRollBarData(const PxU32 antiRollId) const;
	
	/**
	\brief Return the data that describes the filtering of the tire load to produce smoother handling at large time-steps.
	*/
	PX_FORCE_INLINE const PxVehicleTireLoadFilterData& getTireLoadFilterData() const 
	{
		return mNormalisedLoadFilter;
	}

	/**
	\brief Set the suspension data of the idth wheel
	\param[in] id is the wheel index.
	\param[in] susp is the suspension data to be applied.
	*/
	void setSuspensionData(const PxU32 id, const PxVehicleSuspensionData& susp);

	/**
	\brief Set the wheel data of the idth wheel
	\param[in] id is the wheel index.
	\param[in] wheel is the wheel data to be applied.
	*/
	void setWheelData(const PxU32 id, const PxVehicleWheelData& wheel);

	/**
	\brief Set the tire data of the idth wheel
	\param[in] id is the wheel index.
	\param[in] tire is the tire data to be applied.
	*/
	void setTireData(const PxU32 id, const PxVehicleTireData& tire);

	/**
	\brief Set the direction of travel of the suspension of the idth wheel
	\param[in] id is the wheel index
	\param[in] dir is the suspension travel direction to be applied.
	*/
	void setSuspTravelDirection(const PxU32 id, const PxVec3& dir);
	
	/**
	\brief Set the application point of the suspension force of the suspension of the idth wheel.
	\param[in] id is the wheel index
	\param[in] offset is the offset from the rigid body center of mass to the application point of the suspension force.
	\note Specified relative to the centre of mass of the rigid body
	*/
	void setSuspForceAppPointOffset(const PxU32 id, const PxVec3& offset);									
	
	/**
	\brief Set the application point of the tire force of the tire of the idth wheel.
	\param[in] id is the wheel index
	\param[in] offset is the offset from the rigid body center of mass to the application point of the tire force.
	\note Specified relative to the centre of mass of the rigid body
	*/
	void setTireForceAppPointOffset(const PxU32 id, const PxVec3& offset);

	/**
	\brief Set the offset from the rigid body centre of mass to the centre of the idth wheel.
	\param[in] id is the wheel index
	\param[in] offset is the offset from the rigid body center of mass to the center of the wheel at rest.
	\note Specified relative to the centre of mass of the rigid body
	*/
	void setWheelCentreOffset(const PxU32 id, const PxVec3& offset);	

	/**
	\brief Set mapping between wheel id and position of corresponding wheel shape in the list of actor shapes.
	
	\note This mapping is used to pose the correct wheel shapes with the latest wheel rotation angle, steer angle, and suspension travel
	while allowing arbitrary ordering of the wheel shapes in the actor's list of shapes.
	
	\note Use setWheelShapeMapping(i,-1) to register that there is no wheel shape corresponding to the ith wheel
	
	\note Set setWheelShapeMapping(i,k) to register that the ith wheel corresponds to the kth shape in the actor's list of shapes.
	
	\note The default values correspond to setWheelShapeMapping(i,i) for all wheels.
	
	\note Calling this function will also pose the relevant PxShape at the rest position of the wheel.

	\param wheelId is the wheel index

	\param shapeId is the shape index.
	
	@see PxVehicleUpdates, PxVehicleDrive4W::setup, PxVehicleDriveTank::setup, PxVehicleNoDrive::setup, setSceneQueryFilterData, PxRigidActor::getShapes
	*/
	void setWheelShapeMapping(const PxU32 wheelId, const PxI32 shapeId);

	/**
	\brief Set the scene query filter data that will be used for raycasts along the travel
	direction of the specified suspension. The default value is PxFilterData(0,0,0,0)
	\param suspId is the wheel index
	\param sqFilterData is the raycast filter data for the suspension raycast.
	@see setWheelShapeMapping
	*/
	void setSceneQueryFilterData(const PxU32 suspId, const PxFilterData& sqFilterData);

	/**
	\brief Set the data that describes the filtering of the tire load to produce smoother handling at large timesteps.
	\param tireLoadFilter is the smoothing function data.
	*/
	void setTireLoadFilterData(const PxVehicleTireLoadFilterData& tireLoadFilter);

	/**
	\brief Set the anti-roll suspension for a pair of wheels.

	\param antiRoll is the anti-roll suspension.

	\note If an anti-roll bar has already been set for the same logical wheel pair 
	(independent of wheel index order specified by PxVehicleAntiRollBar.mWheel0 and PxVehicleAntiRollBar.mWheel0) 
	then the existing anti-roll bar is updated with a new stiffness parameter antiRoll.mStiffness.  

	\note If the wheel pair specified by antiRoll does not yet have an anti-roll bar then antiRoll is added to 
	a list of anti-roll bars for the vehicle.

	\return If antiRoll represents a new wheel pair then a unique id is assigned to the anti-roll bar and returned. 
	If antiRoll represents an existing wheel pair then the unique id of the existing anti-roll bar is returned.
	The return value is always in range (0, getNbAntiRollBars()).

	\note The return value can be used to query the anti-roll bar with getAntiRollBarData(id).

	\note The number of possible anti-roll bars is limited to half the wheel count.

	\note An existing anti-roll bar can be disabled by calling antiRoll.mStiffness to zero.

	@see PxVehicleWheelsSimData::getAntiRollBarData, PxVehicleAntiRollBarData
	*/
	PxU32 addAntiRollBarData(const PxVehicleAntiRollBarData& antiRoll);

	/**
	\brief Disable a wheel so that zero suspension forces and zero tire forces are applied to the rigid body from this wheel.

	\note If the vehicle has a differential (PxVehicleNW/PxVehicle4W) then the differential (PxVehicleDifferentialNWData/PxVehicleDifferential4WData)
	needs to be configured so that no drive torque is delivered to the disabled wheel.
	
	\note If the vehicle is of type PxVehicleNoDrive then zero drive torque must be applied to the disabled wheel.
	
	\note For tanks (PxVehicleDriveTank) any drive torque that could be delivered to the wheel through the tank differential will be 
	re-directed to the remaining enabled wheels.

	@see enableWheel
	@see PxVehicleDifferentialNWData::setDrivenWheel
	@see PxVehicleDifferential4WData::mFrontLeftRightSplit, PxVehicleDifferential4WData::mRearLeftRightSplit, PxVehicleDifferential4WData::mType
	@see PxVehicleNoDrive::setDriveTorque
	@see PxVehicle4WEnable3WTadpoleMode, PxVehicle4WEnable3WDeltaMode

	\note If a PxShape is associated with the disabled wheel then the association must be broken by calling setWheelShapeMapping(wheelId, -1). 
	@see setWheelShapeMapping

	\note A wheel that is disabled must also simultaneously be given zero wheel rotation speed.
	@see PxVehicleWheelsDynData::setWheelRotationSpeed

	\note Care must be taken with the sprung mass supported by the remaining enabled wheels.  Depending on the desired effect, the mass of the rigid body 
	might need to be distributed among the remaining enabled wheels and suspensions.

	\param[in] wheel is the wheel index.
	*/
	void disableWheel(const PxU32 wheel);

	/**
	\brief Enable a wheel so that suspension forces and tire forces are applied to the rigid body.
	All wheels are enabled by default and remain enabled until they are disabled.
	\param[in] wheel is the wheel index.
	@see disableWheel
	*/
	void enableWheel(const PxU32 wheel);

	/**
	\brief Test if a wheel has been disabled.
	\param[in] wheel is the wheel index.
	*/
	bool getIsWheelDisabled(const PxU32 wheel) const;

	/**
	\brief Set the number of vehicle sub-steps that will be performed when the vehicle's longitudinal 
	speed is below and above a threshold longitudinal speed.
	
	\note More sub-steps provides better stability but with greater computational cost.
	
	\note Typically, vehicles require more sub-steps at very low forward speeds.
	
	\note The threshold longitudinal speed has a default value that is the equivalent of 5 metres per second after accounting for 
	the length scale set in PxTolerancesScale.  

	\note The sub-step count below the threshold longitudinal speed has a default of 3.
	
	\note The sub-step count above the threshold longitudinal speed has a default of 1.
	
	\note Each sub-step has time advancement equal to the time-step passed to PxVehicleUpdates divided by the number of required sub-steps.
	
	\note The contact planes of the most recent suspension line raycast are reused across all sub-steps.
	
	\note Each sub-step computes tire and suspension forces and then advances a velocity, angular velocity and transform.
	
	\note At the end of all sub-steps the vehicle actor is given the velocity and angular velocity that would move the actor from its start transform prior
	to the first sub-step to the transform computed at the end of the last substep, assuming it doesn't collide with anything along the way in the next PhysX SDK update.
	
	\note The global pose of the actor is left unchanged throughout the sub-steps.

	\param[in] thresholdLongitudinalSpeed is a threshold speed that is used to categorize vehicle speed as low speed or high speed.
	\param[in] lowForwardSpeedSubStepCount is the number of sub-steps performed in PxVehicleUpates for vehicles that have longitudinal speed lower than thresholdLongitudinalSpeed.
	\param[in] highForwardSpeedSubStepCount is the number of sub-steps performed in PxVehicleUpdates for vehicles that have longitudinal speed graeter than thresholdLongitudinalSpeed.
	*/
	void setSubStepCount(const PxReal thresholdLongitudinalSpeed, const PxU32 lowForwardSpeedSubStepCount, const PxU32 highForwardSpeedSubStepCount);

	/**
	\brief Set the minimum denominator used in the longitudinal slip calculation.

	\note The longitudinal slip has a theoretical value of (w*r - vz)/|vz|, where w is the angular speed of the wheel; r is the radius of the wheel; 
	and vz is the component of rigid body velocity (computed at the wheel base) that lies along the longitudinal wheel direction. The term |vz|
	normalizes the slip, while preserving the sign of the longitudinal tire slip.   The difficulty here is that when |vz| approaches zero the 
	longitudinal slip approaches infinity. A solution to this problem is to replace the denominator (|vz|) with a value that never falls below a chosen threshold. 
	The longitudinal slip is then calculated with (w*r - vz)/PxMax(|vz|, minLongSlipDenominator).

	\note The default value is the equivalent of 4 metres per second after accounting for the length scale set in PxTolerancesScale.  

	\note Adjust this value upwards if a vehicle has difficulty coming to rest.

	\note Decreasing the timestep (or increasing the number of sub-steps at low longitudinal speed with setSubStepCount) should allow stable stable 
	behavior with smaller values of minLongSlipDenominator.
	*/
	void setMinLongSlipDenominator(const PxReal minLongSlipDenominator);

private:

	/**
	\brief Graph to filter normalised load
	@see setTireLoadFilterData, getTireLoadFilterData
	*/
	PxVehicleTireLoadFilterData mNormalisedLoadFilter;

	/**
	\brief Wheels data organised in blocks of 4 wheels.
	*/
	PxVehicleWheels4SimData* mWheels4SimData;

	/**
	\brief Number of blocks of 4 wheels.
	*/
	PxU32 mNbWheels4;

	/**
	\brief Number of actual wheels (<=(mNbWheels4*4))
	*/
	PxU32 mNbActiveWheels;

	/**
	\brief Anti-roll bars
	*/
	PxVehicleAntiRollBarData* mAntiRollBars;

	/**
	\brief 2 anti-rollbars allocated for each block of 4 wheels.
	*/
	PxU32 mNbAntiRollBars4;

	/**
	\brief Number of active anti-roll bars.
	*/
	PxU32 mNbActiveAntiRollBars;

	/**
	\brief Which of the mNbActiveWheels are active or disabled?
	The default is that all mNbActiveWheels wheels are active.
	*/
	PxU32 mActiveWheelsBitmapBuffer[((PX_MAX_NB_WHEELS + 31) & ~31) >> 5];

	/**
	\brief Threshold longitudinal speed used to decide whether to use 
	mLowForwardSpeedSubStepCount or mHighForwardSpeedSubStepCount as the 
	number of sub-steps that will be peformed.
	*/
	PxF32 mThresholdLongitudinalSpeed;

	/**
	\brief Number of sub-steps that will be performed if the longitudinal speed
	of the vehicle is smaller than mThresholdLongitudinalSpeed.
	*/
	PxU32 mLowForwardSpeedSubStepCount;

	/**
	\brief Number of sub-steps that will be performed if the longitudinal speed
	of the vehicle is greater than or equal to mThresholdLongitudinalSpeed.
	*/
	PxU32 mHighForwardSpeedSubStepCount;

	/**
	\brief Minimum long slip denominator
	*/
	PxF32 mMinLongSlipDenominator;

#if PX_P64_FAMILY
	PxU32 mPad[2];
#else 
	PxU32 mPad[1];
#endif

	/**
	\brief Test if wheel simulation data has been setup with legal values.
	*/
	bool isValid() const;

	/**
	\brief see PxVehicleWheels::allocate
	*/
	static PxU32 computeByteSize(const PxU32 numWheels);
	static PxU8* patchUpPointers(const PxU32 numWheels, PxVehicleWheelsSimData* simData, PxU8* ptrIn);
	PxVehicleWheelsSimData(const PxU32 numWheels);

//serialization
public:
	PxVehicleWheelsSimData(const PxEMPTY) : mNormalisedLoadFilter(PxEmpty) {}
	static void getBinaryMetaData(PxOutputStream& stream);
	PxU32 getNbWheels4() const { return mNbWheels4; }	
	PxU32 getNbSuspensionData() const { return mNbActiveWheels; }
	PxU32 getNbWheelData() const {	return mNbActiveWheels; }	
	PxU32 getNbSuspTravelDirection() const	{ return mNbActiveWheels; }
	PxU32 getNbTireData() const	{ return mNbActiveWheels;	}	
	PxU32 getNbSuspForceAppPointOffset() const	{ return mNbActiveWheels;	}
	PxU32 getNbTireForceAppPointOffset() const	{ return mNbActiveWheels;	}
	PxU32 getNbWheelCentreOffset() const { return mNbActiveWheels;	}
	PxU32 getNbWheelShapeMapping() const { return mNbActiveWheels; }
	PxU32 getNbSceneQueryFilterData() const { return mNbActiveWheels; }
	PxF32 getMinLongSlipDenominator() const {return mMinLongSlipDenominator;}
	void setThresholdLongSpeed(const PxF32 f) {mThresholdLongitudinalSpeed = f;}
	PxF32 getThresholdLongSpeed() const {return mThresholdLongitudinalSpeed;}
	void setLowForwardSpeedSubStepCount(const PxU32 f) {mLowForwardSpeedSubStepCount = f;}
	PxU32 getLowForwardSpeedSubStepCount() const {return mLowForwardSpeedSubStepCount;}
	void setHighForwardSpeedSubStepCount(const PxU32 f) {mHighForwardSpeedSubStepCount = f;}
	PxU32 getHighForwardSpeedSubStepCount() const {return mHighForwardSpeedSubStepCount;}
	void setWheelEnabledState(const PxU32 wheel, const bool state) {if(state) {enableWheel(wheel);} else {disableWheel(wheel);}}
	bool getWheelEnabledState(const PxU32 wheel) const {return !getIsWheelDisabled(wheel);}
	PxU32 getNbWheelEnabledState() const {return mNbActiveWheels;}
	PxU32 getNbAntiRollBars4() const { return mNbAntiRollBars4; }	
	PxU32 getNbAntiRollBarData() const {return mNbActiveAntiRollBars;}
	void setAntiRollBarData(const PxU32 id, const PxVehicleAntiRollBarData& antiRoll);
	PxVehicleWheelsSimData(){}
	~PxVehicleWheelsSimData(){}
//~serialization
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleWheelsSimData) & 15));

/**
\brief Data structure with instanced dynamics data for wheels
*/
class PxVehicleWheelsDynData
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================
public:

	friend class PxVehicleWheels;
	friend class PxVehicleDrive4W;
	friend class PxVehicleDriveTank;
	friend class PxVehicleUpdate;

	PxVehicleWheelsDynData(){}
	~PxVehicleWheelsDynData(){}

	/**
	\brief Set all wheels to their rest state.
	@see setup
	*/
	void setToRestState();

	/**
	\brief Set the tire force shader function
	\param[in] tireForceShaderFn is the shader function that will be used to compute tire forces.
	*/
	void setTireForceShaderFunction(PxVehicleComputeTireForce tireForceShaderFn);

	/**
	\brief Set the tire force shader data for a specific tire
	\param[in] tireId is the wheel index
	\param[in] tireForceShaderData is the data describing the tire.
	*/
	void setTireForceShaderData(const PxU32 tireId, const void* tireForceShaderData);

	/**
	\brief Get the tire force shader data for a specific tire
	*/
	const void* getTireForceShaderData(const PxU32 tireId) const;

	/**
	\brief Set the wheel rotation speed (radians per second) about the rolling axis for the specified wheel.
	\param[in] wheelIdx is the wheel index
	\param[in] speed is the rotation speed to be applied to the wheel.
	*/
	void setWheelRotationSpeed(const PxU32 wheelIdx, const PxReal speed);

	/**
	\brief Return the rotation speed about the rolling axis of a specified wheel .
	*/
	PxReal getWheelRotationSpeed(const PxU32 wheelIdx) const;
	
	/**
	\brief Set the wheel rotation angle (radians) about the rolling axis of the specified wheel.
	\param[in] wheelIdx is the wheel index
	\param[in] angle is the rotation angle to be applied to the wheel.
	*/
	void setWheelRotationAngle(const PxU32 wheelIdx, const PxReal angle);

	/**
	\brief Return the rotation angle about the rolling axis for the specified wheel.
	*/
	PxReal getWheelRotationAngle(const PxU32 wheelIdx) const;

	/**
	\brief Set the user data pointer for the specified wheel
	It has a default value of NULL.
	\param[in] tireIdx is the wheel index
	\param[in] userData is the data to be associated with the wheel.
	*/
	void setUserData(const PxU32 tireIdx, void* userData);

	/**
	\brief Get the user data pointer that was set for the specified wheel
	*/
	void* getUserData(const PxU32 tireIdx) const;

	/**
	\brief Copy the dynamics data of a single wheel unit (wheel, suspension, tire) from srcWheel of src to trgWheel.
	\param[in] src is the data to be copied.
	\param[in] srcWheel is the wheel whose data will be copied from src.
	\param[in] trgWheel is the wheel that will be assigned the copied data.
	*/
	void copy(const PxVehicleWheelsDynData& src, const PxU32 srcWheel, const PxU32 trgWheel);

private:

    /**
	\brief Dynamics data arranged in blocks of 4 wheels.
	*/
	PxVehicleWheels4DynData* mWheels4DynData;

	/**
	\brief Test if wheel dynamics data have legal values.
	*/
	bool isValid() const;

	/**
	\brief Shader data and function for tire force calculations.
	*/
	PxVehicleTireForceCalculator* mTireForceCalculators;
	
	/**
	\brief A userData pointer can be stored for each wheel.
	@see setUserData, getUserData
	*/
	void** mUserDatas;

	/**
	\brief Number of blocks of 4 wheels.
	*/
	PxU32 mNbWheels4;

	/**
	\brief Number of wheels (mNbActiveWheels <= (mNbWheels4*4))
	*/
	PxU32 mNbActiveWheels;

	PxU32 mPad[3];

	/**
	\brief see PxVehicleWheels::allocate
	*/
	static PxU32 computeByteSize(const PxU32 numWheels);
	static PxU8* patchUpPointers(const PxU32 numWheels, PxVehicleWheelsDynData* dynData, PxU8* ptr);
	PxVehicleWheelsDynData(const PxU32 numWheels);

//serialization
public:
	static void getBinaryMetaData(PxOutputStream& stream);	
	PxU32 getNbWheelRotationSpeed() const {	return mNbActiveWheels; }
	PxU32 getNbWheelRotationAngle() const {	return mNbActiveWheels; }	
	PxVehicleWheels4DynData* getWheel4DynData() const { return mWheels4DynData; }
//~serialization
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleWheelsDynData) & 15));

/**
\brief Data structure with instanced dynamics data and configuration data of a vehicle with just wheels
@see PxVehicleDrive, PxVehicleDrive4W, PxVehicleDriveTank
*/
class PxVehicleWheels : public PxBase
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================
public:

	friend class PxVehicleUpdate;
	friend class PxVehicleConstraintShader;

	/**
	\brief Return the type of vehicle 
	@see PxVehicleTypes
	*/
	PX_FORCE_INLINE PxU32 getVehicleType() const {return mType;}

	/**
	\brief Get non-const ptr to PxRigidDynamic instance that is the vehicle's physx representation
	*/
	PX_FORCE_INLINE PxRigidDynamic* getRigidDynamicActor() {return mActor;}

	/**
	\brief Get const ptr to PxRigidDynamic instance that is the vehicle's physx representation
	*/
	PX_FORCE_INLINE const PxRigidDynamic* getRigidDynamicActor() const {return mActor;}
	
	/**
	\brief Compute the rigid body velocity component along the forward vector of the rigid body transform.
	@see PxVehicleSetBasisVectors
	*/
	PxReal computeForwardSpeed() const;

	/**
	\brief Compute the rigid body velocity component along the right vector of the rigid body transform.
	@see PxVehicleSetBasisVectors
	*/
	PxReal computeSidewaysSpeed() const;

	/**
	\brief Data describing the setup of all the wheels/suspensions/tires.
	*/
	PxVehicleWheelsSimData mWheelsSimData;

	/**
	\brief Data describing the dynamic state of all wheels/suspension/tires.
	*/
	PxVehicleWheelsDynData mWheelsDynData;	

protected:

	/**
	\brief Set all wheels to their rest state
	*/
	void setToRestState();

	/**
	\brief Test that all configuration and instanced dynamics data is valid.
	*/
	bool isValid() const;

	/**
	@see PxVehicleDrive4W::allocate, PxVehicleDriveTank::allocate
	*/
	static PxU32 computeByteSize(const PxU32 nbWheels);
	static PxU8* patchupPointers(const PxU32 nbWheels, PxVehicleWheels* vehWheels, PxU8* ptr);
	virtual void init(const PxU32 numWheels);

	/**
	\brief Deallocate a PxVehicleWheels instance.
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
	
	/**
	\brief The rigid body actor that represents the vehicle in the PhysX SDK.
	*/
	PxRigidDynamic* mActor;

private:

	/**
	\brief Count the number of constraint connectors that have hit their callback when deleting a vehicle.
	Can only delete the vehicle's memory when all constraint connectors have hit their callback.
	*/
	PxU32 mNbNonDrivenWheels;
	
	PxU8 mOnConstraintReleaseCounter;

protected:

	/**
	\brief Vehicle type (eVehicleDriveTypes)
	*/
	PxU8 mType;
		
#if PX_P64_FAMILY
	PxU8 mPad0[14];
#else
	PxU8 mPad0[14];
#endif

//serialization
public:
	virtual		void			requiresObjects(PxProcessPxBaseCallback& c);
	virtual		const char*		getConcreteTypeName() const				{	return "PxVehicleWheels"; }
	virtual		bool			isKindOf(const char* name)	const		{	return !::strcmp("PxVehicleWheels", name) || PxBase::isKindOf(name); }
	virtual		void			exportExtraData(PxSerializationContext&);	
				void			importExtraData(PxDeserializationContext&);
				void			resolveReferences(PxDeserializationContext&);
	static		void			getBinaryMetaData(PxOutputStream& stream);
	PX_FORCE_INLINE PxU32 getNbNonDrivenWheels() const { return mNbNonDrivenWheels; }
	PxVehicleWheels(PxType concreteType, PxBaseFlags baseFlags) : PxBase(concreteType, baseFlags) {}
	PxVehicleWheels(PxBaseFlags baseFlags) : PxBase(baseFlags), mWheelsSimData(PxEmpty) {}
	virtual ~PxVehicleWheels() {}
	virtual void release() { free(); }
//~serialization
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleWheels) & 15));

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif //PX_VEHICLE_WHEELS_H
