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

#ifndef PX_VEHICLE_SUSPWHEELTIRE_H
#define PX_VEHICLE_SUSPWHEELTIRE_H
/** \addtogroup vehicle
  @{
*/

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"
#include "foundation/PxVec4.h"
#include "foundation/PxTransform.h"
#include "foundation/PxIO.h"
#include "geometry/PxGeometryHelpers.h"
#include "PxVehicleComponents.h"
#include "PxBatchQueryDesc.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxVehicleConstraintShader;
class PxMaterial;
class PxShape;



class PxVehicleWheels4SimData
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================
public:

	friend class PxVehicleUpdate;

	PxVehicleWheels4SimData();

	bool isValid(const PxU32 id) const;

	static void getBinaryMetaData(PxOutputStream& stream);

public:

	PX_FORCE_INLINE const PxVehicleSuspensionData&		getSuspensionData(const PxU32 id)			const {return mSuspensions[id];}
	PX_FORCE_INLINE const PxVehicleWheelData&			getWheelData(const PxU32 id)				const {return mWheels[id];}
	PX_FORCE_INLINE const PxVehicleTireData&			getTireData(const PxU32 id)					const {return mTires[id];}
	PX_FORCE_INLINE const PxVec3&						getSuspTravelDirection(const PxU32 id)		const {return mSuspDownwardTravelDirections[id];}
	PX_FORCE_INLINE const PxVec3&						getSuspForceAppPointOffset(const PxU32 id)	const {return mSuspForceAppPointOffsets[id];}
	PX_FORCE_INLINE const PxVec3&						getTireForceAppPointOffset(const PxU32 id)	const {return mTireForceAppPointOffsets[id];}
	PX_FORCE_INLINE const PxVec3&						getWheelCentreOffset(const PxU32 id)		const {return mWheelCentreOffsets[id];}
	PX_FORCE_INLINE		  PxI32							getWheelShapeMapping(const PxU32 id)		const {return (PX_MAX_U8 != mWheelShapeMap[id]) ? mWheelShapeMap[id] : -1;}
	PX_FORCE_INLINE	const PxFilterData&					getSceneQueryFilterData(const PxU32 id)		const {return mSqFilterData[id];}
	PX_FORCE_INLINE const PxReal*						getTireRestLoadsArray()						const {return mTireRestLoads;}
	PX_FORCE_INLINE const PxReal*						getRecipTireRestLoadsArray()				const {return mRecipTireRestLoads;}

					void setSuspensionData				(const PxU32 id, const PxVehicleSuspensionData& susp);
					void setWheelData					(const PxU32 id, const PxVehicleWheelData& susp);
					void setTireData					(const PxU32 id, const PxVehicleTireData& tire);
					void setSuspTravelDirection			(const PxU32 id, const PxVec3& dir);
					void setSuspForceAppPointOffset		(const PxU32 id, const PxVec3& offset);
					void setTireForceAppPointOffset		(const PxU32 id, const PxVec3& offset);
					void setWheelCentreOffset			(const PxU32 id, const PxVec3& offset);
					void setWheelShapeMapping			(const PxU32 id, const PxI32 shapeId);
					void setSceneQueryFilterData		(const PxU32 id, const PxFilterData& sqFilterData);

private:

	/**
	\brief Suspension simulation data
	@see setSuspensionData, getSuspensionData
	*/
	PxVehicleSuspensionData			mSuspensions[4];

	/**
	\brief Wheel simulation data
	@see setWheelData, getWheelData
	*/
	PxVehicleWheelData				mWheels[4];

	/**
	\brief Tire simulation data
	@see setTireData, getTireData
	*/
	PxVehicleTireData				mTires[4];

	/**
	\brief Direction of suspension travel, pointing downwards.
	*/
	PxVec3							mSuspDownwardTravelDirections[4];

	/**
	\brief Application point of suspension force specified as an offset from the rigid body centre of mass.
	*/
	PxVec3							mSuspForceAppPointOffsets[4];	//Offset from cm

	/**
	\brief Application point of tire forces specified as an offset from the rigid body centre of mass.
	*/
	PxVec3							mTireForceAppPointOffsets[4];	//Offset from cm

	/**
	\brief Position of wheel center specified as an offset from the rigid body centre of mass.
	*/
	PxVec3							mWheelCentreOffsets[4];			//Offset from cm

	/** 
	\brief Normalized tire load on each tire (load/rest load) at zero suspension jounce under gravity.
	*/
	PxReal							mTireRestLoads[4];	

	/** 
	\brief Reciprocal normalized tire load on each tire at zero suspension jounce under gravity.
	*/
	PxReal							mRecipTireRestLoads[4];	

	/**
	\brief Scene query filter data used by each suspension line.
	Anything relating to the actor belongs in PxVehicleWheels.
	*/
	PxFilterData					mSqFilterData[4];

	/**
	\brief Mapping between wheel id and shape id.
	The PxShape that corresponds to the ith wheel can be found with 
	If mWheelShapeMap[i]<0 then the wheel has no corresponding shape.
	Otherwise, the shape corresponds to:
	PxShape* shapeBuffer[1];
	mActor->getShapes(shapeBuffer,1,mWheelShapeMap[i]);
	Anything relating to the actor belongs in PxVehicleWheels.
	*/
	PxU8							mWheelShapeMap[4];

	PxU32							mPad[3];
};
PX_COMPILE_TIME_ASSERT(0 == (sizeof(PxVehicleWheels4SimData) & 15));

class PxVehicleWheels4DynData
{
//= ATTENTION! =====================================================================================
// Changing the data layout of this class breaks the binary serialization format.  See comments for 
// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
// accordingly.
//==================================================================================================
public:

	friend class PxVehicleUpdate;

	PxVehicleWheels4DynData()
		:	mRaycastResults(NULL),
		    mSweepResults(NULL)
	{
		setToRestState();
	}
	~PxVehicleWheels4DynData()
	{
	}

	bool isValid() const {return true;}

	static void getBinaryMetaData(PxOutputStream& stream);

	void setToRestState()
	{
		for(PxU32 i=0;i<4;i++)
		{
			mWheelSpeeds[i]=0.0f;
			mCorrectedWheelSpeeds[i]=0.0f;
			mWheelRotationAngles[i]=0.0f;
			mTireLowForwardSpeedTimers[i]=0.0f;
			mTireLowSideSpeedTimers[i]=0.0f;
			mJounces[i] = PX_MAX_F32;
		}
		PxMemZero(mQueryOrCachedHitResults, sizeof(SuspLineSweep));

		mRaycastResults = NULL;
		mSweepResults = NULL;
		mHasCachedRaycastHitPlane = false;
	}

	void setInternalDynamicsToZero()
	{
		for(PxU32 i=0;i<4;i++)
		{
			mWheelSpeeds[i] = 0.0f;
			mCorrectedWheelSpeeds[i] = 0.0f;
			mJounces[i] = PX_MAX_F32; //Ensure that the jounce speed is zero when the car wakes up again. 
		}
	}

	/**
	\brief Rotation speeds of wheels 
	@see PxVehicle4WSetToRestState, PxVehicle4WGetWheelRotationSpeed, PxVehicle4WGetEngineRotationSpeed
	*/	
	PxReal mWheelSpeeds[4];

	/**
	\brief Rotation speeds of wheels used to update the wheel rotation angles.
	*/	
	PxReal mCorrectedWheelSpeeds[4];

	/**
	\brief Reported rotation angle about rolling axis.
	@see PxVehicle4WSetToRestState, PxVehicle4WGetWheelRotationAngle
	*/	
	PxReal mWheelRotationAngles[4];

	/**
	\brief Timers used to trigger sticky friction to hold the car perfectly at rest. 
	\note Used only internally.
	*/
	PxReal mTireLowForwardSpeedTimers[4];

	/**
	\brief Timers used to trigger sticky friction to hold the car perfectly at rest. 
	\note Used only internally.
	*/
	PxReal mTireLowSideSpeedTimers[4];

	/**
	\brief Previous suspension jounce.
	\note Used only internally to compute the jounce speed by comparing cached jounce and latest jounce.
	*/
	PxReal mJounces[4];

	struct SuspLineSweep
	{
		/**
		\brief Geometry suspension line sweep used in most recent scene query.
		@see PxVehicleSuspensionSweeps
		*/
		PxGeometryHolder mGometries[4];

		/**
		\brief Start poses of suspension line sweep used in most recent scene query.
		@see PxVehicleSuspensionSweeps 
		*/
		PxTransform mStartPose[4];

		/**
		\brief Directions of suspension line sweeps used in most recent scene query.
		@see PxVehicleSuspensionSweeps
		*/
		PxVec3 mDirs[4];

		/**
		\brief Lengths of suspension line sweeps used in most recent scene query.
		@see PxVehicleSuspensionSweeps
		*/
		PxReal mLengths[4];
	};

	struct SuspLineRaycast
	{
		/**
		\brief Start point of suspension line raycasts used in most recent scene query.
		@see PxVehicleSuspensionRaycasts 
		*/
		PxVec3 mStarts[4];

		/**
		\brief Directions of suspension line raycasts used in most recent scene query.
		@see PxVehicleSuspensionRaycasts
		*/
		PxVec3 mDirs[4];

		/**
		\brief Lengths of suspension line raycasts used in most recent scene query.
		@see PxVehicleSuspensionRaycasts
		*/
		PxReal mLengths[4];

		PxU32 mPad[16];
	};

	struct CachedSuspLineSceneQuerytHitResult
	{
		/**
		\brief Cached raycast hit planes. These are the planes found from the last scene queries.
		@see PxVehicleSuspensionRaycasts, PxVehicleSuspensionSweeps
		*/
		PxVec4 mPlanes[4];

		/**
		\brief Cached friction.
		@see PxVehicleSuspensionRaycasts, PxVehicleSuspensionSweeps
		*/
		PxF32 mFrictionMultipliers[4];

		/**
		\brief Cached raycast hit distance. These are the hit distances found from the last scene queries.
		*/
		PxF32 mDistances[4];

		/**
		\brief Cached raycast hit counts. These are the hit counts found from the last scene queries.
		@see PxVehicleSuspensionRaycasts, , PxVehicleSuspensionSweeps
		*/
		PxU16 mCounts[4];

		/**
		\brief Store 0 if cached results are from raycasts, store 1 if cached results are from sweeps.
		*/
		PxU16 mQueryTypes[4];

		PxU32 mPad1[16];
	};

	/**
	\brief We either have a fresh raycast that was just performed or a cached raycast result that will be used if no raycast was just performed.
	*/
	PxU8 mQueryOrCachedHitResults[sizeof(SuspLineSweep)];

	/**
	\brief Used only internally.
	*/
	void setVehicleConstraintShader(PxVehicleConstraintShader* shader) {mVehicleConstraints=shader;}
	PxVehicleConstraintShader& getVehicletConstraintShader() const {return *mVehicleConstraints;} 

private:

	//Susp limits and sticky tire friction for all wheels.
	PxVehicleConstraintShader* mVehicleConstraints;

public:

	/**
	\brief Set by PxVehicleSuspensionRaycasts
	@see PxVehicleSuspensionRaycasts
	*/
	const PxRaycastQueryResult* mRaycastResults;

	/**
	\brief Set by PxVehicleSuspensionSweeps
	@see PxVehicleSuspensionSweeps
	*/
	const PxSweepQueryResult* mSweepResults;

	/**
	\brief Set true if a raycast hit plane has been recorded and cached.
	This requires a raycast to be performed and then followed by PxVehicleUpdates
	at least once.  Reset to false in setToRestState.
	*/
	bool mHasCachedRaycastHitPlane;


#if PX_P64_FAMILY
	PxU32 mPad[12];
#endif
};
PX_COMPILE_TIME_ASSERT(0==(sizeof(PxVehicleWheels4DynData) & 15));
PX_COMPILE_TIME_ASSERT((0 == (sizeof(PxVehicleWheels4DynData::SuspLineSweep) & 0x0f)));
PX_COMPILE_TIME_ASSERT(sizeof(PxVehicleWheels4DynData::SuspLineRaycast) <= sizeof(PxVehicleWheels4DynData::SuspLineSweep));
PX_COMPILE_TIME_ASSERT(sizeof(PxVehicleWheels4DynData::CachedSuspLineSceneQuerytHitResult) <= sizeof(PxVehicleWheels4DynData::SuspLineSweep));

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif //PX_VEHICLE_SUSPWHEELTIRE_H
