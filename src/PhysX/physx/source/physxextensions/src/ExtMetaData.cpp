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

#include "foundation/PxIO.h"
#include "PxJoint.h"
#include "ExtJoint.h"
#include "ExtD6Joint.h"
#include "ExtFixedJoint.h"
#include "ExtSphericalJoint.h"
#include "ExtDistanceJoint.h"
#include "ExtContactJoint.h"
#include "ExtSphericalJoint.h"
#include "ExtRevoluteJoint.h"
#include "ExtPrismaticJoint.h"
#include "serialization/SnSerializationRegistry.h"
#include "serialization/Binary/SnSerializationContext.h"

using namespace physx;
using namespace Cm;
using namespace Ext;

///////////////////////////////////////////////////////////////////////////////

static void getBinaryMetaData_JointData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,		JointData)

	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	JointData,				PxTransform,				c2b,				0)
#ifdef EXPLICIT_PADDING_METADATA
	PX_DEF_BIN_METADATA_ITEM(stream,		JointData,				PxU32,				        paddingFromFlags,	PxMetaDataFlag::ePADDING)
#endif
	PX_DEF_BIN_METADATA_ITEM(stream,		JointData,				PxConstraintInvMassScale,	invMassScale,	    0)
}

static void getBinaryMetaData_PxD6JointDrive(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_TYPEDEF(stream, 	PxD6JointDriveFlags,	PxU32)

	PX_DEF_BIN_METADATA_CLASS(stream,		PxD6JointDrive)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxD6JointDrive,			PxReal,						stiffness,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxD6JointDrive,			PxReal,						damping,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxD6JointDrive,			PxReal,						forceLimit,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxD6JointDrive,			PxD6JointDriveFlags,		flags,		0)
}

static void getBinaryMetaData_PxJointLimitParameters(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,	PxJointLimitParameters)

	PX_DEF_BIN_METADATA_ITEM(stream,	PxJointLimitParameters,		PxReal,						restitution,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxJointLimitParameters,		PxReal,						bounceThreshold,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxJointLimitParameters,		PxReal,						stiffness,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxJointLimitParameters,		PxReal,						damping,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxJointLimitParameters,		PxReal,						contactDistance,	0)
}

static void getBinaryMetaData_PxJointLinearLimit(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,		PxJointLinearLimit)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	PxJointLinearLimit,		PxJointLimitParameters)

	PX_DEF_BIN_METADATA_ITEM(stream,		PxJointLinearLimit,		PxReal,						value,				0)
}

static void getBinaryMetaData_PxJointLinearLimitPair(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,		PxJointLinearLimitPair)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	PxJointLinearLimitPair, PxJointLimitParameters)

	PX_DEF_BIN_METADATA_ITEM(stream,		PxJointLinearLimitPair,	PxReal,		upper,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxJointLinearLimitPair,	PxReal,		lower,		0)
}

static void getBinaryMetaData_PxJointAngularLimitPair(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,		PxJointAngularLimitPair)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	PxJointAngularLimitPair, PxJointLimitParameters)

	PX_DEF_BIN_METADATA_ITEM(stream,		PxJointAngularLimitPair,	PxReal,		upper,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxJointAngularLimitPair,	PxReal,		lower,		0)
}

static void getBinaryMetaData_PxJointLimitCone(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,		PxJointLimitCone)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	PxJointLimitCone,	PxJointLimitParameters)

	PX_DEF_BIN_METADATA_ITEM(stream,		PxJointLimitCone,	PxReal,		yAngle,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxJointLimitCone,	PxReal,		zAngle,		0)
}

static void getBinaryMetaData_PxJointLimitPyramid(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,		PxJointLimitPyramid)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	PxJointLimitPyramid,	PxJointLimitParameters)

	PX_DEF_BIN_METADATA_ITEM(stream,		PxJointLimitPyramid,	PxReal,		yAngleMin,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxJointLimitPyramid,	PxReal,		yAngleMax,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxJointLimitPyramid,	PxReal,		zAngleMin,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxJointLimitPyramid,	PxReal,		zAngleMax,		0)
}

void PxJoint::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_VCLASS(stream,		PxJoint)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	PxJoint, PxBase)

	PX_DEF_BIN_METADATA_ITEM(stream,		PxJoint, void,	userData,	PxMetaDataFlag::ePTR)
}

///////////////////////////////////////////////////////////////////////////////

static void getBinaryMetaData_RevoluteJointData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_TYPEDEF(stream,		PxRevoluteJointFlags,	PxU16)

	PX_DEF_BIN_METADATA_CLASS(stream,		RevoluteJointData)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	RevoluteJointData,		JointData)

	PX_DEF_BIN_METADATA_ITEM(stream,		RevoluteJointData,		PxReal,					driveVelocity,				0)
	PX_DEF_BIN_METADATA_ITEM(stream,		RevoluteJointData,		PxReal,					driveForceLimit,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,		RevoluteJointData,		PxReal,					driveGearRatio,				0)

	PX_DEF_BIN_METADATA_ITEM(stream,		RevoluteJointData,		PxJointAngularLimitPair,limit,						0)

	PX_DEF_BIN_METADATA_ITEM(stream,		RevoluteJointData,		PxReal,					projectionLinearTolerance,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		RevoluteJointData,		PxReal,					projectionAngularTolerance,	0)

	PX_DEF_BIN_METADATA_ITEM(stream,		RevoluteJointData,		PxRevoluteJointFlags,	jointFlags,					0)
#ifdef EXPLICIT_PADDING_METADATA
	PX_DEF_BIN_METADATA_ITEM(stream,		RevoluteJointData,		PxU16,					paddingFromFlags,			PxMetaDataFlag::ePADDING)
#endif
}

void RevoluteJoint::getBinaryMetaData(PxOutputStream& stream)
{
	getBinaryMetaData_RevoluteJointData(stream);

	PX_DEF_BIN_METADATA_VCLASS(stream,		RevoluteJoint)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	RevoluteJoint,	PxJoint)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	RevoluteJoint,	PxConstraintConnector)

	PX_DEF_BIN_METADATA_ITEM(stream,		RevoluteJoint,	char,					mName,			PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	RevoluteJoint,	PxTransform,			mLocalPose,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,		RevoluteJoint,	PxConstraint,			mPxConstraint,	PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,		RevoluteJoint,	JointData,				mData,			PxMetaDataFlag::ePTR)

	//------ Extra-data ------

	PX_DEF_BIN_METADATA_EXTRA_ITEM(stream,	RevoluteJoint, RevoluteJointData,		mData,			PX_SERIAL_ALIGN)
	PX_DEF_BIN_METADATA_EXTRA_NAME(stream,	RevoluteJoint, mName, 0)
}

///////////////////////////////////////////////////////////////////////////////

static void getBinaryMetaData_SphericalJointData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_TYPEDEF(stream,		PxSphericalJointFlags,	PxU16)

	PX_DEF_BIN_METADATA_CLASS(stream,		SphericalJointData)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	SphericalJointData, 	JointData)

	PX_DEF_BIN_METADATA_ITEM(stream,		SphericalJointData,		PxJointLimitCone,		limit,						0)

	PX_DEF_BIN_METADATA_ITEM(stream,		SphericalJointData,		PxReal,					projectionLinearTolerance,	0)

	PX_DEF_BIN_METADATA_ITEM(stream,		SphericalJointData,		PxSphericalJointFlags,	jointFlags,					0)
#ifdef EXPLICIT_PADDING_METADATA
	PX_DEF_BIN_METADATA_ITEM(stream,		SphericalJointData,		PxU16,					paddingFromFlags,			PxMetaDataFlag::ePADDING)
#endif
}

void SphericalJoint::getBinaryMetaData(PxOutputStream& stream)
{
	getBinaryMetaData_SphericalJointData(stream);

	PX_DEF_BIN_METADATA_VCLASS(stream,		SphericalJoint)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	SphericalJoint, 		PxJoint)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	SphericalJoint, 		PxConstraintConnector)

	PX_DEF_BIN_METADATA_ITEM(stream,		SphericalJoint,			char,						mName,			PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	SphericalJoint,			PxTransform,				mLocalPose,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,		SphericalJoint,			PxConstraint,				mPxConstraint,	PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,		SphericalJoint,			JointData,					mData,			PxMetaDataFlag::ePTR)

	//------ Extra-data ------

	PX_DEF_BIN_METADATA_EXTRA_ITEM(stream,	SphericalJoint, 		SphericalJointData,			mData,			PX_SERIAL_ALIGN)
	PX_DEF_BIN_METADATA_EXTRA_NAME(stream,	SphericalJoint, 		mName, 0)
}

///////////////////////////////////////////////////////////////////////////////

static void getBinaryMetaData_DistanceJointData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_TYPEDEF(stream,		PxDistanceJointFlags,	PxU16)

	PX_DEF_BIN_METADATA_CLASS(stream,		DistanceJointData)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	DistanceJointData,		JointData)

	PX_DEF_BIN_METADATA_ITEM(stream,		DistanceJointData,		PxReal,					minDistance,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,		DistanceJointData,		PxReal,					maxDistance,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,		DistanceJointData,		PxReal,					tolerance,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,		DistanceJointData,		PxReal,					stiffness,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,		DistanceJointData,		PxReal,					damping,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,		DistanceJointData,		PxDistanceJointFlags,	jointFlags,			0)
#ifdef EXPLICIT_PADDING_METADATA
	PX_DEF_BIN_METADATA_ITEM(stream,		DistanceJointData,		PxU16,					paddingFromFlags,	PxMetaDataFlag::ePADDING)
#endif
}

void DistanceJoint::getBinaryMetaData(PxOutputStream& stream)
{
	getBinaryMetaData_DistanceJointData(stream);

	PX_DEF_BIN_METADATA_VCLASS(stream,		DistanceJoint)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	DistanceJoint,			PxJoint)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	DistanceJoint,			PxConstraintConnector)

	PX_DEF_BIN_METADATA_ITEM(stream,		DistanceJoint,			char,					mName,			PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	DistanceJoint,			PxTransform,			mLocalPose,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,		DistanceJoint,			PxConstraint,			mPxConstraint,	PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,		DistanceJoint,			JointData,				mData,			PxMetaDataFlag::ePTR)

	//------ Extra-data ------

	PX_DEF_BIN_METADATA_EXTRA_ITEM(stream,	DistanceJoint, 			DistanceJointData,		mData,			PX_SERIAL_ALIGN)
	PX_DEF_BIN_METADATA_EXTRA_NAME(stream,	DistanceJoint, 			mName,					0)
}

///////////////////////////////////////////////////////////////////////////////

static void getBinaryMetaData_D6JointData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_TYPEDEF(stream,		PxD6Motion::Enum,	PxU32)

	PX_DEF_BIN_METADATA_CLASS(stream,		D6JointData)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	D6JointData,		JointData)

	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	D6JointData,		PxD6Motion::Enum,	    	motion,						0)
	PX_DEF_BIN_METADATA_ITEM(stream,		D6JointData,		PxJointLinearLimit,			distanceLimit,				0)
	PX_DEF_BIN_METADATA_ITEM(stream,		D6JointData,		PxJointLinearLimitPair,		linearLimitX,				0)
	PX_DEF_BIN_METADATA_ITEM(stream,		D6JointData,		PxJointLinearLimitPair,		linearLimitY,				0)
	PX_DEF_BIN_METADATA_ITEM(stream,		D6JointData,		PxJointLinearLimitPair,		linearLimitZ,				0)
	PX_DEF_BIN_METADATA_ITEM(stream,		D6JointData,		PxJointAngularLimitPair,	twistLimit,					0)
	PX_DEF_BIN_METADATA_ITEM(stream,		D6JointData,		PxJointLimitCone,			swingLimit,					0)
	PX_DEF_BIN_METADATA_ITEM(stream,		D6JointData,		PxJointLimitPyramid,		pyramidSwingLimit,			0)

	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	D6JointData,		PxD6JointDrive,				drive,						0)

	PX_DEF_BIN_METADATA_ITEM(stream,		D6JointData,		PxTransform,				drivePosition,				0)
	PX_DEF_BIN_METADATA_ITEM(stream,		D6JointData,		PxVec3,						driveLinearVelocity,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,		D6JointData,		PxVec3,						driveAngularVelocity,		0)

	PX_DEF_BIN_METADATA_ITEM(stream,		D6JointData,		PxU32,						locked,						0)
	PX_DEF_BIN_METADATA_ITEM(stream,		D6JointData,		PxU32,						limited,					0)
	PX_DEF_BIN_METADATA_ITEM(stream,		D6JointData,		PxU32,						driving,					0)

	PX_DEF_BIN_METADATA_ITEM(stream,		D6JointData,		PxReal,						distanceMinDist,			0)

	PX_DEF_BIN_METADATA_ITEM(stream,		D6JointData,		PxReal,						projectionLinearTolerance,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		D6JointData,		PxReal,						projectionAngularTolerance,	0)

	PX_DEF_BIN_METADATA_ITEM(stream,		D6JointData,		bool,						mUseDistanceLimit,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,		D6JointData,		bool,						mUseNewLinearLimits,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,		D6JointData,		bool,						mUseConeLimit,				0)
	PX_DEF_BIN_METADATA_ITEM(stream,		D6JointData,		bool,						mUsePyramidLimits,			0)
}

void D6Joint::getBinaryMetaData(PxOutputStream& stream)
{
	getBinaryMetaData_D6JointData(stream);

	PX_DEF_BIN_METADATA_VCLASS(stream,		D6Joint)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	D6Joint, 			PxJoint)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	D6Joint, 			PxConstraintConnector)

	PX_DEF_BIN_METADATA_ITEM(stream,		D6Joint, 			char,			mName,				PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	D6Joint, 			PxTransform,	mLocalPose,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,		D6Joint, 			PxConstraint,	mPxConstraint,		PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,		D6Joint, 			JointData,		mData,				PxMetaDataFlag::ePTR)

	PX_DEF_BIN_METADATA_ITEM(stream,		D6Joint, 			bool,			mRecomputeMotion,	0)
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	D6Joint, 			bool,			mPadding,			PxMetaDataFlag::ePADDING)

	//------ Extra-data ------

	PX_DEF_BIN_METADATA_EXTRA_ITEM(stream,	D6Joint,			D6JointData,	mData,				PX_SERIAL_ALIGN)
	PX_DEF_BIN_METADATA_EXTRA_NAME(stream,	D6Joint,			mName, 0)
}

///////////////////////////////////////////////////////////////////////////////

static void getBinaryMetaData_PrismaticJointData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_TYPEDEF(stream,		PxPrismaticJointFlags,	PxU16)

	PX_DEF_BIN_METADATA_CLASS(stream,		PrismaticJointData)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	PrismaticJointData, 	JointData)

	PX_DEF_BIN_METADATA_ITEM(stream,		PrismaticJointData,		PxJointLinearLimitPair,	limit,						0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PrismaticJointData,		PxReal,					projectionLinearTolerance,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PrismaticJointData,		PxReal,					projectionAngularTolerance,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PrismaticJointData,		PxPrismaticJointFlags,	jointFlags,					0)
#ifdef EXPLICIT_PADDING_METADATA
	PX_DEF_BIN_METADATA_ITEM(stream,		PrismaticJointData,		PxU16,					paddingFromFlags,			PxMetaDataFlag::ePADDING)
#endif
}

void PrismaticJoint::getBinaryMetaData(PxOutputStream& stream)
{
	getBinaryMetaData_PrismaticJointData(stream);

	PX_DEF_BIN_METADATA_VCLASS(stream,		PrismaticJoint)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	PrismaticJoint, 		PxJoint)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	PrismaticJoint, 		PxConstraintConnector)

	PX_DEF_BIN_METADATA_ITEM(stream,		PrismaticJoint,			char,			mName,			PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	PrismaticJoint,			PxTransform,	mLocalPose,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PrismaticJoint,			PxConstraint,	mPxConstraint,	PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,		PrismaticJoint,			JointData,		mData,			PxMetaDataFlag::ePTR)

	//------ Extra-data ------

	PX_DEF_BIN_METADATA_EXTRA_ITEM(stream,	PrismaticJoint, 		PrismaticJointData, mData, PX_SERIAL_ALIGN)
	PX_DEF_BIN_METADATA_EXTRA_NAME(stream,	PrismaticJoint, 		mName, 0)
}

///////////////////////////////////////////////////////////////////////////////

static void getBinaryMetaData_FixedJointData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,		FixedJointData)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	FixedJointData, JointData)

	PX_DEF_BIN_METADATA_ITEM(stream,		FixedJointData,	PxReal,	projectionLinearTolerance,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		FixedJointData,	PxReal,	projectionAngularTolerance,	0)
}

void FixedJoint::getBinaryMetaData(PxOutputStream& stream)
{
	getBinaryMetaData_FixedJointData(stream);

	PX_DEF_BIN_METADATA_VCLASS(stream,		FixedJoint)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	FixedJoint, PxJoint)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	FixedJoint, PxConstraintConnector)

	PX_DEF_BIN_METADATA_ITEM(stream,		FixedJoint,	char,			mName,			PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	FixedJoint,	PxTransform,	mLocalPose,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,		FixedJoint,	PxConstraint,	mPxConstraint,	PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,		FixedJoint,	JointData,		mData,			PxMetaDataFlag::ePTR)

	//------ Extra-data ------

	PX_DEF_BIN_METADATA_EXTRA_ITEM(stream,	FixedJoint,	FixedJointData, mData,			PX_SERIAL_ALIGN)
	PX_DEF_BIN_METADATA_EXTRA_NAME(stream,	FixedJoint, mName, 0)
}


void getBinaryMetaData_SerializationContext(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_TYPEDEF(stream, PxSerialObjectId,			PxU64)
	PX_DEF_BIN_METADATA_TYPEDEF(stream,	SerialObjectIndex,			PxU32)

	PX_DEF_BIN_METADATA_CLASS(stream,	Sn::ManifestEntry)
	PX_DEF_BIN_METADATA_ITEM(stream,	Sn::ManifestEntry,			PxU32,		offset,   	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	Sn::ManifestEntry,			PxType,	    type,   	0)
		
	PX_DEF_BIN_METADATA_CLASS(stream,	Sn::ImportReference)
	PX_DEF_BIN_METADATA_ITEM(stream,	Sn::ImportReference,		PxSerialObjectId,		id,   	    0)
	PX_DEF_BIN_METADATA_ITEM(stream,	Sn::ImportReference,		PxType,	                type,   	0)
		
	PX_DEF_BIN_METADATA_CLASS(stream,	Sn::ExportReference)
	PX_DEF_BIN_METADATA_ITEM(stream,	Sn::ExportReference,		PxSerialObjectId,		id,   	    0)
	PX_DEF_BIN_METADATA_ITEM(stream,	Sn::ExportReference,		SerialObjectIndex,	    objIndex,  	0)
	
	PX_DEF_BIN_METADATA_CLASS(stream,	Sn::InternalReferencePtr)
	PX_DEF_BIN_METADATA_ITEM(stream,	Sn::InternalReferencePtr,	void,		            reference,  PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,	Sn::InternalReferencePtr,	PxU32,		            kind,   	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	Sn::InternalReferencePtr,	SerialObjectIndex,	    objIndex,  	0)

	PX_DEF_BIN_METADATA_CLASS(stream,	Sn::InternalReferenceIdx)
	PX_DEF_BIN_METADATA_ITEM(stream,	Sn::InternalReferenceIdx,	PxU32,		            reference,  0)
	PX_DEF_BIN_METADATA_ITEM(stream,	Sn::InternalReferenceIdx,	PxU32,		            kind,   	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	Sn::InternalReferenceIdx,	SerialObjectIndex,	    objIndex,  	0)
}

namespace physx
{
namespace Ext
{
void GetExtensionsBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_VCLASS(stream,PxConstraintConnector)

	getBinaryMetaData_JointData(stream);
	getBinaryMetaData_PxD6JointDrive(stream);
	getBinaryMetaData_PxJointLimitParameters(stream);	
	getBinaryMetaData_PxJointLimitCone(stream);	
	getBinaryMetaData_PxJointLimitPyramid(stream);	
	getBinaryMetaData_PxJointLinearLimit(stream);
	getBinaryMetaData_PxJointLinearLimitPair(stream);
	getBinaryMetaData_PxJointAngularLimitPair(stream);
	
	PxJoint::getBinaryMetaData(stream);
	RevoluteJoint::getBinaryMetaData(stream);
	SphericalJoint::getBinaryMetaData(stream);
	DistanceJoint::getBinaryMetaData(stream);
	D6Joint::getBinaryMetaData(stream);
	PrismaticJoint::getBinaryMetaData(stream);
	FixedJoint::getBinaryMetaData(stream);

	getBinaryMetaData_SerializationContext(stream);
}

}
}

///////////////////////////////////////////////////////////////////////////////
