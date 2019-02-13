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

#include "foundation/PxIO.h"
#include "ScActorCore.h"
#include "ScActorSim.h"
#include "ScBodyCore.h"
#include "ScStaticCore.h"
#include "ScConstraintCore.h"
#include "ScMaterialCore.h"
#include "ScShapeCore.h"
#include "ScArticulationCore.h"
#include "ScArticulationJointCore.h"

using namespace physx;
using namespace Ps;
using namespace Cm;
using namespace Sc;

///////////////////////////////////////////////////////////////////////////////

template <typename T> class PxMetaDataArray : public physx::shdfnd::Array<T>
{
public:
    static PX_FORCE_INLINE physx::PxU32 getDataOffset()           { return PX_OFFSET_OF(PxMetaDataArray<T>, mData); }
    static PX_FORCE_INLINE physx::PxU32 getDataSize()             { return PX_SIZE_OF(PxMetaDataArray<T>, mData); }
    static PX_FORCE_INLINE physx::PxU32 getSizeOffset()           { return PX_OFFSET_OF(PxMetaDataArray<T>, mSize); }
    static PX_FORCE_INLINE physx::PxU32 getSizeSize()             { return PX_SIZE_OF(PxMetaDataArray<T>, mSize); }
    static PX_FORCE_INLINE physx::PxU32 getCapacityOffset()       { return PX_OFFSET_OF(PxMetaDataArray<T>, mCapacity); }
    static PX_FORCE_INLINE physx::PxU32 getCapacitySize()         { return PX_SIZE_OF(PxMetaDataArray<T>, mCapacity); }
};

void Sc::ActorCore::getBinaryMetaData(PxOutputStream& stream)
{
	// 16 bytes
	PX_DEF_BIN_METADATA_TYPEDEF(stream,	PxActorFlags,     PxU8)
	PX_DEF_BIN_METADATA_TYPEDEF(stream,	PxDominanceGroup, PxU8)
	PX_DEF_BIN_METADATA_TYPEDEF(stream,	PxClientID,       PxU8)

	PX_DEF_BIN_METADATA_CLASS(stream,	Sc::ActorCore)

	PX_DEF_BIN_METADATA_ITEM(stream,	Sc::ActorCore, ActorSim,			mSim,						PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,	Sc::ActorCore, PxU32,			    mAggregateIDOwnerClient,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	Sc::ActorCore, PxActorFlags,		mActorFlags,				0)
	PX_DEF_BIN_METADATA_ITEM(stream,	Sc::ActorCore, PxU8,				mActorType,					0)
	PX_DEF_BIN_METADATA_ITEM(stream,	Sc::ActorCore, PxU8,				mDominanceGroup,			0)
}

///////////////////////////////////////////////////////////////////////////////

static void getBinaryMetaData_PxsRigidCore(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,	PxsRigidCore)

	PX_DEF_BIN_METADATA_ITEM(stream,	PxsBodyCore, PxTransform,		body2World,				0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxsBodyCore, PxRigidBodyFlags,	mFlags,					0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxsBodyCore, PxU8,				mIdtBody2Actor,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxsBodyCore, PxU16,				solverIterationCounts,	0)
}

namespace
{
	class ShadowPxsBodyCore : public PxsBodyCore
	{
	public:
		static void getBinaryMetaData(PxOutputStream& stream)
		{
			PX_DEF_BIN_METADATA_CLASS(stream,		ShadowPxsBodyCore)
			PX_DEF_BIN_METADATA_BASE_CLASS(stream,	ShadowPxsBodyCore, PxsRigidCore)

			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxTransform,	body2Actor,				0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxReal,		ccdAdvanceCoefficient,	0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxVec3,		linearVelocity,			0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxReal,		maxPenBias,				0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxVec3,		angularVelocity,		0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxReal,		contactReportThreshold,	0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxReal,		maxAngularVelocitySq,	0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxReal,		maxLinearVelocitySq,	0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxReal,		linearDamping,			0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxReal,		angularDamping,			0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxVec3,		inverseInertia,			0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxReal,		inverseMass,			0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxReal,		maxContactImpulse,		0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxReal,		sleepThreshold,			0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxReal,		freezeThreshold,		0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxReal,		wakeCounter,			0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxReal,		solverWakeCounter,		0)
			PX_DEF_BIN_METADATA_ITEM(stream, 		ShadowPxsBodyCore, PxU32,		numCountedInteractions,	0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxU32,		numBodyInteractions,	0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxU16,		isFastMoving,			0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxU16,		lockFlags,				0)
		}
	};
}

static void getBinaryMetaData_PxsBodyCore(PxOutputStream& stream)
{
	getBinaryMetaData_PxsRigidCore(stream);

/*	PX_DEF_BIN_METADATA_CLASS(stream,		PxsBodyCore)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	PxsBodyCore, PxsRigidCore)

	PX_DEF_BIN_METADATA_ITEM(stream,		PxsBodyCore, PxTransform,	body2Actor,				0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsBodyCore, PxReal,		ccdAdvanceCoefficient,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsBodyCore, PxVec3,		linearVelocity,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsBodyCore, PxReal,		maxPenBias,				0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsBodyCore, PxVec3,		angularVelocity,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsBodyCore, PxReal,		contactReportThreshold,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsBodyCore, PxReal,		maxAngularVelocitySq,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsBodyCore, PxReal,		maxLinearVelocitySq,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsBodyCore, PxReal,		linearDamping,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsBodyCore, PxReal,		angularDamping,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsBodyCore, PxVec3,		inverseInertia,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsBodyCore, PxReal,		inverseMass,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsBodyCore, PxReal,		maxContactImpulse,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsBodyCore, PxReal,		sleepThreshold,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsBodyCore, PxReal,		freezeThreshold,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsBodyCore, PxReal,		wakeCounter,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsBodyCore, PxReal,		solverWakeCounter,		0)
	PX_DEF_BIN_METADATA_ITEM(stream, 		PxsBodyCore, PxU32,			numCountedInteractions,	0)*/

	ShadowPxsBodyCore::getBinaryMetaData(stream);
	PX_DEF_BIN_METADATA_TYPEDEF(stream,	PxsBodyCore, ShadowPxsBodyCore)
}

/*
We need to fix the header deps by moving the API out of PhysXCore and into its own dir where other code can get to it.
[25.08.2010 18:34:57] Dilip Sequeira: In the meantime, I think it's Ok to include PxSDK.h, but you're right, we need to be very careful about include deps in that direction.
[25.08.2010 18:38:15] Dilip Sequeira: On the memory thing... PxsBodyCore has 28 bytes of padding at the end, for no reason. In addition, it has two words of padding after the velocity fields, to facilitate SIMD loads. But in fact, Vec3FromVec4 is fast enough such that unless you were using it in an inner loop (which we never are with PxsBodyCore) that padding isn't worth it.
[25.08.2010 18:38:58] Dilip Sequeira: So, we should drop the end-padding, and move the damping values to replace the velocity padding. This probably requires a bit of fixup in the places where we do SIMD writes to the velocity.
[25.08.2010 18:39:18] Dilip Sequeira: Then we're down to 92 bytes of data, and 4 bytes of padding I think.
[25.08.2010 18:50:41] Dilip Sequeira: The reason we don't want to put the sleep data there explicitly is that it isn't LL data so I'd rather not have it in an LL interface struct.
[25.08.2010 19:04:53] Gordon Yeoman nvidia: simd loads are faster when they are 16-byte aligned.  I think the padding might be to ensure the second vector is also 16-byte aligned.  We could drop the second 4-byte pad but dropping the 1st 4-byte pad will likely  have performance implications.
[25.08.2010 19:06:22] Dilip Sequeira: We should still align the vec3s, as now - but we shouldn't use padding to do it, since there are a boatload of scalar data fields floating around in that struct too.
*/
void Sc::BodyCore::getBinaryMetaData(PxOutputStream& stream)
{
	getBinaryMetaData_PxsBodyCore(stream);
	PX_DEF_BIN_METADATA_TYPEDEF(stream, PxRigidBodyFlags, PxU8)

// 176 => 144 bytes
	PX_DEF_BIN_METADATA_CLASS(stream,		Sc::BodyCore)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	Sc::BodyCore, Sc::RigidCore)

	PX_DEF_BIN_METADATA_ITEM(stream,		Sc::BodyCore, PxsBodyCore,	mCore,					0)
	PX_DEF_BIN_METADATA_ITEM(stream,		Sc::BodyCore, SimStateData,	mSimStateData,			PxMetaDataFlag::ePTR)
}

///////////////////////////////////////////////////////////////////////////////

void Sc::ConstraintCore::getBinaryMetaData(PxOutputStream& stream)
{   
	PX_DEF_BIN_METADATA_TYPEDEF(stream, PxConstraintFlags, PxU16)

	PX_DEF_BIN_METADATA_CLASS(stream,	ConstraintCore)

	PX_DEF_BIN_METADATA_ITEM(stream,	ConstraintCore, PxConstraintFlags,		mFlags,					0)
	PX_DEF_BIN_METADATA_ITEM(stream,	ConstraintCore, PxU16,					mPaddingFromFlags,		PxMetaDataFlag::ePADDING)
	PX_DEF_BIN_METADATA_ITEM(stream,	ConstraintCore, PxVec3,					mAppliedForce,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,	ConstraintCore, PxVec3,					mAppliedTorque,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,	ConstraintCore, PxConstraintConnector,	mConnector,				PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,	ConstraintCore, PxConstraintProject,	mProject,				PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,	ConstraintCore, PxConstraintSolverPrep,	mSolverPrep,			PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,	ConstraintCore, PxConstraintVisualize,	mVisualize,				PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,	ConstraintCore, PxU32,					mDataSize,				0)
	PX_DEF_BIN_METADATA_ITEM(stream,	ConstraintCore, PxReal,					mLinearBreakForce,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,	ConstraintCore, PxReal,					mAngularBreakForce,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,	ConstraintCore, PxReal,					mMinResponseThreshold,	0)
		
	PX_DEF_BIN_METADATA_ITEM(stream,	ConstraintCore, ConstraintSim,			mSim,					PxMetaDataFlag::ePTR)
}

///////////////////////////////////////////////////////////////////////////////

void Sc::MaterialCore::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_TYPEDEF(stream,	PxCombineMode::Enum, PxU32)
	PX_DEF_BIN_METADATA_TYPEDEF(stream,	PxMaterialFlags, PxU16)

	PX_DEF_BIN_METADATA_CLASS(stream,	MaterialCore)

	// MaterialData
	PX_DEF_BIN_METADATA_ITEM(stream,	MaterialCore, PxReal,			dynamicFriction,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,	MaterialCore, PxReal,			staticFriction,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,	MaterialCore, PxReal,			restitution,			0)

	PX_DEF_BIN_METADATA_ITEM(stream,	MaterialCore, PxMaterialFlags,	flags,					0)
	PX_DEF_BIN_METADATA_ITEM(stream,	MaterialCore, PxU8,				fricRestCombineMode,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	MaterialCore, PxU8,				padding,				PxMetaDataFlag::ePADDING)

	// MaterialCore
	PX_DEF_BIN_METADATA_ITEM(stream,	MaterialCore, PxMaterial,		mNxMaterial,			PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,	MaterialCore, PxU32,			mMaterialIndex,			0)
}

///////////////////////////////////////////////////////////////////////////////

void Sc::RigidCore::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,		Sc::RigidCore)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	Sc::RigidCore, Sc::ActorCore)
}


///////////////////////////////////////////////////////////////////////////////

void Sc::StaticCore::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,		Sc::StaticCore)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	Sc::StaticCore, Sc::RigidCore)

	PX_DEF_BIN_METADATA_ITEM(stream,		Sc::StaticCore, PxsRigidCore,		mCore,					0)

}

///////////////////////////////////////////////////////////////////////////////

static void getBinaryMetaData_PxFilterData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,	PxFilterData)

	PX_DEF_BIN_METADATA_ITEM(stream,	PxFilterData, PxU32,		word0,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxFilterData, PxU32,		word1,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxFilterData, PxU32,		word2,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxFilterData, PxU32,		word3,		0)
}

static void getBinaryMetaData_PxsShapeCore(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,		PxsShapeCore)

	PX_DEF_BIN_METADATA_ITEM(stream,		PxsShapeCore,	PxTransform,		transform,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsShapeCore,	Gu::GeometryUnion,	geometry,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsShapeCore,	PxReal,				contactOffset,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsShapeCore,	PxShapeFlags,		mShapeFlags,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsShapeCore,	PxU8,				mOwnsMaterialIdxMemory,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsShapeCore,	PxU16,				materialIndex,	0)
}

void Sc::ShapeCore::getBinaryMetaData(PxOutputStream& stream)
{
	getBinaryMetaData_PxFilterData(stream);
	getBinaryMetaData_PxsShapeCore(stream);

	PX_DEF_BIN_METADATA_TYPEDEF(stream, PxShapeFlags, PxU8)

// 144 => 128 bytes
	PX_DEF_BIN_METADATA_CLASS(stream,	ShapeCore)

	PX_DEF_BIN_METADATA_ITEM(stream,	ShapeCore, PxFilterData,	mQueryFilterData,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,	ShapeCore, PxFilterData,	mSimulationFilterData,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	ShapeCore, PxsShapeCore,	mCore,					0)
	PX_DEF_BIN_METADATA_ITEM(stream,	ShapeCore, PxReal,		    mRestOffset,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,	ShapeCore, PxReal,			mTorsionalRadius,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,	ShapeCore, PxReal,			mMinTorsionalPatchRadius, 0)
}

///////////////////////////////////////////////////////////////////////////////

static void getBinaryMetaData_ArticulationCore(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream, Dy::ArticulationCore)

	PX_DEF_BIN_METADATA_TYPEDEF(stream, PxArticulationFlags, PxU8)

	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationCore, PxU32, internalDriveIterations, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationCore, PxU32, externalDriveIterations, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationCore, PxU32, maxProjectionIterations, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationCore, PxU16, solverIterationCounts, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationCore, PxReal, separationTolerance, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationCore, PxReal, sleepThreshold, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationCore, PxReal, freezeThreshold, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationCore, PxReal, wakeCounter, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationCore, PxArticulationFlags, flags, 0)
}

void Sc::ArticulationCore::getBinaryMetaData(PxOutputStream& stream)
{
	getBinaryMetaData_ArticulationCore(stream);

	PX_DEF_BIN_METADATA_CLASS(stream,	ArticulationCore)

	PX_DEF_BIN_METADATA_ITEM(stream,	ArticulationCore, ArticulationSim,		mSim,		PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,	ArticulationCore, Dy::ArticulationCore,	mCore,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,	ArticulationCore, PxU32,				mType,		0)
}

///////////////////////////////////////////////////////////////////////////////

static void getBinaryMetaData_ArticulationLimit(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream, Dy::ArticulationLimit)
	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationLimit, PxReal, low, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationLimit, PxReal, high, 0)
}

static void getBinaryMetaData_ArticulationDrive(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream, Dy::ArticulationDrive)
	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationDrive, PxReal, stiffness, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationDrive, PxReal, damping, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationDrive, PxReal, maxForce, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationDrive, bool, isAcceleration, 0)
}

static void getBinaryMetaData_ArticulationJointCoreBase(PxOutputStream& stream)
{
	getBinaryMetaData_ArticulationLimit(stream);
	getBinaryMetaData_ArticulationDrive(stream);

	PX_DEF_BIN_METADATA_CLASS(stream, Dy::ArticulationJointCoreBase)	

	PX_DEF_BIN_METADATA_TYPEDEF(stream, ArticulationJointCoreDirtyFlags, PxU8)
	PX_DEF_BIN_METADATA_TYPEDEF(stream, PxArticulationMotions, PxU8)

	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationJointCoreBase, PxTransform, parentPose, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationJointCoreBase, PxTransform, childPose, 0)

	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream, Dy::ArticulationJointCoreBase, Dy::ArticulationLimit, limits, 0)
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream, Dy::ArticulationJointCoreBase, Dy::ArticulationDrive, drives, 0)

	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream, Dy::ArticulationJointCoreBase, PxReal, targetP, 0)
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream, Dy::ArticulationJointCoreBase, PxReal, targetV, 0)

	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationJointCoreBase, PxQuat, relativeQuat, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationJointCoreBase, PxReal, frictionCoefficient, 0)

	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationJointCoreBase, PxU32, jointOffset, 0)

	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream, Dy::ArticulationJointCoreBase, PxU8, dofIds, 0)

	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream, Dy::ArticulationJointCoreBase, PxArticulationMotions, motion, 0)

	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationJointCoreBase, PxReal, maxJointVelocity, 0)

	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationJointCoreBase, ArticulationJointCoreDirtyFlags, dirtyFlag, 0)

	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationJointCoreBase, bool, prismaticLimited, 0)

	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationJointCore, PxU8, jointType, 0)
}


static void getBinaryMetaData_ArticulationJointCore(PxOutputStream& stream)
{	
	getBinaryMetaData_ArticulationJointCoreBase(stream);
	PX_DEF_BIN_METADATA_CLASS(stream, Dy::ArticulationJointCore)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream, Dy::ArticulationJointCore, Dy::ArticulationJointCoreBase)
	
	PX_DEF_BIN_METADATA_ITEM(stream,		Dy::ArticulationJointCore, PxQuat, targetPosition,					0)
	PX_DEF_BIN_METADATA_ITEM(stream,		Dy::ArticulationJointCore, PxVec3, targetVelocity,					0)
	
	PX_DEF_BIN_METADATA_ITEM(stream,		Dy::ArticulationJointCore, PxReal, spring, 0)
	PX_DEF_BIN_METADATA_ITEM(stream,		Dy::ArticulationJointCore, PxReal, damping, 0)
	PX_DEF_BIN_METADATA_ITEM(stream,		Dy::ArticulationJointCore, PxReal, internalCompliance, 0)
	PX_DEF_BIN_METADATA_ITEM(stream,		Dy::ArticulationJointCore, PxReal, externalCompliance, 0)

	PX_DEF_BIN_METADATA_ITEM(stream,		Dy::ArticulationJointCore, PxReal, swingLimitContactDistance, 0)
	PX_DEF_BIN_METADATA_ITEM(stream,		Dy::ArticulationJointCore, PxReal, tangentialStiffness, 0)
	PX_DEF_BIN_METADATA_ITEM(stream,		Dy::ArticulationJointCore, PxReal, tangentialDamping, 0)

	PX_DEF_BIN_METADATA_ITEM(stream,		Dy::ArticulationJointCore, bool, swingLimited, 0)
	PX_DEF_BIN_METADATA_ITEM(stream,		Dy::ArticulationJointCore, bool, twistLimited, 0)
	
	PX_DEF_BIN_METADATA_ITEM(stream,		Dy::ArticulationJointCore, PxU8, driveType, 0)

	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationJointCore, PxReal, twistLimitContactDistance, 0)

	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationJointCore, PxReal, tanQSwingY, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationJointCore, PxReal, tanQSwingZ, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationJointCore, PxReal, tanQSwingPad, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationJointCore, PxReal, tanQTwistHigh, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationJointCore, PxReal, tanQTwistLow, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationJointCore, PxReal, tanQTwistPad, 0)
}

void Sc::ArticulationJointCore::getBinaryMetaData(PxOutputStream& stream)
{
	getBinaryMetaData_ArticulationJointCore(stream);
	PX_DEF_BIN_METADATA_CLASS(stream,	ArticulationJointCore)
	PX_DEF_BIN_METADATA_ITEM(stream,	ArticulationJointCore, ArticulationJointSim,		mSim,		PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,	ArticulationJointCore, Dy::ArticulationJointCore,	mCore,		0)
	PX_DEF_BIN_METADATA_ITEM(stream, ArticulationJointCore, Dy::ArticulationCore, mArticulation, PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream, ArticulationJointCore, Dy::PxArticulationJointBase, mRootType, PxMetaDataFlag::ePTR)
}

///////////////////////////////////////////////////////////////////////////////

#define PX_DEF_BIN_METADATA_ARRAY(stream, Class, type, array)	\
{ PxMetaDataEntry tmp = {"void", #array".mData", PxU32(PX_OFFSET_OF(Class, array)) + PxMetaDataArray<type>::getDataOffset(), PxMetaDataArray<type>::getDataSize(), 1, 0, PxMetaDataFlag::ePTR, 0};							PX_STORE_METADATA(stream, tmp); } \
{ PxMetaDataEntry tmp = {"PxU32", #array".mSize", PxU32(PX_OFFSET_OF(Class, array)) + PxMetaDataArray<type>::getSizeOffset(), PxMetaDataArray<type>::getSizeSize(), 1, 0, 0, 0};											PX_STORE_METADATA(stream, tmp);	} \
{ PxMetaDataEntry tmp = {"PxU32", #array".mCapacity", PxU32(PX_OFFSET_OF(Class, array)) + PxMetaDataArray<type>::getCapacityOffset(), PxMetaDataArray<type>::getCapacitySize(), 1, 0, PxMetaDataFlag::eCOUNT_MASK_MSB, 0};	PX_STORE_METADATA(stream, tmp);	} \
{ PxMetaDataEntry tmp = {#type, 0, PxU32(PX_OFFSET_OF(Class, array)) + PxMetaDataArray<type>::getSizeOffset(), PxMetaDataArray<type>::getSizeSize(), 0, 0, PxMetaDataFlag::eEXTRA_DATA, 0};									PX_STORE_METADATA(stream, tmp); }





