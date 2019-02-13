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
#include "ScbShape.h"
#include "ScbBody.h"
#include "ScbRigidStatic.h"
#include "ScbConstraint.h"
#include "ScbArticulation.h"
#include "ScbArticulationJoint.h"
#include "ScbAggregate.h"

using namespace physx;

///////////////////////////////////////////////////////////////////////////////

void Scb::Base::getBinaryMetaData(PxOutputStream& stream)
{
	// 28 => 12 bytes
	PX_DEF_BIN_METADATA_TYPEDEF(stream,	ScbType::Enum, PxU32)

	PX_DEF_BIN_METADATA_CLASS(stream,	Scb::Base)

	PX_DEF_BIN_METADATA_ITEM(stream,	Scb::Base, Scb::Scene,	mScene,			PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,	Scb::Base, PxU32,		mControlState,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	Scb::Base, PxU8*,		mStreamPtr,		PxMetaDataFlag::ePTR)
}

///////////////////////////////////////////////////////////////////////////////

void Scb::Shape::getBinaryMetaData(PxOutputStream& stream)
{
	// 176 => 160 bytes
	PX_DEF_BIN_METADATA_CLASS(stream,		Scb::Shape)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	Scb::Shape, Scb::Base)
	PX_DEF_BIN_METADATA_ITEM(stream,		Scb::Shape, ShapeCore,	mShape,	0)
}

///////////////////////////////////////////////////////////////////////////////

void Scb::Actor::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,		Scb::Actor)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	Scb::Actor, Scb::Base)
}

///////////////////////////////////////////////////////////////////////////////

void Scb::RigidObject::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,		Scb::RigidObject)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	Scb::RigidObject, Scb::Actor)
}

///////////////////////////////////////////////////////////////////////////////

void Scb::Body::getBinaryMetaData(PxOutputStream& stream)
{
	// 240 => 224 bytes
	PX_DEF_BIN_METADATA_CLASS(stream,		Scb::Body)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	Scb::Body, Scb::RigidObject)

#ifdef EXPLICIT_PADDING_METADATA
	PX_DEF_BIN_METADATA_ITEM(stream,		Scb::Body, PxU32,			mPaddingScbBody1,		PxMetaDataFlag::ePADDING)
#endif
	PX_DEF_BIN_METADATA_ITEM(stream,		Scb::Body, Sc::BodyCore,	mBodyCore,				0)
	PX_DEF_BIN_METADATA_ITEM(stream,		Scb::Body, PxTransform,		mBufferedBody2World,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		Scb::Body, PxVec3,			mBufferedLinVelocity,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		Scb::Body, PxVec3,			mBufferedAngVelocity,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		Scb::Body, PxReal,			mBufferedWakeCounter,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		Scb::Body, PxU32,			mBufferedIsSleeping,  	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		Scb::Body, PxU32,			mBodyBufferFlags,		0)
}

///////////////////////////////////////////////////////////////////////////////

void Scb::RigidStatic::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,		Scb::RigidStatic)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	Scb::RigidStatic, Scb::RigidObject)

	PX_DEF_BIN_METADATA_ITEM(stream,		Scb::RigidStatic, Sc::StaticCore,	mStatic,		0)
}

///////////////////////////////////////////////////////////////////////////////

void Scb::Articulation::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,		Scb::Articulation)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	Scb::Articulation, Scb::Base)

	PX_DEF_BIN_METADATA_ITEM(stream,		Scb::Articulation, ArticulationCore,	mArticulation,				0)
	PX_DEF_BIN_METADATA_ITEM(stream,		Scb::Articulation, PxReal,				mBufferedWakeCounter,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,		Scb::Articulation, PxU8,				mBufferedIsSleeping,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,		Scb::Articulation, PxArticulationFlags,	mBufferedArticulationFlags,	0)
}

///////////////////////////////////////////////////////////////////////////////

void Scb::ArticulationJoint::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,		Scb::ArticulationJoint)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	Scb::ArticulationJoint, Scb::Base)

	PX_DEF_BIN_METADATA_ITEM(stream,		Scb::ArticulationJoint, ArticulationJointCore,	mJoint,			0)
}

///////////////////////////////////////////////////////////////////////////////

void Scb::Constraint::getBinaryMetaData(PxOutputStream& stream)
{
	// 120 => 108 bytes
	PX_DEF_BIN_METADATA_CLASS(stream,		Scb::Constraint)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	Scb::Constraint, Scb::Base)

	PX_DEF_BIN_METADATA_ITEM(stream,		Scb::Constraint, ConstraintCore,	mConstraint,				0)
	PX_DEF_BIN_METADATA_ITEM(stream,		Scb::Constraint, PxVec3,			mBufferedForce,				0)
	PX_DEF_BIN_METADATA_ITEM(stream,		Scb::Constraint, PxVec3,			mBufferedTorque,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,		Scb::Constraint, PxConstraintFlags,	mBrokenFlag,				0)
#ifdef EXPLICIT_PADDING_METADATA
	PX_DEF_BIN_METADATA_ITEM(stream,		Scb::Constraint, PxU16,				mPaddingFromBrokenFlags,	PxMetaDataFlag::ePADDING)
#endif
}

///////////////////////////////////////////////////////////////////////////////

void Scb::Aggregate::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,		Scb::Aggregate)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	Scb::Aggregate, Scb::Base)

	PX_DEF_BIN_METADATA_ITEM(stream,		Scb::Aggregate, PxAggregate,mPxAggregate,	PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,		Scb::Aggregate, PxU32,		mAggregateID,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		Scb::Aggregate, PxU32,		mMaxNbActors,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		Scb::Aggregate, bool,		mSelfCollide,	0)

#ifdef EXPLICIT_PADDING_METADATA
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	Scb::Aggregate, bool,		mPaddingFromBool,	PxMetaDataFlag::ePADDING)
#endif
}

///////////////////////////////////////////////////////////////////////////////

