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

#include "ExtFixedJoint.h"
#include "ExtConstraintHelper.h"
#include "CmVisualization.h"

#include "common/PxSerialFramework.h"

using namespace physx;
using namespace Ext;

PxFixedJoint* physx::PxFixedJointCreate(PxPhysics& physics, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1)
{
	PX_CHECK_AND_RETURN_NULL(localFrame0.isSane(), "PxFixedJointCreate: local frame 0 is not a valid transform"); 
	PX_CHECK_AND_RETURN_NULL(localFrame1.isSane(), "PxFixedJointCreate: local frame 1 is not a valid transform"); 
	PX_CHECK_AND_RETURN_NULL((actor0 && actor0->is<PxRigidBody>()) || (actor1 && actor1->is<PxRigidBody>()), "PxFixedJointCreate: at least one actor must be dynamic");
	PX_CHECK_AND_RETURN_NULL(actor0 != actor1, "PxFixedJointCreate: actors must be different");

	FixedJoint* j;
	PX_NEW_SERIALIZED(j, FixedJoint)(physics.getTolerancesScale(), actor0, localFrame0, actor1, localFrame1);

	if(j->attach(physics, actor0, actor1))
		return j;

	PX_DELETE(j);
	return NULL;
}

PxReal FixedJoint::getProjectionLinearTolerance() const
{ 
	return data().projectionLinearTolerance; 
}

void FixedJoint::setProjectionLinearTolerance(PxReal tolerance)
{ 
	PX_CHECK_AND_RETURN(PxIsFinite(tolerance) && tolerance >=0, "PxFixedJoint::setProjectionLinearTolerance: invalid parameter");
	data().projectionLinearTolerance = tolerance; 
	markDirty(); 
}

PxReal FixedJoint::getProjectionAngularTolerance() const
{ 
	return data().projectionAngularTolerance; 
}

void FixedJoint::setProjectionAngularTolerance(PxReal tolerance)	
{ 
	PX_CHECK_AND_RETURN(PxIsFinite(tolerance) && tolerance >=0 && tolerance <= PxPi, "PxFixedJoint::setProjectionAngularTolerance: invalid parameter");
	data().projectionAngularTolerance = tolerance; markDirty(); 
}

bool FixedJoint::attach(PxPhysics &physics, PxRigidActor* actor0, PxRigidActor* actor1)
{
	mPxConstraint = physics.createConstraint(actor0, actor1, *this, sShaders, sizeof(FixedJointData));
	return mPxConstraint!=NULL;
}

void FixedJoint::exportExtraData(PxSerializationContext& stream) const
{
	if(mData)
	{
		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(mData, sizeof(FixedJointData));
	}
	stream.writeName(mName);
}

void FixedJoint::importExtraData(PxDeserializationContext& context)
{
	if(mData)
		mData = context.readExtraData<FixedJointData, PX_SERIAL_ALIGN>();
	context.readName(mName);
}

void FixedJoint::resolveReferences(PxDeserializationContext& context)
{
	setPxConstraint(resolveConstraintPtr(context, getPxConstraint(), getConnector(), sShaders));	
}

FixedJoint* FixedJoint::createObject(PxU8*& address, PxDeserializationContext& context)
{
	FixedJoint* obj = new (address) FixedJoint(PxBaseFlag::eIS_RELEASABLE);
	address += sizeof(FixedJoint);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}

// global function to share the joint shaders with API capture	
const PxConstraintShaderTable* Ext::GetFixedJointShaderTable() 
{ 
	return &FixedJoint::getConstraintShaderTable();
}

//~PX_SERIALIZATION

static void FixedJointProject(const void* constantBlock, PxTransform& bodyAToWorld, PxTransform& bodyBToWorld, bool projectToA)
{
	const FixedJointData &data = *reinterpret_cast<const FixedJointData*>(constantBlock);

	PxTransform cA2w, cB2w, cB2cA, projected;
	joint::computeDerived(data, bodyAToWorld, bodyBToWorld, cA2w, cB2w, cB2cA);

	bool linearTrunc, angularTrunc;
	projected.p = joint::truncateLinear(cB2cA.p, data.projectionLinearTolerance, linearTrunc);
	projected.q = joint::truncateAngular(cB2cA.q, PxSin(data.projectionAngularTolerance/2), PxCos(data.projectionAngularTolerance/2), angularTrunc);
	
	if(linearTrunc || angularTrunc)
		joint::projectTransforms(bodyAToWorld, bodyBToWorld, cA2w, cB2w, projected, data, projectToA);
}

static void FixedJointVisualize(PxConstraintVisualizer& viz, const void* constantBlock, const PxTransform& body0Transform, const PxTransform& body1Transform, PxU32 flags)
{
	if(flags & PxConstraintVisualizationFlag::eLOCAL_FRAMES)
	{
		const FixedJointData& data = *reinterpret_cast<const FixedJointData*>(constantBlock);

		PxTransform cA2w, cB2w;
		joint::computeJointFrames(cA2w, cB2w, data, body0Transform, body1Transform);
		viz.visualizeJointFrames(cA2w, cB2w);
	}
}

static PxU32 FixedJointSolverPrep(Px1DConstraint* constraints,
	PxVec3& body0WorldOffset,
	PxU32 /*maxConstraints*/,
	PxConstraintInvMassScale& invMassScale,
	const void* constantBlock,
	const PxTransform& bA2w,
	const PxTransform& bB2w,
	bool /*useExtendedLimits*/,
	PxVec3& cA2wOut, PxVec3& cB2wOut)
{
	const FixedJointData& data = *reinterpret_cast<const FixedJointData*>(constantBlock);

	PxTransform cA2w, cB2w;
	joint::ConstraintHelper ch(constraints, invMassScale, cA2w, cB2w, body0WorldOffset, data, bA2w, bB2w);

	if (cA2w.q.dot(cB2w.q)<0.0f)	// minimum dist quat (equiv to flipping cB2bB.q, which we don't use anywhere)
		cB2w.q = -cB2w.q;

	PxVec3 ra, rb;
	ch.prepareLockedAxes(cA2w.q, cB2w.q, cA2w.transformInv(cB2w.p), 7, 7, ra, rb);
	cA2wOut = ra + bA2w.p;
	cB2wOut = rb + bB2w.p;


	return ch.getCount();
}

PxConstraintShaderTable Ext::FixedJoint::sShaders = { FixedJointSolverPrep, FixedJointProject, FixedJointVisualize, PxConstraintFlag::Enum(0) };
