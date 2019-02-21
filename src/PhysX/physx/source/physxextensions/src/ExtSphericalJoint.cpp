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

#include "ExtSphericalJoint.h"
#include "ExtConstraintHelper.h"
#include "CmConeLimitHelper.h"
#include "CmRenderOutput.h"
#include "CmVisualization.h"

#include "common/PxSerialFramework.h"

using namespace physx;
using namespace Ext;

PxSphericalJoint* physx::PxSphericalJointCreate(PxPhysics& physics, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1)
{
	PX_CHECK_AND_RETURN_NULL(localFrame0.isSane(), "PxSphericalJointCreate: local frame 0 is not a valid transform"); 
	PX_CHECK_AND_RETURN_NULL(localFrame1.isSane(), "PxSphericalJointCreate: local frame 1 is not a valid transform"); 
	PX_CHECK_AND_RETURN_NULL(actor0 != actor1, "PxSphericalJointCreate: actors must be different");
	PX_CHECK_AND_RETURN_NULL((actor0 && actor0->is<PxRigidBody>()) || (actor1 && actor1->is<PxRigidBody>()), "PxSphericalJointCreate: at least one actor must be dynamic");

	SphericalJoint* j;
	PX_NEW_SERIALIZED(j, SphericalJoint)(physics.getTolerancesScale(), actor0, localFrame0, actor1, localFrame1);

	if(j->attach(physics, actor0, actor1))
		return j;

	PX_DELETE(j);
	return NULL;
}

void SphericalJoint::setProjectionLinearTolerance(PxReal tolerance)
{	
	PX_CHECK_AND_RETURN(PxIsFinite(tolerance) && tolerance >=0, "PxSphericalJoint::setProjectionLinearTolerance: invalid parameter");
	data().projectionLinearTolerance = tolerance;
	markDirty(); 
}

PxReal SphericalJoint::getProjectionLinearTolerance() const	
{	
	return data().projectionLinearTolerance;		
}

void SphericalJoint::setLimitCone(const PxJointLimitCone &limit)
{	
	PX_CHECK_AND_RETURN(limit.isValid(), "PxSphericalJoint::setLimit: invalid parameter");
	data().limit = limit; 
	markDirty();
}

PxJointLimitCone SphericalJoint::getLimitCone() const
{	
	return data().limit; 
}

PxSphericalJointFlags SphericalJoint::getSphericalJointFlags(void) const
{ 
	return data().jointFlags; 
}

void SphericalJoint::setSphericalJointFlags(PxSphericalJointFlags flags)
{ 
	data().jointFlags = flags; 
}

void SphericalJoint::setSphericalJointFlag(PxSphericalJointFlag::Enum flag, bool value)
{
	if(value)
		data().jointFlags |= flag;
	else
		data().jointFlags &= ~flag;
	markDirty();
}

PxReal SphericalJoint::getSwingYAngle()	const
{
	return getSwingYAngle_Internal();
}

PxReal SphericalJoint::getSwingZAngle()	const
{
	return getSwingZAngle_Internal();
}

bool SphericalJoint::attach(PxPhysics &physics, PxRigidActor* actor0, PxRigidActor* actor1)
{
	mPxConstraint = physics.createConstraint(actor0, actor1, *this, sShaders, sizeof(SphericalJointData));
	return mPxConstraint!=NULL;
}

void SphericalJoint::exportExtraData(PxSerializationContext& stream)
{
	if(mData)
	{
		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(mData, sizeof(SphericalJointData));
	}
	stream.writeName(mName);
}

void SphericalJoint::importExtraData(PxDeserializationContext& context)
{
	if(mData)
		mData = context.readExtraData<SphericalJointData, PX_SERIAL_ALIGN>();
	context.readName(mName);
}

void SphericalJoint::resolveReferences(PxDeserializationContext& context)
{
	setPxConstraint(resolveConstraintPtr(context, getPxConstraint(), getConnector(), sShaders));	
}

SphericalJoint* SphericalJoint::createObject(PxU8*& address, PxDeserializationContext& context)
{
	SphericalJoint* obj = new (address) SphericalJoint(PxBaseFlag::eIS_RELEASABLE);
	address += sizeof(SphericalJoint);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}

// global function to share the joint shaders with API capture	
const PxConstraintShaderTable* Ext::GetSphericalJointShaderTable() 
{ 
	return &SphericalJoint::getConstraintShaderTable();
}

//~PX_SERIALIZATION

static void SphericalJointProject(const void* constantBlock, PxTransform& bodyAToWorld, PxTransform& bodyBToWorld, bool projectToA)
{
	const SphericalJointData& data = *reinterpret_cast<const SphericalJointData*>(constantBlock);

	PxTransform cA2w, cB2w, cB2cA, projected;
	joint::computeDerived(data, bodyAToWorld, bodyBToWorld, cA2w, cB2w, cB2cA);

	bool linearTrunc;
	projected.p = joint::truncateLinear(cB2cA.p, data.projectionLinearTolerance, linearTrunc);

	if(linearTrunc)
	{
		projected.q = cB2cA.q;
		joint::projectTransforms(bodyAToWorld, bodyBToWorld, cA2w, cB2w, projected, data, projectToA);
	}
}

static void SphericalJointVisualize(PxConstraintVisualizer& viz, const void* constantBlock, const PxTransform& body0Transform, const PxTransform& body1Transform, PxU32 flags)
{
	const SphericalJointData& data = *reinterpret_cast<const SphericalJointData*>(constantBlock);

	PxTransform cA2w, cB2w;
	joint::computeJointFrames(cA2w, cB2w, data, body0Transform, body1Transform);
	if(flags & PxConstraintVisualizationFlag::eLOCAL_FRAMES)
		viz.visualizeJointFrames(cA2w, cB2w);

	if((flags & PxConstraintVisualizationFlag::eLIMITS) && (data.jointFlags & PxSphericalJointFlag::eLIMIT_ENABLED))
	{
		if(cA2w.q.dot(cB2w.q)<0.0f)
			cB2w.q = -cB2w.q;

		const PxTransform cB2cA = cA2w.transformInv(cB2w);	
		PxQuat swing, twist;
		Ps::separateSwingTwist(cB2cA.q,swing,twist);

		// PT: TODO: refactor with D6 joint code
		const PxReal pad = data.limit.isSoft() ? 0.0f : data.limit.contactDistance;
		const PxVec3 swingAngle(0.0f, computeSwingAngle(swing.y, swing.w), computeSwingAngle(swing.z, swing.w));
		Cm::ConeLimitHelperTanLess coneHelper(data.limit.yAngle, data.limit.zAngle, pad);
		viz.visualizeLimitCone(cA2w, PxTan(data.limit.zAngle/4), PxTan(data.limit.yAngle/4), !coneHelper.contains(swingAngle));
	}
}

static PxU32 SphericalJointSolverPrep(Px1DConstraint* constraints,
	PxVec3& body0WorldOffset,
	PxU32 /*maxConstraints*/,
	PxConstraintInvMassScale& invMassScale,
	const void* constantBlock,							  
	const PxTransform& bA2w,
	const PxTransform& bB2w,
	bool /*useExtendedLimits*/,
	PxVec3& cA2wOut, PxVec3& cB2wOut)
{
	const SphericalJointData& data = *reinterpret_cast<const SphericalJointData*>(constantBlock);

	PxTransform cA2w, cB2w;
	joint::ConstraintHelper ch(constraints, invMassScale, cA2w, cB2w, body0WorldOffset, data, bA2w, bB2w);

	if(cB2w.q.dot(cA2w.q)<0.0f)
		cB2w.q = -cB2w.q;

	if(data.jointFlags & PxSphericalJointFlag::eLIMIT_ENABLED)
	{
		PxQuat swing, twist;
		Ps::separateSwingTwist(cA2w.q.getConjugate() * cB2w.q, swing, twist);
		PX_ASSERT(PxAbs(swing.x)<1e-6f);

		// PT: TODO: refactor with D6 joint code
		PxVec3 axis;
		PxReal error;
		const PxReal pad = data.limit.isSoft() ? 0.0f : data.limit.contactDistance;
		const Cm::ConeLimitHelperTanLess coneHelper(data.limit.yAngle, data.limit.zAngle, pad);
		const bool active = coneHelper.getLimit(swing, axis, error);				
		if(active)
			ch.angularLimit(cA2w.rotate(axis), error, data.limit);
	}

	PxVec3 ra, rb;
	ch.prepareLockedAxes(cA2w.q, cB2w.q, cA2w.transformInv(cB2w.p), 7, 0, ra, rb);
	cA2wOut = ra + bA2w.p;
	cB2wOut = rb + bB2w.p;

	return ch.getCount();
}

PxConstraintShaderTable Ext::SphericalJoint::sShaders = { SphericalJointSolverPrep, SphericalJointProject, SphericalJointVisualize, PxConstraintFlag::Enum(0) };
