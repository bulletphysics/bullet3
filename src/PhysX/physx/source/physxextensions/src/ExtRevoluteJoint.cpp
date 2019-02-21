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

#include "ExtRevoluteJoint.h"
#include "PsUtilities.h"
#include "ExtConstraintHelper.h"
#include "CmRenderOutput.h"
#include "PsMathUtils.h"
#include "CmVisualization.h"
#include "CmUtils.h"

#include "common/PxSerialFramework.h"

using namespace physx;
using namespace Ext;

PxRevoluteJoint* physx::PxRevoluteJointCreate(PxPhysics& physics, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1)
{
	PX_CHECK_AND_RETURN_NULL(localFrame0.isSane(), "PxRevoluteJointCreate: local frame 0 is not a valid transform"); 
	PX_CHECK_AND_RETURN_NULL(localFrame1.isSane(), "PxRevoluteJointCreate: local frame 1 is not a valid transform"); 
	PX_CHECK_AND_RETURN_NULL(actor0 != actor1, "PxRevoluteJointCreate: actors must be different");
	PX_CHECK_AND_RETURN_NULL((actor0 && actor0->is<PxRigidBody>()) || (actor1 && actor1->is<PxRigidBody>()), "PxRevoluteJointCreate: at least one actor must be dynamic");

	RevoluteJoint* j;
	PX_NEW_SERIALIZED(j, RevoluteJoint)(physics.getTolerancesScale(), actor0, localFrame0, actor1, localFrame1);
	if(j->attach(physics, actor0, actor1))
		return j;

	PX_DELETE(j);
	return NULL;
}

PxReal RevoluteJoint::getAngle() const
{
	return getTwistAngle_Internal();
}

PxReal RevoluteJoint::getVelocity() const
{
	return getRelativeAngularVelocity().magnitude();
}

PxJointAngularLimitPair RevoluteJoint::getLimit()	const
{ 
	return data().limit;	
}

void RevoluteJoint::setLimit(const PxJointAngularLimitPair& limit)
{ 
	PX_CHECK_AND_RETURN(limit.isValid(), "PxRevoluteJoint::setLimit: limit invalid");
	PX_CHECK_AND_RETURN(limit.lower>-PxTwoPi && limit.upper<PxTwoPi , "PxRevoluteJoint::twist limit must be strictly between -2*PI and 2*PI");

	data().limit = limit; 
	markDirty();	
}

PxReal RevoluteJoint::getDriveVelocity() const
{ 
	return data().driveVelocity;
}

void RevoluteJoint::setDriveVelocity(PxReal velocity, bool autowake)
{ 
	PX_CHECK_AND_RETURN(PxIsFinite(velocity), "PxRevoluteJoint::setDriveVelocity: invalid parameter");
	data().driveVelocity = velocity; 
	if(autowake)
		wakeUpActors();
	markDirty(); 
}

PxReal RevoluteJoint::getDriveForceLimit() const
{ 
	return data().driveForceLimit;	
}

void RevoluteJoint::setDriveForceLimit(PxReal forceLimit)
{ 
	PX_CHECK_AND_RETURN(PxIsFinite(forceLimit), "PxRevoluteJoint::setDriveForceLimit: invalid parameter");
	data().driveForceLimit = forceLimit; 
	markDirty(); 
}

PxReal RevoluteJoint::getDriveGearRatio() const
{ 
	return data().driveGearRatio;	
}

void RevoluteJoint::setDriveGearRatio(PxReal gearRatio)
{ 
	PX_CHECK_AND_RETURN(PxIsFinite(gearRatio) && gearRatio>0, "PxRevoluteJoint::setDriveGearRatio: invalid parameter");
	data().driveGearRatio = gearRatio; 
	markDirty(); 
}

void RevoluteJoint::setProjectionAngularTolerance(PxReal tolerance)
{ 
	PX_CHECK_AND_RETURN(PxIsFinite(tolerance) && tolerance>=0 && tolerance<=PxPi, "PxRevoluteJoint::setProjectionAngularTolerance: invalid parameter");
	data().projectionAngularTolerance = tolerance;
	markDirty();	
}

PxReal RevoluteJoint::getProjectionAngularTolerance() const	
{ 
	return data().projectionAngularTolerance; 
}

void RevoluteJoint::setProjectionLinearTolerance(PxReal tolerance)
{ 
	PX_CHECK_AND_RETURN(PxIsFinite(tolerance) && tolerance >=0, "PxRevoluteJoint::setProjectionLinearTolerance: invalid parameter");
	data().projectionLinearTolerance = tolerance;
	markDirty(); 
}

PxReal RevoluteJoint::getProjectionLinearTolerance() const
{ 
	return data().projectionLinearTolerance;		
}

PxRevoluteJointFlags RevoluteJoint::getRevoluteJointFlags(void)	const
{ 
	return data().jointFlags; 
}

void RevoluteJoint::setRevoluteJointFlags(PxRevoluteJointFlags flags)
{ 
	data().jointFlags = flags; 
}

void RevoluteJoint::setRevoluteJointFlag(PxRevoluteJointFlag::Enum flag, bool value)
{
	if(value)
		data().jointFlags |= flag;
	else
		data().jointFlags &= ~flag;
	markDirty();
}

bool RevoluteJoint::attach(PxPhysics &physics, PxRigidActor* actor0, PxRigidActor* actor1)
{
	mPxConstraint = physics.createConstraint(actor0, actor1, *this, sShaders, sizeof(RevoluteJointData));
	return mPxConstraint!=NULL;
}

void RevoluteJoint::exportExtraData(PxSerializationContext& stream)
{
	if(mData)
	{
		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(mData, sizeof(RevoluteJointData));
	}
	stream.writeName(mName);
}

void RevoluteJoint::importExtraData(PxDeserializationContext& context)
{
	if(mData)
		mData = context.readExtraData<RevoluteJointData, PX_SERIAL_ALIGN>();
	context.readName(mName);
}

void RevoluteJoint::resolveReferences(PxDeserializationContext& context)
{
	setPxConstraint(resolveConstraintPtr(context, getPxConstraint(), getConnector(), sShaders));	
}

RevoluteJoint* RevoluteJoint::createObject(PxU8*& address, PxDeserializationContext& context)
{
	RevoluteJoint* obj = new (address) RevoluteJoint(PxBaseFlag::eIS_RELEASABLE);
	address += sizeof(RevoluteJoint);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}

// global function to share the joint shaders with API capture	
const PxConstraintShaderTable* Ext::GetRevoluteJointShaderTable() 
{ 
	return &RevoluteJoint::getConstraintShaderTable();
}

//~PX_SERIALIZATION

static void RevoluteJointProject(const void* constantBlock, PxTransform& bodyAToWorld, PxTransform& bodyBToWorld, bool projectToA)
{
	const RevoluteJointData& data = *reinterpret_cast<const RevoluteJointData*>(constantBlock);

	PxTransform cA2w, cB2w, cB2cA, projected;
	joint::computeDerived(data, bodyAToWorld, bodyBToWorld, cA2w, cB2w, cB2cA, false);

	bool linearTrunc, angularTrunc;
	projected.p = joint::truncateLinear(cB2cA.p, data.projectionLinearTolerance, linearTrunc);

	PxQuat swing, twist, projSwing;
	Ps::separateSwingTwist(cB2cA.q, swing, twist);
	projSwing = joint::truncateAngular(swing, PxSin(data.projectionAngularTolerance/2), PxCos(data.projectionAngularTolerance/2), angularTrunc);
	
	if(linearTrunc || angularTrunc)
	{
		projected.q = projSwing * twist;
		joint::projectTransforms(bodyAToWorld, bodyBToWorld, cA2w, cB2w, projected, data, projectToA);
	}
}

static PxQuat computeTwist(const PxTransform& cA2w, const PxTransform& cB2w)
{
	// PT: following code is the same as this part of the "getAngle" function:
	//	const PxQuat q = getRelativeTransform().q;
	//	PxQuat swing, twist;
	//	Ps::separateSwingTwist(q, swing, twist);
	// But it's done a little bit more efficiently since we don't need the swing quat.

	// PT: rotation part of "const PxTransform cB2cA = cA2w.transformInv(cB2w);"
	const PxQuat cB2cAq = cA2w.q.getConjugate() * cB2w.q;

	// PT: twist part of "Ps::separateSwingTwist(cB2cAq,swing,twist)" (more or less)
	return PxQuat(cB2cAq.x, 0.0f, 0.0f, cB2cAq.w);
}

// PT: this version is similar to the "getAngle" function, but the twist is computed slightly differently.
static PX_FORCE_INLINE PxReal computePhi(const PxTransform& cA2w, const PxTransform& cB2w)
{
	PxQuat twist = computeTwist(cA2w, cB2w);
	twist.normalize();

	PxReal angle = twist.getAngle();
	if(twist.x<0.0f)
		angle = -angle;
	return angle;
}

static void RevoluteJointVisualize(PxConstraintVisualizer& viz, const void* constantBlock, const PxTransform& body0Transform, const PxTransform& body1Transform, PxU32 flags)
{
	const RevoluteJointData& data = *reinterpret_cast<const RevoluteJointData*>(constantBlock);

	PxTransform cA2w, cB2w;
	joint::computeJointFrames(cA2w, cB2w, data, body0Transform, body1Transform);
	if(flags & PxConstraintVisualizationFlag::eLOCAL_FRAMES)
		viz.visualizeJointFrames(cA2w, cB2w);

	if((data.jointFlags & PxRevoluteJointFlag::eLIMIT_ENABLED) && (flags & PxConstraintVisualizationFlag::eLIMITS))
	{
		const PxReal angle = computePhi(cA2w, cB2w);
		const PxReal pad = data.limit.contactDistance;
		const PxReal low = data.limit.lower;
		const PxReal high = data.limit.upper;

		const bool active = isLimitActive(data.limit, pad, angle, low, high);
		viz.visualizeAngularLimit(cA2w, data.limit.lower, data.limit.upper, active);
	}
}

static PxU32 RevoluteJointSolverPrep(Px1DConstraint* constraints,
	PxVec3& body0WorldOffset,
	PxU32 /*maxConstraints*/,
	PxConstraintInvMassScale& invMassScale,
	const void* constantBlock,
	const PxTransform& bA2w,
	const PxTransform& bB2w,
	bool useExtendedLimits,
	PxVec3& cA2wOut, PxVec3& cB2wOut)
{
	const RevoluteJointData& data = *reinterpret_cast<const RevoluteJointData*>(constantBlock);

	PxTransform cA2w, cB2w;
	joint::ConstraintHelper ch(constraints, invMassScale, cA2w, cB2w, body0WorldOffset, data, bA2w, bB2w);

	const PxJointAngularLimitPair& limit = data.limit;

	const bool limitEnabled = data.jointFlags & PxRevoluteJointFlag::eLIMIT_ENABLED;
	const bool limitIsLocked = limitEnabled && limit.lower >= limit.upper;

	// PT: it is a mistake to use the neighborhood operator since it
	// prevents us from using the quat's double-cover feature.
	if(!useExtendedLimits && cB2w.q.dot(cA2w.q)<0.0f)
		cB2w.q = -cB2w.q;

	PxVec3 ra, rb;
	ch.prepareLockedAxes(cA2w.q, cB2w.q, cA2w.transformInv(cB2w.p), 7, PxU32(limitIsLocked ? 7 : 6), ra, rb);
	cA2wOut = ra + bA2w.p;
	cB2wOut = rb + bB2w.p;

	if(limitIsLocked)
		return ch.getCount();

	const PxVec3 axis = cA2w.rotate(PxVec3(1.0f, 0.0f, 0.0f));

	if(data.jointFlags & PxRevoluteJointFlag::eDRIVE_ENABLED)
	{
		Px1DConstraint* c = ch.getConstraintRow();

		c->solveHint		= PxConstraintSolveHint::eNONE;
		c->linear0			= PxVec3(0.0f);
		c->angular0			= -axis;
		c->linear1			= PxVec3(0.0f);
		c->angular1			= -axis * data.driveGearRatio;
		c->velocityTarget	= data.driveVelocity;
		c->minImpulse		= -data.driveForceLimit;
		c->maxImpulse		= data.driveForceLimit;
		c->flags |= Px1DConstraintFlag::eANGULAR_CONSTRAINT;
		if(data.jointFlags & PxRevoluteJointFlag::eDRIVE_FREESPIN)
		{
			if(data.driveVelocity > 0.0f)
				c->minImpulse = 0.0f;
			if(data.driveVelocity < 0.0f)
				c->maxImpulse = 0.0f;
		}
		c->flags |= Px1DConstraintFlag::eHAS_DRIVE_LIMIT;
	}

	if(limitEnabled)
	{
		const PxReal phi = computePhi(cA2w, cB2w);
		ch.anglePair(phi, data.limit.lower, data.limit.upper, data.limit.contactDistance, axis, limit);
	}

	return ch.getCount();
}

PxConstraintShaderTable Ext::RevoluteJoint::sShaders = { RevoluteJointSolverPrep, RevoluteJointProject, RevoluteJointVisualize, PxConstraintFlag::Enum(0) };
