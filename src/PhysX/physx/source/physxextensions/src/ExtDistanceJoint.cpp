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

#include "ExtDistanceJoint.h"
#include "ExtConstraintHelper.h"
#include "common/PxSerialFramework.h"

using namespace physx;
using namespace Ext;

PxDistanceJoint* physx::PxDistanceJointCreate(PxPhysics& physics, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1)
{
	PX_CHECK_AND_RETURN_NULL(localFrame0.isSane(), "PxDistanceJointCreate: local frame 0 is not a valid transform"); 
	PX_CHECK_AND_RETURN_NULL(localFrame1.isSane(), "PxDistanceJointCreate: local frame 1 is not a valid transform"); 
	PX_CHECK_AND_RETURN_NULL(actor0 != actor1, "PxDistanceJointCreate: actors must be different");
	PX_CHECK_AND_RETURN_NULL((actor0 && actor0->is<PxRigidBody>()) || (actor1 && actor1->is<PxRigidBody>()), "PxD6JointCreate: at least one actor must be dynamic");

	DistanceJoint* j;
	PX_NEW_SERIALIZED(j, DistanceJoint)(physics.getTolerancesScale(), actor0, localFrame0, actor1, localFrame1);
	if(j->attach(physics, actor0, actor1))
		return j;

	PX_DELETE(j);
	return NULL;
}

PxReal DistanceJoint::getDistance() const
{
	return getRelativeTransform().p.magnitude();
}

void DistanceJoint::setMinDistance(PxReal distance)	
{ 
	PX_CHECK_AND_RETURN(PxIsFinite(distance), "PxDistanceJoint::setMinDistance: invalid parameter");
	data().minDistance = distance;
	markDirty();
}

PxReal DistanceJoint::getMinDistance() const
{ 
	return data().minDistance;		
}

void DistanceJoint::setMaxDistance(PxReal distance)	
{ 
	PX_CHECK_AND_RETURN(PxIsFinite(distance), "PxDistanceJoint::setMaxDistance: invalid parameter");
	data().maxDistance = distance;
	markDirty();
}

PxReal DistanceJoint::getMaxDistance() const	
{ 
	return data().maxDistance;			
}

void DistanceJoint::setTolerance(PxReal tolerance) 
{ 
	PX_CHECK_AND_RETURN(PxIsFinite(tolerance), "PxDistanceJoint::setTolerance: invalid parameter");
	data().tolerance = tolerance;
	markDirty();
}

PxReal DistanceJoint::getTolerance() const
{ 
	return data().tolerance;			
}

void DistanceJoint::setStiffness(PxReal stiffness)
{ 
	PX_CHECK_AND_RETURN(PxIsFinite(stiffness), "PxDistanceJoint::setStiffness: invalid parameter");
	data().stiffness = stiffness;
	markDirty();
}

PxReal DistanceJoint::getStiffness() const	
{ 
	return data().stiffness;
}

void DistanceJoint::setDamping(PxReal damping)
{ 
	PX_CHECK_AND_RETURN(PxIsFinite(damping), "PxDistanceJoint::setDamping: invalid parameter");
	data().damping = damping;
	markDirty();	
}

PxReal DistanceJoint::getDamping() const
{ 
	return data().damping;
}

PxDistanceJointFlags DistanceJoint::getDistanceJointFlags(void) const
{ 
	return data().jointFlags;		
}

void DistanceJoint::setDistanceJointFlags(PxDistanceJointFlags flags) 
{ 
	data().jointFlags = flags; 
	markDirty();	
}

void DistanceJoint::setDistanceJointFlag(PxDistanceJointFlag::Enum flag, bool value)
{
	if(value)
		data().jointFlags |= flag;
	else
		data().jointFlags &= ~flag;
	markDirty();
}

bool DistanceJoint::attach(PxPhysics &physics, PxRigidActor* actor0, PxRigidActor* actor1)
{
	mPxConstraint = physics.createConstraint(actor0, actor1, *this, sShaders, sizeof(DistanceJointData));
	return mPxConstraint!=NULL;
}

void DistanceJoint::exportExtraData(PxSerializationContext& stream)
{
	if(mData)
	{
		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(mData, sizeof(DistanceJointData));
	}
	stream.writeName(mName);
}

void DistanceJoint::importExtraData(PxDeserializationContext& context)
{
	if(mData)
		mData = context.readExtraData<DistanceJointData, PX_SERIAL_ALIGN>();

	context.readName(mName);
}

void DistanceJoint::resolveReferences(PxDeserializationContext& context)
{
	setPxConstraint(resolveConstraintPtr(context, getPxConstraint(), getConnector(), sShaders));	
}

DistanceJoint* DistanceJoint::createObject(PxU8*& address, PxDeserializationContext& context)
{
	DistanceJoint* obj = new (address) DistanceJoint(PxBaseFlag::eIS_RELEASABLE);
	address += sizeof(DistanceJoint);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}

// global function to share the joint shaders with API capture	
const PxConstraintShaderTable* Ext::GetDistanceJointShaderTable() 
{ 
	return &DistanceJoint::getConstraintShaderTable();
}

//~PX_SERIALIZATION

static void DistanceJointProject(const void* /*constantBlock*/, PxTransform& /*bodyAToWorld*/, PxTransform& /*bodyBToWorld*/, bool /*projectToA*/)
{
	// TODO
}

static void DistanceJointVisualize(PxConstraintVisualizer& viz, const void* constantBlock, const PxTransform& body0Transform, const PxTransform& body1Transform, PxU32 flags)
{
	const DistanceJointData& data = *reinterpret_cast<const DistanceJointData*>(constantBlock);

	PxTransform cA2w, cB2w;
	joint::computeJointFrames(cA2w, cB2w, data, body0Transform, body1Transform);
	if(flags & PxConstraintVisualizationFlag::eLOCAL_FRAMES)
		viz.visualizeJointFrames(cA2w, cB2w);

	// PT: we consider the following is part of the joint's "limits" since that's the only available flag we have
	if(flags & PxConstraintVisualizationFlag::eLIMITS)
	{
		const bool enforceMax = (data.jointFlags & PxDistanceJointFlag::eMAX_DISTANCE_ENABLED);
		const bool enforceMin = (data.jointFlags & PxDistanceJointFlag::eMIN_DISTANCE_ENABLED);
		if(!enforceMin && !enforceMax)
			return;

		PxVec3 dir = cB2w.p - cA2w.p;
		const float currentDist = dir.normalize();

		PxU32 color = 0x00ff00;
		if(enforceMax && currentDist>data.maxDistance)
			color = 0xff0000;
		if(enforceMin && currentDist<data.minDistance)
			color = 0x0000ff;

		viz.visualizeLine(cA2w.p, cB2w.p, color);
	}
}

PX_FORCE_INLINE void setupContraint(Px1DConstraint& c, const PxVec3& direction, const PxVec3& angular0, const PxVec3& angular1, const DistanceJointData& data)
{
	// constraint is breakable, so we need to output forces

	c.flags = Px1DConstraintFlag::eOUTPUT_FORCE;

	c.linear0 = direction;		c.angular0 = angular0;
	c.linear1 = direction;		c.angular1 = angular1;		

	if(data.jointFlags & PxDistanceJointFlag::eSPRING_ENABLED)
	{
		c.flags |= Px1DConstraintFlag::eSPRING;
		c.mods.spring.stiffness= data.stiffness;
		c.mods.spring.damping	= data.damping;
	}
}

static PxU32 DistanceJointSolverPrep(Px1DConstraint* constraints,
	PxVec3& body0WorldOffset,
	PxU32 /*maxConstraints*/,
	PxConstraintInvMassScale& invMassScale,
	const void* constantBlock,
	const PxTransform& bA2w,
	const PxTransform& bB2w,
	bool /*useExtendedLimits*/,
	PxVec3& cA2wOut, PxVec3& cB2wOut)
{
	const DistanceJointData& data = *reinterpret_cast<const DistanceJointData*>(constantBlock);

	PxTransform cA2w, cB2w;
	joint::ConstraintHelper ch(constraints, invMassScale, cA2w, cB2w, body0WorldOffset, data, bA2w, bB2w);

	cA2wOut = cB2w.p;
	cB2wOut = cB2w.p;

	PxVec3 direction = cA2w.p - cB2w.p;
	const PxReal distance = direction.normalize();

	const bool enforceMax = (data.jointFlags & PxDistanceJointFlag::eMAX_DISTANCE_ENABLED);
	const bool enforceMin = (data.jointFlags & PxDistanceJointFlag::eMIN_DISTANCE_ENABLED);

#define EPS_REAL 1.192092896e-07F

	if(distance < EPS_REAL)
		direction = PxVec3(1.0f, 0.0f, 0.0f);

	Px1DConstraint* c = constraints;

	const PxVec3 angular0 = ch.getRa().cross(direction);
	const PxVec3 angular1 = ch.getRb().cross(direction);

	setupContraint(*c, direction, angular0, angular1, data); 

	//add tolerance so we don't have contact-style jitter problem.

	if(data.minDistance == data.maxDistance && enforceMin && enforceMax)
	{
		const PxReal error = distance - data.maxDistance;
		c->geometricError = error >  data.tolerance ? error - data.tolerance :
			error < -data.tolerance ? error + data.tolerance : 0.0f;
	}
	else if(enforceMax && distance > data.maxDistance)
	{
		c->geometricError = distance - data.maxDistance - data.tolerance;
		c->maxImpulse = 0.0f;
	}
	else if(enforceMin && distance < data.minDistance)
	{
		c->geometricError = distance - data.minDistance + data.tolerance;	
		c->minImpulse = 0.0f;
	}
	else
	{
		if(enforceMin && enforceMax)
		{
			// since we dont know the current rigid velocity, we need to insert row for both limits
			Px1DConstraint* minConstraint = constraints;
			minConstraint->geometricError = distance - data.minDistance;
			minConstraint->minImpulse = 0.0f;
			minConstraint->maxImpulse = FLT_MAX;
			minConstraint->flags |= Px1DConstraintFlag::eKEEPBIAS;

			Px1DConstraint* maxConstraint = constraints;
			maxConstraint++;

			setupContraint(*maxConstraint, direction, angular0, angular1, data);

			maxConstraint->geometricError = distance - data.maxDistance;
			maxConstraint->minImpulse = -FLT_MAX;
			maxConstraint->maxImpulse = 0.0f;
			maxConstraint->flags |= Px1DConstraintFlag::eKEEPBIAS;

			return 2;
		}
		else if(enforceMax)
		{			
			c->geometricError = distance - data.maxDistance;
			c->minImpulse = -FLT_MAX;
			c->maxImpulse = 0.0f;
			c->flags |= Px1DConstraintFlag::eKEEPBIAS;
			return 0;
		}
		else if(enforceMin)
		{			
			c->geometricError = distance - data.minDistance;
			c->minImpulse = 0.0f;
			c->maxImpulse = FLT_MAX;
			c->flags |= Px1DConstraintFlag::eKEEPBIAS;
			return 0;
		}		
	}

	return 1;
}

PxConstraintShaderTable Ext::DistanceJoint::sShaders = { DistanceJointSolverPrep, DistanceJointProject, DistanceJointVisualize, PxConstraintFlag::Enum(0) };
