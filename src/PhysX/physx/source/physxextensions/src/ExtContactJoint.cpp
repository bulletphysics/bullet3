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

#include "ExtContactJoint.h"
#include "ExtConstraintHelper.h"
#include "common/PxSerialFramework.h"

using namespace physx;
using namespace Ext;

PxContactJoint* physx::PxContactJointCreate(PxPhysics& physics, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1)
{
	PX_CHECK_AND_RETURN_NULL(localFrame0.isSane(), "PxContactJointCreate: local frame 0 is not a valid transform");
	PX_CHECK_AND_RETURN_NULL(localFrame1.isSane(), "PxContactJointCreate: local frame 1 is not a valid transform");
	PX_CHECK_AND_RETURN_NULL(actor0 != actor1, "PxContactJointCreate: actors must be different");
	PX_CHECK_AND_RETURN_NULL((actor0 && actor0->is<PxRigidBody>()) || (actor1 && actor1->is<PxRigidBody>()), "PxContactJointCreate: at least one actor must be dynamic");

	ContactJoint* j;
	PX_NEW_SERIALIZED(j, ContactJoint)(physics.getTolerancesScale(), actor0, localFrame0, actor1, localFrame1);
	if (j->attach(physics, actor0, actor1))
		return j;

	PX_DELETE(j);
	return NULL;
}

PxVec3 ContactJoint::getContact() const
{
	return data().contact;
}

void ContactJoint::setContact(const PxVec3& contact)
{
	PX_CHECK_AND_RETURN(contact.isFinite(), "PxContactJoint::setContact: invalid parameter");
	data().contact = contact;
	markDirty();
}

PxVec3 ContactJoint::getContactNormal() const
{
	return data().normal;
}

void ContactJoint::setContactNormal(const PxVec3& normal)
{
	PX_CHECK_AND_RETURN(normal.isFinite(), "PxContactJoint::setContactNormal: invalid parameter");
	data().normal = normal;
	markDirty();
}

PxReal ContactJoint::getPenetration() const
{
	return data().penetration;
}

void ContactJoint::setPenetration(PxReal penetration)
{
	PX_CHECK_AND_RETURN(PxIsFinite(penetration), "ContactJoint::setPenetration: invalid parameter");
	data().penetration = penetration;
	markDirty();
}


PxReal ContactJoint::getResititution() const
{
	return data().restitution;
}

void ContactJoint::setResititution(const PxReal restitution)
{
	PX_CHECK_AND_RETURN(PxIsFinite(restitution) && restitution >= 0.f && restitution <= 1.f, "ContactJoint::setResititution: invalid parameter");
	data().restitution = restitution;
	markDirty();
}

PxReal ContactJoint::getBounceThreshold() const
{
	return data().bounceThreshold;
}

void ContactJoint::setBounceThreshold(const PxReal bounceThreshold)
{
	PX_CHECK_AND_RETURN(PxIsFinite(bounceThreshold) && bounceThreshold > 0.f, "ContactJoint::setBounceThreshold: invalid parameter");
	data().bounceThreshold = bounceThreshold;
	markDirty();
}

bool ContactJoint::attach(PxPhysics &physics, PxRigidActor* actor0, PxRigidActor* actor1)
{
	mPxConstraint = physics.createConstraint(actor0, actor1, *this, sShaders, sizeof(ContactJointData));
	return mPxConstraint != NULL;
}


void ContactJoint::exportExtraData(PxSerializationContext& stream)
{
	if (mData)
	{
		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(mData, sizeof(ContactJointData));
	}
	stream.writeName(mName);
}

void ContactJoint::importExtraData(PxDeserializationContext& context)
{
	if (mData)
		mData = context.readExtraData<ContactJointData, PX_SERIAL_ALIGN>();

	context.readName(mName);
}

void ContactJoint::resolveReferences(PxDeserializationContext& context)
{
	setPxConstraint(resolveConstraintPtr(context, getPxConstraint(), getConnector(), sShaders));
}

void ContactJoint::computeJacobians(PxJacobianRow* jacobian) const
{
	const PxVec3 cp = data().contact;
	const PxVec3 normal = data().normal;

	PxRigidActor* actor0, *actor1;
	this->getActors(actor0, actor1);

	PxVec3 raXn(0.f), rbXn(0.f);

	if (actor0 && actor0->is<PxRigidBody>())
	{
		PxRigidBody* dyn = actor0->is<PxRigidBody>();
		PxTransform cmassPose = dyn->getGlobalPose() * dyn->getCMassLocalPose();
		raXn = (cp - cmassPose.p).cross(normal);
	}

	if (actor1 && actor1->is<PxRigidBody>())
	{
		PxRigidBody* dyn = actor1->is<PxRigidBody>();
		PxTransform cmassPose = dyn->getGlobalPose() * dyn->getCMassLocalPose();
		rbXn = (cp - cmassPose.p).cross(normal);
	}

	jacobian->linear0  = normal;
	jacobian->angular0 = raXn;
	jacobian->linear1 = -normal;
	jacobian->angular1 = -rbXn;
	
}
PxU32 ContactJoint::getNbJacobianRows() const
{
	return 1;
}


ContactJoint* ContactJoint::createObject(PxU8*& address, PxDeserializationContext& context)
{
	ContactJoint* obj = new (address) ContactJoint(PxBaseFlag::eIS_RELEASABLE);
	address += sizeof(ContactJoint);
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}

// global function to share the joint shaders with API capture	
const PxConstraintShaderTable* Ext::GetContactJointShaderTable()
{
	return &ContactJoint::getConstraintShaderTable();
}

//~PX_SERIALIZATION

static void ContactJointProject(const void* /*constantBlock*/, PxTransform& /*bodyAToWorld*/, PxTransform& /*bodyBToWorld*/, bool /*projectToA*/)
{
	// Not required
}

static void ContactJointVisualize(PxConstraintVisualizer& /*viz*/, const void* /*constantBlock*/, const PxTransform& /*body0Transform*/, const PxTransform& /*body1Transform*/, PxU32 /*flags*/)
{
	//TODO
}

static PxU32 ContactJointSolverPrep(Px1DConstraint* constraints,
	PxVec3& body0WorldOffset,
	PxU32 /*maxConstraints*/,
	PxConstraintInvMassScale& /*invMassScale*/,
	const void* constantBlock,
	const PxTransform& bA2w,
	const PxTransform& bB2w,
	bool,
	PxVec3& cA2wOut, PxVec3& cB2wOut)
{
	const ContactJointData& data = *reinterpret_cast<const ContactJointData*>(constantBlock);

	const PxVec3& contact = data.contact;
	const PxVec3& normal = data.normal;

	cA2wOut = contact;
	cB2wOut = contact;

	const PxVec3 ra = contact - bA2w.p;
	const PxVec3 rb = contact - bB2w.p;

	body0WorldOffset = PxVec3(0.f);

	Px1DConstraint& con = constraints[0];
	con.linear0 = normal;
	con.linear1 = normal;
	con.angular0 = ra.cross(normal);
	con.angular1 = rb.cross(normal);

	con.geometricError = data.penetration;
	con.minImpulse = 0.f;
	con.maxImpulse = PX_MAX_F32;

	con.velocityTarget = 0.f;
	con.forInternalUse = 0.f;
	con.solveHint = 0;
	con.flags = Px1DConstraintFlag::eOUTPUT_FORCE;
	con.mods.bounce.restitution = data.restitution;
	con.mods.bounce.velocityThreshold = data.bounceThreshold;

	return 1;
}

PxConstraintShaderTable Ext::ContactJoint::sShaders = { ContactJointSolverPrep, ContactJointProject, ContactJointVisualize, PxConstraintFlag::Enum(0) };
