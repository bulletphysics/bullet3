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

#include "ScScene.h"
#include "ScConstraintProjectionManager.h"
#include "ScBodySim.h"
#include "ScStaticSim.h"
#include "PxsContext.h"
#include "ScConstraintCore.h"
#include "ScConstraintSim.h"
#include "ScConstraintInteraction.h"
#include "ScElementSimInteraction.h"
#include "CmVisualization.h"
#include "ScObjectIDTracker.h"
#include "DyContext.h"

using namespace physx;

PX_FORCE_INLINE void invalidateConstraintGroupsOnAdd(Sc::ConstraintProjectionManager& cpm, Sc::BodySim* b0, Sc::BodySim* b1, Sc::ConstraintSim& constraint)
{
	// constraint groups get built by starting from dirty constraints that need projection. If a non-projecting constraint gets added
	// we need to restart the whole process (we do not want to track dirty non-projecting constraints because of a scenario where
	// all constraints of a group get switched to non-projecting which should kill the group and not rebuild a new one).
	if (b0 && b0->getConstraintGroup())
		cpm.invalidateGroup(*b0->getConstraintGroup(), &constraint);
	if (b1 && b1->getConstraintGroup())
		cpm.invalidateGroup(*b1->getConstraintGroup(), &constraint);
}

Sc::ConstraintSim::ConstraintSim(ConstraintCore& core, RigidCore* r0, RigidCore* r1, Scene& scene) :
	mScene		(scene),
	mCore		(core),
	mInteraction(NULL),
	mFlags		(0)
{
	mBodies[0] = (r0 && (r0->getActorCoreType() != PxActorType::eRIGID_STATIC)) ? static_cast<BodySim*>(r0->getSim()) : 0;
	mBodies[1] = (r1 && (r1->getActorCoreType() != PxActorType::eRIGID_STATIC)) ? static_cast<BodySim*>(r1->getSim()) : 0;
	
	mLowLevelConstraint.index = scene.getConstraintIDTracker().createID();
	Ps::Array<Dy::ConstraintWriteback, Ps::VirtualAllocator>& writeBackPool = scene.getDynamicsContext()->getConstraintWriteBackPool();
	if (mLowLevelConstraint.index >= writeBackPool.capacity())
	{
		writeBackPool.reserve(writeBackPool.capacity() * 2);
	}

	writeBackPool.resize(PxMax(writeBackPool.size(), mLowLevelConstraint.index + 1));
	writeBackPool[mLowLevelConstraint.index].initialize();

	if (!createLLConstraint())
		return;

	PxReal linBreakForce, angBreakForce;
	core.getBreakForce(linBreakForce, angBreakForce);
	if ((linBreakForce < PX_MAX_F32) || (angBreakForce < PX_MAX_F32))
		setFlag(eBREAKABLE);

	core.setSim(this);

	ConstraintProjectionManager& cpm = scene.getProjectionManager();
	if (!needsProjection())
		invalidateConstraintGroupsOnAdd(cpm, mBodies[0], mBodies[1], *this);
	else
		cpm.addToPendingGroupUpdates(*this);

	ConstraintSim* cs = this;  // to make the Wii U compiler happy
	mInteraction = mScene.getConstraintInteractionPool()->construct(cs, 
																	r0 ? *r0->getSim() : scene.getStaticAnchor(), 
																	r1 ? *r1->getSim() : scene.getStaticAnchor());

	PX_ASSERT(!mInteraction->isRegistered());  // constraint interactions must not register in the scene, there is a list of Sc::ConstraintSim instead
}

Sc::ConstraintSim::~ConstraintSim()
{
	PX_ASSERT(mInteraction);  // This is fine now, a body which gets removed from the scene removes all constraints automatically
	PX_ASSERT(!mInteraction->isRegistered());  // constraint interactions must not register in the scene, there is a list of Sc::ConstraintSim instead

	if (readFlag(ConstraintSim::ePENDING_GROUP_UPDATE))
		mScene.getProjectionManager().removeFromPendingGroupUpdates(*this);

	if (!isBroken())
		mInteraction->destroy();

	mScene.getConstraintIDTracker().releaseID(mLowLevelConstraint.index);
	mScene.getConstraintInteractionPool()->destroy(mInteraction);

	destroyLLConstraint();

	mCore.setSim(NULL);
}

bool Sc::ConstraintSim::createLLConstraint()
{
	Dy::Constraint& llc = mLowLevelConstraint;
	ConstraintCore& core = getCore();
	PxU32 constantBlockSize = core.getConstantBlockSize();

	void* constantBlock = mScene.allocateConstraintBlock(constantBlockSize);
	if(!constantBlock)
	{
		Ps::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "Constraint: could not allocate low-level resources.");
		return false;
	}

	//Ensure the constant block isn't just random data because some functions may attempt to use it before it is
	//setup.  Specifically pvd visualization of joints
	//-CN

	PxMemZero( constantBlock, constantBlockSize);

	core.getBreakForce(llc.linBreakForce, llc.angBreakForce);
	llc.flags					= core.getFlags();
	llc.constantBlockSize		= PxU16(constantBlockSize);

	llc.solverPrep				= core.getSolverPrep();
	llc.project					= core.getProject();
	llc.constantBlock			= constantBlock;

	//llc.index = mLowLevelConstraint.index;
	llc.body0					= mBodies[0] ? &mBodies[0]->getLowLevelBody() : 0;
	llc.body1					= mBodies[1] ? &mBodies[1]->getLowLevelBody() : 0;
	llc.bodyCore0				= mBodies[0] ? &llc.body0->getCore() : NULL;
	llc.bodyCore1				= mBodies[1] ? &llc.body1->getCore() : NULL;

	llc.minResponseThreshold	= core.getMinResponseThreshold();

	return true;
}

void Sc::ConstraintSim::destroyLLConstraint()
{
	if(mLowLevelConstraint.constantBlock)
	{
		mScene.deallocateConstraintBlock(mLowLevelConstraint.constantBlock,
										 mLowLevelConstraint.constantBlockSize);
	}
}

void Sc::ConstraintSim::preBodiesChange()
{
	PX_ASSERT(mInteraction);

	BodySim* b = getConstraintGroupBody();
	if (b)
		mScene.getProjectionManager().invalidateGroup(*b->getConstraintGroup(), this);

	if (!isBroken())
		mInteraction->destroy();

	mScene.getConstraintInteractionPool()->destroy(mInteraction);
	mInteraction = NULL;
}

void Sc::ConstraintSim::postBodiesChange(RigidCore* r0, RigidCore* r1)
{
	PX_ASSERT(mInteraction == NULL);

	BodySim* b0 = (r0 && (r0->getActorCoreType() != PxActorType::eRIGID_STATIC)) ? static_cast<BodySim*>(r0->getSim()) : 0;
	BodySim* b1 = (r1 && (r1->getActorCoreType() != PxActorType::eRIGID_STATIC)) ? static_cast<BodySim*>(r1->getSim()) : 0;

	ConstraintProjectionManager& cpm = mScene.getProjectionManager();
	PxConstraintFlags::InternalType projectionNeeded = getCore().getFlags() & PxConstraintFlag::ePROJECTION;  // can not use "needsProjection()" because that takes into account whether the constraint is broken
	if (!projectionNeeded)
		invalidateConstraintGroupsOnAdd(cpm, b0, b1, *this);
	else if (!readFlag(ConstraintSim::ePENDING_GROUP_UPDATE))
		cpm.addToPendingGroupUpdates(*this);

	Dy::Constraint& c = mLowLevelConstraint;

	c.body0 = b0 ? &b0->getLowLevelBody() : NULL;
	c.body1 = b1 ? &b1->getLowLevelBody() : NULL;

	c.bodyCore0 = c.body0 ? &c.body0->getCore() : NULL;
	c.bodyCore1 = c.body1 ? &c.body1->getCore() : NULL;

	mBodies[0] = b0;
	mBodies[1] = b1;

	ConstraintSim* cs = this;  // to make the Wii U compiler happy
	mInteraction = mScene.getConstraintInteractionPool()->construct(cs, 
																	r0 ? *r0->getSim() : mScene.getStaticAnchor(), 
																	r1 ? *r1->getSim() : mScene.getStaticAnchor());
}

void Sc::ConstraintSim::checkMaxForceExceeded()
{
	PX_ASSERT(readFlag(eCHECK_MAX_FORCE_EXCEEDED));

	Dy::ConstraintWriteback& solverOutput = mScene.getDynamicsContext()->getConstraintWriteBackPool()[mLowLevelConstraint.index];
	if(solverOutput.broken)
	{
		setFlag(ConstraintSim::eBROKEN);
		mScene.addBrokenConstraint(&mCore);
		mCore.breakApart();
		mInteraction->destroy();

		// update related SIPs
		{
			ActorSim& a0 = mInteraction->getActorSim0();
			ActorSim& a1 = mInteraction->getActorSim1();
			ActorSim& actor = (a0.getActorInteractionCount()<  a1.getActorInteractionCount()) ? a0 : a1;

			actor.setActorsInteractionsDirty(InteractionDirtyFlag::eFILTER_STATE, NULL, InteractionFlag::eRB_ELEMENT);
			// because broken constraints can re-enable contact response between the two bodies
		}

		PX_ASSERT(!readFlag(eCHECK_MAX_FORCE_EXCEEDED));
	}
}

void Sc::ConstraintSim::getForce(PxVec3& lin, PxVec3& ang)
{
	const PxReal recipDt = mScene.getOneOverDt();
	Dy::ConstraintWriteback& solverOutput= mScene.getDynamicsContext()->getConstraintWriteBackPool()[mLowLevelConstraint.index];
	lin = solverOutput.linearImpulse * recipDt;
	ang = solverOutput.angularImpulse * recipDt;
}

void Sc::ConstraintSim::setBreakForceLL(PxReal linear, PxReal angular)
{
	PxU8 wasBreakable = readFlag(eBREAKABLE);
	PxU8 isBreakable;
	if ((linear < PX_MAX_F32) || (angular < PX_MAX_F32))
		isBreakable = eBREAKABLE;
	else
		isBreakable = 0;

	if (isBreakable != wasBreakable)
	{
		if (isBreakable)
		{
			PX_ASSERT(!readFlag(eCHECK_MAX_FORCE_EXCEEDED));
			setFlag(eBREAKABLE);
			if (mInteraction->readInteractionFlag(InteractionFlag::eIS_ACTIVE))
				mScene.addActiveBreakableConstraint(this, mInteraction);
		}
		else
		{
			if (readFlag(eCHECK_MAX_FORCE_EXCEEDED))
				mScene.removeActiveBreakableConstraint(this);
			clearFlag(eBREAKABLE);
		}
	}

	mLowLevelConstraint.linBreakForce = linear;
	mLowLevelConstraint.angBreakForce = angular;
}

void Sc::ConstraintSim::postFlagChange(PxConstraintFlags oldFlags, PxConstraintFlags newFlags)
{
	mLowLevelConstraint.flags = newFlags;

	// PT: don't convert to bool if not needed
	const PxU32 hadProjection   = (oldFlags & PxConstraintFlag::ePROJECTION);
	const PxU32 needsProjection = (newFlags & PxConstraintFlag::ePROJECTION);

	if(needsProjection && !hadProjection)
	{
		PX_ASSERT(!readFlag(ConstraintSim::ePENDING_GROUP_UPDATE)); // Non-projecting constrainst should not be part of the update list

		Sc::BodySim* b0 = getBody(0);
		Sc::BodySim* b1 = getBody(1);
		if ((!b0 || b0->getConstraintGroup()) && (!b1 || b1->getConstraintGroup()))
		{
			// Already part of a constraint group but not as a projection constraint -> re-generate projection tree
			PX_ASSERT(b0 != NULL || b1 != NULL);
			if (b0)
				b0->getConstraintGroup()->markForProjectionTreeRebuild(mScene.getProjectionManager());
			else
				b1->getConstraintGroup()->markForProjectionTreeRebuild(mScene.getProjectionManager());
		}
		else
		{
			// Not part of a constraint group yet
			mScene.getProjectionManager().addToPendingGroupUpdates(*this);
		}
	}
	else if(!needsProjection && hadProjection)
	{
		if (!readFlag(ConstraintSim::ePENDING_GROUP_UPDATE))
		{
			Sc::BodySim* b = getConstraintGroupBody();
			if (b)
			{
				PX_ASSERT(b->getConstraintGroup());
				mScene.getProjectionManager().invalidateGroup(*b->getConstraintGroup(), NULL);
			}
			// This is conservative but it could be the case that this constraint with projection was the only
			// one in the group and thus the whole group must be killed. If we had a counter for the number of
			// projecting constraints per group, we could just update the projection tree if the counter was
			// larger than 1. But switching the projection flag does not seem likely anyway.
		}
		else
			mScene.getProjectionManager().removeFromPendingGroupUpdates(*this);  // Was part of a group which got invalidated

		PX_ASSERT(!readFlag(ConstraintSim::ePENDING_GROUP_UPDATE));  // make sure the expected post-condition is met for all paths
	}
}

Sc::RigidSim& Sc::ConstraintSim::getRigid(PxU32 i)
{
	PX_ASSERT(mInteraction);

	if (i == 0)
		return static_cast<RigidSim&>(mInteraction->getActorSim0());
	else
		return static_cast<RigidSim&>(mInteraction->getActorSim1());
}

bool Sc::ConstraintSim::hasDynamicBody()
{
	return (mBodies[0] && (!mBodies[0]->isKinematic())) || (mBodies[1] && (!mBodies[1]->isKinematic()));
}

static void constrainMotion(PxsRigidBody* body, PxTransform& targetPose)
{
	//Now constraint deltaPos and deltaRot
	const PxU32 lockFlags = body->mCore->lockFlags;

	if (lockFlags)
	{
		const PxTransform& currBody2World = body->mCore->body2World;

		PxVec3 deltaPos = targetPose.p - currBody2World.p;

		PxQuat deltaQ = targetPose.q * currBody2World.q.getConjugate();

		if (deltaQ.w < 0)	//shortest angle.
			deltaQ = -deltaQ;

		PxReal angle;
		PxVec3 axis;
		deltaQ.toRadiansAndUnitAxis(angle, axis);
		PxVec3 deltaRot = axis * angle;

		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_X)
			deltaPos.x = 0.0f;
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_Y)
			deltaPos.y = 0.0f;
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_Z)
			deltaPos.z = 0.0f;
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_X)
			deltaRot.x = 0.0f;
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y)
			deltaRot.y = 0.0f;
		if (lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z)
			deltaRot.z = 0.0f;

		targetPose.p = currBody2World.p + deltaPos;

		PxReal w2 = deltaRot.magnitudeSquared();
		if (w2 != 0.0f)
		{
			PxReal w = PxSqrt(w2);

			const PxReal v = w * 0.5f;
			PxReal s, q;
			Ps::sincos(v, s, q);
			s /= w;

			const PxVec3 pqr = deltaRot * s;
			const PxQuat quatVel(pqr.x, pqr.y, pqr.z, 0);
			PxQuat result = quatVel * currBody2World.q;

			result += currBody2World.q * q;

			targetPose.q = result.getNormalized();
		}
		else
		{
			targetPose.q = currBody2World.q;
		}
	}
}

void Sc::ConstraintSim::projectPose(BodySim* childBody, Ps::Array<BodySim*>& projectedBodies)
{
#if PX_DEBUG
	// We expect bodies in low level constraints to have same order as high level counterpart
	PxsRigidBody* b0 = mLowLevelConstraint.body0;
	PxsRigidBody* b1 = mLowLevelConstraint.body1;
	PX_ASSERT(	(childBody == getBody(0) && &childBody->getLowLevelBody() == b0) ||
				(childBody == getBody(1) && &childBody->getLowLevelBody() == b1) );
#endif

	Dy::Constraint& constraint = getLowLevelConstraint();
	bool projectToBody0 = childBody == getBody(1);

	PxsRigidBody* body0 = constraint.body0, 
				* body1 = constraint.body1;

	PxTransform body0ToWorld = body0 ? body0->getPose() : PxTransform(PxIdentity);
	PxTransform body1ToWorld = body1 ? body1->getPose() : PxTransform(PxIdentity);

	(*constraint.project)(constraint.constantBlock, body0ToWorld, body1ToWorld, projectToBody0);

	if(projectToBody0)
	{
		PX_ASSERT(body1);
		//Constrain new pose to valid world motion
		constrainMotion(body1, body1ToWorld);
		body1->setPose(body1ToWorld);
		projectedBodies.pushBack(getBody(1));
	}
	else
	{
		PX_ASSERT(body0);
		//Constrain new pose to valid world motion
		constrainMotion(body0, body0ToWorld);
		body0->setPose(body0ToWorld);
		projectedBodies.pushBack(getBody(0));
	}
}

bool Sc::ConstraintSim::needsProjection()
{
	const Dy::ConstraintWriteback& solverOutput = mScene.getDynamicsContext()->getConstraintWriteBackPool()[mLowLevelConstraint.index];
	return (getCore().getFlags() & PxConstraintFlag::ePROJECTION ) && !solverOutput.broken;
}

PX_INLINE Sc::BodySim* Sc::ConstraintSim::getConstraintGroupBody()
{
	BodySim* b = NULL;
	if (mBodies[0] && mBodies[0]->getConstraintGroup())
		b = mBodies[0];
	else if (mBodies[1] && mBodies[1]->getConstraintGroup())
		b = mBodies[1];

	return b;
}

void Sc::ConstraintSim::visualize(PxRenderBuffer& output)
{
	if(!(getCore().getFlags() & PxConstraintFlag::eVISUALIZATION))
		return;

	PxsRigidBody* b0 = mLowLevelConstraint.body0;
	PxsRigidBody* b1 = mLowLevelConstraint.body1;
	
	const PxTransform idt(PxIdentity);
	const PxTransform& t0 = b0 ? b0->getPose() : idt;
	const PxTransform& t1 = b1 ? b1->getPose() : idt;

	const PxReal frameScale = mScene.getVisualizationScale() * mScene.getVisualizationParameter(PxVisualizationParameter::eJOINT_LOCAL_FRAMES);
	const PxReal limitScale = mScene.getVisualizationScale() * mScene.getVisualizationParameter(PxVisualizationParameter::eJOINT_LIMITS);

	Cm::RenderOutput renderOut(static_cast<Cm::RenderBuffer &>(output));
	Cm::ConstraintImmediateVisualizer viz(frameScale, limitScale, renderOut);

	PxU32 flags = 0;
	if(frameScale!=0.0f)
		flags |= PxConstraintVisualizationFlag::eLOCAL_FRAMES;
	if(limitScale!=0.0f)
		flags |= PxConstraintVisualizationFlag::eLIMITS;

	mCore.getVisualize()(viz, mLowLevelConstraint.constantBlock, t0, t1, flags);
}

void Sc::ConstraintSim::setConstantsLL(void* addr)
{
	PxMemCopy(mLowLevelConstraint.constantBlock, addr, mLowLevelConstraint.constantBlockSize);

	getAnyBody()->getScene().getSimulationController()->updateJoint(mInteraction->getEdgeIndex(), &mLowLevelConstraint);
}
