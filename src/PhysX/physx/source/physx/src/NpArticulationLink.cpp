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


#include "NpArticulationLink.h"
#include "NpArticulationJoint.h"
#include "NpWriteCheck.h"
#include "NpReadCheck.h"
#include "ScbArticulation.h"
#include "CmVisualization.h"
#include "CmConeLimitHelper.h"
#include "CmUtils.h"
#include "NpArticulation.h"

using namespace physx;

// PX_SERIALIZATION
void NpArticulationLink::requiresObjects(PxProcessPxBaseCallback& c)
{
	NpArticulationLinkT::requiresObjects(c);
	
	if(mInboundJoint)
		c.process(*mInboundJoint);
}

void NpArticulationLink::exportExtraData(PxSerializationContext& stream)
{
	NpArticulationLinkT::exportExtraData(stream);
	Cm::exportInlineArray(mChildLinks, stream);
}

void NpArticulationLink::importExtraData(PxDeserializationContext& context)
{
	NpArticulationLinkT::importExtraData(context);
	Cm::importInlineArray(mChildLinks, context);
}

void NpArticulationLink::resolveReferences(PxDeserializationContext& context)
{	
    context.translatePxBase(mRoot);
    context.translatePxBase(mInboundJoint);
    context.translatePxBase(mParent);
       
    NpArticulationLinkT::resolveReferences(context);

    const PxU32 nbLinks = mChildLinks.size();
    for(PxU32 i=0;i<nbLinks;i++)
        context.translatePxBase(mChildLinks[i]);
}

NpArticulationLink* NpArticulationLink::createObject(PxU8*& address, PxDeserializationContext& context)
{
	NpArticulationLink* obj = new (address) NpArticulationLink(PxBaseFlags(0));
	address += sizeof(NpArticulationLink);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}
//~PX_SERIALIZATION

NpArticulationLink::NpArticulationLink(const PxTransform& bodyPose, PxArticulationBase& root, NpArticulationLink* parent)
: NpArticulationLinkT(PxConcreteType::eARTICULATION_LINK, PxBaseFlag::eOWNS_MEMORY, PxActorType::eARTICULATION_LINK, bodyPose)
, mRoot(&root)
, mInboundJoint(NULL)
, mParent(parent)
, mLLIndex(0xffffffff)
, mInboundJointDof(0xffffffff)
{
	PX_ASSERT(mBody.getScbType() == ScbType::eBODY);
	mBody.setScbType(ScbType::eBODY_FROM_ARTICULATION_LINK);

	if (parent)
		parent->addToChildList(*this);
}


NpArticulationLink::~NpArticulationLink()
{
}

void NpArticulationLink::releaseInternal()
{
	NpPhysics::getInstance().notifyDeletionListenersUserRelease(this, userData);

	NpArticulationLinkT::release();

	PxArticulationImpl* impl = reinterpret_cast<PxArticulationImpl*>(mRoot->getImpl());
	impl->removeLinkFromList(*this);

	if (mParent)
		mParent->removeFromChildList(*this);

	if (mInboundJoint)
		mInboundJoint->release();

	NpScene* npScene = NpActor::getAPIScene(*this);
	if (npScene)
		npScene->getScene().removeActor(mBody, true, false);

	mBody.destroy();
}


void NpArticulationLink::release()
{
	NP_WRITE_CHECK(NpActor::getOwnerScene(*this));

	PxArticulationImpl* impl = reinterpret_cast<PxArticulationImpl*>(mRoot->getImpl());

	PX_UNUSED(impl);

	if (impl->getRoot() == this && NpActor::getOwnerScene(*this) != NULL)
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxArticulationLink::release(): root link may not be released while articulation is in a scene");
		return;
	}

	//! this function doesn't get called when the articulation root is released
	// therefore, put deregistration code etc. into dtor, not here

	if (mChildLinks.empty())
	{
		releaseInternal();
	}
	else
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxArticulationLink::release(): Only leaf articulation links can be released. Release call failed");
	}
}



PxTransform NpArticulationLink::getGlobalPose() const
{
	NP_READ_CHECK(NpActor::getOwnerScene(*this));

	//!!!AL TODO: Need to start from root and compute along the branch to reflect double buffered state of root link
	return getScbBodyFast().getBody2World() * getScbBodyFast().getBody2Actor().getInverse();
}

void NpArticulationLink::setLinearDamping(PxReal linearDamping)
{
	NP_WRITE_CHECK(NpActor::getOwnerScene(*this));
	PX_CHECK_AND_RETURN(PxIsFinite(linearDamping), "NpArticulationLink::setLinearDamping: invalid float");
	PX_CHECK_AND_RETURN(linearDamping >= 0, "NpArticulationLink::setLinearDamping: The linear damping must be nonnegative!");

	getScbBodyFast().setLinearDamping(linearDamping);
}

PxReal NpArticulationLink::getLinearDamping() const
{
	NP_READ_CHECK(NpActor::getOwnerScene(*this));

	return getScbBodyFast().getLinearDamping();
}

void NpArticulationLink::setAngularDamping(PxReal angularDamping)
{
	NP_WRITE_CHECK(NpActor::getOwnerScene(*this));
	PX_CHECK_AND_RETURN(PxIsFinite(angularDamping), "NpArticulationLink::setAngularDamping: invalid float");
	PX_CHECK_AND_RETURN(angularDamping >= 0, "NpArticulationLink::setAngularDamping: The angular damping must be nonnegative!")

	getScbBodyFast().setAngularDamping(angularDamping);
}

PxReal NpArticulationLink::getAngularDamping() const
{
	NP_READ_CHECK(NpActor::getOwnerScene(*this));

	return getScbBodyFast().getAngularDamping();
}

PxArticulationBase& NpArticulationLink::getArticulation() const
{
	NP_READ_CHECK(NpActor::getOwnerScene(*this));
	return *mRoot;
}

PxArticulationReducedCoordinate& NpArticulationLink::getArticulationReducedCoordinate() const
{
	NP_READ_CHECK(NpActor::getOwnerScene(*this));
	//return *mRoot;
	PxArticulationReducedCoordinate* ret = NULL;
	return *ret;
}


PxArticulationJointBase* NpArticulationLink::getInboundJoint() const
{
	NP_READ_CHECK(NpActor::getOwnerScene(*this));
	return mInboundJoint;
}

PxU32 NpArticulationLink::getInboundJointDof() const
{
	NP_READ_CHECK(NpActor::getOwnerScene(*this));

	return mInboundJointDof;
}

PxU32 NpArticulationLink::getNbChildren() const
{
	NP_READ_CHECK(NpActor::getOwnerScene(*this));
	return mChildLinks.size();
}


PxU32 NpArticulationLink::getChildren(PxArticulationLink** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	NP_READ_CHECK(NpActor::getOwnerScene(*this));
	return Cm::getArrayOfPointers(userBuffer, bufferSize, startIndex, mChildLinks.begin(), mChildLinks.size());
}

PxU32 NpArticulationLink::getLinkIndex() const
{
	NP_READ_CHECK(NpActor::getOwnerScene(*this));
	return mLLIndex;
}


void NpArticulationLink::setCMassLocalPose(const PxTransform& pose)
{
	NP_WRITE_CHECK(NpActor::getOwnerScene(*this));
	PX_CHECK_AND_RETURN(pose.isSane(), "PxArticulationLink::setCMassLocalPose: invalid parameter");

	PxTransform p = pose.getNormalized();
	PxTransform oldpose = getScbBodyFast().getBody2Actor();
	PxTransform comShift = p.transformInv(oldpose);

	NpArticulationLinkT::setCMassLocalPoseInternal(p);

	if(mInboundJoint)
	{
		Scb::ArticulationJoint &j = mInboundJoint->getImpl()->getScbArticulationJoint();
		j.setChildPose(comShift.transform(j.getChildPose()));
	}

	for(PxU32 i=0; i<mChildLinks.size(); i++)
	{
		Scb::ArticulationJoint &j = static_cast<NpArticulationJoint*>(mChildLinks[i]->getInboundJoint())->getScbArticulationJoint();
		j.setParentPose(comShift.transform(j.getParentPose()));
	}
}


void NpArticulationLink::addForce(const PxVec3& force, PxForceMode::Enum mode, bool autowake)
{
	NpScene* scene = NpActor::getOwnerScene(*this);
	PX_UNUSED(scene);

	PX_CHECK_AND_RETURN(force.isFinite(), "NpArticulationLink::addForce: force is not valid.");
	NP_WRITE_CHECK(scene);
	PX_CHECK_AND_RETURN(scene, "NpArticulationLink::addForce: articulation link must be in a scene!");

	addSpatialForce(&force, 0, mode);

	reinterpret_cast<PxArticulationImpl*>(mRoot->getImpl())->wakeUpInternal((!force.isZero()), autowake);
}


void NpArticulationLink::addTorque(const PxVec3& torque, PxForceMode::Enum mode, bool autowake)
{
	NpScene* scene = NpActor::getOwnerScene(*this);
	PX_UNUSED(scene);

	PX_CHECK_AND_RETURN(torque.isFinite(), "NpArticulationLink::addTorque: force is not valid.");
	NP_WRITE_CHECK(scene);
	PX_CHECK_AND_RETURN(scene, "NpArticulationLink::addTorque: articulation link must be in a scene!");

	addSpatialForce(0, &torque, mode);

	reinterpret_cast<PxArticulationImpl*>(mRoot->getImpl())->wakeUpInternal((!torque.isZero()), autowake);
}

void NpArticulationLink::setForceAndTorque(const PxVec3& force, const PxVec3& torque, PxForceMode::Enum mode)
{
	NpScene* scene = NpActor::getOwnerScene(*this);
	PX_UNUSED(scene);

	PX_CHECK_AND_RETURN(torque.isFinite(), "NpArticulationLink::setForceAndTorque: torque is not valid.");
	PX_CHECK_AND_RETURN(force.isFinite(), "NpArticulationLink::setForceAndTorque: force is not valid.");
	NP_WRITE_CHECK(scene);
	PX_CHECK_AND_RETURN(scene, "NpArticulationLink::addTorque: articulation link must be in a scene!");

	setSpatialForce(&force, &torque, mode);

	reinterpret_cast<PxArticulationImpl*>(mRoot->getImpl())->wakeUpInternal((!torque.isZero()), true);

}

void NpArticulationLink::clearForce(PxForceMode::Enum mode)
{
	NpScene* scene = NpActor::getOwnerScene(*this);
	PX_UNUSED(scene);
	NP_WRITE_CHECK(scene);
	PX_CHECK_AND_RETURN(scene, "NpArticulationLink::clearForce: articulation link must be in a scene!");

	clearSpatialForce(mode, true, false);
}

void NpArticulationLink::clearTorque(PxForceMode::Enum mode)
{
	NpScene* scene = NpActor::getOwnerScene(*this);
	PX_UNUSED(scene);
	NP_WRITE_CHECK(scene);
	PX_CHECK_AND_RETURN(scene, "NpArticulationLink::clearTorque: articulation link must be in a scene!");

	clearSpatialForce(mode, false, true);
}

void NpArticulationLink::setGlobalPoseInternal(const PxTransform& pose, bool autowake)
{
	NpScene* scene = NpActor::getOwnerScene(*this);

	PX_CHECK_AND_RETURN(pose.isValid(), "NpArticulationLink::setGlobalPose pose is not valid.");

	NP_WRITE_CHECK(scene);

#if PX_CHECKED
	if (scene)
		scene->checkPositionSanity(*this, pose, "PxArticulationLink::setGlobalPose");
#endif

	PxTransform body2World = pose * getScbBodyFast().getBody2Actor();
	getScbBodyFast().setBody2World(body2World, false);

	if (scene && autowake)
		reinterpret_cast<PxArticulationImpl*>(mRoot->getImpl())->wakeUpInternal(false, true);

	if (scene)
		reinterpret_cast<PxArticulationImpl*>(mRoot->getImpl())->setGlobalPose();
}

void NpArticulationLink::setGlobalPose(const PxTransform& pose)
{
	// clow: no need to test inputs here, it's done in the setGlobalPose function already
	setGlobalPose(pose, true);
}

void NpArticulationLink::setGlobalPose(const PxTransform& pose, bool autowake)
{
	PX_CHECK_AND_RETURN(mRoot->getType() == PxArticulationBase::eMaximumCoordinate, "NpArticulationLink::setGlobalPose teleport isn't allowed in the reduced coordinate system.");
	
	setGlobalPoseInternal(pose, autowake);
}


void NpArticulationLink::setLinearVelocity(const PxVec3& velocity, bool autowake)
{
	NpScene* scene = NpActor::getOwnerScene(*this);

	PX_CHECK_AND_RETURN(velocity.isFinite(), "NpArticulationLink::setLinearVelocity velocity is not valid.");

	NP_WRITE_CHECK(scene);
	
	getScbBodyFast().setLinearVelocity(velocity);

	if (scene)
		reinterpret_cast<PxArticulationImpl*>(mRoot->getImpl())->wakeUpInternal((!velocity.isZero()), autowake);
}


void NpArticulationLink::setAngularVelocity(const PxVec3& velocity, bool autowake)
{
	NpScene* scene = NpActor::getOwnerScene(*this);

	PX_CHECK_AND_RETURN(velocity.isFinite(), "NpArticulationLink::setAngularVelocity velocity is not valid.");

	NP_WRITE_CHECK(scene);

	getScbBodyFast().setAngularVelocity(velocity);

	if (scene)
		reinterpret_cast<PxArticulationImpl*>(mRoot->getImpl())->wakeUpInternal((!velocity.isZero()), autowake);
}

void NpArticulationLink::setMaxAngularVelocity(PxReal maxAngularVelocity)
{
	NP_WRITE_CHECK(NpActor::getOwnerScene(*this));
	PX_CHECK_AND_RETURN(PxIsFinite(maxAngularVelocity), "NpArticulationLink::setMaxAngularVelocity: invalid float");
	PX_CHECK_AND_RETURN(maxAngularVelocity >= 0.0f, "NpArticulationLink::setMaxAngularVelocity: threshold must be non-negative!");

	getScbBodyFast().setMaxAngVelSq(maxAngularVelocity * maxAngularVelocity);
}

PxReal NpArticulationLink::getMaxAngularVelocity() const
{
	NP_READ_CHECK(NpActor::getOwnerScene(*this));

	return PxSqrt(getScbBodyFast().getMaxAngVelSq());
}

void NpArticulationLink::setMaxLinearVelocity(PxReal maxLinearVelocity)
{
	NP_WRITE_CHECK(NpActor::getOwnerScene(*this));
	PX_CHECK_AND_RETURN(PxIsFinite(maxLinearVelocity), "NpArticulationLink::setMaxAngularVelocity: invalid float");
	PX_CHECK_AND_RETURN(maxLinearVelocity >= 0.0f, "NpArticulationLink::setMaxAngularVelocity: threshold must be non-negative!");

	getScbBodyFast().setMaxLinVelSq(maxLinearVelocity * maxLinearVelocity);
}

PxReal NpArticulationLink::getMaxLinearVelocity() const
{
	NP_READ_CHECK(NpActor::getOwnerScene(*this));

	return PxSqrt(getScbBodyFast().getMaxLinVelSq());
}


#if PX_ENABLE_DEBUG_VISUALIZATION
void NpArticulationLink::visualize(Cm::RenderOutput& out, NpScene* scene)
{
	NpArticulationLinkT::visualize(out, scene);

	if (getScbBodyFast().getActorFlags() & PxActorFlag::eVISUALIZATION)
	{
		PX_ASSERT(getScene());
		PxReal scale = getScene()->getVisualizationParameter(PxVisualizationParameter::eSCALE);

		PxReal massAxes = scale * getScene()->getVisualizationParameter(PxVisualizationParameter::eBODY_MASS_AXES);
		if (massAxes != 0)
		{
			PxU32 color = 0xff;
			color = (color<<16 | color<<8 | color);
			PxVec3 dims = invertDiagInertia(getScbBodyFast().getInverseInertia());
			dims = getDimsFromBodyInertia(dims, 1.0f / getScbBodyFast().getInverseMass());

			out << color << getScbBodyFast().getBody2World() << Cm::DebugBox(dims * 0.5f);
		}	
		PxReal frameScale = scale * getScene()->getVisualizationParameter(PxVisualizationParameter::eJOINT_LOCAL_FRAMES);
		PxReal limitScale = scale * getScene()->getVisualizationParameter(PxVisualizationParameter::eJOINT_LIMITS);
		if ( frameScale != 0.0f || limitScale != 0.0f )
		{
			Cm::ConstraintImmediateVisualizer viz( frameScale, limitScale, out );
			visualizeJoint( viz );
		}
	}
}

static PX_FORCE_INLINE PxReal computePhi(const PxQuat& q)
{
	PxQuat twist = q;
	twist.normalize();

	PxReal angle = twist.getAngle();
	if (twist.x<0.0f)
		angle = -angle;
	return angle;
}

static PX_FORCE_INLINE float	computeSwingAngle(float swingYZ, float swingW)
{
	return 4.0f * PxAtan2(swingYZ, 1.0f + swingW);	// tan (t/2) = sin(t)/(1+cos t), so this is the quarter angle
}

static PX_FORCE_INLINE void separateSwingTwist(const PxQuat& q, PxQuat& twist, PxQuat& swing1, PxQuat& swing2)
{
	twist = q.x != 0.0f ? PxQuat(q.x, 0, 0, q.w).getNormalized() : PxQuat(PxIdentity);
	PxQuat swing = q * twist.getConjugate();
	swing1 = swing.y != 0.f ? PxQuat(0.f, swing.y, 0.f, swing.w).getNormalized() : PxQuat(PxIdentity);
	swing = swing * swing1.getConjugate();
	swing2 = swing.z != 0.f ? PxQuat(0.f, 0.f, swing.z, swing.w).getNormalized() : PxQuat(PxIdentity);
}


void NpArticulationLink::visualizeJoint(PxConstraintVisualizer& jointViz)
{
	NpArticulationLink* parent = getParent();
	if(parent)
	{
		PxTransform cA2w = getGlobalPose().transform(mInboundJoint->getChildPose());
		PxTransform cB2w = parent->getGlobalPose().transform(mInboundJoint->getParentPose());
	
		jointViz.visualizeJointFrames(cA2w, cB2w);

		PxArticulationJointImpl* impl = mInboundJoint->getImpl();

		if (impl->mType == PxArticulationBase::eMaximumCoordinate)
		{

			PxTransform parentFrame = cB2w;

			if (cA2w.q.dot(cB2w.q) < 0)
				cB2w.q = -cB2w.q;

			PxTransform cB2cA = cA2w.transformInv(cB2w);

			PxQuat swing, twist;
			Ps::separateSwingTwist(cB2cA.q, swing, twist);

			PxMat33 cA2w_m(cA2w.q), cB2w_m(cB2w.q);

			PxReal tqPhi = Ps::tanHalf(twist.x, twist.w);		// always support (-pi, +pi)

			PxReal lower, upper, yLimit, zLimit;

			impl->getScbArticulationJoint().getTwistLimit(lower, upper);
			impl->getScbArticulationJoint().getSwingLimit(yLimit, zLimit);
			PxReal swingPad = impl->getScbArticulationJoint().getSwingLimitContactDistance(), twistPad = impl->getScbArticulationJoint().getTwistLimitContactDistance();
			jointViz.visualizeAngularLimit(parentFrame, lower, upper, PxAbs(tqPhi) > PxTan(upper - twistPad));

			PxVec3 tanQSwing = PxVec3(0, Ps::tanHalf(swing.z, swing.w), -Ps::tanHalf(swing.y, swing.w));
			Cm::ConeLimitHelper coneHelper(PxTan(yLimit / 4), PxTan(zLimit / 4), PxTan(swingPad / 4));
			jointViz.visualizeLimitCone(parentFrame, PxTan(yLimit / 4), PxTan(zLimit / 4), !coneHelper.contains(tanQSwing));
		}
		else
		{
			//(1) visualize any angular dofs/limits...

			const PxMat33 cA2w_m(cA2w.q), cB2w_m(cB2w.q);

			PxTransform parentFrame = cB2w;

			if (cA2w.q.dot(cB2w.q) < 0)
				cB2w.q = -cB2w.q;

			//const PxTransform cB2cA = cA2w.transformInv(cB2w);

			const PxTransform cA2cB = cB2w.transformInv(cA2w);

			Sc::ArticulationJointCore& joint = impl->getScbArticulationJoint().getScArticulationJoint();

			PxQuat swing1, swing2, twist;
			separateSwingTwist(cA2cB.q, twist, swing1, swing2);

			const PxReal pad = 0.01f;

			if(joint.getMotion(PxArticulationAxis::eTWIST))
			{
				PxReal lowLimit, highLimit;

				const PxReal angle = computePhi(twist);
				joint.getLimit(PxArticulationAxis::Enum(PxArticulationAxis::eTWIST), lowLimit, highLimit);

				bool active = (angle-pad) < lowLimit || (angle+pad) > highLimit;

				PxTransform tmp = parentFrame;

				jointViz.visualizeAngularLimit(tmp, lowLimit, highLimit, active);
			}

			if (joint.getMotion(PxArticulationAxis::eSWING1))
			{
				PxReal lowLimit, highLimit;

				joint.getLimit(PxArticulationAxis::Enum(PxArticulationAxis::eSWING1), lowLimit, highLimit);

				const PxReal angle = computeSwingAngle(swing1.y, swing1.w);

				bool active = (angle - pad) < lowLimit || (angle + pad) > highLimit;

				PxTransform tmp = parentFrame;
				tmp.q = tmp.q * PxQuat(-PxPiDivTwo, PxVec3(0.f, 0.f, 1.f));

				
				jointViz.visualizeAngularLimit(tmp, -highLimit, -lowLimit, active);
			}

			if (joint.getMotion(PxArticulationAxis::eSWING2))
			{
				PxReal lowLimit, highLimit;

				joint.getLimit(PxArticulationAxis::Enum(PxArticulationAxis::eSWING2), lowLimit, highLimit);

				const PxReal angle = computeSwingAngle(swing2.z, swing2.w);

				bool active = (angle - pad) < lowLimit || (angle + pad) > highLimit;

				PxTransform tmp = parentFrame;
				tmp.q = tmp.q * PxQuat(PxPiDivTwo, PxVec3(0.f, 1.f, 0.f));

				jointViz.visualizeAngularLimit(tmp, -highLimit, -lowLimit, active);
			}

			for (PxU32 i = PxArticulationAxis::eX; i <= PxArticulationAxis::eZ; ++i)
			{
				if (joint.getMotion(PxArticulationAxis::Enum(i)) == PxArticulationMotion::eLIMITED)
				{
					PxU32 index = i - PxArticulationAxis::eX;
					PxReal lowLimit, highLimit;
					joint.getLimit(PxArticulationAxis::Enum(i), lowLimit, highLimit);
					PxReal ordinate = cA2cB.p[index];
					PxVec3 origin = cB2w.p;
					PxVec3 axis = cA2w_m[index];
					const bool active = ordinate < lowLimit || ordinate > highLimit;
					const PxVec3 p0 = origin + axis * lowLimit;
					const PxVec3 p1 = origin + axis * highLimit;
					jointViz.visualizeLine(p0, p1, active ? 0xff0000u : 0xffffffu);
				}
			}
		}
	}	
}
#endif  // PX_ENABLE_DEBUG_VISUALIZATION
