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


#include "NpRigidStatic.h"
#include "NpPhysics.h"
#include "ScbNpDeps.h"
#include "NpScene.h"
#include "NpRigidActorTemplateInternal.h"

using namespace physx;

NpRigidStatic::NpRigidStatic(const PxTransform& pose)
: NpRigidStaticT(PxConcreteType::eRIGID_STATIC, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE)
, mRigidStatic(pose)
{
}

NpRigidStatic::~NpRigidStatic()
{
}

// PX_SERIALIZATION
void NpRigidStatic::requiresObjects(PxProcessPxBaseCallback& c)
{
	NpRigidStaticT::requiresObjects(c);	
}

NpRigidStatic* NpRigidStatic::createObject(PxU8*& address, PxDeserializationContext& context)
{
	NpRigidStatic* obj = new (address) NpRigidStatic(PxBaseFlag::eIS_RELEASABLE);
	address += sizeof(NpRigidStatic);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}
//~PX_SERIALIZATION

void NpRigidStatic::release()
{
	releaseActorT(this, mRigidStatic);
}

void NpRigidStatic::setGlobalPose(const PxTransform& pose, bool /*wake*/)
{
	PX_CHECK_AND_RETURN(pose.isSane(), "PxRigidStatic::setGlobalPose: pose is not valid.");

	NP_WRITE_CHECK(NpActor::getOwnerScene(*this));

	NpScene* npScene = NpActor::getAPIScene(*this);
#if PX_CHECKED
	if(npScene)
		npScene->checkPositionSanity(*this, pose, "PxRigidStatic::setGlobalPose");
#endif

	mRigidStatic.setActor2World(pose.getNormalized());

	if(npScene)
	{
		mShapeManager.markAllSceneQueryForUpdate(npScene->getSceneQueryManagerFast(), *this);
		npScene->getSceneQueryManagerFast().get(Sq::PruningIndex::eSTATIC).invalidateTimestamp();
	}

#if PX_SUPPORT_PVD
	// have to do this here since this call gets not forwarded to Scb::RigidStatic
	Scb::Scene* scbScene = NpActor::getScbFromPxActor(*this).getScbSceneForAPI();
	if(scbScene)
		scbScene->getScenePvdClient().updatePvdProperties(&mRigidStatic);
#endif

	// invalidate the pruning structure if the actor bounds changed
	if (mShapeManager.getPruningStructure())
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxRigidStatic::setGlobalPose: Actor is part of a pruning structure, pruning structure is now invalid!");
		mShapeManager.getPruningStructure()->invalidate(this);
	}

	updateShaderComs();
}

PxTransform NpRigidStatic::getGlobalPose() const
{
	NP_READ_CHECK(NpActor::getOwnerScene(*this));
	return mRigidStatic.getActor2World();
}

PxU32 physx::NpRigidStaticGetShapes(Scb::RigidStatic& rigid, void* const *&shapes)
{
	NpRigidStatic* a = static_cast<NpRigidStatic*>(rigid.getScRigidCore().getPxActor());
	NpShapeManager& sm = a->getShapeManager();
	shapes = reinterpret_cast<void *const *>(sm.getShapes());
	return sm.getNbShapes();
}

void NpRigidStatic::switchToNoSim()
{
	getScbRigidStaticFast().switchToNoSim(false);
}

void NpRigidStatic::switchFromNoSim()
{
	getScbRigidStaticFast().switchFromNoSim(false);
}

#if PX_CHECKED
bool NpRigidStatic::checkConstraintValidity() const
{
	// Perhaps NpConnectorConstIterator would be worth it...
	NpConnectorIterator iter = (const_cast<NpRigidStatic*>(this))->getConnectorIterator(NpConnectorType::eConstraint); 
	while (PxBase* ser = iter.getNext())
	{
		NpConstraint* c = static_cast<NpConstraint*>(ser);
		if(!c->NpConstraint::isValid())
			return false;
	}
	return true;
}
#endif

#if PX_ENABLE_DEBUG_VISUALIZATION
void NpRigidStatic::visualize(Cm::RenderOutput& out, NpScene* scene)
{
	NpRigidStaticT::visualize(out, scene);

	if (getScbRigidStaticFast().getActorFlags() & PxActorFlag::eVISUALIZATION)
	{
		Scb::Scene& scbScene = scene->getScene();
		PxReal scale = scbScene.getVisualizationParameter(PxVisualizationParameter::eSCALE);

		//visualize actor frames
		PxReal actorAxes = scale * scbScene.getVisualizationParameter(PxVisualizationParameter::eACTOR_AXES);
		if (actorAxes != 0)
			out << getGlobalPose() << Cm::DebugBasis(PxVec3(actorAxes));
	}
}
#endif

