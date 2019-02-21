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

#include "PxRaycastCCD.h"

using namespace physx;

#include "geometry/PxBoxGeometry.h"
#include "geometry/PxSphereGeometry.h"
#include "geometry/PxCapsuleGeometry.h"
#include "geometry/PxConvexMeshGeometry.h"
#include "geometry/PxConvexMesh.h"
#include "PxScene.h"
#include "PxRigidDynamic.h"
#include "extensions/PxShapeExt.h"
#include "PsArray.h"

namespace physx
{
class RaycastCCDManagerInternal
{
	PX_NOCOPY(RaycastCCDManagerInternal)
	public:
				RaycastCCDManagerInternal(PxScene* scene) : mScene(scene)	{}
				~RaycastCCDManagerInternal(){}

		bool	registerRaycastCCDObject(PxRigidDynamic* actor, PxShape* shape);

		void	doRaycastCCD(bool doDynamicDynamicCCD);

		struct CCDObject
		{
			PX_FORCE_INLINE	CCDObject(PxRigidDynamic* actor, PxShape* shape, const PxVec3& witness) : mActor(actor), mShape(shape), mWitness(witness)	{}
			PxRigidDynamic*	mActor;
			PxShape*		mShape;
			PxVec3			mWitness;
		};

	private:
		PxScene*						mScene;
		physx::shdfnd::Array<CCDObject>	mObjects;
};
}

static PxVec3 getShapeCenter(PxShape* shape, const PxTransform& pose)
{
	PxVec3 offset(0.0f);
	if(shape->getGeometryType()==PxGeometryType::eCONVEXMESH)
	{
		PxConvexMeshGeometry geometry;
		bool status = shape->getConvexMeshGeometry(geometry);
		PX_UNUSED(status);
		PX_ASSERT(status);

		PxReal mass;
		PxMat33 localInertia;
		PxVec3 localCenterOfMass;
		geometry.convexMesh->getMassInformation(mass, localInertia, localCenterOfMass);

		offset += localCenterOfMass;
	}
	return pose.transform(offset);
}

static PX_FORCE_INLINE PxVec3 getShapeCenter(PxRigidActor* actor, PxShape* shape)
{
	const PxTransform pose = PxShapeExt::getGlobalPose(*shape, *actor);
	return getShapeCenter(shape, pose);
}

static PxReal computeInternalRadius(PxRigidActor* actor, PxShape* shape, const PxVec3& dir)
{
	const PxBounds3 bounds = PxShapeExt::getWorldBounds(*shape, *actor);
	const PxReal diagonal = (bounds.maximum - bounds.minimum).magnitude();
	const PxReal offsetFromOrigin = diagonal * 2.0f;

	PxTransform pose = PxShapeExt::getGlobalPose(*shape, *actor);

	PxReal internalRadius = 0.0f;
	const PxReal length = offsetFromOrigin*2.0f;

	switch(shape->getGeometryType())
	{
		case PxGeometryType::eSPHERE:
		{
			PxSphereGeometry geometry;
			bool status = shape->getSphereGeometry(geometry);
			PX_UNUSED(status);
			PX_ASSERT(status);

			internalRadius = geometry.radius;
		}
		break;

		case PxGeometryType::eBOX:
		case PxGeometryType::eCAPSULE:
		{
			pose.p = PxVec3(0.0f);
			const PxVec3 virtualOrigin = pose.p + dir * offsetFromOrigin;

			PxRaycastHit hit;
			PxU32 nbHits = PxGeometryQuery::raycast(virtualOrigin, -dir, shape->getGeometry().any(), pose, length, PxHitFlags(0), 1, &hit);
			PX_UNUSED(nbHits);
			PX_ASSERT(nbHits);

			internalRadius = offsetFromOrigin - hit.distance;
		}
		break;

		case PxGeometryType::eCONVEXMESH:
		{
			PxVec3 shapeCenter = getShapeCenter(shape, pose);
			shapeCenter -= pose.p;
			pose.p = PxVec3(0.0f);

			const PxVec3 virtualOrigin = shapeCenter + dir * offsetFromOrigin;
			PxRaycastHit hit;
			PxU32 nbHits = PxGeometryQuery::raycast(virtualOrigin, -dir, shape->getGeometry().any(), pose, length, PxHitFlags(0), 1, &hit);
			PX_UNUSED(nbHits);
			PX_ASSERT(nbHits);

			internalRadius = offsetFromOrigin - hit.distance;
		}
		break;

		case PxGeometryType::ePLANE:
		case PxGeometryType::eHEIGHTFIELD:
		case PxGeometryType::eTRIANGLEMESH:
		case PxGeometryType::eGEOMETRY_COUNT:
		case PxGeometryType::eINVALID:
		break;
	}
	return internalRadius;
}

class CCDRaycastFilterCallback : public PxQueryFilterCallback
{
public:
	CCDRaycastFilterCallback(PxRigidActor* actor, PxShape* shape) : mActor(actor), mShape(shape){}

	PxRigidActor*	mActor;
	PxShape*		mShape;

	virtual PxQueryHitType::Enum preFilter(const PxFilterData&, const PxShape* shape, const PxRigidActor* actor, PxHitFlags&)
	{
		if(mActor==actor && mShape==shape)
			return PxQueryHitType::eNONE;
		return PxQueryHitType::eBLOCK;
	}

	virtual PxQueryHitType::Enum postFilter(const PxFilterData&, const PxQueryHit&)
	{
		return PxQueryHitType::eNONE;
	}
};

static bool CCDRaycast(PxScene* scene, PxRigidActor* actor, PxShape* shape, const PxVec3& origin, const PxVec3& unitDir, const PxReal distance, PxRaycastHit& hit, bool dyna_dyna)
{
	const PxQueryFlags qf(dyna_dyna ? PxQueryFlags(PxQueryFlag::eSTATIC|PxQueryFlag::eDYNAMIC|PxQueryFlag::ePREFILTER) : PxQueryFlags(PxQueryFlag::eSTATIC));
	const PxQueryFilterData filterData(PxFilterData(), qf);

	CCDRaycastFilterCallback CB(actor, shape);

	PxRaycastBuffer buf1;
	scene->raycast(origin, unitDir, distance, buf1, PxHitFlags(0), filterData, &CB);
	hit = buf1.block;
	return buf1.hasBlock;
}

static PxRigidDynamic* canDoCCD(PxRigidActor& actor, PxShape* /*shape*/)
{
	if(actor.getConcreteType()!=PxConcreteType::eRIGID_DYNAMIC)
		return NULL;	// PT: no need to do it for statics
	PxRigidDynamic* dyna = static_cast<PxRigidDynamic*>(&actor);

	const PxU32 nbShapes = dyna->getNbShapes();
	if(nbShapes!=1)
		return NULL;	// PT: only works with simple actors for now

	if(dyna->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC)
		return NULL;	// PT: no need to do it for kinematics

	return dyna;
}

static bool doRaycastCCD(PxScene* scene, const RaycastCCDManagerInternal::CCDObject& object, PxTransform& newPose, PxVec3& newShapeCenter, bool dyna_dyna)
{
	PxRigidDynamic* dyna = canDoCCD(*object.mActor, object.mShape);
	if(!dyna)
		return true;

	bool updateCCDWitness = true;

	const PxVec3 offset = newPose.p - newShapeCenter;
	const PxVec3& origin = object.mWitness;
	const PxVec3& dest = newShapeCenter;

	PxVec3 dir = dest - origin;
	const PxReal length = dir.magnitude();
	if(length!=0.0f)
	{
		dir /= length;

		const PxReal internalRadius = computeInternalRadius(object.mActor, object.mShape, dir);

		PxRaycastHit hit;
		if(internalRadius!=0.0f && CCDRaycast(scene, object.mActor, object.mShape, origin, dir, length, hit, dyna_dyna))
		{
			updateCCDWitness = false;

			const PxReal radiusLimit = internalRadius * 0.75f;
			if(hit.distance>radiusLimit)
			{
				newShapeCenter = origin + dir * (hit.distance - radiusLimit);
			}
			else
			{
				if(hit.actor->getConcreteType()==PxConcreteType::eRIGID_DYNAMIC)
					return true;

				newShapeCenter = origin;
			}

			newPose.p = offset + newShapeCenter;
			const PxTransform shapeLocalPose = object.mShape->getLocalPose();
			const PxTransform inverseShapeLocalPose = shapeLocalPose.getInverse();
			const PxTransform newGlobalPose = newPose * inverseShapeLocalPose;
			dyna->setGlobalPose(newGlobalPose);
		}
	}
	return updateCCDWitness;
}

bool RaycastCCDManagerInternal::registerRaycastCCDObject(PxRigidDynamic* actor, PxShape* shape)
{
	if(!actor || !shape)
		return false;

	mObjects.pushBack(CCDObject(actor, shape, getShapeCenter(actor, shape)));
	return true;
}

void RaycastCCDManagerInternal::doRaycastCCD(bool doDynamicDynamicCCD)
{
	const PxU32 nbObjects = mObjects.size();
	for(PxU32 i=0;i<nbObjects;i++)
	{
		CCDObject& object = mObjects[i];

		if(object.mActor->isSleeping())
			continue;

		PxTransform newPose = PxShapeExt::getGlobalPose(*object.mShape, *object.mActor);
		PxVec3 newShapeCenter = getShapeCenter(object.mShape, newPose);

		if(::doRaycastCCD(mScene, object, newPose, newShapeCenter, doDynamicDynamicCCD))
			object.mWitness = newShapeCenter;
	}
}

RaycastCCDManager::RaycastCCDManager(PxScene* scene)
{
	mImpl = new RaycastCCDManagerInternal(scene);
}

RaycastCCDManager::~RaycastCCDManager()
{
	delete mImpl;
}

bool RaycastCCDManager::registerRaycastCCDObject(PxRigidDynamic* actor, PxShape* shape)
{
	return mImpl->registerRaycastCCDObject(actor, shape);
}

void RaycastCCDManager::doRaycastCCD(bool doDynamicDynamicCCD)
{
	mImpl->doRaycastCCD(doDynamicDynamicCCD);
}
