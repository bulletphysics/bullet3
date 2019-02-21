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


#include "PxRigidBodyExt.h"
#include "PxShapeExt.h"
#include "PxMassProperties.h"

#include "ExtInertiaTensor.h"
#include "PsAllocator.h"
#include "PsFoundation.h"

#include "PxShape.h"
#include "PxScene.h"

#include "PxBoxGeometry.h"
#include "PxSphereGeometry.h"
#include "PxCapsuleGeometry.h"
#include "PxPlaneGeometry.h"
#include "PxConvexMeshGeometry.h"
#include "PxTriangleMeshGeometry.h"
#include "PxHeightFieldGeometry.h"
#include "PxGeometryHelpers.h"

#include "PxConvexMesh.h"

#include "PxBatchQuery.h"

#include "PxRigidDynamic.h"
#include "PxRigidStatic.h"
#include "CmUtils.h"

using namespace physx;
using namespace Cm;

static bool computeMassAndDiagInertia(Ext::InertiaTensorComputer& inertiaComp, 
		PxVec3& diagTensor, PxQuat& orient, PxReal& massOut, PxVec3& coM, bool lockCOM, const PxRigidBody& body, const char* errorStr)
{
	// The inertia tensor and center of mass is relative to the actor at this point. Transform to the
	// body frame directly if CoM is specified, else use computed center of mass
	if (lockCOM)
	{
		inertiaComp.translate(-coM);  // base the tensor on user's desired center of mass.
	}
	else
	{
		//get center of mass - has to be done BEFORE centering.
		coM = inertiaComp.getCenterOfMass();

		//the computed result now needs to be centered around the computed center of mass:
		inertiaComp.center();
	}
	// The inertia matrix is now based on the body's center of mass desc.massLocalPose.p
	
	massOut = inertiaComp.getMass();
	diagTensor = PxDiagonalize(inertiaComp.getInertia(), orient);

	if ((diagTensor.x > 0.0f) && (diagTensor.y > 0.0f) && (diagTensor.z > 0.0f))
		return true;
	else
	{
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, 
								"%s: inertia tensor has negative components (ill-conditioned input expected). Approximation for inertia tensor will be used instead.", errorStr);

		// keep center of mass but use the AABB as a crude approximation for the inertia tensor
		PxBounds3 bounds = body.getWorldBounds();
		PxTransform pose = body.getGlobalPose();
		bounds = PxBounds3::transformFast(pose.getInverse(), bounds);
		Ext::InertiaTensorComputer it(false);
		it.setBox(bounds.getExtents());
		it.scaleDensity(massOut / it.getMass());
		PxMat33 inertia = it.getInertia();
		diagTensor = PxVec3(inertia.column0.x, inertia.column1.y, inertia.column2.z);
		orient = PxQuat(PxIdentity);

		return true;
	}
}

static bool computeMassAndInertia(bool multipleMassOrDensity, PxRigidBody& body, const PxReal* densities, const PxReal* masses, PxU32 densityOrMassCount, bool includeNonSimShapes, Ext::InertiaTensorComputer& computer)
{
	PX_ASSERT(!densities || !masses);
	PX_ASSERT((densities || masses) && (densityOrMassCount > 0));

	Ext::InertiaTensorComputer inertiaComp(true);

	Ps::InlineArray<PxShape*, 16> shapes("PxShape*"); shapes.resize(body.getNbShapes());

	body.getShapes(shapes.begin(), shapes.size());

	PxU32 validShapeIndex = 0;
	PxReal currentMassOrDensity;
	const PxReal* massOrDensityArray;
	if (densities)
	{
		massOrDensityArray = densities;
		currentMassOrDensity = densities[0];
	}
	else
	{
		massOrDensityArray = masses;
		currentMassOrDensity = masses[0];
	}
	if (!PxIsFinite(currentMassOrDensity))
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, 
			"computeMassAndInertia: Provided mass or density has no valid value");
		return false;
	}

	for(PxU32 i=0; i < shapes.size(); i++)
	{
		if ((!(shapes[i]->getFlags() & PxShapeFlag::eSIMULATION_SHAPE)) && (!includeNonSimShapes))
			continue; 

		if (multipleMassOrDensity)
		{
			if (validShapeIndex < densityOrMassCount)
			{
				currentMassOrDensity = massOrDensityArray[validShapeIndex];

				if (!PxIsFinite(currentMassOrDensity))
				{
					Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, 
						"computeMassAndInertia: Provided mass or density has no valid value");
					return false;
				}
			}
			else
			{
				Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, 
					"computeMassAndInertia: Not enough mass/density values provided for all (simulation) shapes");
				return false;
			}
		}

		Ext::InertiaTensorComputer it(false);

		switch(shapes[i]->getGeometryType())
		{
		case PxGeometryType::eSPHERE : 
			{
				PxSphereGeometry g;
				bool ok = shapes[i]->getSphereGeometry(g);
				PX_ASSERT(ok);
				PX_UNUSED(ok);
				PxTransform temp(shapes[i]->getLocalPose());

				it.setSphere(g.radius, &temp);
			}
			break;

		case PxGeometryType::eBOX : 
			{
				PxBoxGeometry g;
				bool ok = shapes[i]->getBoxGeometry(g);
				PX_ASSERT(ok);
				PX_UNUSED(ok);
				PxTransform temp(shapes[i]->getLocalPose());

				it.setBox(g.halfExtents, &temp);
			}
			break;

		case PxGeometryType::eCAPSULE : 
			{
				PxCapsuleGeometry g;
				bool ok = shapes[i]->getCapsuleGeometry(g);
				PX_ASSERT(ok);
				PX_UNUSED(ok);
				PxTransform temp(shapes[i]->getLocalPose());

				it.setCapsule(0, g.radius, g.halfHeight, &temp);
			}
			break;

		case PxGeometryType::eCONVEXMESH : 
			{
				PxConvexMeshGeometry g;
				bool ok = shapes[i]->getConvexMeshGeometry(g);
				PX_ASSERT(ok);
				PX_UNUSED(ok);
				PxConvexMesh& convMesh = *g.convexMesh;

				PxReal convMass;
				PxMat33 convInertia;
				PxVec3 convCoM;
				convMesh.getMassInformation(convMass, reinterpret_cast<PxMat33&>(convInertia), convCoM);

				if (!g.scale.isIdentity())
				{
					//scale the mass properties
					convMass *= (g.scale.scale.x * g.scale.scale.y * g.scale.scale.z);
					convCoM = g.scale.rotation.rotateInv(g.scale.scale.multiply(g.scale.rotation.rotate(convCoM)));
					convInertia = PxMassProperties::scaleInertia(convInertia, g.scale.rotation, g.scale.scale);
				}

				it = Ext::InertiaTensorComputer(convInertia, convCoM, convMass);
				it.transform(shapes[i]->getLocalPose());
			}
			break;
		case PxGeometryType::eHEIGHTFIELD:
		case PxGeometryType::ePLANE:
		case PxGeometryType::eTRIANGLEMESH:
		case PxGeometryType::eINVALID:
		case PxGeometryType::eGEOMETRY_COUNT:
			{

				Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, 
					"computeMassAndInertia: Dynamic actor with illegal collision shapes");
				return false;
			}
		}

		if (densities)
			it.scaleDensity(currentMassOrDensity);
		else if (multipleMassOrDensity)  // mass per shape -> need to scale density per shape
			it.scaleDensity(currentMassOrDensity / it.getMass());

		inertiaComp.add(it);

		validShapeIndex++;
	}

	if (validShapeIndex && masses && (!multipleMassOrDensity))  // at least one simulation shape and single mass for all shapes -> scale density at the end
	{
		inertiaComp.scaleDensity(currentMassOrDensity / inertiaComp.getMass());
	}

	computer = inertiaComp;
	return true;
}

static bool updateMassAndInertia(bool multipleMassOrDensity, PxRigidBody& body, const PxReal* densities, PxU32 densityCount, const PxVec3* massLocalPose, bool includeNonSimShapes)
{
	bool success;

	// default values in case there were no shapes
	PxReal massOut = 1.0f;
	PxVec3 diagTensor(1.f,1.f,1.f);
	PxQuat orient = PxQuat(PxIdentity);
	bool lockCom = massLocalPose != NULL;
	PxVec3 com = lockCom ? *massLocalPose : PxVec3(0);
	const char* errorStr = "PxRigidBodyExt::updateMassAndInertia";

	if (densities && densityCount)
	{
		Ext::InertiaTensorComputer inertiaComp(true);
		if(computeMassAndInertia(multipleMassOrDensity, body, densities, NULL, densityCount, includeNonSimShapes, inertiaComp))
		{
			if(inertiaComp.getMass()!=0 && computeMassAndDiagInertia(inertiaComp, diagTensor, orient, massOut, com, lockCom, body, errorStr))
				success = true;
			else
				success = false;  // body with no shapes provided or computeMassAndDiagInertia() failed
		}
		else
		{
			Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, 
				"%s: Mass and inertia computation failed, setting mass to 1 and inertia to (1,1,1)", errorStr);

			success = false;
		}
	}
	else
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, 
			"%s: No density specified, setting mass to 1 and inertia to (1,1,1)", errorStr);

		success = false;
	}

	PX_ASSERT(orient.isFinite());
	PX_ASSERT(diagTensor.isFinite());
	PX_ASSERT(PxIsFinite(massOut));

	body.setMass(massOut);
	body.setMassSpaceInertiaTensor(diagTensor);
	body.setCMassLocalPose(PxTransform(com, orient));

	return success;
}

bool PxRigidBodyExt::updateMassAndInertia(PxRigidBody& body, const PxReal* densities, PxU32 densityCount, const PxVec3* massLocalPose, bool includeNonSimShapes)
{
	return ::updateMassAndInertia(true, body, densities, densityCount, massLocalPose, includeNonSimShapes);
}

bool PxRigidBodyExt::updateMassAndInertia(PxRigidBody& body, PxReal density, const PxVec3* massLocalPose, bool includeNonSimShapes)
{
	return ::updateMassAndInertia(false, body, &density, 1, massLocalPose, includeNonSimShapes);
}

static bool setMassAndUpdateInertia(bool multipleMassOrDensity, PxRigidBody& body, const PxReal* masses, PxU32 massCount, const PxVec3* massLocalPose, bool includeNonSimShapes)
{
	bool success;

	// default values in case there were no shapes
	PxReal massOut = 1.0f;
	PxVec3 diagTensor(1.0f,1.0f,1.0f);
	PxQuat orient = PxQuat(PxIdentity);
	bool lockCom = massLocalPose != NULL;
	PxVec3 com = lockCom ? *massLocalPose : PxVec3(0);
	const char* errorStr = "PxRigidBodyExt::setMassAndUpdateInertia";

	if(masses && massCount)
	{
		Ext::InertiaTensorComputer inertiaComp(true);
		if(computeMassAndInertia(multipleMassOrDensity, body, NULL, masses, massCount, includeNonSimShapes, inertiaComp))
		{
			success = true;

			if (inertiaComp.getMass()!=0 && !computeMassAndDiagInertia(inertiaComp, diagTensor, orient, massOut, com, lockCom, body, errorStr))
				success = false;  // computeMassAndDiagInertia() failed (mass zero?)

			if (massCount == 1)
				massOut = masses[0]; // to cover special case where body has no simulation shape
		}
		else
		{
			Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, 
				"%s: Mass and inertia computation failed, setting mass to 1 and inertia to (1,1,1)", errorStr);

			success = false;
		}
	}
	else
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, 
			"%s: No mass specified, setting mass to 1 and inertia to (1,1,1)", errorStr);
		success = false;
	}

	PX_ASSERT(orient.isFinite());
	PX_ASSERT(diagTensor.isFinite());

	body.setMass(massOut);
	body.setMassSpaceInertiaTensor(diagTensor);
	body.setCMassLocalPose(PxTransform(com, orient));

	return success;
}

bool PxRigidBodyExt::setMassAndUpdateInertia(PxRigidBody& body, const PxReal* masses, PxU32 massCount, const PxVec3* massLocalPose, bool includeNonSimShapes)
{
	return ::setMassAndUpdateInertia(true, body, masses, massCount, massLocalPose, includeNonSimShapes);
}

bool PxRigidBodyExt::setMassAndUpdateInertia(PxRigidBody& body, PxReal mass, const PxVec3* massLocalPose, bool includeNonSimShapes)
{
	return ::setMassAndUpdateInertia(false, body, &mass, 1, massLocalPose, includeNonSimShapes);
}

PxMassProperties PxRigidBodyExt::computeMassPropertiesFromShapes(const PxShape* const* shapes, PxU32 shapeCount)
{
	Ps::InlineArray<PxMassProperties, 16> massProps;
	massProps.reserve(shapeCount);
	Ps::InlineArray<PxTransform, 16> localTransforms;
	localTransforms.reserve(shapeCount);

	for(PxU32 shapeIdx=0; shapeIdx < shapeCount; shapeIdx++)
	{
		const PxShape* shape = shapes[shapeIdx];
		PxMassProperties mp(shape->getGeometry().any());
		massProps.pushBack(mp);
		localTransforms.pushBack(shape->getLocalPose());
	}

	return PxMassProperties::sum(massProps.begin(), localTransforms.begin(), shapeCount);
}

PX_INLINE void addForceAtPosInternal(PxRigidBody& body, const PxVec3& force, const PxVec3& pos, PxForceMode::Enum mode, bool wakeup)
{
	if(mode == PxForceMode::eACCELERATION || mode == PxForceMode::eVELOCITY_CHANGE)
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, 
			"PxRigidBodyExt::addForce methods do not support eACCELERATION or eVELOCITY_CHANGE modes");
		return;
	}

	const PxTransform globalPose = body.getGlobalPose();
	const PxVec3 centerOfMass = globalPose.transform(body.getCMassLocalPose().p);

	const PxVec3 torque = (pos - centerOfMass).cross(force);
	body.addForce(force, mode, wakeup);
	body.addTorque(torque, mode, wakeup);
}

void PxRigidBodyExt::addForceAtPos(PxRigidBody& body, const PxVec3& force, const PxVec3& pos, PxForceMode::Enum mode, bool wakeup)
{
	addForceAtPosInternal(body, force, pos, mode, wakeup);
}

void PxRigidBodyExt::addForceAtLocalPos(PxRigidBody& body, const PxVec3& force, const PxVec3& pos, PxForceMode::Enum mode, bool wakeup)
{
	//transform pos to world space
	const PxVec3 globalForcePos = body.getGlobalPose().transform(pos);

	addForceAtPosInternal(body, force, globalForcePos, mode, wakeup);
}

void PxRigidBodyExt::addLocalForceAtPos(PxRigidBody& body, const PxVec3& force, const PxVec3& pos, PxForceMode::Enum mode, bool wakeup)
{
	const PxVec3 globalForce = body.getGlobalPose().rotate(force);

	addForceAtPosInternal(body, globalForce, pos, mode, wakeup);
}

void PxRigidBodyExt::addLocalForceAtLocalPos(PxRigidBody& body, const PxVec3& force, const PxVec3& pos, PxForceMode::Enum mode, bool wakeup)
{
	const PxTransform globalPose = body.getGlobalPose();
	const PxVec3 globalForcePos = globalPose.transform(pos);
	const PxVec3 globalForce = globalPose.rotate(force);

	addForceAtPosInternal(body, globalForce, globalForcePos, mode, wakeup);
}

PX_INLINE PxVec3 getVelocityAtPosInternal(const PxRigidBody& body, const PxVec3& point)
{
	PxVec3 velocity = body.getLinearVelocity();
	velocity       += body.getAngularVelocity().cross(point);
	
	return velocity;
}

PxVec3 PxRigidBodyExt::getVelocityAtPos(const PxRigidBody& body, const PxVec3& point)
{
	const PxTransform globalPose = body.getGlobalPose();
	const PxVec3 centerOfMass    = globalPose.transform(body.getCMassLocalPose().p);
	const PxVec3 rpoint          = point - centerOfMass;

	return getVelocityAtPosInternal(body, rpoint);
}

PxVec3 PxRigidBodyExt::getLocalVelocityAtLocalPos(const PxRigidBody& body, const PxVec3& point)
{
	const PxTransform globalPose = body.getGlobalPose();
	const PxVec3 centerOfMass    = globalPose.transform(body.getCMassLocalPose().p);
	const PxVec3 rpoint          = globalPose.transform(point) - centerOfMass;

	return getVelocityAtPosInternal(body, rpoint);
}

PxVec3 PxRigidBodyExt::getVelocityAtOffset(const PxRigidBody& body, const PxVec3& point)
{
	const PxTransform globalPose = body.getGlobalPose();
	const PxVec3 centerOfMass    = globalPose.rotate(body.getCMassLocalPose().p);
	const PxVec3 rpoint          = point - centerOfMass;

	return getVelocityAtPosInternal(body, rpoint);
}

void PxRigidBodyExt::computeVelocityDeltaFromImpulse(const PxRigidBody& body, const PxTransform& globalPose, const PxVec3& point, const PxVec3& impulse, const PxReal invMassScale, 
														const PxReal invInertiaScale, PxVec3& linearVelocityChange, PxVec3& angularVelocityChange)
{
	const PxVec3 centerOfMass = globalPose.transform(body.getCMassLocalPose().p);
	const PxReal invMass = body.getInvMass() * invMassScale;
	const PxVec3 invInertiaMS = body.getMassSpaceInvInertiaTensor() * invInertiaScale;

	PxMat33 invInertia;
	transformInertiaTensor(invInertiaMS, PxMat33(globalPose.q), invInertia);
	linearVelocityChange = impulse * invMass;
	const PxVec3 rXI = (point - centerOfMass).cross(impulse);
	angularVelocityChange = invInertia * rXI;
}

void PxRigidBodyExt::computeLinearAngularImpulse(const PxRigidBody& body, const PxTransform& globalPose, const PxVec3& point, const PxVec3& impulse, const PxReal invMassScale, 
														const PxReal invInertiaScale, PxVec3& linearImpulse, PxVec3& angularImpulse)
{
	const PxVec3 centerOfMass = globalPose.transform(body.getCMassLocalPose().p);
	linearImpulse = impulse * invMassScale;
	angularImpulse = (point - centerOfMass).cross(impulse) * invInertiaScale;
}



//=================================================================================
// Single closest hit compound sweep
bool PxRigidBodyExt::linearSweepSingle(
	PxRigidBody& body, PxScene& scene, const PxVec3& unitDir, const PxReal distance,
	PxHitFlags outputFlags, PxSweepHit& closestHit, PxU32& shapeIndex,
	const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall,
	const PxQueryCache* cache, const PxReal inflation)
{
	shapeIndex = 0xFFFFffff;
	PxReal closestDist = distance;
	PxU32 nbShapes = body.getNbShapes();
	for(PxU32 i=0; i < nbShapes; i++)
	{
		PxShape* shape = NULL;
		body.getShapes(&shape, 1, i);
		PX_ASSERT(shape != NULL);
		PxTransform pose = PxShapeExt::getGlobalPose(*shape, body);
		PxQueryFilterData fd;
		fd.flags = filterData.flags;
		PxU32 or4 = (filterData.data.word0 | filterData.data.word1 | filterData.data.word2 | filterData.data.word3);
		fd.data = or4 ? filterData.data : shape->getQueryFilterData();
		PxGeometryHolder anyGeom = shape->getGeometry();

		PxSweepBuffer subHit; // touching hits are not allowed to be returned from the filters
		scene.sweep(anyGeom.any(), pose, unitDir, distance, subHit, outputFlags, fd, filterCall, cache, inflation);
		if (subHit.hasBlock && subHit.block.distance < closestDist)
		{
			closestDist = subHit.block.distance;
			closestHit = subHit.block;
			shapeIndex = i;
		}
	}

	return (shapeIndex != 0xFFFFffff);
}

//=================================================================================
// Multiple hits compound sweep
// AP: we might be able to improve the return results API but no time for it in 3.3
PxU32 PxRigidBodyExt::linearSweepMultiple(
	PxRigidBody& body, PxScene& scene, const PxVec3& unitDir, const PxReal distance, PxHitFlags outputFlags,
	PxSweepHit* hitBuffer, PxU32* hitShapeIndices, PxU32 hitBufferSize, PxSweepHit& block, PxI32& blockingHitShapeIndex,
	bool& overflow, const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall,
	const PxQueryCache* cache, const PxReal inflation)
{
	overflow = false;
	blockingHitShapeIndex = -1;

	for (PxU32 i = 0; i < hitBufferSize; i++)
		hitShapeIndices[i] = 0xFFFFffff;

	PxI32 sumNbResults = 0;

	PxU32 nbShapes = body.getNbShapes();
	PxF32 shrunkMaxDistance = distance;
	for(PxU32 i=0; i < nbShapes; i++)
	{
		PxShape* shape = NULL;
		body.getShapes(&shape, 1, i);
		PX_ASSERT(shape != NULL);
		PxTransform pose = PxShapeExt::getGlobalPose(*shape, body);
		PxQueryFilterData fd;
		fd.flags = filterData.flags;
		PxU32 or4 = (filterData.data.word0 | filterData.data.word1 | filterData.data.word2 | filterData.data.word3);
		fd.data = or4 ? filterData.data : shape->getQueryFilterData();
		PxGeometryHolder anyGeom = shape->getGeometry();

		PxU32 bufSizeLeft = hitBufferSize-sumNbResults;
		PxSweepHit extraHit;
		PxSweepBuffer buffer(bufSizeLeft == 0 ? &extraHit : hitBuffer+sumNbResults, bufSizeLeft == 0 ? 1 : hitBufferSize-sumNbResults);
		scene.sweep(anyGeom.any(), pose, unitDir, shrunkMaxDistance, buffer, outputFlags, fd, filterCall, cache, inflation);

		// Check and abort on overflow. Assume overflow if result count is bufSize.
		PxU32 nbNewResults = buffer.getNbTouches();
		overflow |= (nbNewResults >= bufSizeLeft);
		if (bufSizeLeft == 0) // this is for when we used the extraHit buffer
			nbNewResults = 0;

		// set hitShapeIndices for each new non-blocking hit
		for (PxU32 j = 0; j < nbNewResults; j++)
			if (sumNbResults + PxU32(j) < hitBufferSize)
				hitShapeIndices[sumNbResults+j] = i;

		if (buffer.hasBlock) // there's a blocking hit in the most recent sweepMultiple results
		{
			// overwrite the return result blocking hit with the new blocking hit if under
			if (blockingHitShapeIndex == -1 || buffer.block.distance < block.distance)
			{
				blockingHitShapeIndex = PxI32(i);
				block = buffer.block;
			}

			// Remove all the old touching hits below the new maxDist
			// sumNbResults is not updated yet at this point
			//   and represents the count accumulated so far excluding the very last query
			PxI32 nbNewResultsSigned = PxI32(nbNewResults); // need a signed version, see nbNewResultsSigned-- below
			for (PxI32 j = sumNbResults-1; j >= 0; j--) // iterate over "old" hits (up to shapeIndex-1)
				if (buffer.block.distance < hitBuffer[j].distance)
				{
					// overwrite with last "new" hit
					PxI32 sourceIndex = PxI32(sumNbResults)+nbNewResultsSigned-1; PX_ASSERT(sourceIndex >= j);
					hitBuffer[j] = hitBuffer[sourceIndex];
					hitShapeIndices[j] = hitShapeIndices[sourceIndex];
					nbNewResultsSigned--; // can get negative, that means we are shifting the last results array
				}

			sumNbResults += nbNewResultsSigned;
		} else // if there was no new blocking hit we don't need to do anything special, simply append all results to touch array
			sumNbResults += nbNewResults;

		PX_ASSERT(sumNbResults >= 0 && sumNbResults <= PxI32(hitBufferSize));
	}

	return PxU32(sumNbResults);
}

void PxRigidBodyExt::computeVelocityDeltaFromImpulse(const PxRigidBody& body, const PxVec3& impulsiveForce, const PxVec3& impulsiveTorque, PxVec3& deltaLinearVelocity, PxVec3& deltaAngularVelocity)
{
	{
		const PxF32 recipMass = body.getInvMass();
		deltaLinearVelocity = impulsiveForce*recipMass;
	}

	{
		const PxTransform globalPose = body.getGlobalPose();
		const PxTransform cmLocalPose = body.getCMassLocalPose();
		const PxTransform body2World = globalPose*cmLocalPose;
		PxMat33 M(body2World.q);

		const PxVec3 recipInertiaBodySpace = body.getMassSpaceInvInertiaTensor();

		PxMat33 recipInertiaWorldSpace;
		const float	axx = recipInertiaBodySpace.x*M(0,0), axy = recipInertiaBodySpace.x*M(1,0), axz = recipInertiaBodySpace.x*M(2,0);
		const float	byx = recipInertiaBodySpace.y*M(0,1), byy = recipInertiaBodySpace.y*M(1,1), byz = recipInertiaBodySpace.y*M(2,1);
		const float	czx = recipInertiaBodySpace.z*M(0,2), czy = recipInertiaBodySpace.z*M(1,2), czz = recipInertiaBodySpace.z*M(2,2);
		recipInertiaWorldSpace(0,0) = axx*M(0,0) + byx*M(0,1) + czx*M(0,2);
		recipInertiaWorldSpace(1,1) = axy*M(1,0) + byy*M(1,1) + czy*M(1,2);
		recipInertiaWorldSpace(2,2) = axz*M(2,0) + byz*M(2,1) + czz*M(2,2);
		recipInertiaWorldSpace(0,1) = recipInertiaWorldSpace(1,0) = axx*M(1,0) + byx*M(1,1) + czx*M(1,2);
		recipInertiaWorldSpace(0,2) = recipInertiaWorldSpace(2,0) = axx*M(2,0) + byx*M(2,1) + czx*M(2,2);
		recipInertiaWorldSpace(1,2) = recipInertiaWorldSpace(2,1) = axy*M(2,0) + byy*M(2,1) + czy*M(2,2);

		deltaAngularVelocity = recipInertiaWorldSpace*(impulsiveTorque);
	}
}


