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

#include "NpShapeManager.h"
#include "NpFactory.h"
#include "ScbRigidObject.h"
#include "NpActor.h"
#include "SqPruningStructure.h"
#include "NpScene.h"
#include "NpPtrTableStorageManager.h"
#include "NpRigidDynamic.h"
#include "NpArticulationLink.h"
#include "ScBodySim.h"
#include "GuBounds.h"
#include "CmUtils.h"
#include "PsAlloca.h"

using namespace physx;
using namespace Sq;
using namespace Gu;
using namespace Cm;

static PX_FORCE_INLINE bool isSceneQuery(const NpShape& shape) { return shape.getFlagsFast() & PxShapeFlag::eSCENE_QUERY_SHAPE; }

NpShapeManager::NpShapeManager()
	: mSqCompoundId(INVALID_PRUNERHANDLE), mPruningStructure(NULL)
{
}

// PX_SERIALIZATION
NpShapeManager::NpShapeManager(const PxEMPTY) :
	mShapes			(PxEmpty),
	mSceneQueryData	(PxEmpty) 
{	
}

NpShapeManager::~NpShapeManager() 
{ 
	PX_ASSERT(!mPruningStructure);
	PtrTableStorageManager& sm = NpFactory::getInstance().getPtrTableStorageManager();
	mShapes.clear(sm);
	mSceneQueryData.clear(sm);
}

void NpShapeManager::exportExtraData(PxSerializationContext& stream)
{ 
	mShapes.exportExtraData(stream);							
	mSceneQueryData.exportExtraData(stream);
}

void NpShapeManager::importExtraData(PxDeserializationContext& context)
{ 
	mShapes.importExtraData(context);	
	mSceneQueryData.importExtraData(context);
}
//~PX_SERIALIZATION

void NpShapeManager::attachShape(NpShape& shape, PxRigidActor& actor)
{
	PX_ASSERT(!mPruningStructure);

	PtrTableStorageManager& sm = NpFactory::getInstance().getPtrTableStorageManager();

	const PxU32 index = getNbShapes();
	mShapes.add(&shape, sm);	
	mSceneQueryData.add(reinterpret_cast<void*>(size_t(SQ_INVALID_PRUNER_DATA)), sm);

	NpScene* scene = NpActor::getAPIScene(actor);		
	if(scene && isSceneQuery(shape))
		setupSceneQuery(scene->getSceneQueryManagerFast(), actor, index);

	Scb::RigidObject& ro = static_cast<Scb::RigidObject&>(NpActor::getScbFromPxActor(actor));
	ro.onShapeAttach(shape.getScbShape());	

	PX_ASSERT(!shape.isExclusive() || shape.getActor()==NULL);
	shape.onActorAttach(actor);
}
				 
bool NpShapeManager::detachShape(NpShape& s, PxRigidActor& actor, bool wakeOnLostTouch)
{
	PX_ASSERT(!mPruningStructure);

	const PxU32 index = mShapes.find(&s);
	if(index==0xffffffff)
		return false;

	NpScene* scene = NpActor::getAPIScene(actor);
	if(scene && isSceneQuery(s))
	{
		scene->getSceneQueryManagerFast().removePrunerShape(mSqCompoundId, getPrunerData(index));
		// if this is the last shape of a compound shape, we have to remove the compound id 
		// and in case of a dynamic actor, remove it from the active list
		if(isSqCompound() && (mShapes.getCount() == 1))
		{
			mSqCompoundId = INVALID_PRUNERHANDLE;
			const PxType actorType = actor.getConcreteType();
			const bool isDynamic = actorType == PxConcreteType::eRIGID_DYNAMIC || actorType == PxConcreteType::eARTICULATION_LINK;
			if(isDynamic)
			{
				// for PxRigidDynamic and PxArticulationLink we need to remove the compound rigid flag and remove them from active list
				if(actor.is<PxRigidDynamic>())				
					const_cast<NpRigidDynamic&>(static_cast<const NpRigidDynamic&>(actor)).getScbBodyFast().getScBody().getSim()->disableCompound();
				else
				{
					if(actor.is<PxArticulationLink>())
						const_cast<NpArticulationLink&>(static_cast<const NpArticulationLink&>(actor)).getScbBodyFast().getScBody().getSim()->disableCompound();
				}
			}
		} 
	}

	Scb::RigidObject& ro = static_cast<Scb::RigidObject&>(NpActor::getScbFromPxActor(actor));
	ro.onShapeDetach(s.getScbShape(), wakeOnLostTouch, (s.getRefCount() == 1));
	PtrTableStorageManager& sm = NpFactory::getInstance().getPtrTableStorageManager();
	mShapes.replaceWithLast(index, sm);
	mSceneQueryData.replaceWithLast(index, sm);
	
	s.onActorDetach();
	return true;
}

void NpShapeManager::detachAll(NpScene* scene, const PxRigidActor& actor)
{
	// assumes all SQ data has been released, which is currently the responsbility of the owning actor
	const PxU32 nbShapes = getNbShapes();
	NpShape*const *shapes = getShapes();

	if(scene)
		teardownAllSceneQuery(scene->getSceneQueryManagerFast(), actor); 

	// actor cleanup in Scb/Sc will remove any outstanding references corresponding to sim objects, so we don't need to do that here.
	for(PxU32 i=0;i<nbShapes;i++)
		shapes[i]->onActorDetach();

	PtrTableStorageManager& sm = NpFactory::getInstance().getPtrTableStorageManager();

	mShapes.clear(sm);
	mSceneQueryData.clear(sm);
}

PxU32 NpShapeManager::getShapes(PxShape** buffer, PxU32 bufferSize, PxU32 startIndex) const
{
	return getArrayOfPointers(buffer, bufferSize, startIndex, getShapes(), getNbShapes());
}

PxBounds3 NpShapeManager::getWorldBounds(const PxRigidActor& actor) const
{
	PxBounds3 bounds(PxBounds3::empty());

	const PxU32 nbShapes = getNbShapes();
	const PxTransform actorPose = actor.getGlobalPose();
	NpShape*const* PX_RESTRICT shapes = getShapes();

	for(PxU32 i=0;i<nbShapes;i++)
		bounds.include(Gu::computeBounds(shapes[i]->getScbShape().getGeometry(), actorPose * shapes[i]->getLocalPoseFast()));
		
	return bounds;
}

void NpShapeManager::clearShapesOnRelease(Scb::Scene& s, PxRigidActor& r)
{
	PX_ASSERT(static_cast<Scb::RigidObject&>(NpActor::getScbFromPxActor(r)).isSimDisabledInternally());
	
	const PxU32 nbShapes = getNbShapes();
	NpShape*const* PX_RESTRICT shapes = getShapes();

	for(PxU32 i=0;i<nbShapes;i++)
	{
		Scb::Shape& scbShape = shapes[i]->getScbShape();
		scbShape.checkUpdateOnRemove<false>(&s);
#if PX_SUPPORT_PVD
		s.getScenePvdClient().releasePvdInstance(&scbShape, r);
#else
		PX_UNUSED(r);
#endif
	}
}

void NpShapeManager::releaseExclusiveUserReferences()
{
	// when the factory is torn down, release any shape owner refs that are still outstanding
	const PxU32 nbShapes = getNbShapes();
	NpShape*const* PX_RESTRICT shapes = getShapes();
	for(PxU32 i=0;i<nbShapes;i++)
	{
		if(shapes[i]->isExclusiveFast() && shapes[i]->getRefCount()>1)
			shapes[i]->release();
	}
}

void NpShapeManager::setupSceneQuery(SceneQueryManager& sqManager, const PxRigidActor& actor, const NpShape& shape)
{ 
	PX_ASSERT(shape.getFlags() & PxShapeFlag::eSCENE_QUERY_SHAPE);
	const PxU32 index = mShapes.find(&shape);
	PX_ASSERT(index!=0xffffffff);
	setupSceneQuery(sqManager, actor, index);
}

void NpShapeManager::teardownSceneQuery(SceneQueryManager& sqManager, const NpShape& shape)
{
	const PxU32 index = mShapes.find(&shape);
	PX_ASSERT(index!=0xffffffff);
	teardownSceneQuery(sqManager, index);
}

void NpShapeManager::setupAllSceneQuery(NpScene* scene, const PxRigidActor& actor, bool hasPrunerStructure, const PxBounds3* bounds, const Gu::BVHStructure* bvhStructure)
{ 
	PX_ASSERT(scene);		// shouldn't get here unless we're in a scene
	SceneQueryManager& sqManager = scene->getSceneQueryManagerFast();

	const PxU32 nbShapes = getNbShapes();
	NpShape*const *shapes = getShapes();

	// if BVH structure was provided, we add shapes into compound pruner
	if(bvhStructure)
	{
		addBVHStructureShapes(sqManager, actor, bvhStructure);
	}
	else
	{
		const PxType actorType = actor.getConcreteType();
		const bool isDynamic = actorType == PxConcreteType::eRIGID_DYNAMIC || actorType == PxConcreteType::eARTICULATION_LINK;
		
		for(PxU32 i=0;i<nbShapes;i++)
		{
			if(isSceneQuery(*shapes[i]))
				addPrunerShape(sqManager, i, *shapes[i], actor, isDynamic, bounds ? bounds + i : NULL, hasPrunerStructure);
		}
	}
}

void NpShapeManager::teardownAllSceneQuery(SceneQueryManager& sqManager, const PxRigidActor& actor)
{
	NpShape*const *shapes = getShapes();
	const PxU32 nbShapes = getNbShapes();

	if(isSqCompound())
	{
		const PxType actorType = actor.getConcreteType();
		const bool isDynamic = actorType == PxConcreteType::eRIGID_DYNAMIC || actorType == PxConcreteType::eARTICULATION_LINK;
		sqManager.removeCompoundActor(mSqCompoundId, isDynamic);
		for(PxU32 i=0;i<nbShapes;i++)
		{
			setPrunerData(i, SQ_INVALID_PRUNER_DATA);
		}
		mSqCompoundId = INVALID_PRUNERHANDLE;
		return;
	}

	for(PxU32 i=0;i<nbShapes;i++)
	{
		if(isSceneQuery(*shapes[i]))
			sqManager.removePrunerShape(INVALID_PRUNERHANDLE, getPrunerData(i));

		setPrunerData(i, SQ_INVALID_PRUNER_DATA);
	}
}

void NpShapeManager::markAllSceneQueryForUpdate(SceneQueryManager& sqManager, const PxRigidActor& actor)
{
	if(isSqCompound())
	{		
		const PxType actorType = actor.getConcreteType();
		const bool isDynamic = actorType == PxConcreteType::eRIGID_DYNAMIC || actorType == PxConcreteType::eARTICULATION_LINK;
		sqManager.updateCompoundActor(mSqCompoundId, actor.getGlobalPose(), isDynamic);
		return;
	}

	const PxU32 nbShapes = getNbShapes();

	for(PxU32 i=0;i<nbShapes;i++)
	{
		const PrunerData data = getPrunerData(i);
		if(data!=SQ_INVALID_PRUNER_DATA)
			sqManager.markForUpdate(INVALID_PRUNERHANDLE, data);
	}
}

Sq::PrunerData NpShapeManager::findSceneQueryData(const NpShape& shape) const
{
	const PxU32 index = mShapes.find(&shape);
	PX_ASSERT(index!=0xffffffff);
	PX_ASSERT(!isSqCompound()); // used in cases we know it is not a compound

	return getPrunerData(index);
}


Sq::PrunerData NpShapeManager::findSceneQueryData(const NpShape& shape, Sq::PrunerCompoundId& compoundId) const
{
	const PxU32 index = mShapes.find(&shape);
	PX_ASSERT(index!=0xffffffff);

	compoundId = mSqCompoundId;
	return getPrunerData(index);
}

//
// internal methods
// 

void NpShapeManager::addBVHStructureShapes(SceneQueryManager& sqManager, const PxRigidActor& actor, const Gu::BVHStructure* bvhStructure)
{
	PX_ASSERT(bvhStructure);

	const Scb::Actor& scbActor = NpActor::getScbFromPxActor(actor);
	const PxU32 nbShapes = getNbShapes();

	PX_ALLOCA(scbShapes, const Scb::Shape*, nbShapes);
	PX_ALLOCA(prunerData, Sq::PrunerData, nbShapes);

	PxU32 numSqShapes = 0;
	for(PxU32 i = 0; i < nbShapes; i++)
	{
		const NpShape& shape = *getShapes()[i];
		if(isSceneQuery(shape))
			scbShapes[numSqShapes++] = &shape.getScbShape();
	}
	PX_ASSERT(numSqShapes == bvhStructure->getNbBounds());

	mSqCompoundId = static_cast<const Sc::RigidCore&>(NpActor::getScbFromPxActor(actor).getActorCore()).getRigidID();
	sqManager.addCompoundShape(*bvhStructure, mSqCompoundId, actor.getGlobalPose(), prunerData, scbShapes, scbActor);

	numSqShapes = 0;
	for(PxU32 i = 0; i < nbShapes; i++)
	{
		const NpShape& shape = *getShapes()[i];
		if(isSceneQuery(shape))
			setPrunerData(i, prunerData[numSqShapes++]);
	}
}

void NpShapeManager::addPrunerShape(SceneQueryManager& sqManager, PxU32 index, const NpShape& shape, const PxRigidActor& actor, bool dynamic, const PxBounds3* bound, bool hasPrunerStructure)
{
	const Scb::Shape& scbShape = shape.getScbShape();
	const Scb::Actor& scbActor = NpActor::getScbFromPxActor(actor);
	setPrunerData(index, sqManager.addPrunerShape(scbShape, scbActor, dynamic, mSqCompoundId, bound, hasPrunerStructure));
}

void NpShapeManager::setupSceneQuery(SceneQueryManager& sqManager, const PxRigidActor& actor, PxU32 index)
{ 
	const PxType actorType = actor.getConcreteType();
	const bool isDynamic = actorType == PxConcreteType::eRIGID_DYNAMIC || actorType == PxConcreteType::eARTICULATION_LINK;
	addPrunerShape(sqManager, index, *(getShapes()[index]), actor, isDynamic, NULL, false);
}

void NpShapeManager::teardownSceneQuery(SceneQueryManager& sqManager, PxU32 index)
{
	sqManager.removePrunerShape(mSqCompoundId, getPrunerData(index));
	setPrunerData(index, SQ_INVALID_PRUNER_DATA);	
}

#if PX_ENABLE_DEBUG_VISUALIZATION
#include "GuHeightFieldUtil.h"
#include "PxGeometryQuery.h"
#include "PxMeshQuery.h"
#include "GuConvexEdgeFlags.h"
#include "GuMidphaseInterface.h"

static const PxU32 gCollisionShapeColor = PxU32(PxDebugColor::eARGB_MAGENTA);

static void visualizeSphere(const PxSphereGeometry& geometry, RenderOutput& out, const PxTransform& absPose)
{
	out << gCollisionShapeColor;	// PT: no need to output this for each segment!

	out << absPose << DebugCircle(100, geometry.radius);

	PxMat44 rotPose(absPose);
	Ps::swap(rotPose.column1, rotPose.column2);
	rotPose.column1 = -rotPose.column1;
	out << rotPose << DebugCircle(100, geometry.radius);

	Ps::swap(rotPose.column0, rotPose.column2);
	rotPose.column0 = -rotPose.column0;
	out << rotPose << DebugCircle(100, geometry.radius);
}

static void visualizePlane(const PxPlaneGeometry& /*geometry*/, RenderOutput& out, const PxTransform& absPose)
{
	PxMat44 rotPose(absPose);
	Ps::swap(rotPose.column1, rotPose.column2);
	rotPose.column1 = -rotPose.column1;

	Ps::swap(rotPose.column0, rotPose.column2);
	rotPose.column0 = -rotPose.column0;

	out << rotPose << gCollisionShapeColor;	// PT: no need to output this for each segment!
	for(PxReal radius = 2.0f; radius < 20.0f ; radius += 2.0f)
		out << DebugCircle(100, radius*radius);
}

static void visualizeCapsule(const PxCapsuleGeometry& geometry, RenderOutput& out, const PxTransform& absPose)
{
	out << gCollisionShapeColor;
	out.outputCapsule(geometry.radius, geometry.halfHeight, absPose);
}

static void visualizeBox(const PxBoxGeometry& geometry, RenderOutput& out, const PxTransform& absPose)
{
	out << gCollisionShapeColor;
	out << absPose << DebugBox(geometry.halfExtents);
}

static void visualizeConvexMesh(const PxConvexMeshGeometry& geometry, RenderOutput& out, const PxTransform& absPose)
{
	const ConvexMesh* convexMesh = static_cast<const ConvexMesh*>(geometry.convexMesh);
	const ConvexHullData& hullData = convexMesh->getHull();

	const PxVec3* vertices = hullData.getHullVertices();
	const PxU8* indexBuffer = hullData.getVertexData8();
	const PxU32 nbPolygons = convexMesh->getNbPolygonsFast();

	const PxMat44 m44(PxMat33(absPose.q) * geometry.scale.toMat33(), absPose.p);

	out << m44 << gCollisionShapeColor;	// PT: no need to output this for each segment!

	for(PxU32 i=0; i<nbPolygons; i++)
	{
		const PxU32 pnbVertices = hullData.mPolygons[i].mNbVerts;

		PxVec3 begin = m44.transform(vertices[indexBuffer[0]]);	// PT: transform it only once before the loop starts
		for(PxU32 j=1; j<pnbVertices; j++)
		{
			const PxVec3 end = m44.transform(vertices[indexBuffer[j]]);
			out.outputSegment(begin, end);
			begin = end;
		}
		out.outputSegment(begin, m44.transform(vertices[indexBuffer[0]]));

		indexBuffer += pnbVertices;
	}
}

static void getTriangle(const Gu::TriangleMesh&, PxU32 i, PxVec3* wp, const PxVec3* vertices, const void* indices, bool has16BitIndices)
{
	PxU32 ref0, ref1, ref2;

	if(!has16BitIndices)
	{
		const PxU32* dtriangles = reinterpret_cast<const PxU32*>(indices);
		ref0 = dtriangles[i*3+0];
		ref1 = dtriangles[i*3+1];
		ref2 = dtriangles[i*3+2];
	}
	else
	{
		const PxU16* wtriangles = reinterpret_cast<const PxU16*>(indices);
		ref0 = wtriangles[i*3+0];
		ref1 = wtriangles[i*3+1];
		ref2 = wtriangles[i*3+2];
	}

	wp[0] = vertices[ref0];
	wp[1] = vertices[ref1];
	wp[2] = vertices[ref2];
}

static void getTriangle(const Gu::TriangleMesh& mesh, PxU32 i, PxVec3* wp, const PxVec3* vertices, const void* indices, const Matrix34& absPose, bool has16BitIndices)
{
	PxVec3 localVerts[3];
	getTriangle(mesh, i, localVerts, vertices, indices, has16BitIndices);

	wp[0] = absPose.transform(localVerts[0]);
	wp[1] = absPose.transform(localVerts[1]);
	wp[2] = absPose.transform(localVerts[2]);
}

static void visualizeActiveEdges(RenderOutput& out, const Gu::TriangleMesh& mesh, PxU32 nbTriangles, const PxU32* results, const Matrix34& absPose)
{
	const PxU8* extraTrigData = mesh.getExtraTrigData();
	PX_ASSERT(extraTrigData);

	const PxVec3* vertices = mesh.getVerticesFast();
	const void* indices = mesh.getTrianglesFast();

	out << PxU32(PxDebugColor::eARGB_YELLOW);	// PT: no need to output this for each segment!

	const bool has16Bit = mesh.has16BitIndices();
	for(PxU32 i=0; i<nbTriangles; i++)
	{
		const PxU32 index = results ? results[i] : i;

		PxVec3 wp[3];
		getTriangle(mesh, index, wp, vertices, indices, absPose, has16Bit);

		const PxU32 flags = extraTrigData[index];

		if(flags & Gu::ETD_CONVEX_EDGE_01)
			out.outputSegment(wp[0], wp[1]);

		if(flags & Gu::ETD_CONVEX_EDGE_12)
			out.outputSegment(wp[1], wp[2]);

		if(flags & Gu::ETD_CONVEX_EDGE_20)
			out.outputSegment(wp[0], wp[2]);
	}
}

static void visualizeFaceNormals(	PxReal fscale, RenderOutput& out, const TriangleMesh& mesh, PxU32 nbTriangles, const PxVec3* vertices,
									const void* indices, bool has16Bit, const PxU32* results, const Matrix34& absPose, const PxMat44& midt)
{
	if(fscale==0.0f)
		return;

	out << midt << PxU32(PxDebugColor::eARGB_DARKRED);	// PT: no need to output this for each segment!

	for(PxU32 i=0; i<nbTriangles; i++)
	{
		const PxU32 index = results ? results[i] : i;
		PxVec3 wp[3];
		getTriangle(mesh, index, wp, vertices, indices, absPose, has16Bit);

		const PxVec3 center = (wp[0] + wp[1] + wp[2]) / 3.0f;
		PxVec3 normal = (wp[0] - wp[1]).cross(wp[0] - wp[2]);
		PX_ASSERT(!normal.isZero());
		normal = normal.getNormalized();

		out << DebugArrow(center, normal * fscale);
	}
}

static PX_FORCE_INLINE void outputTriangle(PxDebugLine* segments, const PxVec3& v0, const PxVec3& v1, const PxVec3& v2, PxU32 color)
{
	// PT: TODO: use SIMD
	segments[0] = PxDebugLine(v0, v1, color);
	segments[1] = PxDebugLine(v1, v2, color);
	segments[2] = PxDebugLine(v2, v0, color);
}

static void visualizeTriangleMesh(const PxTriangleMeshGeometry& geometry, RenderOutput& out, const PxTransform& pose, const PxBounds3& cullbox, const PxReal fscale, bool visualizeShapes, bool visualizeEdges, bool useCullBox)
{
	const TriangleMesh* triangleMesh = static_cast<const TriangleMesh*>(geometry.triangleMesh);
	
	const PxMat44 midt(PxIdentity);
	const Matrix34 absPose(PxMat33(pose.q) * geometry.scale.toMat33(), pose.p);

	PxU32 nbTriangles = triangleMesh->getNbTrianglesFast();
	const PxU32 nbVertices = triangleMesh->getNbVerticesFast();
	const PxVec3* vertices = triangleMesh->getVerticesFast();
	const void* indices = triangleMesh->getTrianglesFast();
	const bool has16Bit = triangleMesh->has16BitIndices();

	// PT: TODO: don't render the same edge multiple times

	PxU32* results = NULL;
	if(useCullBox)
	{
		const Gu::Box worldBox(
			(cullbox.maximum + cullbox.minimum)*0.5f,
			(cullbox.maximum - cullbox.minimum)*0.5f,
			PxMat33(PxIdentity));
		
		// PT: TODO: use the callback version here to avoid allocating this huge array
		results = reinterpret_cast<PxU32*>(PX_ALLOC_TEMP(sizeof(PxU32)*nbTriangles, "tmp triangle indices"));
		LimitedResults limitedResults(results, nbTriangles, 0);
		Midphase::intersectBoxVsMesh(worldBox, *triangleMesh, pose, geometry.scale, &limitedResults);
		nbTriangles = limitedResults.mNbResults;

		if(visualizeShapes)
		{
			const PxU32 scolor = gCollisionShapeColor;

			out << midt << scolor;	// PT: no need to output this for each segment!

			PxDebugLine* segments = out.reserveSegments(nbTriangles*3);
			for(PxU32 i=0; i<nbTriangles; i++)
			{
				PxVec3 wp[3];
				getTriangle(*triangleMesh, results[i], wp, vertices, indices, absPose, has16Bit);
				outputTriangle(segments, wp[0], wp[1], wp[2], scolor);
				segments+=3;
			}
		}
	}
	else
	{
		if(visualizeShapes)
		{
			const PxU32 scolor = gCollisionShapeColor;

			out << midt << scolor;	// PT: no need to output this for each segment!

			// PT: TODO: use SIMD
			PxVec3* transformed = reinterpret_cast<PxVec3*>(PX_ALLOC(sizeof(PxVec3)*nbVertices, "PxVec3"));
			for(PxU32 i=0;i<nbVertices;i++)
				transformed[i] = absPose.transform(vertices[i]);

			PxDebugLine* segments = out.reserveSegments(nbTriangles*3);
			for(PxU32 i=0; i<nbTriangles; i++)
			{
				PxVec3 wp[3];
				getTriangle(*triangleMesh, i, wp, transformed, indices, has16Bit);
				outputTriangle(segments, wp[0], wp[1], wp[2], scolor);
				segments+=3;
			}

			PX_FREE(transformed);
		}
	}

	visualizeFaceNormals(fscale, out, *triangleMesh, nbTriangles, vertices, indices, has16Bit, results, absPose, midt);

	if(visualizeEdges && triangleMesh->getExtraTrigData())
		visualizeActiveEdges(out, *triangleMesh, nbTriangles, results, absPose);

	if(results)
		PX_FREE(results);
}

static void visualizeHeightField(const PxHeightFieldGeometry& hfGeometry, RenderOutput& out, const PxTransform& absPose, const PxBounds3& cullbox, bool useCullBox)
{
	const HeightField* heightfield = static_cast<const HeightField*>(hfGeometry.heightField);

	// PT: TODO: the debug viz for HFs is minimal at the moment...
	const PxU32 scolor = gCollisionShapeColor;
	const PxMat44 midt = PxMat44(PxIdentity);

	HeightFieldUtil hfUtil(hfGeometry);

	const PxU32 nbRows = heightfield->getNbRowsFast();
	const PxU32 nbColumns = heightfield->getNbColumnsFast();
	const PxU32 nbVerts = nbRows * nbColumns;
	const PxU32 nbTriangles = 2 * nbVerts;

	out << midt << scolor;	// PT: no need to output the same matrix/color for each triangle

	if(useCullBox)
	{
		const PxTransform pose0((cullbox.maximum + cullbox.minimum)*0.5f);
		const PxBoxGeometry boxGeometry((cullbox.maximum - cullbox.minimum)*0.5f);

		PxU32* results = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32)*nbTriangles, "tmp triangle indices"));

		bool overflow = false;
		PxU32 nbTouchedTris = PxMeshQuery::findOverlapHeightField(boxGeometry, pose0, hfGeometry, absPose, results, nbTriangles, 0, overflow);
		
		PxDebugLine* segments = out.reserveSegments(nbTouchedTris*3);

		for(PxU32 i=0; i<nbTouchedTris; i++)
		{
			const PxU32 index = results[i];
			PxTriangle currentTriangle;
			PxMeshQuery::getTriangle(hfGeometry, absPose, index, currentTriangle);

			//The check has been done in the findOverlapHeightField
			//if(heightfield->isValidTriangle(index) && heightfield->getTriangleMaterial(index) != PxHeightFieldMaterial::eHOLE)
			{
				outputTriangle(segments, currentTriangle.verts[0], currentTriangle.verts[1], currentTriangle.verts[2], scolor);
				segments+=3;
			}
		}
		PX_FREE(results);
	}
	else
	{
		// PT: transform vertices only once
		PxVec3* tmpVerts = reinterpret_cast<PxVec3*>(PX_ALLOC(sizeof(PxVec3)*nbVerts, "PxVec3"));
		// PT: TODO: optimize the following line
		for(PxU32 i=0;i<nbVerts;i++)
			tmpVerts[i] = absPose.transform(hfUtil.hf2shapep(heightfield->getVertex(i)));

		for(PxU32 i=0; i<nbTriangles; i++)
		{
			if(heightfield->isValidTriangle(i) && heightfield->getTriangleMaterial(i) != PxHeightFieldMaterial::eHOLE)
			{
				PxU32 vi0, vi1, vi2;
				heightfield->getTriangleVertexIndices(i, vi0, vi1, vi2);

				PxDebugLine* segments = out.reserveSegments(3);
				outputTriangle(segments, tmpVerts[vi0], tmpVerts[vi1], tmpVerts[vi2], scolor);
			}
		}
		PX_FREE(tmpVerts);
	}
}

static void visualize(const PxGeometry& geometry, RenderOutput& out, const PxTransform& absPose, const PxBounds3& cullbox, const PxReal fscale, bool visualizeShapes, bool visualizeEdges, bool useCullBox)
{
	// triangle meshes can render active edges or face normals, but for other types we can just early out if there are no collision shapes
	if(!visualizeShapes && geometry.getType() != PxGeometryType::eTRIANGLEMESH)
		return;

	switch(geometry.getType())
	{
	case PxGeometryType::eSPHERE:
		visualizeSphere(static_cast<const PxSphereGeometry&>(geometry), out, absPose);
		break;
	case PxGeometryType::eBOX:
		visualizeBox(static_cast<const PxBoxGeometry&>(geometry), out, absPose);
		break;
	case PxGeometryType::ePLANE:
		visualizePlane(static_cast<const PxPlaneGeometry&>(geometry), out, absPose);
		break;
	case PxGeometryType::eCAPSULE:
		visualizeCapsule(static_cast<const PxCapsuleGeometry&>(geometry), out, absPose);
		break;
	case PxGeometryType::eCONVEXMESH:
		visualizeConvexMesh(static_cast<const PxConvexMeshGeometry&>(geometry), out, absPose);
		break;
	case PxGeometryType::eTRIANGLEMESH:
		visualizeTriangleMesh(static_cast<const PxTriangleMeshGeometry&>(geometry), out, absPose, cullbox, fscale, visualizeShapes, visualizeEdges, useCullBox);
		break;
	case PxGeometryType::eHEIGHTFIELD:
		visualizeHeightField(static_cast<const PxHeightFieldGeometry&>(geometry), out, absPose, cullbox, useCullBox);
		break;
	case PxGeometryType::eINVALID:
		break;
	case PxGeometryType::eGEOMETRY_COUNT:
		break;
	}
}

void NpShapeManager::visualize(RenderOutput& out, NpScene* scene, const PxRigidActor& actor)
{
	const PxReal scale = scene->getVisualizationParameter(PxVisualizationParameter::eSCALE);
	if(!scale)
		return;

	const PxU32 nbShapes = getNbShapes();
	NpShape*const* PX_RESTRICT shapes = getShapes();

	const bool visualizeCompounds = (nbShapes>1) && scene->getVisualizationParameter(PxVisualizationParameter::eCOLLISION_COMPOUNDS)!=0.0f;

	// PT: moved all these out of the loop, no need to grab them once per shape
	const PxBounds3& cullbox		= scene->getScene().getVisualizationCullingBox();
	const bool visualizeAABBs		= scene->getVisualizationParameter(PxVisualizationParameter::eCOLLISION_AABBS)!=0.0f;
	const bool visualizeShapes		= scene->getVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES)!=0.0f;
	const bool visualizeEdges		= scene->getVisualizationParameter(PxVisualizationParameter::eCOLLISION_EDGES)!=0.0f;
	const float fNormals			= scene->getVisualizationParameter(PxVisualizationParameter::eCOLLISION_FNORMALS);
	const bool visualizeFNormals	= fNormals!=0.0f;
	const bool visualizeCollision	= visualizeShapes || visualizeFNormals || visualizeEdges;
	const bool useCullBox			= !cullbox.isEmpty();
	const bool needsShapeBounds0	= visualizeCompounds || (visualizeCollision && useCullBox);
	const PxReal collisionAxes		= scale * scene->getVisualizationParameter(PxVisualizationParameter::eCOLLISION_AXES);
	const PxReal fscale				= scale * fNormals;

	const PxTransform actorPose = actor.getGlobalPose();

	PxBounds3 compoundBounds(PxBounds3::empty());
	for(PxU32 i=0;i<nbShapes;i++)
	{
		const Scb::Shape& scbShape = shapes[i]->getScbShape();

		const PxTransform absPose = actorPose * scbShape.getShape2Actor();
		const PxGeometry& geom = scbShape.getGeometry();

		const bool shapeDebugVizEnabled = scbShape.getFlags() & PxShapeFlag::eVISUALIZATION;

		const bool needsShapeBounds = needsShapeBounds0 || (visualizeAABBs && shapeDebugVizEnabled);
		const PxBounds3 currentShapeBounds = needsShapeBounds ? Gu::computeBounds(geom, absPose) : PxBounds3::empty();

		if(shapeDebugVizEnabled)
		{
			if(visualizeAABBs)
				out << PxU32(PxDebugColor::eARGB_YELLOW) << PxMat44(PxIdentity) << DebugBox(currentShapeBounds);

			if(collisionAxes != 0.0f)
				out << PxMat44(absPose) << DebugBasis(PxVec3(collisionAxes), 0xcf0000, 0x00cf00, 0x0000cf);

			if(visualizeCollision)
			{
				if(!useCullBox || cullbox.intersects(currentShapeBounds))
					::visualize(geom, out, absPose, cullbox, fscale, visualizeShapes, visualizeEdges, useCullBox);
			}
		}

		if(visualizeCompounds)
			compoundBounds.include(currentShapeBounds);
	}
	if(visualizeCompounds && !compoundBounds.isEmpty())
		out << gCollisionShapeColor << PxMat44(PxIdentity) << DebugBox(compoundBounds);
}
#endif  // PX_ENABLE_DEBUG_VISUALIZATION
