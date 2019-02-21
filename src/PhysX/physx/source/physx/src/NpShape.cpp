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

#include "NpCast.h"
#include "GuConvexMesh.h"
#include "GuTriangleMesh.h"
#include "ScbNpDeps.h"
#include "GuBounds.h"

using namespace physx;
using namespace Sq;

static PX_FORCE_INLINE void updatePvdProperties(const Scb::Shape& shape)
{
#if PX_SUPPORT_PVD
	Scb::Scene* scbScene = shape.getScbSceneForAPI();
	if(scbScene)
		scbScene->getScenePvdClient().updatePvdProperties(&shape);
#else
	PX_UNUSED(shape);
#endif
}

NpShape::NpShape(const PxGeometry& geometry, PxShapeFlags shapeFlags, const PxU16* materialIndices, PxU16 materialCount, bool isExclusive)
:	PxShape				(PxConcreteType::eSHAPE, PxBaseFlag::eOWNS_MEMORY | PxBaseFlag::eIS_RELEASABLE)
,	mActor				(NULL)
,	mShape				(geometry, shapeFlags, materialIndices, materialCount, isExclusive)
,	mName				(NULL)
,	mExclusiveAndActorCount (isExclusive ? EXCLUSIVE_MASK : 0)
{
	PX_ASSERT(mShape.getScShape().getPxShape() == static_cast<PxShape*>(this));

	PxShape::userData = NULL;

	incMeshRefCount();
}

NpShape::~NpShape()
{
	decMeshRefCount();

	PxU32 nbMaterials = mShape.getNbMaterials();
	for (PxU32 i=0; i < nbMaterials; i++)
	{
		NpMaterial* mat = static_cast<NpMaterial*>(mShape.getMaterial(i));
		mat->decRefCount();
	}
}

void NpShape::onRefCountZero()
{
	NpFactory::getInstance().onShapeRelease(this);
	// see NpShape.h for ref counting semantics for shapes
	NpDestroy(getScbShape());
}

// PX_SERIALIZATION

NpShape::NpShape(PxBaseFlags baseFlags) : PxShape(baseFlags), mShape(PxEmpty) 
{
	mExclusiveAndActorCount &= EXCLUSIVE_MASK;
}

void NpShape::exportExtraData(PxSerializationContext& stream)
{	
	getScbShape().getScShape().exportExtraData(stream);
	stream.writeName(mName);
}

void NpShape::importExtraData(PxDeserializationContext& context)
{
	getScbShape().getScShape().importExtraData(context);
	context.readName(mName);
}

void NpShape::requiresObjects(PxProcessPxBaseCallback& c)
{
	//meshes
	PxBase* mesh = NULL;
	switch(mShape.getGeometryType())
	{
	case PxGeometryType::eCONVEXMESH:
		mesh = static_cast<const PxConvexMeshGeometry&>(mShape.getGeometry()).convexMesh;
		break;
	case PxGeometryType::eHEIGHTFIELD:
		mesh = static_cast<const PxHeightFieldGeometry&>(mShape.getGeometry()).heightField;
		break;
	case PxGeometryType::eTRIANGLEMESH:
		mesh = static_cast<const PxTriangleMeshGeometry&>(mShape.getGeometry()).triangleMesh;
		break;
	case PxGeometryType::eSPHERE:
	case PxGeometryType::ePLANE:
	case PxGeometryType::eCAPSULE:
	case PxGeometryType::eBOX:
	case PxGeometryType::eGEOMETRY_COUNT:
	case PxGeometryType::eINVALID:
		break;
	}
	
	if(mesh)
		c.process(*mesh);

	//material
	PxU32 nbMaterials = mShape.getNbMaterials();
	for (PxU32 i=0; i < nbMaterials; i++)
	{
		NpMaterial* mat = static_cast<NpMaterial*>(mShape.getMaterial(i));
		c.process(*mat);
	}
}

void NpShape::resolveReferences(PxDeserializationContext& context)
{	
	// getMaterials() only works after material indices have been patched. 
	// in order to get to the new material indices, we need access to the new materials.
	// this only leaves us with the option of acquiring the material through the context given an old material index (we do have the mapping)
	{
		PxU32 nbIndices = mShape.getScShape().getNbMaterialIndices();
		const PxU16* indices = mShape.getScShape().getMaterialIndices();

		for (PxU32 i=0; i < nbIndices; i++)
		{
			PxBase* base = context.resolveReference(PX_SERIAL_REF_KIND_MATERIAL_IDX, size_t(indices[i]));
			PX_ASSERT(base && base->is<PxMaterial>());

			NpMaterial& material = *static_cast<NpMaterial*>(base);
			getScbShape().getScShape().resolveMaterialReference(i, PxU16(material.getHandle()));
		}
	}

	context.translatePxBase(mActor);

	getScbShape().getScShape().resolveReferences(context);	


	incMeshRefCount();

	// Increment materials' refcounts in a second pass. Works better in case of failure above.
	PxU32 nbMaterials = mShape.getNbMaterials();
	for (PxU32 i=0; i < nbMaterials; i++)
	{
		NpMaterial* mat = static_cast<NpMaterial*>(mShape.getMaterial(i));
		mat->incRefCount();
	}	
}

NpShape* NpShape::createObject(PxU8*& address, PxDeserializationContext& context)
{
	NpShape* obj = new (address) NpShape(PxBaseFlag::eIS_RELEASABLE);
	address += sizeof(NpShape);	
	obj->importExtraData(context);
	obj->resolveReferences(context);
	return obj;
}
//~PX_SERIALIZATION

PxU32 NpShape::getReferenceCount() const
{
	return getRefCount();
}

void NpShape::acquireReference()
{
	incRefCount();
}

void NpShape::release()
{
	PX_CHECK_AND_RETURN(getRefCount() > 1 || getActorCount() == 0, "PxShape::release: last reference to a shape released while still attached to an actor!");
	NP_WRITE_CHECK(getOwnerScene());
	releaseInternal();
}

void NpShape::releaseInternal()
{
	decRefCount();
}

Sc::RigidCore& NpShape::getScRigidObjectExclusive() const
{
	const PxType actorType = mActor->getConcreteType();

	if (actorType == PxConcreteType::eRIGID_DYNAMIC)
		return static_cast<NpRigidDynamic&>(*mActor).getScbBodyFast().getScBody();
	else if (actorType == PxConcreteType::eARTICULATION_LINK)
		return static_cast<NpArticulationLink&>(*mActor).getScbBodyFast().getScBody();
	else
		return static_cast<NpRigidStatic&>(*mActor).getScbRigidStaticFast().getScStatic();
}

void NpShape::updateSQ(const char* errorMessage)
{
	if(mActor && (mShape.getFlags() & PxShapeFlag::eSCENE_QUERY_SHAPE))
	{
		NpScene* scene = NpActor::getAPIScene(*mActor);
		NpShapeManager* shapeManager = NpActor::getShapeManager(*mActor);
		if(scene)
		{
			PxU32 compoundId;
			const PrunerData sqData = shapeManager->findSceneQueryData(*this, compoundId);
			scene->getSceneQueryManagerFast().markForUpdate(compoundId, sqData);
		}

		// invalidate the pruning structure if the actor bounds changed
		if(shapeManager->getPruningStructure())
		{
			Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, errorMessage);
			shapeManager->getPruningStructure()->invalidate(mActor);
		}
	}
}

PxGeometryType::Enum NpShape::getGeometryType() const
{
	NP_READ_CHECK(getOwnerScene());

	return mShape.getGeometryType();
}

void NpShape::setGeometry(const PxGeometry& g)
{
	NP_WRITE_CHECK(getOwnerScene());
	PX_CHECK_AND_RETURN(isWritable(), "PxShape::setGeometry: shared shapes attached to actors are not writable.");
	PX_SIMD_GUARD;

	// PT: fixes US2117
	if(g.getType() != getGeometryTypeFast())
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "PxShape::setGeometry(): Invalid geometry type. Changing the type of the shape is not supported.");
		return;
	}

#if PX_CHECKED
	bool isValid = false;
	switch(g.getType())
	{
		case PxGeometryType::eSPHERE:
			isValid = static_cast<const PxSphereGeometry&>(g).isValid();
		break;

		case PxGeometryType::ePLANE:
			isValid = static_cast<const PxPlaneGeometry&>(g).isValid();
		break;

		case PxGeometryType::eCAPSULE:
			isValid = static_cast<const PxCapsuleGeometry&>(g).isValid();
		break;

		case PxGeometryType::eBOX:
			isValid = static_cast<const PxBoxGeometry&>(g).isValid();
		break;

		case PxGeometryType::eCONVEXMESH:
			isValid = static_cast<const PxConvexMeshGeometry&>(g).isValid();
		break;

		case PxGeometryType::eTRIANGLEMESH:
			isValid = static_cast<const PxTriangleMeshGeometry&>(g).isValid();
		break;

		case PxGeometryType::eHEIGHTFIELD:
			isValid = static_cast<const PxHeightFieldGeometry&>(g).isValid();
		break;
		
		case PxGeometryType::eGEOMETRY_COUNT:
		case PxGeometryType::eINVALID:
			break;
	}

	if(!isValid)
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "PxShape::setGeometry(): Invalid geometry!");
		return;
	}
#endif

	decMeshRefCount();

	mShape.setGeometry(g);

	incMeshRefCount();

	updateSQ("PxShape::setGeometry: Shape is a part of pruning structure, pruning structure is now invalid!");
}

PxGeometryHolder NpShape::getGeometry() const
{
	PX_COMPILE_TIME_ASSERT(sizeof(Gu::GeometryUnion)>=sizeof(PxGeometryHolder));
	return reinterpret_cast<const PxGeometryHolder&>(mShape.getGeometry());
}

template<class T>
static PX_FORCE_INLINE bool getGeometryT(const NpShape* npShape, PxGeometryType::Enum type, T& geom)
{
	NP_READ_CHECK(npShape->getOwnerScene());

	if(npShape->getGeometryTypeFast() != type)
		return false;

	geom = static_cast<const T&>(npShape->getScbShape().getGeometry());
	return true;
}

bool NpShape::getBoxGeometry(PxBoxGeometry& g)							const	{ return getGeometryT(this, PxGeometryType::eBOX, g);			}
bool NpShape::getSphereGeometry(PxSphereGeometry& g)					const	{ return getGeometryT(this, PxGeometryType::eSPHERE, g);		}
bool NpShape::getCapsuleGeometry(PxCapsuleGeometry& g)					const	{ return getGeometryT(this, PxGeometryType::eCAPSULE, g);		}
bool NpShape::getPlaneGeometry(PxPlaneGeometry& g)						const	{ return getGeometryT(this, PxGeometryType::ePLANE, g);		}
bool NpShape::getConvexMeshGeometry(PxConvexMeshGeometry& g)			const	{ return getGeometryT(this, PxGeometryType::eCONVEXMESH, g);	}
bool NpShape::getTriangleMeshGeometry(PxTriangleMeshGeometry& g)		const	{ return getGeometryT(this, PxGeometryType::eTRIANGLEMESH, g);	}
bool NpShape::getHeightFieldGeometry(PxHeightFieldGeometry& g)			const	{ return getGeometryT(this, PxGeometryType::eHEIGHTFIELD, g);	}

PxRigidActor* NpShape::getActor() const
{
	NP_READ_CHECK(getOwnerScene());
	return mActor;
}

void NpShape::setLocalPose(const PxTransform& newShape2Actor)
{
	PX_CHECK_AND_RETURN(newShape2Actor.isSane(), "PxShape::setLocalPose: pose is not valid.");
	PX_CHECK_AND_RETURN(isWritable(), "PxShape::setLocalPose: shared shapes attached to actors are not writable.");
	NP_WRITE_CHECK(getOwnerScene());

	mShape.setShape2Actor(newShape2Actor.getNormalized());

	updateSQ("PxShape::setLocalPose: Shape is a part of pruning structure, pruning structure is now invalid!");
}

PxTransform NpShape::getLocalPose() const
{
	NP_READ_CHECK(getOwnerScene());

	return mShape.getShape2Actor();
}

///////////////////////////////////////////////////////////////////////////////

void NpShape::setSimulationFilterData(const PxFilterData& data)
{
	NP_WRITE_CHECK(getOwnerScene());
	PX_CHECK_AND_RETURN(isWritable(), "PxShape::setSimulationFilterData: shared shapes attached to actors are not writable.");
	mShape.setSimulationFilterData(data);
}

PxFilterData NpShape::getSimulationFilterData() const
{
	NP_READ_CHECK(getOwnerScene());
	return mShape.getSimulationFilterData();
}

void NpShape::setQueryFilterData(const PxFilterData& data)
{
	NP_WRITE_CHECK(getOwnerScene());
	PX_CHECK_AND_RETURN(isWritable(), "PxShape::setQueryFilterData: shared shapes attached to actors are not writable.");

	mShape.getScShape().setQueryFilterData(data);	// PT: this one doesn't need double-buffering

	updatePvdProperties(mShape);
}

PxFilterData NpShape::getQueryFilterData() const
{
	NP_READ_CHECK(getOwnerScene());

	return getQueryFilterDataFast();
}

///////////////////////////////////////////////////////////////////////////////

void NpShape::setMaterials(PxMaterial*const* materials, PxU16 materialCount)
{
	NP_WRITE_CHECK(getOwnerScene());
	PX_CHECK_AND_RETURN(isWritable(), "PxShape::setMaterials: shared shapes attached to actors are not writable.");

#if PX_CHECKED
	if (!NpShape::checkMaterialSetup(mShape.getGeometry(), "PxShape::setMaterials()", materials,  materialCount))
		return;
#endif

	PxU32 oldMaterialCount = mShape.getNbMaterials();
	PX_ALLOCA(oldMaterials, PxMaterial*, oldMaterialCount);
	PxU32 tmp = mShape.getMaterials(oldMaterials, oldMaterialCount);
	PX_ASSERT(tmp == oldMaterialCount);
	PX_UNUSED(tmp);

	if (mShape.setMaterials(materials, materialCount))
	{
		for(PxU32 i=0; i < materialCount; i++)
			static_cast<NpMaterial*>(materials[i])->incRefCount();

		for(PxU32 i=0; i < oldMaterialCount; i++)
			static_cast<NpMaterial*>(oldMaterials[i])->decRefCount();
	}
}

PxU16 NpShape::getNbMaterials() const
{
	NP_READ_CHECK(getOwnerScene());

	return mShape.getNbMaterials();
}

PxU32 NpShape::getMaterials(PxMaterial** userBuffer, PxU32 bufferSize, PxU32 startIndex) const
{
	NP_READ_CHECK(getOwnerScene());

	return mShape.getMaterials(userBuffer, bufferSize, startIndex);
}

PxMaterial* NpShape::getMaterialFromInternalFaceIndex(PxU32 faceIndex) const
{
	NP_READ_CHECK(getOwnerScene());

	bool isHf = (getGeometryType() == PxGeometryType::eHEIGHTFIELD);
	bool isMesh = (getGeometryType() == PxGeometryType::eTRIANGLEMESH);
	if( faceIndex == 0xFFFFffff && (isHf || isMesh) )
	{
		Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__,
			"PxShape::getMaterialFromInternalFaceIndex received 0xFFFFffff as input - returning NULL.");
		return NULL;
	}

	PxMaterialTableIndex hitMatTableId = 0;

	if(isHf)
	{
		PxHeightFieldGeometry hfGeom;
		getHeightFieldGeometry(hfGeom);

		hitMatTableId = hfGeom.heightField->getTriangleMaterialIndex(faceIndex);
	}
	else if(isMesh)
	{
		PxTriangleMeshGeometry triGeo;
		getTriangleMeshGeometry(triGeo);

		Gu::TriangleMesh* tm = static_cast<Gu::TriangleMesh*>(triGeo.triangleMesh);
		if(tm->hasPerTriangleMaterials())
			hitMatTableId = triGeo.triangleMesh->getTriangleMaterialIndex(faceIndex);
	}

	return getMaterial(hitMatTableId);
}

void NpShape::setContactOffset(PxReal contactOffset)
{
	NP_WRITE_CHECK(getOwnerScene());

	PX_CHECK_AND_RETURN(PxIsFinite(contactOffset), "PxShape::setContactOffset: invalid float");
	PX_CHECK_AND_RETURN((contactOffset >= 0.0f && contactOffset > mShape.getRestOffset()), "PxShape::setContactOffset: contactOffset should be positive, and greater than restOffset!");
	PX_CHECK_AND_RETURN(isWritable(), "PxShape::setContactOffset: shared shapes attached to actors are not writable.");

	mShape.setContactOffset(contactOffset);
}

PxReal NpShape::getContactOffset() const
{
	NP_READ_CHECK(getOwnerScene());

	return mShape.getContactOffset();
}

void NpShape::setRestOffset(PxReal restOffset)
{
	NP_WRITE_CHECK(getOwnerScene());
	PX_CHECK_AND_RETURN(PxIsFinite(restOffset), "PxShape::setRestOffset: invalid float");
	PX_CHECK_AND_RETURN((restOffset < mShape.getContactOffset()), "PxShape::setRestOffset: restOffset should be less than contactOffset!");
	PX_CHECK_AND_RETURN(isWritable(), "PxShape::setRestOffset: shared shapes attached to actors are not writable.");

	mShape.setRestOffset(restOffset);
}

PxReal NpShape::getRestOffset() const
{
	NP_READ_CHECK(getOwnerScene());

	return mShape.getRestOffset();
}

void NpShape::setTorsionalPatchRadius(PxReal radius)
{
	NP_WRITE_CHECK(getOwnerScene());
	PX_CHECK_AND_RETURN(PxIsFinite(radius), "PxShape::setTorsionalPatchRadius: invalid float");
	PX_CHECK_AND_RETURN((radius >= 0.f), "PxShape::setTorsionalPatchRadius: must be >= 0.f");

	mShape.setTorsionalPatchRadius(radius);
}

PxReal NpShape::getTorsionalPatchRadius() const
{
	NP_READ_CHECK(getOwnerScene());
	return mShape.getTorsionalPatchRadius();
}

void NpShape::setMinTorsionalPatchRadius(PxReal radius)
{
	NP_WRITE_CHECK(getOwnerScene());
	PX_CHECK_AND_RETURN(PxIsFinite(radius), "PxShape::setMinTorsionalPatchRadius: invalid float");
	PX_CHECK_AND_RETURN((radius >= 0.f), "PxShape::setMinTorsionalPatchRadius: must be >= 0.f");

	mShape.setMinTorsionalPatchRadius(radius);
}

PxReal NpShape::getMinTorsionalPatchRadius() const
{
	NP_READ_CHECK(getOwnerScene());
	return mShape.getMinTorsionalPatchRadius();
}

void NpShape::setFlagsInternal(PxShapeFlags inFlags)
{
	const bool hasMeshTypeGeom = mShape.getGeometryType() == PxGeometryType::eTRIANGLEMESH || mShape.getGeometryType() == PxGeometryType::eHEIGHTFIELD;

	if(hasMeshTypeGeom && (inFlags & PxShapeFlag::eTRIGGER_SHAPE))
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, 
			"PxShape::setFlag(s): triangle mesh and heightfield triggers are not supported!");
		return;
	}

	if((inFlags & PxShapeFlag::eSIMULATION_SHAPE) && (inFlags & PxShapeFlag::eTRIGGER_SHAPE))
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, 
			"PxShape::setFlag(s): shapes cannot simultaneously be trigger shapes and simulation shapes.");
		return;
	}

	const PxShapeFlags oldFlags = mShape.getFlags();

	const bool oldIsSimShape = oldFlags & PxShapeFlag::eSIMULATION_SHAPE;
	const bool isSimShape = inFlags & PxShapeFlag::eSIMULATION_SHAPE;

	if(mActor)
	{
		const PxType type = mActor->getConcreteType();

		// PT: US5732 - support kinematic meshes
		bool isKinematic = false;
		if(type==PxConcreteType::eRIGID_DYNAMIC)
		{
			PxRigidDynamic* rigidDynamic = static_cast<PxRigidDynamic*>(mActor);
			isKinematic = rigidDynamic->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC;
		}

		if((type != PxConcreteType::eRIGID_STATIC) && !isKinematic && isSimShape && !oldIsSimShape && (hasMeshTypeGeom || mShape.getGeometryType() == PxGeometryType::ePLANE))
		{
			Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, 
				"PxShape::setFlag(s): triangle mesh, heightfield and plane shapes can only be simulation shapes if part of a PxRigidStatic!");
			return;
		}
	}

	const bool oldHasSceneQuery = oldFlags & PxShapeFlag::eSCENE_QUERY_SHAPE;
	const bool hasSceneQuery = inFlags & PxShapeFlag::eSCENE_QUERY_SHAPE;

	mShape.setFlags(inFlags);

	if(oldHasSceneQuery != hasSceneQuery && mActor)
	{
		NpScene* npScene = getAPIScene();
		NpShapeManager* shapeManager = NpActor::getShapeManager(*mActor);
		if(npScene)
		{
			if(hasSceneQuery)
				shapeManager->setupSceneQuery(npScene->getSceneQueryManagerFast(), *mActor, *this);
			else
				shapeManager->teardownSceneQuery(npScene->getSceneQueryManagerFast(), *this);
		}

		// invalidate the pruning structure if the actor bounds changed
		if(shapeManager->getPruningStructure())
		{
			Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "PxShape::setFlag: Shape is a part of pruning structure, pruning structure is now invalid!");
			shapeManager->getPruningStructure()->invalidate(mActor);
		}
	}
}

void NpShape::setFlag(PxShapeFlag::Enum flag, bool value)
{
	NP_WRITE_CHECK(getOwnerScene());
	PX_CHECK_AND_RETURN(isWritable(), "PxShape::setFlag: shared shapes attached to actors are not writable.");
	PX_SIMD_GUARD;

	PxShapeFlags shapeFlags = mShape.getFlags();
	shapeFlags = value ? shapeFlags | flag : shapeFlags & ~flag;
	
	setFlagsInternal(shapeFlags);
}

void NpShape::setFlags(PxShapeFlags inFlags)
{
	NP_WRITE_CHECK(getOwnerScene());
	PX_CHECK_AND_RETURN(isWritable(), "PxShape::setFlags: shared shapes attached to actors are not writable.");
	PX_SIMD_GUARD;

	setFlagsInternal(inFlags);
}

PxShapeFlags NpShape::getFlags() const
{
	NP_READ_CHECK(getOwnerScene());
	return mShape.getFlags();
}

bool NpShape::isExclusive() const
{
	NP_READ_CHECK(getOwnerScene());
	return (mExclusiveAndActorCount & EXCLUSIVE_MASK) != 0;
}

void NpShape::onActorAttach(PxRigidActor& actor)
{
	incRefCount();
	if(isExclusiveFast())
		mActor = &actor;
	Ps::atomicIncrement(&mExclusiveAndActorCount);
}

void NpShape::onActorDetach()
{
	PX_ASSERT(getActorCount() > 0);
	Ps::atomicDecrement(&mExclusiveAndActorCount);
	if(isExclusiveFast())
		mActor = NULL;
	decRefCount();
}

void NpShape::setName(const char* debugName)		
{
	NP_WRITE_CHECK(getOwnerScene());
	PX_CHECK_AND_RETURN(isWritable(), "PxShape::setName: shared shapes attached to actors are not writable.");

	mName = debugName;

	updatePvdProperties(mShape);
}

const char* NpShape::getName() const
{
	NP_READ_CHECK(getOwnerScene());

	return mName;
}

NpScene* NpShape::getOwnerScene()	const 
{	
	return mActor ? NpActor::getOwnerScene(*mActor) : NULL; 
}

NpScene* NpShape::getAPIScene()	const 
{	
	// gets called when we update SQ structures due to a write - in which case there must be an actor 
	PX_ASSERT(mActor);
	return NpActor::getAPIScene(*mActor);
}

///////////////////////////////////////////////////////////////////////////////

namespace physx
{
Sc::RigidCore* NpShapeGetScRigidObjectFromScbSLOW(const Scb::Shape& scb)
{
	const NpShape* np = getNpShape(&scb);
	return np->NpShape::getActor() ? &np->getScRigidObjectExclusive() : NULL;
}

size_t NpShapeGetScPtrOffset()
{
	const size_t offset = size_t(&(reinterpret_cast<NpShape*>(0)->getScbShape().getScShape()));	
	return offset;
}

void NpShapeIncRefCount(Scb::Shape& scb)
{
	NpShape* np = const_cast<NpShape*>(getNpShape(&scb));
	np->incRefCount();
}

void NpShapeDecRefCount(Scb::Shape& scb)
{
	NpShape* np = const_cast<NpShape*>(getNpShape(&scb));
	np->decRefCount();
}
}

// see NpConvexMesh.h, NpHeightField.h, NpTriangleMesh.h for details on how ref counting works for meshes
Cm::RefCountable* NpShape::getMeshRefCountable()
{
	switch(mShape.getGeometryType())
	{
		case PxGeometryType::eCONVEXMESH:
			return static_cast<Gu::ConvexMesh*>(
				static_cast<const PxConvexMeshGeometry&>(mShape.getGeometry()).convexMesh);

		case PxGeometryType::eHEIGHTFIELD:
			return static_cast<Gu::HeightField*>(
				static_cast<const PxHeightFieldGeometry&>(mShape.getGeometry()).heightField);

		case PxGeometryType::eTRIANGLEMESH:
			return static_cast<Gu::TriangleMesh*>(
				static_cast<const PxTriangleMeshGeometry&>(mShape.getGeometry()).triangleMesh);
		
		case PxGeometryType::eSPHERE:
		case PxGeometryType::ePLANE:
		case PxGeometryType::eCAPSULE:
		case PxGeometryType::eBOX:
		case PxGeometryType::eGEOMETRY_COUNT:
		case PxGeometryType::eINVALID:
			break;
	}
	return NULL;
}

bool NpShape::isWritable()
{
	// a shape is writable if it's exclusive, or it's not connected to any actors (which is true if the ref count is 1 and the user ref is not released.)
	return isExclusiveFast() || (getRefCount()==1 && (mBaseFlags & PxBaseFlag::eIS_RELEASABLE));
}

void NpShape::incMeshRefCount()
{
	Cm::RefCountable* npMesh = getMeshRefCountable();
	if(npMesh)
		npMesh->incRefCount();
}

void NpShape::decMeshRefCount()
{
	Cm::RefCountable* npMesh = getMeshRefCountable();
	if(npMesh)
		npMesh->decRefCount();
}

bool NpShape::checkMaterialSetup(const PxGeometry& geom, const char* errorMsgPrefix, PxMaterial*const* materials, PxU16 materialCount)
{
	for(PxU32 i=0; i<materialCount; ++i)
	{
		if(!materials[i])
		{
			Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, 
					"material pointer %d is NULL!", i);
			return false;
		}
	}

	// check that simple shapes don't get assigned multiple materials
	if (materialCount > 1 && (geom.getType() != PxGeometryType::eHEIGHTFIELD) && (geom.getType() != PxGeometryType::eTRIANGLEMESH))
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, 
			"%s: multiple materials defined for single material geometry!", errorMsgPrefix);
		return false;
	}

	//  verify we provide all materials required
	if (materialCount > 1 && (geom.getType() == PxGeometryType::eTRIANGLEMESH))
	{
		const PxTriangleMeshGeometry& meshGeom = static_cast<const PxTriangleMeshGeometry&>(geom);
		const PxTriangleMesh& mesh = *meshGeom.triangleMesh;
		if(mesh.getTriangleMaterialIndex(0) != 0xffff)
		{
			for(PxU32 i = 0; i < mesh.getNbTriangles(); i++)
			{
				const PxMaterialTableIndex meshMaterialIndex = mesh.getTriangleMaterialIndex(i);
				if(meshMaterialIndex >= materialCount)
				{
					Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, 
						"%s: PxTriangleMesh material indices reference more materials than provided!", errorMsgPrefix);
					break;
				}
			}
		}
	}
	if (materialCount > 1 && (geom.getType() == PxGeometryType::eHEIGHTFIELD))
	{
		const PxHeightFieldGeometry& meshGeom = static_cast<const PxHeightFieldGeometry&>(geom);
		const PxHeightField& mesh = *meshGeom.heightField;
		if(mesh.getTriangleMaterialIndex(0) != 0xffff)
		{
			const PxU32 nbTris = mesh.getNbColumns()*mesh.getNbRows()*2;
			for(PxU32 i = 0; i < nbTris; i++)
			{
				const PxMaterialTableIndex meshMaterialIndex = mesh.getTriangleMaterialIndex(i);
				if(meshMaterialIndex != PxHeightFieldMaterial::eHOLE && meshMaterialIndex >= materialCount)
				{
					Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, 
						"%s: PxHeightField material indices reference more materials than provided!", errorMsgPrefix);
					break;
				}
			}
		}
	}

	return true;
}

///////////////////////////////////////////////////////////////////////////////
