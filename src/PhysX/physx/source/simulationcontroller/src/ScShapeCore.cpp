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

#include "foundation/PxErrorCallback.h"
#include "ScShapeSim.h"
#include "ScPhysics.h"
#include "GuConvexMesh.h"
#include "GuTriangleMesh.h"
#include "GuHeightField.h"
#include "ScMaterialCore.h"

using namespace physx;
using namespace Sc;

// djs: temporary cruft 

static PxConvexMeshGeometryLL extendForLL(const PxConvexMeshGeometry& hlGeom)
{
	PxConvexMeshGeometryLL llGeom;
	static_cast<PxConvexMeshGeometry&>(llGeom) = hlGeom;

	Gu::ConvexMesh* cm = static_cast<Gu::ConvexMesh*>(hlGeom.convexMesh);

	llGeom.hullData = &(cm->getHull());
	llGeom.gpuCompatible = hlGeom.convexMesh->isGpuCompatible();

	return llGeom;
}

static PxTriangleMeshGeometryLL extendForLL(const PxTriangleMeshGeometry& hlGeom)
{
	PxTriangleMeshGeometryLL llGeom;
	static_cast<PxTriangleMeshGeometry&>(llGeom) = hlGeom;

	Gu::TriangleMesh* tm = static_cast<Gu::TriangleMesh*>(hlGeom.triangleMesh);
	llGeom.meshData = tm;
	llGeom.materialIndices = tm->getMaterials();
	llGeom.materials = static_cast<const PxTriangleMeshGeometryLL&>(hlGeom).materials;

	return llGeom;
}

static PxHeightFieldGeometryLL extendForLL(const PxHeightFieldGeometry& hlGeom)
{
	PxHeightFieldGeometryLL llGeom;
	static_cast<PxHeightFieldGeometry&>(llGeom) = hlGeom;

	Gu::HeightField* hf = static_cast<Gu::HeightField*>(hlGeom.heightField);

	llGeom.heightFieldData = &hf->getData();

	llGeom.materials = static_cast<const PxHeightFieldGeometryLL&>(hlGeom).materials;

	return llGeom;
}

ShapeCore::ShapeCore(const PxGeometry& geometry, PxShapeFlags shapeFlags, const PxU16* materialIndices, PxU16 materialCount) :
	mRestOffset(0.0f),
	mTorsionalRadius(0.f),
	mMinTorsionalPatchRadius(0.f)
{
	mCore.mOwnsMaterialIdxMemory	= true;

	PX_ASSERT(materialCount > 0);

	const PxTolerancesScale& scale = Physics::getInstance().getTolerancesScale();
	mCore.geometry.set(geometry);
	mCore.transform			= PxTransform(PxIdentity);
	mCore.contactOffset		= 0.02f * scale.length;

	mCore.mShapeFlags		= shapeFlags;

	setMaterialIndices(materialIndices, materialCount);
}

// PX_SERIALIZATION
ShapeCore::ShapeCore(const PxEMPTY) : 
	mQueryFilterData		(PxEmpty),
	mSimulationFilterData	(PxEmpty),
	mCore					(PxEmpty)
{ 
	mCore.mOwnsMaterialIdxMemory	= false;
}
//~PX_SERIALIZATION

ShapeCore::~ShapeCore()
{
	if(mCore.geometry.getType() == PxGeometryType::eTRIANGLEMESH)
	{
		PxTriangleMeshGeometryLL& meshGeom = mCore.geometry.get<PxTriangleMeshGeometryLL>();
		if(mCore.mOwnsMaterialIdxMemory)
			meshGeom.materials.deallocate();
	}
	else if(mCore.geometry.getType() == PxGeometryType::eHEIGHTFIELD)
	{
		PxHeightFieldGeometryLL& hfGeom = mCore.geometry.get<PxHeightFieldGeometryLL>();
		if(mCore.mOwnsMaterialIdxMemory)
			hfGeom.materials.deallocate();
	}
}

PxU16 Sc::ShapeCore::getNbMaterialIndices() const
{
	const PxGeometryType::Enum geomType = mCore.geometry.getType();

	if((geomType != PxGeometryType::eTRIANGLEMESH) && (geomType != PxGeometryType::eHEIGHTFIELD))
	{
		return 1;
	}
	else if(geomType == PxGeometryType::eTRIANGLEMESH)
	{
		const PxTriangleMeshGeometryLL& meshGeom = mCore.geometry.get<PxTriangleMeshGeometryLL>();
		return meshGeom.materials.numIndices;
	}
	else
	{
		PX_ASSERT(geomType == PxGeometryType::eHEIGHTFIELD);
		const PxHeightFieldGeometryLL& hfGeom = mCore.geometry.get<PxHeightFieldGeometryLL>();
		return hfGeom.materials.numIndices;
	}
}

const PxU16* Sc::ShapeCore::getMaterialIndices() const
{
	const PxGeometryType::Enum geomType = mCore.geometry.getType();

	if((geomType != PxGeometryType::eTRIANGLEMESH) && (geomType != PxGeometryType::eHEIGHTFIELD))
	{
		return &mCore.materialIndex;
	}
	else if(geomType == PxGeometryType::eTRIANGLEMESH)
	{
		const PxTriangleMeshGeometryLL& meshGeom = mCore.geometry.get<PxTriangleMeshGeometryLL>();
		return meshGeom.materials.indices;
	}
	else
	{
		PX_ASSERT(geomType == PxGeometryType::eHEIGHTFIELD);
		const PxHeightFieldGeometryLL& hfGeom = mCore.geometry.get<PxHeightFieldGeometryLL>();
		return hfGeom.materials.indices;
	}
}

PX_FORCE_INLINE void setMaterialsHelper(MaterialIndicesStruct& materials, const PxU16* materialIndices, PxU16 materialIndexCount, PxU8& ownsMemory)
{
	if(materials.numIndices < materialIndexCount)
	{
		if(materials.indices && ownsMemory)
			materials.deallocate();
		materials.allocate(materialIndexCount);
		ownsMemory = true;
	}
	PxMemCopy(materials.indices, materialIndices, sizeof(PxU16)*materialIndexCount);
	materials.numIndices = materialIndexCount;
}

void ShapeCore::setMaterialIndices(const PxU16* materialIndices, PxU16 materialIndexCount)
{
	const PxGeometryType::Enum geomType = mCore.geometry.getType();
	mCore.materialIndex = materialIndices[0];

	if(geomType == PxGeometryType::eTRIANGLEMESH)
	{
		PxTriangleMeshGeometryLL& meshGeom = mCore.geometry.get<PxTriangleMeshGeometryLL>();
		setMaterialsHelper(meshGeom.materials, materialIndices, materialIndexCount, mCore.mOwnsMaterialIdxMemory);
	}
	else if(geomType == PxGeometryType::eHEIGHTFIELD)
	{
		PxHeightFieldGeometryLL& hfGeom = mCore.geometry.get<PxHeightFieldGeometryLL>();
		setMaterialsHelper(hfGeom.materials, materialIndices, materialIndexCount, mCore.mOwnsMaterialIdxMemory);
	}
}

void ShapeCore::setGeometry(const PxGeometry& geom)
{
	const PxGeometryType::Enum oldGeomType = mCore.geometry.getType();
	const PxGeometryType::Enum newGeomType = geom.getType();

	// copy material related data to restore it after the new geometry has been set
	MaterialIndicesStruct materials;
	PX_ASSERT(materials.numIndices == 0);
	
	if(oldGeomType == PxGeometryType::eTRIANGLEMESH)
	{
		PxTriangleMeshGeometryLL& meshGeom = mCore.geometry.get<PxTriangleMeshGeometryLL>();
		materials = meshGeom.materials;
	}
	else if(oldGeomType == PxGeometryType::eHEIGHTFIELD)
	{
		PxHeightFieldGeometryLL& hfGeom = mCore.geometry.get<PxHeightFieldGeometryLL>();
		materials = hfGeom.materials;
	}

	mCore.geometry.set(geom);

	if((newGeomType == PxGeometryType::eTRIANGLEMESH) || (newGeomType == PxGeometryType::eHEIGHTFIELD))
	{
		MaterialIndicesStruct* newMaterials;

		if(newGeomType == PxGeometryType::eTRIANGLEMESH)
		{
			PxTriangleMeshGeometryLL& meshGeom = mCore.geometry.get<PxTriangleMeshGeometryLL>();
			newMaterials = &meshGeom.materials;
		}
		else
		{
			PX_ASSERT(newGeomType == PxGeometryType::eHEIGHTFIELD);
			PxHeightFieldGeometryLL& hfGeom = mCore.geometry.get<PxHeightFieldGeometryLL>();
			newMaterials = &hfGeom.materials;
		}

		if(materials.numIndices != 0)  // old type was mesh type
			*newMaterials = materials;
		else
		{   // old type was non-mesh type
			newMaterials->allocate(1);
			*newMaterials->indices = mCore.materialIndex;
			mCore.mOwnsMaterialIdxMemory = true;
		}
	}
	else if((materials.numIndices != 0) && mCore.mOwnsMaterialIdxMemory)
	{
		// geometry changed to non-mesh type
		materials.deallocate();
	}
}

PxShape* ShapeCore::getPxShape()
{
	return Sc::gOffsetTable.convertScShape2Px(this);
}

const PxShape* ShapeCore::getPxShape() const
{
	return Sc::gOffsetTable.convertScShape2Px(this);
}

// PX_SERIALIZATION

PX_FORCE_INLINE void exportExtraDataMaterials(PxSerializationContext& stream, const MaterialIndicesStruct& materials)
{
	stream.alignData(PX_SERIAL_ALIGN);
	stream.writeData(materials.indices, sizeof(PxU16)*materials.numIndices);
}

void ShapeCore::exportExtraData(PxSerializationContext& stream)
{
	const PxGeometryType::Enum geomType = mCore.geometry.getType();

	if(geomType == PxGeometryType::eTRIANGLEMESH)
	{
		PxTriangleMeshGeometryLL& meshGeom = mCore.geometry.get<PxTriangleMeshGeometryLL>();
		exportExtraDataMaterials(stream, meshGeom.materials);
	}
	else if(geomType == PxGeometryType::eHEIGHTFIELD)
	{
		PxHeightFieldGeometryLL& hfGeom = mCore.geometry.get<PxHeightFieldGeometryLL>();
		exportExtraDataMaterials(stream, hfGeom.materials);
	}
}

void ShapeCore::importExtraData(PxDeserializationContext& context)
{
	const PxGeometryType::Enum geomType = mCore.geometry.getType();

	if(geomType == PxGeometryType::eTRIANGLEMESH)
	{
		MaterialIndicesStruct& materials = mCore.geometry.get<PxTriangleMeshGeometryLL>().materials;
		materials.indices = context.readExtraData<PxU16, PX_SERIAL_ALIGN>(materials.numIndices);
	}
	else if(geomType == PxGeometryType::eHEIGHTFIELD)
	{
		MaterialIndicesStruct& materials = mCore.geometry.get<PxHeightFieldGeometryLL>().materials;
		materials.indices = context.readExtraData<PxU16, PX_SERIAL_ALIGN>(materials.numIndices);
	}
}

void ShapeCore::resolveMaterialReference(PxU32 materialTableIndex, PxU16 materialIndex)
{
	if(materialTableIndex == 0)
	{
		mCore.materialIndex = materialIndex;
	}

	PxGeometry& geom = const_cast<PxGeometry&>(mCore.geometry.getGeometry());

	if(geom.getType() == PxGeometryType::eHEIGHTFIELD)
	{
		PxHeightFieldGeometryLL& hfGeom = static_cast<PxHeightFieldGeometryLL&>(geom);
		hfGeom.materials.indices[materialTableIndex] = materialIndex;
	}
	else if(geom.getType() == PxGeometryType::eTRIANGLEMESH)
	{
		PxTriangleMeshGeometryLL& meshGeom = static_cast<PxTriangleMeshGeometryLL&>(geom);
		meshGeom.materials.indices[materialTableIndex] = materialIndex;
	}
}

void ShapeCore::resolveReferences(PxDeserializationContext& context)
{
	// Resolve geometry pointers if needed
	PxGeometry& geom = const_cast<PxGeometry&>(mCore.geometry.getGeometry());	
	
	switch(geom.getType())
	{
	case PxGeometryType::eCONVEXMESH:
	{
		PxConvexMeshGeometryLL& convexGeom = static_cast<PxConvexMeshGeometryLL&>(geom);
		context.translatePxBase(convexGeom.convexMesh);		

		// update the hullData pointer
		static_cast<PxConvexMeshGeometryLL&>(geom) = extendForLL(convexGeom);
	}
	break;

	case PxGeometryType::eHEIGHTFIELD:
	{
		PxHeightFieldGeometryLL& hfGeom = static_cast<PxHeightFieldGeometryLL&>(geom);
		context.translatePxBase(hfGeom.heightField);

		// update hf pointers
		static_cast<PxHeightFieldGeometryLL&>(geom) = extendForLL(hfGeom);
	}
	break;

	case PxGeometryType::eTRIANGLEMESH:
	{
		PxTriangleMeshGeometryLL& meshGeom = static_cast<PxTriangleMeshGeometryLL&>(geom);
		context.translatePxBase(meshGeom.triangleMesh);		

		// update mesh pointers
		static_cast<PxTriangleMeshGeometryLL&>(geom) = extendForLL(meshGeom);
	}
	break;
	case PxGeometryType::eSPHERE:
	case PxGeometryType::ePLANE:
	case PxGeometryType::eCAPSULE:
	case PxGeometryType::eBOX:
	case PxGeometryType::eGEOMETRY_COUNT:
	case PxGeometryType::eINVALID:
	break;

	}	
}

//~PX_SERIALIZATION
