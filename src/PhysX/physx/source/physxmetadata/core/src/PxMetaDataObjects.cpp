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

#include "PsFoundation.h"
#include "PsUtilities.h"
#include "CmPhysXCommon.h"

#include "PxMetaDataObjects.h"

using namespace physx;

PX_PHYSX_CORE_API PxGeometryType::Enum PxShapeGeometryPropertyHelper::getGeometryType(const PxShape* inShape) const { return inShape->getGeometryType(); }
PX_PHYSX_CORE_API bool PxShapeGeometryPropertyHelper::getGeometry(const PxShape* inShape, PxBoxGeometry& geometry) const { return inShape->getBoxGeometry( geometry ); }
PX_PHYSX_CORE_API bool PxShapeGeometryPropertyHelper::getGeometry(const PxShape* inShape, PxSphereGeometry& geometry) const { return inShape->getSphereGeometry( geometry ); }
PX_PHYSX_CORE_API bool PxShapeGeometryPropertyHelper::getGeometry(const PxShape* inShape, PxCapsuleGeometry& geometry) const { return inShape->getCapsuleGeometry( geometry ); }
PX_PHYSX_CORE_API bool PxShapeGeometryPropertyHelper::getGeometry(const PxShape* inShape, PxPlaneGeometry& geometry) const { return inShape->getPlaneGeometry( geometry ); }
PX_PHYSX_CORE_API bool PxShapeGeometryPropertyHelper::getGeometry(const PxShape* inShape, PxConvexMeshGeometry& geometry) const { return inShape->getConvexMeshGeometry( geometry ); }
PX_PHYSX_CORE_API bool PxShapeGeometryPropertyHelper::getGeometry(const PxShape* inShape, PxTriangleMeshGeometry& geometry) const { return inShape->getTriangleMeshGeometry( geometry ); }
PX_PHYSX_CORE_API bool PxShapeGeometryPropertyHelper::getGeometry(const PxShape* inShape, PxHeightFieldGeometry& geometry) const { return inShape->getHeightFieldGeometry( geometry ); }

PX_PHYSX_CORE_API void PxShapeMaterialsPropertyHelper::setMaterials(PxShape* inShape, PxMaterial*const* materials, PxU16 materialCount) const
{
	inShape->setMaterials( materials, materialCount );
}

PX_PHYSX_CORE_API PxShape* PxRigidActorShapeCollectionHelper::createShape(PxRigidActor* inActor, const PxGeometry& geometry, PxMaterial& material,
														PxShapeFlags shapeFlags ) const
{
	PxMaterial* materialPtr = const_cast<PxMaterial*>(&material);
	PxShape* shape = PxGetPhysics().createShape(geometry, &materialPtr, 1, true, shapeFlags);
	if (shape)
	{
		inActor->attachShape(*shape);	// attach can fail, if e.g. we try and attach a trimesh simulation shape to a dynamic actor
		shape->release();		// if attach fails, we hold the only counted reference, and so this cleans up properly
	}
	return shape;
}
PX_PHYSX_CORE_API PxShape* PxRigidActorShapeCollectionHelper::createShape(PxRigidActor* inActor, const PxGeometry& geometry, PxMaterial *const* materials,
														PxU16 materialCount, PxShapeFlags shapeFlags ) const
{
	PxShape* shape = PxGetPhysics().createShape(geometry, materials, materialCount, true, shapeFlags);
	if (shape)
	{
		inActor->attachShape(*shape);	// attach can fail, if e.g. we try and attach a trimesh simulation shape to a dynamic actor
		shape->release();		// if attach fails, we hold the only counted reference, and so this cleans up properly
	}
	return shape;
}

PX_PHYSX_CORE_API PxArticulationLink*	PxArticulationLinkCollectionPropHelper::createLink(PxArticulation* inArticulation, PxArticulationLink* parent,
																	   const PxTransform& pose) const
{
	PX_CHECK_AND_RETURN_NULL(pose.isValid(), "PxArticulationLinkCollectionPropHelper::createLink pose is not valid.");
	return inArticulation->createLink(parent, pose );
}




inline void SetNbShape( PxSimulationStatistics* inStats, PxGeometryType::Enum data, PxU32 val ) { inStats->nbShapes[data] = val; }
inline PxU32 GetNbShape( const PxSimulationStatistics* inStats, PxGeometryType::Enum data) { return inStats->nbShapes[data]; }


PX_PHYSX_CORE_API NbShapesProperty::NbShapesProperty()
	: PxIndexedPropertyInfo<PxPropertyInfoName::PxSimulationStatistics_NbShapes
			, PxSimulationStatistics
			, PxGeometryType::Enum
			, PxU32> ( "NbShapes", SetNbShape, GetNbShape )
{
}


inline void SetNbDiscreteContactPairs( PxSimulationStatistics* inStats, PxGeometryType::Enum idx1, PxGeometryType::Enum idx2, PxU32 val ) { inStats->nbDiscreteContactPairs[idx1][idx2] = val; }
inline PxU32 GetNbDiscreteContactPairs( const PxSimulationStatistics* inStats, PxGeometryType::Enum idx1, PxGeometryType::Enum idx2 ) { return inStats->nbDiscreteContactPairs[idx1][idx2]; }
PX_PHYSX_CORE_API NbDiscreteContactPairsProperty::NbDiscreteContactPairsProperty()
								: PxDualIndexedPropertyInfo<PxPropertyInfoName::PxSimulationStatistics_NbDiscreteContactPairs
															, PxSimulationStatistics
															, PxGeometryType::Enum
															, PxGeometryType::Enum
															, PxU32> ( "NbDiscreteContactPairs", SetNbDiscreteContactPairs, GetNbDiscreteContactPairs )
{
}

inline void SetNbModifiedContactPairs( PxSimulationStatistics* inStats, PxGeometryType::Enum idx1, PxGeometryType::Enum idx2, PxU32 val ) { inStats->nbModifiedContactPairs[idx1][idx2] = val; }
inline PxU32 GetNbModifiedContactPairs( const PxSimulationStatistics* inStats, PxGeometryType::Enum idx1, PxGeometryType::Enum idx2 ) { return inStats->nbModifiedContactPairs[idx1][idx2]; }
PX_PHYSX_CORE_API NbModifiedContactPairsProperty::NbModifiedContactPairsProperty()
								: PxDualIndexedPropertyInfo<PxPropertyInfoName::PxSimulationStatistics_NbModifiedContactPairs
															, PxSimulationStatistics
															, PxGeometryType::Enum
															, PxGeometryType::Enum
															, PxU32> ( "NbModifiedContactPairs", SetNbModifiedContactPairs, GetNbModifiedContactPairs )
{
}

inline void SetNbCCDPairs( PxSimulationStatistics* inStats, PxGeometryType::Enum idx1, PxGeometryType::Enum idx2, PxU32 val ) { inStats->nbCCDPairs[idx1][idx2] = val; }
inline PxU32 GetNbCCDPairs( const PxSimulationStatistics* inStats, PxGeometryType::Enum idx1, PxGeometryType::Enum idx2 ) { return inStats->nbCCDPairs[idx1][idx2]; }
PX_PHYSX_CORE_API NbCCDPairsProperty::NbCCDPairsProperty()
								: PxDualIndexedPropertyInfo<PxPropertyInfoName::PxSimulationStatistics_NbCCDPairs
															, PxSimulationStatistics
															, PxGeometryType::Enum
															, PxGeometryType::Enum
															, PxU32> ( "NbCCDPairs", SetNbCCDPairs, GetNbCCDPairs )
{
}

inline void SetNbTriggerPairs( PxSimulationStatistics* inStats, PxGeometryType::Enum idx1, PxGeometryType::Enum idx2, PxU32 val ) { inStats->nbTriggerPairs[idx1][idx2] = val; }
inline PxU32 GetNbTriggerPairs( const PxSimulationStatistics* inStats, PxGeometryType::Enum idx1, PxGeometryType::Enum idx2 ) { return inStats->nbTriggerPairs[idx1][idx2]; }
PX_PHYSX_CORE_API NbTriggerPairsProperty::NbTriggerPairsProperty()
								: PxDualIndexedPropertyInfo<PxPropertyInfoName::PxSimulationStatistics_NbTriggerPairs
															, PxSimulationStatistics
															, PxGeometryType::Enum
															, PxGeometryType::Enum
															, PxU32> ( "NbTriggerPairs", SetNbTriggerPairs, GetNbTriggerPairs )
{
}

inline PxSimulationStatistics GetStats( const PxScene* inScene ) { PxSimulationStatistics stats; inScene->getSimulationStatistics( stats ); return stats; }
PX_PHYSX_CORE_API SimulationStatisticsProperty::SimulationStatisticsProperty() 
	: PxReadOnlyPropertyInfo<PxPropertyInfoName::PxScene_SimulationStatistics, PxScene, PxSimulationStatistics >( "SimulationStatistics", GetStats )
{
}

