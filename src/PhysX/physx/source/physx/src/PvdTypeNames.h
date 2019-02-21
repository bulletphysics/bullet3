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
#ifndef PVD_TYPE_NAMES_H
#define PVD_TYPE_NAMES_H
#if PX_SUPPORT_PVD
#include "PxPvdObjectModelBaseTypes.h"
#include "PxMetaDataObjects.h"
#include "PxHeightFieldSample.h"

namespace physx
{
namespace Vd
{
struct PvdSqHit;
struct PvdRaycast;
struct PvdOverlap;
struct PvdSweep;

struct PvdHullPolygonData
{
	PxU16 mNumVertices;
	PxU16 mIndexBase;
	PvdHullPolygonData(PxU16 numVert, PxU16 idxBase) : mNumVertices(numVert), mIndexBase(idxBase)
	{
	}
};


struct PxArticulationLinkUpdateBlock
{
	PxTransform GlobalPose;
	PxVec3 LinearVelocity;
	PxVec3 AngularVelocity;
};
struct PxRigidDynamicUpdateBlock : public PxArticulationLinkUpdateBlock
{
	bool IsSleeping;
};

struct PvdContact
{
	PxVec3 point;
	PxVec3 axis;
	const void* shape0;
	const void* shape1;
	PxF32 separation;
	PxF32 normalForce;
	PxU32 internalFaceIndex0;
	PxU32 internalFaceIndex1;
	bool normalForceAvailable;
};

struct PvdPositionAndRadius
{
	PxVec3 position;
	PxF32 radius;
};

} //Vd
} //physx

namespace physx
{
namespace pvdsdk
{

#define DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(type) DEFINE_PVD_TYPE_NAME_MAP(physx::type, "physx3", #type)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxPhysics)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxScene)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxTolerancesScale)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxTolerancesScaleGeneratedValues)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxSceneDescGeneratedValues)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxSceneDesc)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxSimulationStatistics)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxSimulationStatisticsGeneratedValues)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxMaterial)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxMaterialGeneratedValues)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxHeightField)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxHeightFieldDesc)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxHeightFieldDescGeneratedValues)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxTriangleMesh)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxConvexMesh)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxActor)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxRigidActor)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxRigidBody)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxRigidDynamic)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxRigidDynamicGeneratedValues)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxRigidStatic)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxRigidStaticGeneratedValues)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxShape)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxShapeGeneratedValues)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxGeometry)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxBoxGeometry)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxPlaneGeometry)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxCapsuleGeometry)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxSphereGeometry)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxHeightFieldGeometry)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxTriangleMeshGeometry)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxConvexMeshGeometry)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxBoxGeometryGeneratedValues)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxPlaneGeometryGeneratedValues)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxCapsuleGeometryGeneratedValues)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxSphereGeometryGeneratedValues)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxHeightFieldGeometryGeneratedValues)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxTriangleMeshGeometryGeneratedValues)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxConvexMeshGeometryGeneratedValues)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxHeightFieldSample)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxConstraint)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxConstraintGeneratedValues)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxArticulationBase)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxArticulationBaseGeneratedValues)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxArticulation)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxArticulationGeneratedValues)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxArticulationLink)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxArticulationLinkGeneratedValues)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxArticulationJointBase)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxArticulationJointBaseGeneratedValues)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxArticulationJoint)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxArticulationJointGeneratedValues)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxArticulationJointReducedCoordinate)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxAggregate)
DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP(PxAggregateGeneratedValues)

#undef DEFINE_NATIVE_PVD_PHYSX3_TYPE_MAP

#define DEFINE_NATIVE_PVD_TYPE_MAP(type) DEFINE_PVD_TYPE_NAME_MAP(physx::Vd::type, "physx3", #type)
DEFINE_NATIVE_PVD_TYPE_MAP(PvdHullPolygonData)
DEFINE_NATIVE_PVD_TYPE_MAP(PxRigidDynamicUpdateBlock)
DEFINE_NATIVE_PVD_TYPE_MAP(PxArticulationLinkUpdateBlock)
DEFINE_NATIVE_PVD_TYPE_MAP(PvdContact)
DEFINE_NATIVE_PVD_TYPE_MAP(PvdRaycast)
DEFINE_NATIVE_PVD_TYPE_MAP(PvdSweep)
DEFINE_NATIVE_PVD_TYPE_MAP(PvdOverlap)
DEFINE_NATIVE_PVD_TYPE_MAP(PvdSqHit)
DEFINE_NATIVE_PVD_TYPE_MAP(PvdPositionAndRadius)

#undef DEFINE_NATIVE_PVD_TYPE_MAP


DEFINE_PVD_TYPE_ALIAS(physx::PxFilterData, U32Array4)


}
}

#endif

#endif
