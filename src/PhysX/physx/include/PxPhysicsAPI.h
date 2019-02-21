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


#ifndef PX_PHYSICS_NXPHYSICS_API
#define PX_PHYSICS_NXPHYSICS_API
/** \addtogroup physics
@{
*/

/**
This is the main include header for the Physics SDK, for users who
want to use a single #include file.

Alternatively, one can instead directly #include a subset of the below files.
*/

// Foundation SDK 
#include "foundation/Px.h"
#include "foundation/PxAllocatorCallback.h"
#include "foundation/PxAssert.h"
#include "foundation/PxBitAndData.h"
#include "foundation/PxBounds3.h"
#include "foundation/PxErrorCallback.h"
#include "foundation/PxErrors.h"
#include "foundation/PxFlags.h"
#include "foundation/PxIntrinsics.h"
#include "foundation/PxIO.h"
#include "foundation/PxMat33.h"
#include "foundation/PxMat44.h"
#include "foundation/PxMath.h"
#include "foundation/PxMathUtils.h"
#include "foundation/PxPlane.h"
#include "foundation/PxPreprocessor.h"
#include "foundation/PxQuat.h"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxStrideIterator.h"
#include "foundation/PxTransform.h"
#include "foundation/PxUnionCast.h"
#include "foundation/PxVec2.h"
#include "foundation/PxVec3.h"
#include "foundation/PxVec4.h"

//Not physics specific utilities and common code
#include "common/PxCoreUtilityTypes.h"
#include "common/PxPhysXCommonConfig.h"
#include "common/PxRenderBuffer.h"
#include "common/PxBase.h"
#include "common/PxTolerancesScale.h"
#include "common/PxTypeInfo.h"
#include "common/PxStringTable.h"
#include "common/PxSerializer.h"
#include "common/PxMetaData.h"
#include "common/PxMetaDataFlags.h"
#include "common/PxSerialFramework.h"
#include "common/PxPhysicsInsertionCallback.h"

//Task Manager
#include "task/PxTask.h"

// Cuda Mananger
#if PX_SUPPORT_GPU_PHYSX
#include "gpu/PxGpu.h"
#endif

//Geometry Library
#include "geometry/PxBoxGeometry.h"
#include "geometry/PxBVHStructure.h"
#include "geometry/PxCapsuleGeometry.h"
#include "geometry/PxConvexMesh.h"
#include "geometry/PxConvexMeshGeometry.h"
#include "geometry/PxGeometry.h"
#include "geometry/PxGeometryHelpers.h"
#include "geometry/PxGeometryQuery.h"
#include "geometry/PxHeightField.h"
#include "geometry/PxHeightFieldDesc.h"
#include "geometry/PxHeightFieldFlag.h"
#include "geometry/PxHeightFieldGeometry.h"
#include "geometry/PxHeightFieldSample.h"
#include "geometry/PxMeshQuery.h"
#include "geometry/PxMeshScale.h"
#include "geometry/PxPlaneGeometry.h"
#include "geometry/PxSimpleTriangleMesh.h"
#include "geometry/PxSphereGeometry.h"
#include "geometry/PxTriangle.h"
#include "geometry/PxTriangleMesh.h"
#include "geometry/PxTriangleMeshGeometry.h"


// PhysX Core SDK
#include "PxActor.h"
#include "PxAggregate.h"
#include "PxArticulation.h"
#include "PxArticulationReducedCoordinate.h"
#include "PxArticulationJoint.h"
#include "PxArticulationJointReducedCoordinate.h"
#include "PxArticulationLink.h"
#include "PxBatchQuery.h"
#include "PxBatchQueryDesc.h"
#include "PxClient.h"
#include "PxConstraint.h"
#include "PxConstraintDesc.h"
#include "PxContact.h"
#include "PxContactModifyCallback.h"
#include "PxDeletionListener.h"
#include "PxFiltering.h"
#include "PxForceMode.h"
#include "PxFoundation.h"
#include "PxLockedData.h"
#include "PxMaterial.h"
#include "PxPhysics.h"
#include "PxPhysicsVersion.h"
#include "PxPhysXConfig.h"
#include "PxQueryFiltering.h"
#include "PxQueryReport.h"
#include "PxRigidActor.h"
#include "PxRigidBody.h"
#include "PxRigidDynamic.h"
#include "PxRigidStatic.h"
#include "PxScene.h"
#include "PxSceneDesc.h"
#include "PxSceneLock.h"
#include "PxShape.h"
#include "PxSimulationEventCallback.h"
#include "PxSimulationStatistics.h"
#include "PxVisualizationParameter.h"
#include "PxPruningStructure.h"

//Character Controller
#include "characterkinematic/PxBoxController.h"
#include "characterkinematic/PxCapsuleController.h"
#include "characterkinematic/PxController.h"
#include "characterkinematic/PxControllerBehavior.h"
#include "characterkinematic/PxControllerManager.h"
#include "characterkinematic/PxControllerObstacles.h"
#include "characterkinematic/PxExtended.h"

//Cooking (data preprocessing)
#include "cooking/Pxc.h"
#include "cooking/PxConvexMeshDesc.h"
#include "cooking/PxCooking.h"
#include "cooking/PxTriangleMeshDesc.h"
#include "cooking/PxBVH33MidphaseDesc.h"
#include "cooking/PxBVH34MidphaseDesc.h"
#include "cooking/PxMidphaseDesc.h"

//Extensions to the SDK
#include "extensions/PxDefaultStreams.h"
#include "extensions/PxDistanceJoint.h"
#include "extensions/PxExtensionsAPI.h"
#include "extensions/PxFixedJoint.h"
#include "extensions/PxJoint.h"
#include "extensions/PxJointLimit.h"
#include "extensions/PxPrismaticJoint.h"
#include "extensions/PxRevoluteJoint.h"
#include "extensions/PxRigidBodyExt.h"
#include "extensions/PxShapeExt.h"
#include "extensions/PxSimpleFactory.h"
#include "extensions/PxSmoothNormals.h"
#include "extensions/PxSphericalJoint.h"
#include "extensions/PxStringTableExt.h"
#include "extensions/PxTriangleMeshExt.h"
#include "extensions/PxConvexMeshExt.h"

//Serialization
#include "extensions/PxSerialization.h"
#include "extensions/PxBinaryConverter.h"
#include "extensions/PxRepXSerializer.h"

//Vehicle Simulation
#include "vehicle/PxVehicleComponents.h"
#include "vehicle/PxVehicleDrive.h"
#include "vehicle/PxVehicleDrive4W.h"
#include "vehicle/PxVehicleDriveTank.h"
#include "vehicle/PxVehicleSDK.h"
#include "vehicle/PxVehicleShaders.h"
#include "vehicle/PxVehicleTireFriction.h"
#include "vehicle/PxVehicleUpdate.h"
#include "vehicle/PxVehicleUtilControl.h"
#include "vehicle/PxVehicleUtilSetup.h"
#include "vehicle/PxVehicleUtilTelemetry.h"
#include "vehicle/PxVehicleWheels.h"
#include "vehicle/PxVehicleNoDrive.h"
#include "vehicle/PxVehicleDriveNW.h"

//Connecting the SDK to Visual Debugger
#include "pvd/PxPvdSceneClient.h"
#include "pvd/PxPvd.h"
#include "pvd/PxPvdTransport.h"
/** @} */
#endif
