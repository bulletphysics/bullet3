/* Copyright (C) 2006-2009 Erwin Coumans & Charlie C
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
*
* 1. The origin of this software must not be misrepresented; you must not
*    claim that you wrote the original software. If you use this software
*    in a product, an acknowledgment in the product documentation would be
*    appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
*    misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/
// Auto generated from makesdna dna.c
#ifndef __BULLETCOMMON_H__
#define __BULLETCOMMON_H__

// put an empty struct in the case
typedef struct bInvalidHandle {
	int unused;
}bInvalidHandle;

namespace Bullet {
    class PointerArray;
    class btPhysicsSystem;
    class ListBase;
    class btVector3FloatData;
    class btVector3DoubleData;
    class btMatrix3x3FloatData;
    class btMatrix3x3DoubleData;
    class btTransformFloatData;
    class btTransformDoubleData;
    class btBvhSubtreeInfoData;
    class btOptimizedBvhNodeFloatData;
    class btOptimizedBvhNodeDoubleData;
    class btQuantizedBvhNodeData;
    class btQuantizedBvhFloatData;
    class btQuantizedBvhDoubleData;
    class btCollisionShapeData;
    class btStaticPlaneShapeData;
    class btConvexInternalShapeData;
    class btPositionAndRadius;
    class btMultiSphereShapeData;
    class btIntIndexData;
    class btShortIntIndexData;
    class btMeshPartData;
    class btStridingMeshInterfaceData;
    class btTriangleMeshShapeData;
    class btCompoundShapeChildData;
    class btCompoundShapeData;
    class btCylinderShapeData;
    class btCapsuleShapeData;
    class btTriangleInfoData;
    class btTriangleInfoMapData;
    class btGImpactMeshShapeData;
    class btConvexHullShapeData;
    class btCollisionObjectDoubleData;
    class btCollisionObjectFloatData;
    class btRigidBodyFloatData;
    class btRigidBodyDoubleData;
    class btConstraintInfo1;
    class btTypedConstraintData;
    class btPoint2PointConstraintFloatData;
    class btPoint2PointConstraintDoubleData;
    class btHingeConstraintDoubleData;
    class btHingeConstraintFloatData;
    class btConeTwistConstraintData;
    class btGeneric6DofConstraintData;
    class btSliderConstraintData;
}
#endif//__BULLETCOMMON_H__