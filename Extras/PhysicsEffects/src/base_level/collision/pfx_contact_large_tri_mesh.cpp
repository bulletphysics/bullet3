/*
Physics Effects Copyright(C) 2010 Sony Computer Entertainment Inc.
All rights reserved.

Physics Effects is open software; you can redistribute it and/or
modify it under the terms of the BSD License.

Physics Effects is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the BSD License for more details.

A copy of the BSD License is distributed with
Physics Effects under the filename: physics_effects_license.txt
*/

#include "../../../include/physics_effects/base_level/base/pfx_vec_utils.h"
#include "pfx_contact_tri_mesh_sphere.h"
#include "pfx_contact_tri_mesh_box.h"
#include "pfx_contact_tri_mesh_capsule.h"
#include "pfx_contact_tri_mesh_cylinder.h"
#include "pfx_contact_tri_mesh_convex.h"
#include "pfx_contact_large_tri_mesh.h"
#include "pfx_intersect_common.h"
#include "pfx_mesh_common.h"


namespace sce {
namespace PhysicsEffects {


PfxInt32 pfxContactLargeTriMesh(
				PfxContactCache &contacts,
				const PfxLargeTriMesh *lmeshA,
				const PfxTransform3 &transformA,
				const PfxShape &shapeB,
				const PfxTransform3 &transformB,
				PfxFloat distanceThreshold)
{
	PfxTransform3 transformAB;
	PfxMatrix3 matrixAB;
	PfxVector3 offsetAB;
	
	// Bローカル→Aローカルへの変換
	transformAB = orthoInverse(transformA) * transformB;
	matrixAB = transformAB.getUpper3x3();
	offsetAB = transformAB.getTranslation();
	
	// -----------------------------------------------------
	// LargeTriMeshに含まれるTriMeshのAABBと凸体のAABBを判定し、
	// 交差するものを個別に衝突判定する。※LargeMesh座標系
	
	PfxVector3 shapeHalf(0.0f);
	PfxVector3 shapeCenter = offsetAB;
	

	switch(shapeB.getType()) {
		case kPfxShapeSphere:
		shapeHalf = PfxVector3(shapeB.getSphere().m_radius);
		break;
		
		case kPfxShapeCapsule:
		{
			PfxCapsule capsule = shapeB.getCapsule();
			shapeHalf = absPerElem(matrixAB) * PfxVector3(capsule.m_halfLen+capsule.m_radius,capsule.m_radius,capsule.m_radius);
		}
		break;
		
		case kPfxShapeCylinder:
		{
			PfxCylinder cylinder = shapeB.getCylinder();
			shapeHalf = absPerElem(matrixAB) * PfxVector3(cylinder.m_halfLen,cylinder.m_radius,cylinder.m_radius);
		}
		break;
		
		case kPfxShapeBox:
		shapeHalf = absPerElem(matrixAB) * shapeB.getBox().m_half;
		break;
		
		case kPfxShapeConvexMesh:
	shapeHalf = absPerElem(matrixAB) * shapeB.getConvexMesh()->m_half;
		break;
		
		default:
		break;
	}

	// -----------------------------------------------------
	// アイランドとの衝突判定

	PfxVecInt3 aabbMinL,aabbMaxL;
	lmeshA->getLocalPosition((shapeCenter-shapeHalf),(shapeCenter+shapeHalf),aabbMinL,aabbMaxL);
	
	PfxUInt32 numIslands = lmeshA->m_numIslands;

	{
	for(PfxUInt32 i=0;i<numIslands;i++) {
		// AABBチェック
		PfxAabb16 aabbB = lmeshA->m_aabbList[i];
		if(aabbMaxL.getX() < pfxGetXMin(aabbB) || aabbMinL.getX() > pfxGetXMax(aabbB)) continue;
		if(aabbMaxL.getY() < pfxGetYMin(aabbB) || aabbMinL.getY() > pfxGetYMax(aabbB)) continue;
		if(aabbMaxL.getZ() < pfxGetZMin(aabbB) || aabbMinL.getZ() > pfxGetZMax(aabbB)) continue;
		
		PfxTriMesh *island = &lmeshA->m_islands[i];

			// 衝突判定
			PfxContactCache localContacts;
			switch(shapeB.getType()) {
				case kPfxShapeSphere:
				pfxContactTriMeshSphere(localContacts,island,transformA,shapeB.getSphere(),transformB,distanceThreshold);
				break;
				
				case kPfxShapeCapsule:
				pfxContactTriMeshCapsule(localContacts,island,transformA,shapeB.getCapsule(),transformB,distanceThreshold);
				break;
				
				case kPfxShapeBox:
				pfxContactTriMeshBox(localContacts,island,transformA,shapeB.getBox(),transformB,distanceThreshold);
				break;
				
				case kPfxShapeCylinder:
				pfxContactTriMeshCylinder(localContacts,island,transformA,shapeB.getCylinder(),transformB,distanceThreshold);
				break;
				
				case kPfxShapeConvexMesh:
			pfxContactTriMeshConvex(localContacts,island,transformA,*shapeB.getConvexMesh(),transformB,distanceThreshold);
				break;
				
				default:
				break;
			}

			
			// 衝突点を追加
			for(int j=0;j<localContacts.getNumContacts();j++) {
				PfxSubData subData = localContacts.getSubData(j);
				subData.setIslandId(i);
				contacts.addContactPoint(
					localContacts.getDistance(j),
					localContacts.getNormal(j),
					localContacts.getLocalPointA(j),
					localContacts.getLocalPointB(j),
					subData);
			}
		}
	}


	return contacts.getNumContacts();
}
} //namespace PhysicsEffects
} //namespace sce
