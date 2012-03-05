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
#include "pfx_contact_tri_mesh_cylinder.h"
#include "pfx_intersect_common.h"
#include "pfx_mesh_common.h"
#include "pfx_gjk_solver.h"
#include "pfx_gjk_support_func.h"

namespace sce {
namespace PhysicsEffects {

static SCE_PFX_FORCE_INLINE
bool pfxContactTriangleCylinder(PfxContactCache &contacts,PfxUInt32 facetId,
							const PfxVector3 &normal,const PfxVector3 &p0,const PfxVector3 &p1,const PfxVector3 &p2,
							const PfxFloat thickness,const PfxFloat angle0,const PfxFloat angle1,const PfxFloat angle2,
							PfxUInt32 edgeChk,
							const PfxCylinder &cylinder)
{
	PfxVector3 facetPnts[6] = {
		p0,p1,p2,p0-thickness*normal,p1-thickness*normal,p2-thickness*normal
	};
	

	PfxPoint3 pA(0.0f),pB(0.0f);
	PfxVector3 nml(0.0f);
	PfxGjkSolver gjk;
	
	gjk.setup((void*)facetPnts,(void*)&cylinder,pfxGetSupportVertexTriangleWithThickness,pfxGetSupportVertexCylinder);
	PfxFloat d = gjk.collide(nml,pA,pB,PfxTransform3::identity(),PfxTransform3::identity(),SCE_PFX_FLT_MAX);
	if(d >= 0.0f) return false;
	
	PfxVector3 pointsOnTriangle = PfxVector3(pA);
	PfxVector3 pointsOnConvex = PfxVector3(pB);
	PfxVector3 axis = nml;
	
	// 面上の最近接点が凸エッジ上でない場合は法線を変える
	if( ((edgeChk&0x01)&&pfxPointOnLine(pointsOnTriangle,p0,p1)) ||
		((edgeChk&0x02)&&pfxPointOnLine(pointsOnTriangle,p1,p2)) ||
		((edgeChk&0x04)&&pfxPointOnLine(pointsOnTriangle,p2,p0)) ) {
		axis=-normal;
	}
	
	PfxSubData subData;
	subData.setFacetId(facetId);
	contacts.addContactPoint(-length(pointsOnTriangle-pointsOnConvex),axis,pA,pB,subData);
	
	return true;
}

PfxInt32 pfxContactTriMeshCylinder(
	PfxContactCache &contacts,
	const PfxTriMesh *meshA,
	const PfxTransform3 &transformA,
	const PfxCylinder &cylinderB,
	const PfxTransform3 &transformB,
	PfxFloat distanceThreshold)
{
	(void) distanceThreshold;

	PfxTransform3 transformAB, transformBA;
	PfxMatrix3 matrixBA;
	PfxVector3 offsetBA;

	// Bローカル→Aローカルへの変換.
	transformAB = orthoInverse(transformA) * transformB;

	// Aローカル→Bローカルへの変換.
	transformBA = orthoInverse(transformAB);

	matrixBA = transformBA.getUpper3x3();
	offsetBA = transformBA.getTranslation();

	//-------------------------------------------
	// 判定する面を絞り込む.

	PfxUInt8 SCE_PFX_ALIGNED(16) selFacets[SCE_PFX_NUMMESHFACETS] = { 0 };
	PfxUInt32 numSelFacets = 0;

	// ※cylinderB座標系
	PfxVector3 aabbB(cylinderB.m_halfLen,cylinderB.m_radius,cylinderB.m_radius);
	numSelFacets = pfxGatherFacets(meshA,(PfxFloat*)&aabbB,offsetBA,matrixBA,selFacets);

	if(numSelFacets == 0) {
		return 0;
	}
	
	//-------------------------------------------
	// 面ごとに衝突を検出
	
	PfxContactCache localContacts;
	
	for(PfxUInt32 f = 0; f < numSelFacets; f++) {
		const PfxFacet &facet = meshA->m_facets[selFacets[f]];
		
		PfxVector3 facetNormal = matrixBA * pfxReadVector3(facet.m_normal);

		PfxVector3 facetPntsA[3] = {
			offsetBA + matrixBA * meshA->m_verts[facet.m_vertIds[0]],
			offsetBA + matrixBA * meshA->m_verts[facet.m_vertIds[1]],
			offsetBA + matrixBA * meshA->m_verts[facet.m_vertIds[2]],
		};

		const PfxEdge *edge[3] = {
			&meshA->m_edges[facet.m_edgeIds[0]],
			&meshA->m_edges[facet.m_edgeIds[1]],
			&meshA->m_edges[facet.m_edgeIds[2]],
		};
		
		PfxUInt32 edgeChk = 
			((edge[0]->m_angleType==SCE_PFX_EDGE_CONVEX)?0x00:0x01) |
			((edge[1]->m_angleType==SCE_PFX_EDGE_CONVEX)?0x00:0x02) |
			((edge[2]->m_angleType==SCE_PFX_EDGE_CONVEX)?0x00:0x04);

		pfxContactTriangleCylinder(localContacts,selFacets[f],
							facetNormal,facetPntsA[0],facetPntsA[1],facetPntsA[2],
							facet.m_thickness,
							0.5f*SCE_PFX_PI*(edge[0]->m_tilt/255.0f),
							0.5f*SCE_PFX_PI*(edge[1]->m_tilt/255.0f),
							0.5f*SCE_PFX_PI*(edge[2]->m_tilt/255.0f),
							edgeChk,cylinderB);
	}
	
	for(int i=0;i<localContacts.getNumContacts();i++) {
		PfxSubData subData = localContacts.getSubData(i);
		
		const PfxFacet &facet = meshA->m_facets[subData.getFacetId()];
		
		PfxTriangle triangleA(
			meshA->m_verts[facet.m_vertIds[0]],
			meshA->m_verts[facet.m_vertIds[1]],
			meshA->m_verts[facet.m_vertIds[2]]);
		
		PfxFloat s=0.0f,t=0.0f;
		pfxGetLocalCoords(PfxVector3(localContacts.getLocalPointA(i)),triangleA,s,t);
		subData.m_type = PfxSubData::MESH_INFO;
		subData.setFacetLocalS(s);
		subData.setFacetLocalT(t);
		
		contacts.addContactPoint(
			localContacts.getDistance(i),
			transformB.getUpper3x3() * localContacts.getNormal(i),
			transformAB * localContacts.getLocalPointA(i),
			localContacts.getLocalPointB(i),
			subData);
	}
	
	return contacts.getNumContacts();
}
} //namespace PhysicsEffects
} //namespace sce
