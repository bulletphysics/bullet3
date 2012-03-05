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
#include "pfx_intersect_common.h"
#include "pfx_mesh_common.h"

namespace sce {
namespace PhysicsEffects {

static SCE_PFX_FORCE_INLINE
bool pfxContactTriangleSphere(PfxContactCache &contacts,PfxUInt32 facetId,
	const PfxVector3 &normal,const PfxVector3 &p0,const PfxVector3 &p1,const PfxVector3 &p2,
	const PfxFloat thickness,const PfxFloat angle0,const PfxFloat angle1,const PfxFloat angle2,
	PfxUInt32 edgeChk,
	PfxFloat sphereRadius,const PfxVector3 &spherePos)
{
	PfxVector3 facetPnts[3] = {
		p0,p1,p2,
	};
	
	// 早期判定
	{
		PfxPlane planeA(normal,p0);
		PfxFloat len1 = planeA.onPlane(spherePos);
		
		if(len1 >= sphereRadius || len1 < -thickness-sphereRadius) return false;
		
	}

	// 球と面の最近接点を計算
	{
		PfxTriangle triangleA(p0,p1,p2);
		PfxVector3 pntA;
		// pfxClosestPointTriangle(spherePos,triangleA,pntA);
		bool insideTriangle = false;
		while(1) {
		    PfxVector3 ab = p1 - p0;
		    PfxVector3 ac = p2 - p0;
		    PfxVector3 ap = spherePos - p0;
		    PfxFloat d1 = dot(ab, ap);
		    PfxFloat d2 = dot(ac, ap);
			if(d1 <= 0.0f && d2 <= 0.0f) {
				pntA = p0;
				break;
			}

		    PfxVector3 bp = spherePos - p1;
		    PfxFloat d3 = dot(ab, bp);
		    PfxFloat d4 = dot(ac, bp);
			if (d3 >= 0.0f && d4 <= d3) {
				pntA = p1;
				break;
			}

		    PfxFloat vc = d1*d4 - d3*d2;
		    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
		        PfxFloat v = d1 / (d1 - d3);
		        pntA = p0 + v * ab;
				break;
		    }

		    PfxVector3 cp = spherePos - p2;
		    PfxFloat d5 = dot(ab, cp);
		    PfxFloat d6 = dot(ac, cp);
			if (d6 >= 0.0f && d5 <= d6) {
				pntA = p2;
				break;
			}

		    PfxFloat vb = d5*d2 - d1*d6;
		    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
		        PfxFloat w = d2 / (d2 - d6);
		        pntA = p0 + w * ac;
				break;
		    }

		    PfxFloat va = d3*d6 - d5*d4;
		    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
		        PfxFloat w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		        pntA = p1 + w * (p2 - p1);
				break;
		    }

		    PfxFloat den = 1.0f / (va + vb + vc);
		    PfxFloat v = vb * den;
		    PfxFloat w = vc * den;
		    pntA = p0 + ab * v + ac * w;
		    insideTriangle = true;
			break;
		}
		PfxVector3 distVec = pntA - spherePos;
		PfxFloat l = length(distVec);
		
		if(!insideTriangle && l >= sphereRadius) return false;
		
		// 分離軸
		PfxVector3 sepAxis = (l < 0.00001f || insideTriangle) ? -normal : distVec / l;

		// 球上の衝突点
		PfxVector3 pointsOnSphere = spherePos + sphereRadius * sepAxis;
		PfxVector3 pointsOnTriangle = pntA;

		// 面上の最近接点が凸エッジ上でない場合は法線を変える
		if( ((edgeChk&0x01)&&pfxPointOnLine(pointsOnTriangle,p0,p1)) ||
			((edgeChk&0x02)&&pfxPointOnLine(pointsOnTriangle,p1,p2)) ||
			((edgeChk&0x04)&&pfxPointOnLine(pointsOnTriangle,p2,p0)) ) {
			sepAxis=-normal;
		}

		PfxSubData subData;
		subData.setFacetId(facetId);
		contacts.addContactPoint(-length(pointsOnSphere-pointsOnTriangle),sepAxis,PfxPoint3(pointsOnTriangle),PfxPoint3(pointsOnSphere),subData);
	}

	return true;
}

PfxInt32 pfxContactTriMeshSphere(
	PfxContactCache &contacts,
	const PfxTriMesh *meshA,
	const PfxTransform3 &transformA,
	const PfxSphere &sphereB,
	const PfxTransform3 &transformB,
	PfxFloat distanceThreshold)
{
	(void) distanceThreshold;

	PfxTransform3 transformAB,transformBA;
	PfxMatrix3 matrixBA;
	PfxVector3 offsetBA;

	// Bローカル→Aローカルへの変換
	transformAB = orthoInverse(transformA) * transformB;

	// Aローカル→Bローカルへの変換
	transformBA = orthoInverse(transformAB);

	matrixBA = transformBA.getUpper3x3();
	offsetBA = transformBA.getTranslation();

	//-------------------------------------------
	// 判定する面を絞り込む

	PfxUInt8 SCE_PFX_ALIGNED(16) selFacets[SCE_PFX_NUMMESHFACETS] = {0};
	PfxUInt32 numSelFacets = 0;

	PfxVector3 aabbB(sphereB.m_radius);
	numSelFacets = pfxGatherFacets(meshA,(PfxFloat*)&aabbB,offsetBA,matrixBA,selFacets);

	if(numSelFacets == 0) {
		return 0;
	}

	//-----------------------------------------------
	// 判定

	PfxContactCache localContacts;

	// TriangleMeshの面->sphereの判定
	// ※TriangleMesh座標系
	{
		for(PfxUInt32 f = 0; f < numSelFacets; f++ ) {
			const PfxFacet &facet = meshA->m_facets[selFacets[f]];

			const PfxVector3 facetNormal = pfxReadVector3(facet.m_normal);

			const PfxVector3 facetPnts[3] = {
				meshA->m_verts[facet.m_vertIds[0]],
				meshA->m_verts[facet.m_vertIds[1]],
				meshA->m_verts[facet.m_vertIds[2]],
			};
			
			const PfxEdge *edge[3] = {
				&meshA->m_edges[facet.m_edgeIds[0]],
				&meshA->m_edges[facet.m_edgeIds[1]],
				&meshA->m_edges[facet.m_edgeIds[2]],
			};
			
			PfxVector3 sepAxis,pntA,pntB;
			
			PfxUInt32 edgeChk = 
				((edge[0]->m_angleType==SCE_PFX_EDGE_CONVEX)?0x00:0x01) |
				((edge[1]->m_angleType==SCE_PFX_EDGE_CONVEX)?0x00:0x02) |
				((edge[2]->m_angleType==SCE_PFX_EDGE_CONVEX)?0x00:0x04);
			
			pfxContactTriangleSphere(localContacts,selFacets[f],
									facetNormal,facetPnts[0],facetPnts[1],facetPnts[2],
									facet.m_thickness,
									0.5f*SCE_PFX_PI*(edge[0]->m_tilt/255.0f),
									0.5f*SCE_PFX_PI*(edge[1]->m_tilt/255.0f),
									0.5f*SCE_PFX_PI*(edge[2]->m_tilt/255.0f),
									edgeChk,
									sphereB.m_radius,transformAB.getTranslation());
		}
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
			transformA.getUpper3x3() * localContacts.getNormal(i),
			localContacts.getLocalPointA(i),
			transformBA * localContacts.getLocalPointB(i),
			subData);
	}

	return contacts.getNumContacts();
}
} //namespace PhysicsEffects
} //namespace sce
