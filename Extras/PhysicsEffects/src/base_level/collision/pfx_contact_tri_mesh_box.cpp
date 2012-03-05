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
#include "pfx_contact_tri_mesh_box.h"
#include "pfx_intersect_common.h"
#include "pfx_mesh_common.h"

namespace sce {
namespace PhysicsEffects {

static SCE_PFX_FORCE_INLINE
bool checkSAT(const PfxVector3 &axis,PfxFloat AMin,PfxFloat AMax,PfxFloat BMin,PfxFloat BMax,PfxFloat &distMin,PfxVector3 &axisMin)
{
	// ■ 非接触.

	// A:          +----+
	// B:+----+
	if(BMax <= AMin) {
		return true;
	}

	// A:+----+
	// B:          +----+
	else if(AMax <= BMin) {
		return true;
	}

	// ■ 内包

	//A:      +--+
	//B:   +------+
	if(BMin < AMin && AMax < BMax) {
		PfxFloat d = AMin-BMax;
		if(distMin < d) {
			distMin = d;
			axisMin = axis;
		}
	}

	//A:   +------+
	//B:    +--+
	else if(AMin < BMin && BMax < AMax) {
		PfxFloat d = AMin-BMax;
		if(distMin < d) {
			distMin = d;
			axisMin = axis;
		}
	}

	// ■ 接触
	
	// A:   +----+
	// B:+----+
	else if(BMin < AMin && AMin < BMax) {
		PfxFloat d = AMin-BMax;
		if(distMin < d) {
			distMin = d;
			axisMin = axis;
		}
	}

	// A:   +----+
	// B:      +----+
	else if(AMin < BMin && BMin < AMax) {
		PfxFloat d = BMin-AMax;
		if(distMin < d) {
			distMin = d;
			axisMin = -axis;
		}
	}

	return false;
}

static SCE_PFX_FORCE_INLINE
bool pfxContactTriangleBox(PfxContactCache &contacts,PfxUInt32 facetId,
							const PfxVector3 &normal,const PfxVector3 &p0,const PfxVector3 &p1,const PfxVector3 &p2,
							const PfxFloat thickness,const PfxFloat angle0,const PfxFloat angle1,const PfxFloat angle2,
							PfxUInt32 edgeChk,
							const PfxVector3 &boxHalf)
{
	const PfxFloat epsilon = 0.00001f;
	
	// 最も浅い貫通深度とそのときの分離軸
	PfxFloat distMin = -SCE_PFX_FLT_MAX;
	PfxVector3 axisMin(0.0f);

	//-------------------------------------------
	// １．分離軸判定
	{
		PfxVector3 facetPnts[6] = {
			p0,p1,p2,p0-thickness*normal,p1-thickness*normal,p2-thickness*normal
		};

		PfxVector3 sideNml[3] = {
			normalize(cross((facetPnts[1] - facetPnts[0]),normal)),
			normalize(cross((facetPnts[2] - facetPnts[1]),normal)),
			normalize(cross((facetPnts[0] - facetPnts[2]),normal)),
		};


		// Trianglesの面 -> Box
		{
			// 分離軸
			const PfxVector3 sepAxis = normal;

			// 分離平面
			PfxPlane planeA(sepAxis,p0);
			
			// Boxを分離軸に投影して範囲を取得
			PfxFloat r = dot(boxHalf,absPerElem(sepAxis));
			PfxFloat boxOffset = planeA.onPlane(PfxVector3(0.0f));
			PfxFloat boxMax = boxOffset + r;
			PfxFloat boxMin = boxOffset - r;
			
			// 判定
			if(boxMin > 0.0f || boxMax < -thickness) {
				return false;
			}

			if(distMin < boxMin) {
				distMin = boxMin;
				axisMin = -sepAxis;
			}
		}

		// Box -> Triangles
		for(int bf=0;bf<3;bf++) {
			// 分離軸
			PfxVector3 sepAxis(0.0f);
			sepAxis[bf] = 1.0f;

			// 分離軸の方向をチェック
			if(dot(normal,sepAxis) > 0.0f)
				sepAxis = -sepAxis;

			// Trianglesを分離軸に投影して範囲を取得
			PfxFloat triMin,triMax;
			pfxGetProjAxisPnts6(facetPnts,sepAxis,triMin,triMax);

			// Boxを分離軸に投影して範囲を取得
			PfxFloat boxMin = -boxHalf[bf];
			PfxFloat boxMax =  boxHalf[bf];
			
			if(checkSAT(sepAxis,triMin,triMax,boxMin,boxMax,distMin,axisMin)) {
				return false;
			}
		}

		// エッジ Triangles面のエッジ(x3)×Boxのエッジ(x3)
		for(int e=0;e<3;e++) {
			PfxVector3 dir = normalize(facetPnts[(e+1)%3] - facetPnts[e]);

			for(int i=0;i<3;i++) {
				PfxVector3 boxEdge(0.0f);
				boxEdge[i] = 1.0f;
				
				// エッジが平行であれば判定しない
				if(pfxIsSameDirection(dir,boxEdge)) continue;

				PfxVector3 sepAxis = normalize(cross(dir,boxEdge));

				// 分離軸の方向をチェック
				if(dot(normal,sepAxis) > 0.0f)
					sepAxis = -sepAxis;

				// Trianglesを分離軸に投影して範囲を取得
				PfxFloat triMin,triMax;
				pfxGetProjAxisPnts6(facetPnts,sepAxis,triMin,triMax);

				// Boxを分離軸に投影して範囲を取得
				PfxFloat r = dot(boxHalf,absPerElem(sepAxis));
				PfxFloat boxMin = -r;
				PfxFloat boxMax = r;

				if(checkSAT(sepAxis,triMin,triMax,boxMin,boxMax,distMin,axisMin)) {
					return false;
				}
			}
		}

		// 面に厚みがある場合の補助的な判定（交差するかしないかだけを判定）
		if(thickness > 0.0f) {
			// 厚み側面の法線
			for(int i=0;i<3;i++) {
				// 分離平面
				PfxPlane planeA(sideNml[i],facetPnts[i]);

				// Boxを分離軸に投影して範囲を取得
				PfxFloat r = dot(boxHalf,absPerElem(sideNml[i]));
				PfxFloat boxOffset = planeA.onPlane(PfxVector3(0.0f));
				PfxFloat boxMin = boxOffset - r;

				if(boxMin > 0.0f) {
					return false;
				}
			}

			// ２つの厚み側面のなすエッジ3×ボックスのエッジ3
			for(int e=0;e<3;e++) {
				PfxVector3 edgeVec = normalize(cross(sideNml[(e+1)%3],sideNml[e]));

				for(int i=0;i<3;i++) {
					PfxVector3 boxEdge(0.0f);
					boxEdge[i] = 1.0f;
					
					// エッジが平行であれば判定しない
					if(pfxIsSameDirection(edgeVec,boxEdge)) continue;

					PfxVector3 sepAxis = normalize(cross(edgeVec,boxEdge));

					// Trianglesを分離軸に投影して範囲を取得
					PfxFloat triMin,triMax;
					pfxGetProjAxisPnts3(facetPnts,sepAxis,triMin,triMax);

					// Boxを分離軸に投影して範囲を取得
					PfxFloat r = dot(boxHalf,absPerElem(sepAxis));
					PfxFloat boxMin = -r;
					PfxFloat boxMax = r;

					if(triMax < boxMin || boxMax < triMin) {
						return false;
					}
				}
			}
		}
	}

	//-------------------------------------------
	// ２．衝突点の探索
	{
		// 分離軸方向に引き離す(最近接を判定するため、交差回避させる)
		PfxVector3 sepAxis = 1.1f * fabsf(distMin) * axisMin;
		
		const PfxVector3 facetPnts[3] = {
			p0 + sepAxis,
			p1 + sepAxis,
			p2 + sepAxis,
		};
		
		const PfxVector3 boxPnts[8] = {
			mulPerElem(PfxVector3(-1.0f,-1.0f,-1.0f),boxHalf),
			mulPerElem(PfxVector3(-1.0f,-1.0f, 1.0f),boxHalf),
			mulPerElem(PfxVector3( 1.0f,-1.0f, 1.0f),boxHalf),
			mulPerElem(PfxVector3( 1.0f,-1.0f,-1.0f),boxHalf),
			mulPerElem(PfxVector3(-1.0f, 1.0f,-1.0f),boxHalf),
			mulPerElem(PfxVector3(-1.0f, 1.0f, 1.0f),boxHalf),
			mulPerElem(PfxVector3( 1.0f, 1.0f, 1.0f),boxHalf),
			mulPerElem(PfxVector3( 1.0f, 1.0f,-1.0f),boxHalf),
		};
		
		//--------------------------------------------------------------------
		// 衝突点の探索
		
		PfxClosestPoints cp;
		PfxVector3 sA,sB;
		
		// エッジ間の最短距離と座標値を算出
		{
			const int boxIds[] = {
				0,1,
				1,2,
				2,3,
				3,0,
				4,5,
				5,6,
				6,7,
				7,4,
				0,4,
				3,7,
				2,6,
				1,5,
			};

			for(int i=0;i<3;i++) {
				for(int j=0;j<12;j++) {
					pfxClosestTwoLines(facetPnts[i],facetPnts[(i+1)%3],boxPnts[boxIds[j*2]],boxPnts[boxIds[j*2+1]],sA,sB);
					cp.add(PfxPoint3(sA),PfxPoint3(sB),lengthSqr(sA-sB));
				}
			}
		}
		
		// Triangleの頂点 -> Boxの面
		{
#ifdef SCE_PFX_USE_GEOMETRY
			PfxFloatInVec sqrDist;
			PfxGeomAabb aabb(PfxPoint3(0.0f),boxHalf);
			{
				PfxPoint3 closestPoint = sce::Geometry::Aos::closestPoint(PfxPoint3(facetPnts[0]),aabb,&sqrDist);
				cp.add(PfxPoint3(facetPnts[0]),closestPoint,sqrDist);
			}
			{
				PfxPoint3 closestPoint = sce::Geometry::Aos::closestPoint(PfxPoint3(facetPnts[1]),aabb,&sqrDist);
				cp.add(PfxPoint3(facetPnts[1]),closestPoint,sqrDist);
			}
			{
				PfxPoint3 closestPoint = sce::Geometry::Aos::closestPoint(PfxPoint3(facetPnts[2]),aabb,&sqrDist);
				cp.add(PfxPoint3(facetPnts[2]),closestPoint,sqrDist);
			}
#else
			pfxClosestPointAABB(facetPnts[0],boxHalf,sB);
			cp.add(PfxPoint3(facetPnts[0]),PfxPoint3(sB),lengthSqr(sB-facetPnts[0]));
			
			pfxClosestPointAABB(facetPnts[1],boxHalf,sB);
			cp.add(PfxPoint3(facetPnts[1]),PfxPoint3(sB),lengthSqr(sB-facetPnts[1]));
			
			pfxClosestPointAABB(facetPnts[2],boxHalf,sB);
			cp.add(PfxPoint3(facetPnts[2]),PfxPoint3(sB),lengthSqr(sB-facetPnts[2]));
#endif
		}

		// Boxの頂点 -> Trianglesの面
		PfxTriangle triangleA(facetPnts[0],facetPnts[1],facetPnts[2]);
		for(int i=0;i<8;i++) {
			pfxClosestPointTriangle(boxPnts[i],triangleA,sA);
			cp.add(PfxPoint3(sA),PfxPoint3(boxPnts[i]),lengthSqr(sA-boxPnts[i]));
		}
		
		for(int i=0;i<cp.numPoints;i++) {
			if(cp.distSqr[i] < cp.closestDistSqr + epsilon) {
				cp.pA[i] -= sepAxis;
				// 面上の最近接点が凸エッジ上でない場合は法線を変える
				if( ((edgeChk&0x01)&&pfxPointOnLine(PfxVector3(cp.pA[i]),p0,p1)) ||
					((edgeChk&0x02)&&pfxPointOnLine(PfxVector3(cp.pA[i]),p1,p2)) ||
					((edgeChk&0x04)&&pfxPointOnLine(PfxVector3(cp.pA[i]),p2,p0)) ) {
					axisMin=-normal;
				}
				
				PfxSubData subData;
				subData.setFacetId(facetId);
				contacts.addContactPoint(-length(cp.pB[i]-cp.pA[i]),axisMin,cp.pA[i],cp.pB[i],subData);
			}
		}
	}

	return true;
}

// 分離軸が見つかった場合はすぐに抜けるため最短距離が返されるわけではないことに注意
PfxInt32 pfxContactTriMeshBox(
	PfxContactCache &contacts,
	const PfxTriMesh *meshA,
	const PfxTransform3 &transformA,
	const PfxBox &boxB,
	const PfxTransform3 &transformB,
	PfxFloat distanceThreshold)
{
	(void) distanceThreshold;

	PfxTransform3 transformAB, transformBA;
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

	// ※boxB座標系
	PfxVector3 aabbB = boxB.m_half;
	numSelFacets = pfxGatherFacets(meshA,(PfxFloat*)&aabbB,offsetBA,matrixBA,selFacets);
	
	if(numSelFacets == 0) {
		return 0;
	}
	
	//-------------------------------------------
	// 分離軸判定(SAT)
	// ※Box座標系 (Bローカル)で判定
	
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

		pfxContactTriangleBox(localContacts,selFacets[f],
							facetNormal,facetPntsA[0],facetPntsA[1],facetPntsA[2],
							facet.m_thickness,
							0.5f*SCE_PFX_PI*(edge[0]->m_tilt/255.0f),
							0.5f*SCE_PFX_PI*(edge[1]->m_tilt/255.0f),
							0.5f*SCE_PFX_PI*(edge[2]->m_tilt/255.0f),
							edgeChk,boxB.m_half);
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
