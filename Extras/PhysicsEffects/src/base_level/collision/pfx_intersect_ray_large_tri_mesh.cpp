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

#include "../../../include/physics_effects/base_level/collision/pfx_large_tri_mesh.h"
#include "pfx_intersect_common.h"
#include "pfx_mesh_common.h"
#include "pfx_intersect_ray_large_tri_mesh.h"


namespace sce {
namespace PhysicsEffects {


static SCE_PFX_FORCE_INLINE
PfxBool pfxIntersectRayTriMesh(const PfxTriMesh &mesh,const PfxVector3 &rayStart,const PfxVector3 &rayDir,
	PfxUInt32 facetMode,PfxFloat &variable,PfxVector3 &normal,PfxSubData &subData)
{
	bool ret = false;
	PfxFloat nearest_t = variable;
	PfxVector3 nearest_nml(0.0f);
	PfxUInt32 nearest_f = 0;

	for(PfxUInt32 f=0;f<mesh.m_numFacets;f++) {
		const PfxFacet &facet = mesh.m_facets[f];
		
		PfxVector3 facetCenter = pfxReadVector3(facet.m_center);
		PfxVector3 facetHalf = pfxReadVector3(facet.m_half);

		PfxFloat cur_t = 1.0f;

		if( !pfxIntersectRayAABBFast(rayStart,rayDir,facetCenter,facetHalf,cur_t) )
			continue;
		
		if( nearest_t <= cur_t ) continue;
		
		PfxTriangle triangle(
			mesh.m_verts[facet.m_vertIds[0]],
			mesh.m_verts[facet.m_vertIds[1]],
			mesh.m_verts[facet.m_vertIds[2]]);
		
		if(facetMode == SCE_PFX_RAY_FACET_MODE_FRONT_ONLY && 
			pfxIntersectRayTriangleWithoutBackFace(rayStart,rayDir,triangle,cur_t) && cur_t < nearest_t) {
			nearest_f = f;
			nearest_t = cur_t;
			nearest_nml = pfxReadVector3(facet.m_normal);
			ret = true;
		}
		else if(facetMode == SCE_PFX_RAY_FACET_MODE_BACK_ONLY && 
			pfxIntersectRayTriangleWithoutFrontFace(rayStart,rayDir,triangle,cur_t) && cur_t < nearest_t) {
			nearest_f = f;
			nearest_t = cur_t;
			nearest_nml = pfxReadVector3(facet.m_normal);
			ret = true;
		}
		else if(facetMode == SCE_PFX_RAY_FACET_MODE_FRONT_AND_BACK && 
			pfxIntersectRayTriangle(rayStart,rayDir,triangle,cur_t) && cur_t < nearest_t) {
			nearest_f = f;
			nearest_t = cur_t;
			nearest_nml = pfxReadVector3(facet.m_normal);
			ret = true;
		}
	}
	
	if(ret) {
		// 面のローカル座標を算出
		const PfxFacet &facet = mesh.m_facets[nearest_f];

		PfxTriangle triangle(
			mesh.m_verts[facet.m_vertIds[0]],
			mesh.m_verts[facet.m_vertIds[1]],
			mesh.m_verts[facet.m_vertIds[2]]);
		
		PfxFloat s=0.0f,t=0.0f;
		pfxGetLocalCoords(rayStart+t*rayDir,triangle,s,t);
		
		subData.m_type = PfxSubData::MESH_INFO;
		subData.setFacetLocalS(s);
		subData.setFacetLocalT(t);
		subData.setFacetId(nearest_f);
		
		variable = nearest_t;
		normal = nearest_nml;
	}

	return ret;
}

PfxBool pfxIntersectRayLargeTriMesh(const PfxRayInput &ray,PfxRayOutput &out,const void *shape,const PfxTransform3 &transform)
{
	PfxBool ret = false;

	const PfxLargeTriMesh &largeMesh = *((PfxLargeTriMesh*)shape);
	
	// レイをラージメッシュのローカル座標へ変換
	PfxTransform3 transformLMesh = orthoInverse(transform);
	PfxVector3 rayStartPosition = transformLMesh.getUpper3x3() * ray.m_startPosition + transformLMesh.getTranslation();
	PfxVector3 rayDirection = transformLMesh.getUpper3x3() * ray.m_direction;
	
	PfxVecInt3 s,e,aabbMinL,aabbMaxL;

	s = largeMesh.getLocalPosition(rayStartPosition);
	e = largeMesh.getLocalPosition(rayStartPosition+rayDirection);

	aabbMinL = minPerElem(s,e);
	aabbMaxL = maxPerElem(s,e);
	
	PfxUInt32 numIslands = largeMesh.m_numIslands;
	
	{
	for(PfxUInt32 i=0;i<numIslands;i++) {
		PfxAabb16 aabbB = largeMesh.m_aabbList[i];
		if(aabbMaxL.getX() < pfxGetXMin(aabbB) || aabbMinL.getX() > pfxGetXMax(aabbB)) continue;
		if(aabbMaxL.getY() < pfxGetYMin(aabbB) || aabbMinL.getY() > pfxGetYMax(aabbB)) continue;
		if(aabbMaxL.getZ() < pfxGetZMin(aabbB) || aabbMinL.getZ() > pfxGetZMax(aabbB)) continue;
		// Todo:早期終了チェック

			PfxVector3 aabbMin,aabbMax;
			aabbMin = largeMesh.getWorldPosition(PfxVecInt3((PfxFloat)pfxGetXMin(aabbB),(PfxFloat)pfxGetYMin(aabbB),(PfxFloat)pfxGetZMin(aabbB)));
			aabbMax = largeMesh.getWorldPosition(PfxVecInt3((PfxFloat)pfxGetXMax(aabbB),(PfxFloat)pfxGetYMax(aabbB),(PfxFloat)pfxGetZMax(aabbB)));

			PfxFloat tmpVariable = 1.0f;
			PfxVector3 tmpNormal;

			if( !pfxIntersectRayAABBFast(
				rayStartPosition,rayDirection,
				(aabbMax+aabbMin)*0.5f,
				(aabbMax-aabbMin)*0.5f,
				tmpVariable) )
				continue;
			
			if( out.m_variable <= tmpVariable ) continue;

			// アイランドとの交差チェック
		const PfxTriMesh *island = &largeMesh.m_islands[i];
			
			PfxSubData subData;
			tmpVariable = out.m_variable;

			if( pfxIntersectRayTriMesh(*island,rayStartPosition,rayDirection,ray.m_facetMode,tmpVariable,tmpNormal,subData) &&
				tmpVariable < out.m_variable ) {
				out.m_contactFlag = true;
				out.m_variable = tmpVariable;
				out.m_contactPoint = ray.m_startPosition + tmpVariable * ray.m_direction;
				out.m_contactNormal = transform.getUpper3x3() * tmpNormal;
				subData.setIslandId(i);
				out.m_subData = subData;
				ret = true;
			}

		}
	}

	return ret;
}
} //namespace PhysicsEffects
} //namespace sce
