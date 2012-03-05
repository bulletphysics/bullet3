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

#ifndef _SCE_PFX_LARGE_TRI_MESH_H
#define _SCE_PFX_LARGE_TRI_MESH_H

#include "pfx_tri_mesh.h"

namespace sce {
namespace PhysicsEffects {

#define SCE_PFX_MAX_LARGETRIMESH_ISLANDS 256

///////////////////////////////////////////////////////////////////////////////
// Large Mesh

class SCE_PFX_ALIGNED(16) PfxLargeTriMesh
{
public:
	//J ラージメッシュのサイズ
	//E Size of a large mesh
	PfxVector3 m_half;

	//J 含まれるメッシュの総数
	//E Number of mesh groups
	PfxUInt16 m_numIslands;

	//J アイランドAABB配列（ソート済み）
	//E Array of island AABB (Sorted)
	PfxUInt8 m_axis; // 0 = X , 1 = Y , 2 = Z
	SCE_PFX_PADDING(1,1)
	PfxAabb16 *m_aabbList;
	SCE_PFX_PADDING(2,4)

	//J アイランド配列
	//E Array of island
	PfxTriMesh *m_islands;

	PfxLargeTriMesh()
	{
		m_numIslands = 0;
		m_islands = NULL;
		m_aabbList = NULL;
	}
	
	inline bool testAABB(int islandId,const PfxVector3 &center,const PfxVector3 &half) const;
	
	//J ワールド座標値をラージメッシュローカルに変換する
	//E Convert a position in the world coordinate into a position in the local coordinate
	inline PfxVecInt3 getLocalPosition(const PfxVector3 &worldPosition) const;
	inline void getLocalPosition(
		const PfxVector3 &worldMinPosition,const PfxVector3 &worldMaxPosition,
		PfxVecInt3 &localMinPosition,PfxVecInt3 &localMaxPosition) const;
	
	//J ラージメッシュローカル座標値をワールドに変換する
	//E Convert a position in the local coordinate into a position in the world coordinate
	inline PfxVector3 getWorldPosition(const PfxVecInt3 &localPosition) const;
};

inline
bool PfxLargeTriMesh::testAABB(int islandId,const PfxVector3 &center,const PfxVector3 &half) const 
{
	PfxVecInt3 aabbMinL = getLocalPosition(center-half);
	PfxVecInt3 aabbMaxL = getLocalPosition(center+half);
	
	if(aabbMaxL.getX() < pfxGetXMin(m_aabbList[islandId]) || aabbMinL.getX() > pfxGetXMax(m_aabbList[islandId])) return false;
	if(aabbMaxL.getY() < pfxGetYMin(m_aabbList[islandId]) || aabbMinL.getY() > pfxGetYMax(m_aabbList[islandId])) return false;
	if(aabbMaxL.getZ() < pfxGetZMin(m_aabbList[islandId]) || aabbMinL.getZ() > pfxGetZMax(m_aabbList[islandId])) return false;
	
	return true;
}

inline
PfxVecInt3 PfxLargeTriMesh::getLocalPosition(const PfxVector3 &worldPosition) const 
{
	const PfxVector3 sz(65535.0f);
	PfxVector3 tmp = divPerElem(worldPosition+m_half,2.0f*m_half);
	tmp = mulPerElem(sz,minPerElem(maxPerElem(tmp,PfxVector3(0.0f)),PfxVector3(1.0f))); // clamp 0.0 - 1.0
	return PfxVecInt3(tmp);
}

inline
void PfxLargeTriMesh::getLocalPosition(
		const PfxVector3 &worldMinPosition,const PfxVector3 &worldMaxPosition,
		PfxVecInt3 &localMinPosition,PfxVecInt3 &localMaxPosition) const
{
	const PfxVector3 sz(65535.0f);
	PfxVector3 qmin = divPerElem(worldMinPosition+m_half,2.0f*m_half);
	qmin = mulPerElem(sz,minPerElem(maxPerElem(qmin,PfxVector3(0.0f)),PfxVector3(1.0f))); // clamp 0.0 - 1.0

	PfxVector3 qmax = divPerElem(worldMaxPosition+m_half,2.0f*m_half);
	qmax = mulPerElem(sz,minPerElem(maxPerElem(qmax,PfxVector3(0.0f)),PfxVector3(1.0f))); // clamp 0.0 - 1.0

localMinPosition = PfxVecInt3(floorf(qmin[0]),floorf(qmin[1]),floorf(qmin[2]));
localMaxPosition = PfxVecInt3(ceilf(qmax[0]),ceilf(qmax[1]),ceilf(qmax[2]));
}

inline
PfxVector3 PfxLargeTriMesh::getWorldPosition(const PfxVecInt3 &localPosition) const 
{
	PfxVector3 sz(65535.0f),lp(localPosition);
	PfxVector3 tmp = divPerElem(lp,sz);
	return mulPerElem(tmp,2.0f*m_half) - m_half;
}

} // namespace PhysicsEffects
} // namespace sce

#endif // _SCE_PFX_LARGE_TRI_MESH_H