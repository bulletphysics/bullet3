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

#ifndef _SCE_PFX_TRI_MESH_H
#define _SCE_PFX_TRI_MESH_H

#include "../base/pfx_common.h"
#include "../base/pfx_vec_utils.h"
#include "pfx_aabb.h"

namespace sce {
namespace PhysicsEffects {

//J メッシュのリソース制限
//E Define some limitations of a triangle mesh
#define SCE_PFX_NUMMESHFACETS		64
#define SCE_PFX_NUMMESHEDGES		192
#define SCE_PFX_NUMMESHVERTICES		128

//J エッジの角
//E Edge types
#define SCE_PFX_EDGE_FLAT    0
#define SCE_PFX_EDGE_CONVEX  1
#define SCE_PFX_EDGE_CONCAVE 2

///////////////////////////////////////////////////////////////////////////////
// Edge
struct PfxEdge
{
	PfxUInt8 m_vertId[2];
	PfxUInt8 m_angleType;
	PfxUInt8 m_tilt;
};

inline
bool operator ==(const PfxEdge &e1,const PfxEdge &e2)
{
    return  (e1.m_vertId[0] == e2.m_vertId[0] && e1.m_vertId[1] == e2.m_vertId[1]) || 
			(e1.m_vertId[1] == e2.m_vertId[0] && e1.m_vertId[0] == e2.m_vertId[1]);
}

///////////////////////////////////////////////////////////////////////////////
// Facet

struct PfxFacet
{
	PfxFloat m_normal[3];
	PfxFloat m_thickness;
	PfxUInt8 m_group;
	PfxUInt8 m_vertIds[3];
	PfxUInt8 m_edgeIds[3];
	PfxUInt8 m_userData;
	PfxFloat m_half[3];
	PfxFloat m_center[3];
};

///////////////////////////////////////////////////////////////////////////////
// Mesh

struct SCE_PFX_ALIGNED(16) PfxTriMesh
{
	PfxUInt8 m_numVerts;
	PfxUInt8 m_numEdges;
	PfxUInt8 m_numFacets;
	PfxUInt8 m_reserved;
	PfxFacet m_facets[SCE_PFX_NUMMESHFACETS];
	PfxEdge  m_edges[SCE_PFX_NUMMESHEDGES];
	SCE_PFX_PADDING(1,12)
	PfxVector3 m_verts[SCE_PFX_NUMMESHVERTICES];
	PfxVector3 m_half;
	
	PfxTriMesh()
	{
		m_numVerts = m_numEdges = m_numFacets = 0;
	}
	
	inline void updateAABB();
};

inline
void PfxTriMesh::updateAABB()
{
	PfxVector3 halfMax(0);
	
	for(PfxUInt8 i=0;i<m_numFacets;i++) {
		PfxVector3 pnts[6] = {
			m_verts[m_facets[i].m_vertIds[0]],
			m_verts[m_facets[i].m_vertIds[1]],
			m_verts[m_facets[i].m_vertIds[2]],
			m_verts[m_facets[i].m_vertIds[0]] - m_facets[i].m_thickness * pfxReadVector3(m_facets[i].m_normal),
			m_verts[m_facets[i].m_vertIds[1]] - m_facets[i].m_thickness * pfxReadVector3(m_facets[i].m_normal),
			m_verts[m_facets[i].m_vertIds[2]] - m_facets[i].m_thickness * pfxReadVector3(m_facets[i].m_normal)
		};
		
		PfxVector3 facetAABBmin,facetAABBmax,facetHalf,facetCenter;
		facetAABBmin = minPerElem(pnts[5],minPerElem(pnts[4],minPerElem(pnts[3],minPerElem(pnts[2],minPerElem(pnts[0],pnts[1])))));
		facetAABBmax = maxPerElem(pnts[5],maxPerElem(pnts[4],maxPerElem(pnts[3],maxPerElem(pnts[2],maxPerElem(pnts[0],pnts[1])))));
		facetHalf = 0.5f * (facetAABBmax - facetAABBmin) + PfxVector3(0.00001f); // Slightly stretch to avoid collision hole
		facetCenter = 0.5f * (facetAABBmax + facetAABBmin);
		pfxStoreVector3(facetHalf,m_facets[i].m_half);
		pfxStoreVector3(facetCenter,m_facets[i].m_center);
		halfMax = maxPerElem(absPerElem(facetAABBmax),halfMax);
	}
	m_half = halfMax;
}

///////////////////////////////////////////////////////////////////////////////
// Convex Mesh

struct SCE_PFX_ALIGNED(16) PfxConvexMesh
{
	PfxUInt8 m_numVerts;
	PfxUInt8 m_numIndices;
	PfxUInt16 m_indices[SCE_PFX_NUMMESHFACETS*3];
	SCE_PFX_PADDING(1,14)
	PfxVector3 m_verts[SCE_PFX_NUMMESHVERTICES];
	PfxVector3 m_half;
	SCE_PFX_PADDING(2,16)

	PfxConvexMesh()
	{
		m_numVerts = m_numIndices = 0;
	}
	
	inline void updateAABB();
};

inline
void PfxConvexMesh::updateAABB()
{
	PfxVector3 halfMax(0);
	for(int i=0;i<m_numVerts;i++) {
		halfMax = maxPerElem(absPerElem(m_verts[i]),halfMax);
	}
	m_half = halfMax;
}

} // namespace PhysicsEffects
} // namespace sce

#endif // _SCE_PFX_TRI_MESH_H
