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
#include "pfx_gjk_support_func.h"
#include "../../../include/physics_effects/base_level/collision/pfx_tri_mesh.h"
#include "../../../include/physics_effects/base_level/collision/pfx_box.h"
#include "../../../include/physics_effects/base_level/collision/pfx_capsule.h"
#include "../../../include/physics_effects/base_level/collision/pfx_cylinder.h"
#include "../../../include/physics_effects/base_level/collision/pfx_sphere.h"

namespace sce {
namespace PhysicsEffects {

#define SCE_PFX_GJK_MARGIN 0.025f

///////////////////////////////////////////////////////////////////////////////
// Support Function

void pfxGetSupportVertexTriangle(void *shape,const PfxVector3 &seperatingAxis,PfxVector3 &supportVertex)
{
	PfxVector3 *vtx = (PfxVector3*)shape;
	
PfxFloat d0 = dot(vtx[0],seperatingAxis);
PfxFloat d1 = dot(vtx[1],seperatingAxis);
PfxFloat d2 = dot(vtx[2],seperatingAxis);

int reti = 2;

if(d0 > d1 && d0 > d2) {
	reti = 0;
}
else if(d1 > d2) {
	reti = 1;
}

	supportVertex = vtx[reti] + SCE_PFX_GJK_MARGIN * normalize(seperatingAxis);
}

void pfxGetSupportVertexTriangleWithThickness(void *shape,const PfxVector3 &seperatingAxis,PfxVector3 &supportVertex)
{
	PfxVector3 *vtx = (PfxVector3*)shape;
	
PfxFloat d[6];
d[0] = dot(vtx[0],seperatingAxis);
d[1] = dot(vtx[1],seperatingAxis);
d[2] = dot(vtx[2],seperatingAxis);
d[3] = dot(vtx[3],seperatingAxis);
d[4] = dot(vtx[4],seperatingAxis);
d[5] = dot(vtx[5],seperatingAxis);

int reti = 0;
for(int i=1;i<6;i++) {
	if(d[reti] < d[i]) {
		reti = i;
	}
}

	supportVertex = vtx[reti] + SCE_PFX_GJK_MARGIN * normalize(seperatingAxis);
}

void pfxGetSupportVertexConvex(void *shape,const PfxVector3 &seperatingAxis,PfxVector3 &supportVertex)
{
	PfxConvexMesh *mesh = (PfxConvexMesh*)shape;
	int reti = 0;
PfxFloat dmax = dot(mesh->m_verts[0],seperatingAxis);
for(int i=1;i<mesh->m_numVerts;i++) {
	PfxFloat d = dot(mesh->m_verts[i],seperatingAxis);
	if(d > dmax) {
		dmax =d;
		reti = i;
	}
}
	supportVertex = mesh->m_verts[reti] + SCE_PFX_GJK_MARGIN * normalize(seperatingAxis);
}

void pfxGetSupportVertexBox(void *shape,const PfxVector3 &seperatingAxis,PfxVector3 &supportVertex)
{
	PfxBox *box = (PfxBox*)shape;
	PfxVector3 boxHalf = box->m_half + PfxVector3(SCE_PFX_GJK_MARGIN);
	supportVertex[0] = seperatingAxis[0]>0.0f?boxHalf[0]:-boxHalf[0];
	supportVertex[1] = seperatingAxis[1]>0.0f?boxHalf[1]:-boxHalf[1];
	supportVertex[2] = seperatingAxis[2]>0.0f?boxHalf[2]:-boxHalf[2];
}

void pfxGetSupportVertexCapsule(void *shape,const PfxVector3 &seperatingAxis,PfxVector3 &supportVertex)
{
	PfxCapsule *capsule = (PfxCapsule*)shape;
	PfxVector3 u(1.0f,0.0f,0.0f);

	PfxFloat udotv = dot(seperatingAxis,u);
	PfxVector3 dir = u * (udotv > 0.0f ? capsule->m_halfLen : -capsule->m_halfLen);
	supportVertex = dir + normalize(seperatingAxis) * (capsule->m_radius + SCE_PFX_GJK_MARGIN);
}

void pfxGetSupportVertexSphere(void *shape,const PfxVector3 &seperatingAxis,PfxVector3 &supportVertex)
{
	PfxSphere *sphere = (PfxSphere*)shape;
	supportVertex = normalize(seperatingAxis) * (sphere->m_radius + SCE_PFX_GJK_MARGIN);
}

void pfxGetSupportVertexCylinder(void *shape,const PfxVector3 &seperatingAxis,PfxVector3 &supportVertex)
{
	PfxCylinder *cylinder = (PfxCylinder*)shape;
	PfxVector3 u(1.0f,0.0f,0.0f);

	PfxFloat udotv = dot(seperatingAxis,u);
	PfxFloat s = seperatingAxis[1]*seperatingAxis[1]+seperatingAxis[2]*seperatingAxis[2];
	if(s < 0.000001f) {
		supportVertex = u * (udotv > 0.0f ? cylinder->m_halfLen + SCE_PFX_GJK_MARGIN : -cylinder->m_halfLen-SCE_PFX_GJK_MARGIN);
	}
	else {
		PfxVector3 dir = u * (udotv > 0.0f ? cylinder->m_halfLen : -cylinder->m_halfLen);
		PfxVector3 vYZ = seperatingAxis;
		vYZ[0] = 0.0f;
		vYZ /= sqrtf(s);
		supportVertex = dir + vYZ * (cylinder->m_radius) + normalize(seperatingAxis) * SCE_PFX_GJK_MARGIN;
	}
}
} //namespace PhysicsEffects
} //namespace sce
