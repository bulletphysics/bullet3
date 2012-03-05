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

#ifndef _SCE_PFX_GJK_SUPPORT_FUNC_H
#define _SCE_PFX_GJK_SUPPORT_FUNC_H

namespace sce {
namespace PhysicsEffects {

#include "../../../include/physics_effects/base_level/base/pfx_common.h"

void pfxGetSupportVertexTriangle(void *shape,const PfxVector3 &seperatingAxis,PfxVector3 &supportVertex);
void pfxGetSupportVertexTriangleWithThickness(void *shape,const PfxVector3 &seperatingAxis,PfxVector3 &supportVertex);
void pfxGetSupportVertexConvex(void *shape,const PfxVector3 &seperatingAxis,PfxVector3 &supportVertex);
void pfxGetSupportVertexBox(void *shape,const PfxVector3 &seperatingAxis,PfxVector3 &supportVertex);
void pfxGetSupportVertexCapsule(void *shape,const PfxVector3 &seperatingAxis,PfxVector3 &supportVertex);
void pfxGetSupportVertexSphere(void *shape,const PfxVector3 &seperatingAxis,PfxVector3 &supportVertex);
void pfxGetSupportVertexCylinder(void *shape,const PfxVector3 &seperatingAxis,PfxVector3 &supportVertex);

} //namespace PhysicsEffects
} //namespace sce

#endif // _SCE_PFX_GJK_SUPPORT_FUNC_H
