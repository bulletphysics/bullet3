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

#ifndef _SCE_PFX_INTERSECT_RAY_FUNC_H
#define _SCE_PFX_INTERSECT_RAY_FUNC_H

#include "../../../include/physics_effects/base_level/collision/pfx_ray.h"

namespace sce {
namespace PhysicsEffects {

typedef PfxBool (*PfxIntersectRayFunc)(
				const PfxRayInput &ray,PfxRayOutput &out,
				const PfxShape &shape,const PfxTransform3 &transform);

PfxIntersectRayFunc pfxGetIntersectRayFunc(PfxUInt8 shapeType);

PfxInt32 pfxSetIntersectRayFunc(PfxUInt8 shapeType,PfxIntersectRayFunc func);

} //namespace PhysicsEffects
} //namespace sce

#endif /* _SCE_PFX_INTERSECT_RAY_FUNC_H */
