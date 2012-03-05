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

#ifndef _SCE_PFX_DETECT_COLLISION_FUNC_H
#define _SCE_PFX_DETECT_COLLISION_FUNC_H

#include "../../base_level/collision/pfx_contact_cache.h"

namespace sce {
namespace PhysicsEffects {

typedef void (*pfx_detect_collision_func)(
				PfxContactCache &contacts,
				const PfxShape & shapeA,const PfxTransform3 &offsetTransformA,const PfxTransform3 &worldTransformA,int shapeIdA,
				const PfxShape & shapeB,const PfxTransform3 &offsetTransformB,const PfxTransform3 &worldTransformB,int shapeIdB,
				float contactThreshold);

pfx_detect_collision_func pfxGetDetectCollisionFunc(PfxUInt8 shapeTypeA,PfxUInt8 shapeTypeB);

int pfxSetDetectCollisionFunc(PfxUInt8 shapeTypeA,PfxUInt8 shapeTypeB,pfx_detect_collision_func func);

} //namespace PhysicsEffects
} //namespace sce
#endif // _SCE_PFX_DETECT_COLLISION_FUNC_H
