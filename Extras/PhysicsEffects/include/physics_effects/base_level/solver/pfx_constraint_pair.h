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

#ifndef _SCE_PFX_CONSTRAINT_PAIR_H
#define _SCE_PFX_CONSTRAINT_PAIR_H

#include "../sort/pfx_sort_data.h"
#include "../broadphase/pfx_broadphase_pair.h"

namespace sce {
namespace PhysicsEffects {

typedef PfxSortData16 PfxConstraintPair;

//J	PfxBroadphasePairと共通
//E Same as PfxBroadphasePair

SCE_PFX_FORCE_INLINE void pfxSetConstraintId(PfxConstraintPair &pair,PfxUInt32 i)	{pair.set32(2,i);}
SCE_PFX_FORCE_INLINE void pfxSetNumConstraints(PfxConstraintPair &pair,PfxUInt8 n)	{pair.set8(7,n);}

SCE_PFX_FORCE_INLINE PfxUInt32 pfxGetConstraintId(const PfxConstraintPair &pair)	{return pair.get32(2);}
SCE_PFX_FORCE_INLINE PfxUInt8  pfxGetNumConstraints(const PfxConstraintPair &pair)	{return pair.get8(7);}

} //namespace PhysicsEffects
} //namespace sce
#endif // _SCE_PFX_CONSTRAINT_PAIR_H
