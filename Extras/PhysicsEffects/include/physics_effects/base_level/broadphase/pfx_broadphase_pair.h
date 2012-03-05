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

#ifndef _SCE_PFX_BROADPHASE_PAIR_H
#define _SCE_PFX_BROADPHASE_PAIR_H

#include "../sort/pfx_sort_data.h"

namespace sce {
namespace PhysicsEffects {

typedef PfxSortData16 PfxBroadphasePair;

SCE_PFX_FORCE_INLINE void pfxSetObjectIdA(PfxBroadphasePair &pair,PfxUInt16 i)	{pair.set16(0,i);}
SCE_PFX_FORCE_INLINE void pfxSetObjectIdB(PfxBroadphasePair &pair,PfxUInt16 i)	{pair.set16(1,i);}
SCE_PFX_FORCE_INLINE void pfxSetMotionMaskA(PfxBroadphasePair &pair,PfxUInt8 i)		{pair.set8(4,i);}
SCE_PFX_FORCE_INLINE void pfxSetMotionMaskB(PfxBroadphasePair &pair,PfxUInt8 i)		{pair.set8(5,i);}
SCE_PFX_FORCE_INLINE void pfxSetBroadphaseFlag(PfxBroadphasePair &pair,PfxUInt8 f)	{pair.set8(6,(pair.get8(6)&0xf0)|(f&0x0f));}
SCE_PFX_FORCE_INLINE void pfxSetActive(PfxBroadphasePair &pair,PfxBool b)			{pair.set8(6,(pair.get8(6)&0x0f)|((b?1:0)<<4));}
SCE_PFX_FORCE_INLINE void pfxSetContactId(PfxBroadphasePair &pair,PfxUInt32 i)		{pair.set32(2,i);}

SCE_PFX_FORCE_INLINE PfxUInt16 pfxGetObjectIdA(const PfxBroadphasePair &pair)	{return pair.get16(0);}
SCE_PFX_FORCE_INLINE PfxUInt16 pfxGetObjectIdB(const PfxBroadphasePair &pair)	{return pair.get16(1);}
SCE_PFX_FORCE_INLINE PfxUInt8  pfxGetMotionMaskA(const PfxBroadphasePair &pair)		{return pair.get8(4);}
SCE_PFX_FORCE_INLINE PfxUInt8  pfxGetMotionMaskB(const PfxBroadphasePair &pair)		{return pair.get8(5);}
SCE_PFX_FORCE_INLINE PfxUInt8  pfxGetBroadphaseFlag(const PfxBroadphasePair &pair)	{return pair.get8(6)&0x0f;}
SCE_PFX_FORCE_INLINE PfxBool   pfxGetActive(const PfxBroadphasePair &pair)			{return (pair.get8(6)>>4)!=0;}
SCE_PFX_FORCE_INLINE PfxUInt32 pfxGetContactId(const PfxBroadphasePair &pair)		{return pair.get32(2);}

} //namespace PhysicsEffects
} //namespace sce

#endif // _SCE_PFX_BROADPHASE_PAIR_H
