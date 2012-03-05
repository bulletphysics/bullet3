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

#ifndef _SCE_PFX_BROADPHASE_PROXY_H
#define _SCE_PFX_BROADPHASE_PROXY_H

namespace sce {
namespace PhysicsEffects {

typedef PfxSortData32 PfxBroadphaseProxy;

//J	AABBパラメータはPfxAabbと共通
//E PfxBroadphaseProxy shares AABB parameters with PfxAabb32

SCE_PFX_FORCE_INLINE void pfxSetObjectId(PfxBroadphaseProxy &proxy,PfxUInt16 i)	   {proxy.set16(6,i);}
SCE_PFX_FORCE_INLINE void pfxSetMotionMask(PfxBroadphaseProxy &proxy,PfxUInt8 i)   {proxy.set8(14,i);}
SCE_PFX_FORCE_INLINE void pfxSetProxyFlag(PfxBroadphaseProxy &proxy,PfxUInt8 i)    {proxy.set8(15,i);}
SCE_PFX_FORCE_INLINE void pfxSetSelf(PfxBroadphaseProxy &proxy,PfxUInt32 i)        {proxy.set32(5,i);}
SCE_PFX_FORCE_INLINE void pfxSetTarget(PfxBroadphaseProxy &proxy,PfxUInt32 i)      {proxy.set32(6,i);}

SCE_PFX_FORCE_INLINE PfxUInt16 pfxGetObjectId(const PfxBroadphaseProxy &proxy)     {return proxy.get16(6);}
SCE_PFX_FORCE_INLINE PfxUInt8  pfxGetMotionMask(const PfxBroadphaseProxy &proxy)   {return proxy.get8(14);}
SCE_PFX_FORCE_INLINE PfxUInt8  pfxGetProxyFlag(const PfxBroadphaseProxy &proxy)	   {return proxy.get8(15);}
SCE_PFX_FORCE_INLINE PfxUInt32 pfxGetSelf(const PfxBroadphaseProxy &proxy)		   {return proxy.get32(5);}
SCE_PFX_FORCE_INLINE PfxUInt32 pfxGetTarget(const PfxBroadphaseProxy &proxy)	   {return proxy.get32(6);}

} //namespace PhysicsEffects
} //namespace sce
#endif // _SCE_PFX_BROADPHASE_PROXY_H
