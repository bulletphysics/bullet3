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

#ifndef _SCE_PFX_AABB_H
#define _SCE_PFX_AABB_H

#include "../sort/pfx_sort_data.h"

namespace sce {
namespace PhysicsEffects {

typedef PfxSortData16 PfxAabb16;
typedef PfxSortData32 PfxAabb32;

SCE_PFX_FORCE_INLINE void pfxSetXMin(PfxAabb16& aabb,PfxUInt16 i) {aabb.set16(0,i);}
SCE_PFX_FORCE_INLINE void pfxSetXMax(PfxAabb16& aabb,PfxUInt16 i) {aabb.set16(1,i);}
SCE_PFX_FORCE_INLINE void pfxSetYMin(PfxAabb16& aabb,PfxUInt16 i) {aabb.set16(2,i);}
SCE_PFX_FORCE_INLINE void pfxSetYMax(PfxAabb16& aabb,PfxUInt16 i) {aabb.set16(3,i);}
SCE_PFX_FORCE_INLINE void pfxSetZMin(PfxAabb16& aabb,PfxUInt16 i) {aabb.set16(4,i);}
SCE_PFX_FORCE_INLINE void pfxSetZMax(PfxAabb16& aabb,PfxUInt16 i) {aabb.set16(5,i);}
SCE_PFX_FORCE_INLINE void pfxSetXYZMin(PfxAabb16 &aabb,PfxUInt16 i,int axis) {aabb.set16(axis<<1,i);}
SCE_PFX_FORCE_INLINE void pfxSetXYZMax(PfxAabb16 &aabb,PfxUInt16 i,int axis) {aabb.set16((axis<<1)+1,i);}

SCE_PFX_FORCE_INLINE PfxUInt16 pfxGetXMin(const PfxAabb16& aabb) {return aabb.get16(0);}
SCE_PFX_FORCE_INLINE PfxUInt16 pfxGetXMax(const PfxAabb16& aabb) {return aabb.get16(1);}
SCE_PFX_FORCE_INLINE PfxUInt16 pfxGetYMin(const PfxAabb16& aabb) {return aabb.get16(2);}
SCE_PFX_FORCE_INLINE PfxUInt16 pfxGetYMax(const PfxAabb16& aabb) {return aabb.get16(3);}
SCE_PFX_FORCE_INLINE PfxUInt16 pfxGetZMin(const PfxAabb16& aabb) {return aabb.get16(4);}
SCE_PFX_FORCE_INLINE PfxUInt16 pfxGetZMax(const PfxAabb16& aabb) {return aabb.get16(5);}
SCE_PFX_FORCE_INLINE PfxUInt16 pfxGetXYZMin(const PfxAabb16 &aabb,int axis) {return aabb.get16(axis<<1);}
SCE_PFX_FORCE_INLINE PfxUInt16 pfxGetXYZMax(const PfxAabb16 &aabb,int axis) {return aabb.get16((axis<<1)+1);}

SCE_PFX_FORCE_INLINE void pfxSetXMin(PfxAabb32& aabb,PfxUInt16 i) {aabb.set16(0,i);}
SCE_PFX_FORCE_INLINE void pfxSetXMax(PfxAabb32& aabb,PfxUInt16 i) {aabb.set16(1,i);}
SCE_PFX_FORCE_INLINE void pfxSetYMin(PfxAabb32& aabb,PfxUInt16 i) {aabb.set16(2,i);}
SCE_PFX_FORCE_INLINE void pfxSetYMax(PfxAabb32& aabb,PfxUInt16 i) {aabb.set16(3,i);}
SCE_PFX_FORCE_INLINE void pfxSetZMin(PfxAabb32& aabb,PfxUInt16 i) {aabb.set16(4,i);}
SCE_PFX_FORCE_INLINE void pfxSetZMax(PfxAabb32& aabb,PfxUInt16 i) {aabb.set16(5,i);}
SCE_PFX_FORCE_INLINE void pfxSetXYZMin(PfxAabb32 &aabb,PfxUInt16 i,int axis) {aabb.set16(axis<<1,i);}
SCE_PFX_FORCE_INLINE void pfxSetXYZMax(PfxAabb32 &aabb,PfxUInt16 i,int axis) {aabb.set16((axis<<1)+1,i);}

SCE_PFX_FORCE_INLINE PfxUInt16 pfxGetXMin(const PfxAabb32& aabb) {return aabb.get16(0);}
SCE_PFX_FORCE_INLINE PfxUInt16 pfxGetXMax(const PfxAabb32& aabb) {return aabb.get16(1);}
SCE_PFX_FORCE_INLINE PfxUInt16 pfxGetYMin(const PfxAabb32& aabb) {return aabb.get16(2);}
SCE_PFX_FORCE_INLINE PfxUInt16 pfxGetYMax(const PfxAabb32& aabb) {return aabb.get16(3);}
SCE_PFX_FORCE_INLINE PfxUInt16 pfxGetZMin(const PfxAabb32& aabb) {return aabb.get16(4);}
SCE_PFX_FORCE_INLINE PfxUInt16 pfxGetZMax(const PfxAabb32& aabb) {return aabb.get16(5);}
SCE_PFX_FORCE_INLINE PfxUInt16 pfxGetXYZMin(const PfxAabb32 &aabb,int axis) {return aabb.get16(axis<<1);}
SCE_PFX_FORCE_INLINE PfxUInt16 pfxGetXYZMax(const PfxAabb32 &aabb,int axis) {return aabb.get16((axis<<1)+1);}


#define SCE_PFX_TEST_AABB(aabbA,aabbB) \
if(pfxGetXMax(aabbA) < pfxGetXMin(aabbB) || pfxGetXMin(aabbA) > pfxGetXMax(aabbB)) return false;\
if(pfxGetYMax(aabbA) < pfxGetYMin(aabbB) || pfxGetYMin(aabbA) > pfxGetYMax(aabbB)) return false;\
if(pfxGetZMax(aabbA) < pfxGetZMin(aabbB) || pfxGetZMin(aabbA) > pfxGetZMax(aabbB)) return false;\
return true;

SCE_PFX_FORCE_INLINE
bool pfxTestAabb(const PfxAabb16 &aabbA,const PfxAabb16 &aabbB)
{
SCE_PFX_TEST_AABB(aabbA,aabbB)
}

SCE_PFX_FORCE_INLINE
bool pfxTestAabb(const PfxAabb32 &aabbA,const PfxAabb32 &aabbB)
{
SCE_PFX_TEST_AABB(aabbA,aabbB)
}

SCE_PFX_FORCE_INLINE
bool pfxTestAabb(const PfxAabb32 &aabbA,const PfxAabb16 &aabbB)
{
SCE_PFX_TEST_AABB(aabbA,aabbB)
}

SCE_PFX_FORCE_INLINE
bool pfxTestAabb(const PfxAabb16 &aabbA,const PfxAabb32 &aabbB)
{
SCE_PFX_TEST_AABB(aabbA,aabbB)
}


SCE_PFX_FORCE_INLINE
PfxAabb16 pfxMergeAabb(const PfxAabb16 &aabbA,const PfxAabb16 &aabbB)
{
	PfxAabb16 aabb = aabbA;
	pfxSetXMin(aabb,SCE_PFX_MIN(pfxGetXMin(aabbA),pfxGetXMin(aabbB)));
	pfxSetXMax(aabb,SCE_PFX_MAX(pfxGetXMax(aabbA),pfxGetXMax(aabbB)));
	pfxSetYMin(aabb,SCE_PFX_MIN(pfxGetYMin(aabbA),pfxGetYMin(aabbB)));
	pfxSetYMax(aabb,SCE_PFX_MAX(pfxGetYMax(aabbA),pfxGetYMax(aabbB)));
	pfxSetZMin(aabb,SCE_PFX_MIN(pfxGetZMin(aabbA),pfxGetZMin(aabbB)));
	pfxSetZMax(aabb,SCE_PFX_MAX(pfxGetZMax(aabbA),pfxGetZMax(aabbB)));
	return aabb;
}

SCE_PFX_FORCE_INLINE
PfxAabb32 pfxMergeAabb(const PfxAabb32 &aabbA,const PfxAabb32 &aabbB)
{
	PfxAabb32 aabb = aabbA;
	pfxSetXMin(aabb,SCE_PFX_MIN(pfxGetXMin(aabbA),pfxGetXMin(aabbB)));
	pfxSetXMax(aabb,SCE_PFX_MAX(pfxGetXMax(aabbA),pfxGetXMax(aabbB)));
	pfxSetYMin(aabb,SCE_PFX_MIN(pfxGetYMin(aabbA),pfxGetYMin(aabbB)));
	pfxSetYMax(aabb,SCE_PFX_MAX(pfxGetYMax(aabbA),pfxGetYMax(aabbB)));
	pfxSetZMin(aabb,SCE_PFX_MIN(pfxGetZMin(aabbA),pfxGetZMin(aabbB)));
	pfxSetZMax(aabb,SCE_PFX_MAX(pfxGetZMax(aabbA),pfxGetZMax(aabbB)));
	return aabb;
}
} //namespace PhysicsEffects
} //namespace sce

#endif // _SCE_PFX_AABB_H
