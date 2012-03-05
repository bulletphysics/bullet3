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

#ifndef _SCE_PFX_SORT_DATA_H
#define _SCE_PFX_SORT_DATA_H

#include "../base/pfx_common.h"



namespace sce {
namespace PhysicsEffects {

#define SCE_PFX_SENTINEL_KEY	0xffffffff

struct SCE_PFX_ALIGNED(16) PfxSortData16 {
	union {
		PfxUInt8   i8data[16];
		PfxUInt16  i16data[8];
		PfxUInt32  i32data[4];
	};

void set8(int slot,PfxUInt8 data)   {i8data[slot] = data;}
void set16(int slot,PfxUInt16 data) {i16data[slot] = data;}
void set32(int slot,PfxUInt32 data) {i32data[slot] = data;}
PfxUInt8 get8(int slot)   const {return i8data[slot];}
PfxUInt16 get16(int slot) const {return i16data[slot];}
PfxUInt32 get32(int slot) const {return i32data[slot];}
};

struct SCE_PFX_ALIGNED(16) PfxSortData32 {
	union {
		PfxUInt8   i8data[32];
		PfxUInt16  i16data[16];
		PfxUInt32  i32data[8];
	};

void set8(int slot,PfxUInt8 data)   {i8data[slot] = data;}
void set16(int slot,PfxUInt16 data) {i16data[slot] = data;}
void set32(int slot,PfxUInt32 data) {i32data[slot] = data;}
PfxUInt8 get8(int slot)   const {return i8data[slot];}
PfxUInt16 get16(int slot) const {return i16data[slot];}
PfxUInt32 get32(int slot) const {return i32data[slot];}
};

SCE_PFX_FORCE_INLINE
void pfxSetKey(PfxSortData16 &sortData,PfxUInt32 key) {sortData.set32(3,key);}

SCE_PFX_FORCE_INLINE
PfxUInt32 pfxGetKey(const PfxSortData16 &sortData) {return sortData.get32(3);}

SCE_PFX_FORCE_INLINE
void pfxSetKey(PfxSortData32 &sortData,PfxUInt32 key) {sortData.set32(7,key);}

SCE_PFX_FORCE_INLINE
PfxUInt32 pfxGetKey(const PfxSortData32 &sortData) {return sortData.get32(7);}

SCE_PFX_FORCE_INLINE
PfxUInt32 pfxCreateUniqueKey(PfxUInt32 i,PfxUInt32 j)
{
	PfxUInt32 minIdx = SCE_PFX_MIN(i,j);
	PfxUInt32 maxIdx = SCE_PFX_MAX(i,j);
	return (maxIdx<<16)|(minIdx&0xffff);
}

} // namespace PhysicsEffects
} // namespace sce

#endif // _SCE_PFX_SORT_DATA_H
