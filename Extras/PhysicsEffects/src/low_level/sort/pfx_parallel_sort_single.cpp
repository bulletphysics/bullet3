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

#include "../../../include/physics_effects/base_level/base/pfx_perf_counter.h"
#include "../../../include/physics_effects/low_level/sort/pfx_parallel_sort.h"
#include "../../../include/physics_effects/base_level/sort/pfx_sort.h"

///////////////////////////////////////////////////////////////////////////////
// SINGLE THREAD

namespace sce {
namespace PhysicsEffects {

PfxInt32 pfxParallelSort(
	PfxSortData16 *data,PfxUInt32 numData,
	void *workBuff,PfxUInt32 workBytes)
{
	if(!SCE_PFX_PTR_IS_ALIGNED16(workBuff)) return SCE_PFX_ERR_INVALID_ALIGN;
	if(SCE_PFX_AVAILABLE_BYTES_ALIGN16(workBuff,workBytes) < sizeof(PfxSortData16) * numData) return SCE_PFX_ERR_OUT_OF_BUFFER;

	SCE_PFX_PUSH_MARKER("pfxParallelSort");
	pfxSort(data,(PfxSortData16*)workBuff,numData);
	SCE_PFX_POP_MARKER();

	return SCE_PFX_OK;
}

PfxInt32 pfxParallelSort(
	PfxSortData32 *data,PfxUInt32 numData,
	void *workBuff,PfxUInt32 workBytes)
{
	if(!SCE_PFX_PTR_IS_ALIGNED16(workBuff)) return SCE_PFX_ERR_INVALID_ALIGN;
	if(SCE_PFX_AVAILABLE_BYTES_ALIGN16(workBuff,workBytes) < sizeof(PfxSortData32) * numData) return SCE_PFX_ERR_OUT_OF_BUFFER;

	SCE_PFX_PUSH_MARKER("pfxParallelSort");
	pfxSort(data,(PfxSortData32*)workBuff,numData);
	SCE_PFX_POP_MARKER();

	return SCE_PFX_OK;
}

} //namespace PhysicsEffects
} //namespace sce
