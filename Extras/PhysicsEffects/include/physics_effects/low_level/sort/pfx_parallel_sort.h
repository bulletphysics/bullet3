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

#ifndef _SCE_PFX_PARALLEL_SORT_H
#define _SCE_PFX_PARALLEL_SORT_H

#include "../../base_level/sort/pfx_sort_data.h"
#include "../task/pfx_task_manager.h"

///////////////////////////////////////////////////////////////////////////////
// Parallel Sort
namespace sce {
namespace PhysicsEffects {

PfxInt32 pfxParallelSort(PfxSortData16 *data,PfxUInt32 numData,void *workBuff,PfxUInt32 workBytes);

PfxInt32 pfxParallelSort(PfxSortData32 *data,PfxUInt32 numData,void *workBuff,PfxUInt32 workBytes);

PfxInt32 pfxParallelSort(PfxSortData16 *data,PfxUInt32 numData,void *workBuff,PfxUInt32 workBytes,
	PfxTaskManager *taskManager);

PfxInt32 pfxParallelSort(PfxSortData32 *data,PfxUInt32 numData,void *workBuff,PfxUInt32 workBytes,
	PfxTaskManager *taskManager);

} //namespace PhysicsEffects
} //namespace sce
#endif // _SCE_PFX_PARALLEL_SORT_H
