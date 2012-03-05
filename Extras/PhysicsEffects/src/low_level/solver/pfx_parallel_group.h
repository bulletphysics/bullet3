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

#ifndef _SCE_PFX_PARALLEL_GROUP_H
#define _SCE_PFX_PARALLEL_GROUP_H

///////////////////////////////////////////////////////////////////////////////
// Parallel Group

#define SCE_PFX_MAX_SOLVER_PHASES 64	// 最大フェーズ数
#define SCE_PFX_MAX_SOLVER_BATCHES 32	// １フェーズに含まれる最大並列処理バッチ
#define SCE_PFX_MAX_SOLVER_PAIRS  64	// １バッチに含まれる最大ペア数
#define SCE_PFX_MIN_SOLVER_PAIRS  16	// １バッチに含まれる最小ペア数

namespace sce {
namespace PhysicsEffects {

struct SCE_PFX_ALIGNED(128) PfxParallelBatch {
	PfxUInt16 pairIndices[SCE_PFX_MAX_SOLVER_PAIRS];
};

struct SCE_PFX_ALIGNED(128) PfxParallelGroup {
	PfxUInt16 numPhases;
	PfxUInt16 numBatches[SCE_PFX_MAX_SOLVER_PHASES]; // 各フェーズの保持する並列実行可能なバッチの数
	PfxUInt16 numPairs[SCE_PFX_MAX_SOLVER_PHASES*SCE_PFX_MAX_SOLVER_BATCHES]; // 各バッチの保持するペアの数
SCE_PFX_PADDING(1,126)
};

} //namespace PhysicsEffects
} //namespace sce
#endif // _SCE_PFX_PARALLEL_GROUP_H
