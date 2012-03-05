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

#ifndef _SCE_PFX_CHECK_SOLVER_H_
#define _SCE_PFX_CHECK_SOLVER_H_

#include "../../../include/physics_effects/base_level/rigidbody/pfx_rigid_state.h"
#include "../../../include/physics_effects/base_level/solver/pfx_constraint_pair.h"

namespace sce {
namespace PhysicsEffects {

// Solver check table
/*
	-----------------MotionTypeA
	|0	1	0	1	0
	|1	1	1	1	0
	|0	1	0	1	0
	|1	1	1	0	0
	|0	0	0	0	0
MotionTypeB
 */

static SCE_PFX_FORCE_INLINE
PfxBool pfxCheckSolverTable(ePfxMotionType i,ePfxMotionType j)
{
	const PfxUInt32 solverTable = 0x00af2b80;

	SCE_PFX_ASSERT(i < kPfxMotionTypeCount);
	SCE_PFX_ASSERT(j < kPfxMotionTypeCount);

	PfxUInt32 idx = j * kPfxMotionTypeCount + i;
	PfxUInt32 mask = 1 << (kPfxMotionTypeCount*kPfxMotionTypeCount-1-idx);
	return (solverTable & mask) != 0;
}

static SCE_PFX_FORCE_INLINE
PfxBool pfxCheckSolver(const PfxConstraintPair &pair)
{
	PfxUInt32 motionA = pfxGetMotionMaskA(pair)&SCE_PFX_MOTION_MASK_TYPE;
	PfxUInt32 motionB = pfxGetMotionMaskB(pair)&SCE_PFX_MOTION_MASK_TYPE;
	PfxUInt32 sleepA = pfxGetMotionMaskA(pair)&SCE_PFX_MOTION_MASK_SLEEPING;
	PfxUInt32 sleepB = pfxGetMotionMaskB(pair)&SCE_PFX_MOTION_MASK_SLEEPING;
	
	return
		pfxGetActive(pair) &&
		pfxCheckSolverTable((ePfxMotionType)motionA,(ePfxMotionType)motionB) && // モーションタイプ別衝突判定テーブル
		!((sleepA != 0 && sleepB != 0) || (sleepA != 0 && motionB == kPfxMotionTypeFixed) || (sleepB != 0 && motionA == kPfxMotionTypeFixed));// スリープ時のチェック
}

} //namespace PhysicsEffects
} //namespace sce

#endif /* _SCE_PFX_CHECK_SOLVER_H_ */
